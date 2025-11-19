#include "pipe.hpp"

/*--------------------------------------------------------------------
Helper struct for FEM and renderer discretization
--------------------------------------------------------------------*/

struct PipeMergedSegment {

    vector<PipeComponent> components;
    scalar L_total;
    uint Ne_seg;

    PipeMergedSegment() : L_total(0), Ne_seg(0) {}

    void add_component(const PipeComponent &component) {
        components.push_back(component);
        L_total += component.L;
    }

    // Get weighted average diameter for a specific element
    pair<scalar, scalar> get_element_diameters(scalar element_start, scalar element_end) {
        scalar Do_weighted = 0.0;
        scalar Di_weighted = 0.0;
        scalar L_element = element_end - element_start;

        scalar current_pos = 0.0;
        for (const auto &comp : components) {
            scalar start = max(element_start, current_pos);
            scalar end = min(element_end, current_pos + comp.L);
            if (end > start) {
                scalar overlap = end - start;
                Do_weighted += comp.Do * overlap;
                Di_weighted += comp.Di * overlap;
            }
            current_pos += comp.L;
        }
        return {Do_weighted / L_element, Di_weighted / L_element};
    }
};

enum class ProfileShape {
    FLAT,        // Regular pipe section or middle of component
    L_LEFT,      // Start of component with tool joint
    L_RIGHT,     // End of component with tool joint
    DOUBLE_STEP, // Short component with tool joints at both ends
    MIDDLE_STEP  // For stabilizers
};

void create_pipe_fem_distribution(const Config &config, vector<PipeComponent> &pipe_assembly, vector<scalar> &ro_vec,
                                  vector<scalar> &ri_vec, vector<scalar> &S_vec, scalar &dS_min);

void create_pipe_fem_distribution_test(const Config &config, vector<PipeComponent> &pipe_assembly,
                                       vector<scalar> &ro_vec, vector<scalar> &ri_vec, vector<scalar> &S_vec,
                                       scalar &dS_min);

void create_pipe_contact_points(const Config &config, const vector<PipeComponent> &pipe_assembly,
                                const vector<scalar> &S_vec, vector<scalar> &Sc_vec, vector<scalar> &rc_vec,
                                int n_stab_cp, int n_tool_cp);

void create_bow_spring_contact_points(const Config &config, const vector<PipeComponent> &pipe_assembly,
                                      const vector<Centralizer> &centralizers, vector<scalar> &Sbs_vec);

void create_eccentric_mass_vectors(const Config &config, const vector<PipeComponent> &pipe_assembly,
                                   vector<scalar> &Sec_vec, vector<scalar> &L_ec_vec, vector<scalar> &r_ec_vec);

void create_eccentric_mass_distribution(const Config &config, const Pipe &pipe, const vector<scalar> &L_ec_vec,
                                        const vector<scalar> &Sec_vec, const vector<scalar> &r_ec_vec, byte *buf);

void create_pipe_renderer_components(const Config &config, const Pipe &pipe, vector<PipeComponent> &pipe_assembly,
                                     PipeRenderComponents *render_components, byte *buf);

template <PipeField field> inline void allocate_single_pipe_field(ArenaBump &arena_h, Pipe &pipe, uint count) {
    static_assert(field < PipeField::COUNT);
    using T = typename PipeFieldTraits<field>::variable_type;
    Offset<T> offset{};
    arena_h.allocate(&offset, count);
    pipe.offsets[(uint)field] = {.offset = offset.offset, .count = offset.count};
}

Pipe create_and_allocate_pipe(uint N, uint Nc, uint Nbs, ArenaBump &arena_h) {
    Pipe pipe = {};
    pipe.N = N;
    pipe.Nc = Nc;
    pipe.Nbs = Nbs;

    allocate_single_pipe_field<PipeField::S>(arena_h, pipe, N);
    allocate_single_pipe_field<PipeField::ro>(arena_h, pipe, N - 1);
    allocate_single_pipe_field<PipeField::ri>(arena_h, pipe, N - 1);
    allocate_single_pipe_field<PipeField::ec>(arena_h, pipe, N - 1);
    allocate_single_pipe_field<PipeField::num_ic_to_ie>(arena_h, pipe, N);
    allocate_single_pipe_field<PipeField::num_ibs_to_ie>(arena_h, pipe, N);
    allocate_single_pipe_field<PipeField::num_ir_to_ie>(arena_h, pipe, N);
    allocate_single_pipe_field<PipeField::Sc>(arena_h, pipe, Nc);
    allocate_single_pipe_field<PipeField::rc>(arena_h, pipe, Nc);
    allocate_single_pipe_field<PipeField::Sbs>(arena_h, pipe, Nbs);

    return pipe;
}

Polygon create_polygon(const PipeComponent &component, const vector<scalar> &S_splits, uint k);

Pipe pipe_create(Config &config, vector<PipeComponent> &pipe_assembly, PipeRenderComponents *render_components,
                 vector<Centralizer> &centralizers, ArenaBump &arena_h) {
    /*--------------------------------------------------------------------
    Main constructor for component based build system
    --------------------------------------------------------------------*/

    vector<scalar> ro_vec;
    vector<scalar> ri_vec;
    vector<scalar> S_vec;

    // Calculate total assembly length first
    scalar total_length = 0.0;
    for (const auto &component : pipe_assembly) {
        total_length += component.L;
    }

    if (config.dS_min_node_target > total_length) {
        THROW_RUNTIME_ERROR("Target dS_min (" + std::to_string(config.dS_min_node_target) +
                            ") cannot be larger than total assembly length (" + std::to_string(total_length) + ")\n");
    }
    scalar dS_min = total_length;
    cout << "Creating pipe fem distribution\n";
    create_pipe_fem_distribution_test(config, pipe_assembly, ro_vec, ri_vec, S_vec, dS_min);

    uint N = S_vec.size();
    assert(ri_vec.size() == N - 1);
    assert(S_vec.size() == N);

    // Next parsing the contact points
    vector<scalar> Sc_vec;
    vector<scalar> rc_vec;
    vector<scalar> Sec_vec;  // Eccentric mass positions
    vector<scalar> Sbs_vec;  // Bow-spring contact point positions
    vector<scalar> L_ec_vec; // Length span of eccentric masses
    vector<scalar> r_ec_vec; // Radial position of eccentric masses
    uint Nc, Nbs, Nec;

    if (config.conf_stat.string_type == StringType::CASING_STRING && centralizers.size() > 0) {
        Nbs = centralizers.size();
        if (Nbs > 0) {
            create_bow_spring_contact_points(config, pipe_assembly, centralizers, Sbs_vec);
        }
    } else {
        Nbs = 0;
    }

    cout << "Creating contact points distribution\n";
    create_pipe_contact_points(config, pipe_assembly, S_vec, Sc_vec, rc_vec, config.N_stab, config.N_tj);
    Nc = Sc_vec.size();

    if (config.conf_stat.string_type == StringType::DRILL_STRING) {
        create_eccentric_mass_vectors(config, pipe_assembly, Sec_vec, L_ec_vec, r_ec_vec);
        Nec = Sec_vec.size();
    } else {
        Nec = 0;
    }

    Pipe pipe = create_and_allocate_pipe(N, Nc, Nbs, arena_h);

    byte *buf = arena_h.buf;
    ArrayView<scalar> ro = pipe.get_field<PipeField::ro>(buf);
    ArrayView<scalar> ri = pipe.get_field<PipeField::ri>(buf);
    ArrayView<scalar> S = pipe.get_field<PipeField::S>(buf);
    ArrayView<scalar> ec = pipe.get_field<PipeField::ec>(buf);

    for (uint i = 0; i < N - 1; i++) {
        ro[i] = ro_vec[i];
        ri[i] = ri_vec[i];
        S[i] = S_vec[i];
        ec[i] = 0.0; // Default eccentricity to zero
    }
    S[N - 1] = S_vec[N - 1];
    assert(is_close(S_vec[N - 1], S_vec.back()));

    pipe.N = N;
    pipe.Nc = Nc;
    pipe.Nbs = Nbs;
    pipe.nu = config.conf_stat.nu;
    pipe.L_tot = S_vec.back();
    pipe.dS_min = dS_min;

    if (Nc > 0) {
        assert(Sc_vec.size() == Nc);
        assert(rc_vec.size() == Nc);
        ArrayView<scalar> Sc = pipe.get_field<PipeField::Sc>(buf);
        ArrayView<scalar> rc = pipe.get_field<PipeField::rc>(buf);
        ArrayView<uint> num_ic_to_ie = pipe.get_field<PipeField::num_ic_to_ie>(buf);
        assert(rc_vec.size() == Nc);
        for (uint i = 0; i < Nc; i++) {
            Sc[i] = Sc_vec[i];
            rc[i] = rc_vec[i];
        }

        num_ic_to_ie[0] = 0;
        // Find the element indices of the contact points
        for (uint ie = 1; ie < N; ie++) {
            num_ic_to_ie[ie] = num_ic_to_ie[ie - 1]; // Store absolute vertice index
            for (uint ic = 0; ic < Nc; ic++) {
                if (Sc[ic] >= S[ie - 1] && Sc[ic] < S[ie]) {
                    num_ic_to_ie[ie]++;
                }
            }
            assert(num_ic_to_ie[ie] >= num_ic_to_ie[ie - 1]);
        }

        num_ic_to_ie[N - 1]++;

        assert(num_ic_to_ie[N - 1] == Nc);
    }

    if (Nbs > 0) {
        assert(Sbs_vec.size() == Nbs);
        ArrayView<scalar> Sbs = pipe.get_field<PipeField::Sbs>(buf);
        ArrayView<uint> num_ibs_to_ie = pipe.get_field<PipeField::num_ibs_to_ie>(buf);
        for (uint i = 0; i < Nbs; i++) {
            Sbs[i] = Sbs_vec[i];
        }
        num_ibs_to_ie[0] = 0;
        // Find the element indices of the contact points
        for (uint ie = 1; ie < N; ie++) {
            num_ibs_to_ie[ie] = num_ibs_to_ie[ie - 1]; // Store absolute vertice index
            for (uint ibs = 0; ibs < Nbs; ibs++) {
                if (Sbs[ibs] >= S[ie - 1] && Sbs[ibs] < S[ie]) {
                    num_ibs_to_ie[ie]++;
                }
            }
            assert(num_ibs_to_ie[ie] >= num_ibs_to_ie[ie - 1]);
        }
        assert(num_ibs_to_ie[N - 1] == Nbs);
    }

    if (Nec > 0) {
        assert(config.conf_stat.string_type == StringType::DRILL_STRING);
        if (config.conf_stat.string_type == StringType::DRILL_STRING) {
            create_eccentric_mass_distribution(config, pipe, L_ec_vec, Sec_vec, r_ec_vec, buf);
        }
    }

    if (config.gui_enabled) {
        assert(config.dS_max_render_target > 0.0);
        create_pipe_renderer_components(config, pipe, pipe_assembly, render_components, buf);
    }

    return pipe;
}

void create_pipe_fem_distribution(const Config &config, vector<PipeComponent> &pipe_assembly, vector<scalar> &ro_vec,
                                  vector<scalar> &ri_vec, vector<scalar> &S_vec, scalar &dS_min) {

    vector<PipeMergedSegment> merged_segments;

    // First pass: Identify segments and merge short ones
    PipeMergedSegment current_segment;
    for (uint i = 0; i < pipe_assembly.size(); i++) {
        const PipeComponent &component = pipe_assembly[i];
        current_segment.add_component(component);

        bool is_last_component = (i == pipe_assembly.size() - 1);
        if (current_segment.L_total >= config.dS_min_node_target) {
            merged_segments.push_back(current_segment);
            current_segment = PipeMergedSegment();
        } else if (is_last_component) {
            // At the end with a short segment - merge backwards
            assert(!merged_segments.empty());

            while (current_segment.L_total < config.dS_min_node_target) {
                PipeMergedSegment &prev_segment = merged_segments.back();

                // Merge current components into previous segment
                for (auto &comp : current_segment.components) {
                    prev_segment.add_component(comp);
                }
                current_segment = prev_segment;
            }
        }
    }

    scalar L = 0.0;

    // Second pass: create discretization from merged segments
    for (auto &segment : merged_segments) {

        uint Ne_seg = (uint)floor(segment.L_total / config.dS_min_node_target);
        assert(Ne_seg > 0);
        scalar Le = segment.L_total / Ne_seg;
        segment.Ne_seg = Ne_seg;
        dS_min = min(Le, dS_min);
        // Create elements
        for (uint k = 0; k < Ne_seg; k++) {
            scalar element_start = k * Le;
            scalar element_end = element_start + Le;
            const auto [Do_avg, Di_avg] = segment.get_element_diameters(element_start, element_end);
            ro_vec.push_back(Do_avg / 2.0);
            ri_vec.push_back(Di_avg / 2.0);
            S_vec.push_back(element_start + L);
        }
        L += segment.L_total;
    }
    S_vec.push_back(L);

    // Third pass: Find the top node for each component
    scalar component_start = 0.0;
    uint i_component = 0;
    for (const auto &segment : merged_segments) {
        for (const auto &component : segment.components) {
            // Topnode: Either at the top of the component or above it (if no coinding node)
            for (uint i = 0; i < S_vec.size(); i++) {
                if (S_vec[i] <= component_start) {
                    pipe_assembly[i_component].i_top_node = i;
                }
            }
            assert(pipe_assembly[i_component].i_top_node < S_vec.size());
            i_component++;
            component_start += component.L;
        }
    }
}

void create_pipe_fem_distribution_test(const Config &config, vector<PipeComponent> &pipe_assembly,
                                       vector<scalar> &ro_vec, vector<scalar> &ri_vec, vector<scalar> &S_vec,
                                       scalar &dS_min) {

    vector<scalar> special_nodes; // To store required node positions (e.g., stabilizer centers)
    scalar L = 0.0;
    for (const auto &comp : pipe_assembly) {
        if (comp.has_stabilizer) {
            scalar stab_center = L + comp.S_stabilizer;
            special_nodes.push_back(stab_center);
        }
        L += comp.L;
    }

    // Remove duplicates and sort
    std::sort(special_nodes.begin(), special_nodes.end());
    special_nodes.erase(std::unique(special_nodes.begin(), special_nodes.end(),
                                    [](scalar a, scalar b) { return std::abs(a - b) < SMALL_SCALAR; }),
                        special_nodes.end());

    vector<PipeMergedSegment> merged_segments;

    // First pass: Identify segments and merge short ones (unchanged)
    PipeMergedSegment current_segment;
    for (uint i = 0; i < pipe_assembly.size(); i++) {
        const PipeComponent &component = pipe_assembly[i];
        current_segment.add_component(component);

        bool is_last_component = (i == pipe_assembly.size() - 1);
        if (current_segment.L_total >= config.dS_min_node_target) {
            merged_segments.push_back(current_segment);
            current_segment = PipeMergedSegment();
        } else if (is_last_component) {
            assert(!merged_segments.empty());
            while (current_segment.L_total < config.dS_min_node_target) {
                PipeMergedSegment &prev_segment = merged_segments.back();
                for (auto &comp : current_segment.components) {
                    prev_segment.add_component(comp);
                }
                current_segment = prev_segment;
            }
        }
    }

    L = 0.0;
    S_vec.clear();

    // Second pass: create discretization from merged segments
    for (auto &segment : merged_segments) {
        scalar seg_start = L;
        scalar seg_end = L + segment.L_total;

        // Find all special nodes (stabilizer centers) in this segment
        vector<scalar> seg_special_nodes;
        for (scalar s : special_nodes) {
            if (s > seg_start + SMALL_SCALAR && s < seg_end - SMALL_SCALAR)
                seg_special_nodes.push_back(s);
        }

        // Always start with the segment start
        vector<scalar> nodes;
        nodes.push_back(seg_start);

        // Insert special nodes in order
        for (scalar s : seg_special_nodes)
            nodes.push_back(s);

        // Always end with the segment end
        nodes.push_back(seg_end);

        // Now, for each interval between nodes, distribute nodes as needed
        for (size_t j = 0; j < nodes.size() - 1; ++j) {
            scalar interval_start = nodes[j];
            scalar interval_end = nodes[j + 1];
            scalar interval_length = interval_end - interval_start;

            // Compute number of elements in this interval
            uint Ne = std::max(1u, static_cast<uint>(std::round(interval_length / config.dS_min_node_target)));
            scalar Le = interval_length / Ne;
            dS_min = std::min(Le, dS_min);

            for (uint k = 0; k < Ne; ++k) {
                scalar S = interval_start + k * Le;
                if (S_vec.empty() || std::abs(S - S_vec.back()) > SMALL_SCALAR)
                    S_vec.push_back(S);
            }
        }
        // Ensure the segment end is included
        if (S_vec.empty() || std::abs(seg_end - S_vec.back()) > SMALL_SCALAR)
            S_vec.push_back(seg_end);

        L += segment.L_total;
    }

    // Now fill ro_vec and ri_vec as before
    for (size_t i = 0; i < S_vec.size() - 1; ++i) {
        scalar element_start = S_vec[i];
        scalar element_end = S_vec[i + 1];
        // Find which segment this element belongs to
        scalar seg_L = 0.0;
        for (auto &segment : merged_segments) {
            if (element_start >= seg_L && element_end <= seg_L + segment.L_total + SMALL_SCALAR) {
                const auto [Do_avg, Di_avg] = segment.get_element_diameters(element_start - seg_L, element_end - seg_L);
                ro_vec.push_back(Do_avg / 2.0);
                ri_vec.push_back(Di_avg / 2.0);
                break;
            }
            seg_L += segment.L_total;
        }
    }

    // Third pass: Find the top node for each component (unchanged)
    scalar component_start = 0.0;
    uint i_component = 0;
    for (const auto &segment : merged_segments) {
        for (const auto &component : segment.components) {
            for (uint i = 0; i < S_vec.size(); i++) {
                if (S_vec[i] <= component_start) {
                    pipe_assembly[i_component].i_top_node = i;
                }
            }
            assert(pipe_assembly[i_component].i_top_node < S_vec.size());
            i_component++;
            component_start += component.L;
        }
    }
}

// Helper: Returns correct outer radius at local position within a PipeComponent
scalar get_pipe_radius_at_S(const PipeComponent &comp, scalar local_S) {
    if (comp.has_tool_joints && local_S <= comp.L_tool + SMALL_SCALAR)
        return comp.D_tool / 2.0;
    if (comp.has_tool_joints && local_S >= comp.L - comp.L_tool - SMALL_SCALAR)
        return comp.D_tool / 2.0;
    if (comp.has_stabilizer) {
        scalar stab_start = comp.S_stabilizer - 0.5 * comp.L_stabilizer;
        scalar stab_end = comp.S_stabilizer + 0.5 * comp.L_stabilizer;
        if (local_S >= stab_start - SMALL_SCALAR && local_S <= stab_end + SMALL_SCALAR)
            return comp.D_stabilizer / 2.0;
    }
    return comp.Do / 2.0;
}

scalar get_max_contact_radius_at_S(const vector<PipeComponent> &pipe_assembly, scalar S) {
    constexpr scalar EPS = SMALL_SCALAR;
    scalar L = 0.0;
    for (size_t i = 0; i < pipe_assembly.size(); ++i) {
        const PipeComponent &comp = pipe_assembly[i];
        scalar start = L;
        scalar end = L + comp.L;
        // Check if S is at the start of this component
        if (std::abs(S - start) < EPS && i > 0) {
            // At start: compare previous end and current start
            scalar local_S_prev = pipe_assembly[i - 1].L;
            scalar r_prev = get_pipe_radius_at_S(pipe_assembly[i - 1], local_S_prev);
            scalar r_curr = get_pipe_radius_at_S(comp, 0.0);
            return std::max(r_prev, r_curr);
        }
        // Check if S is at the end of this component
        if (std::abs(S - end) < EPS && i + 1 < pipe_assembly.size()) {
            // At end: compare current end and next start
            scalar r_curr = get_pipe_radius_at_S(comp, comp.L);
            scalar r_next = get_pipe_radius_at_S(pipe_assembly[i + 1], 0.0);
            return std::max(r_curr, r_next);
        }
        // If S is inside this component
        if (S > start - EPS && S < end + EPS) {
            scalar local_S = S - start;
            return get_pipe_radius_at_S(comp, local_S);
        }
        L += comp.L;
    }
    // If not found, fallback to last component
    const PipeComponent &last = pipe_assembly.back();
    scalar local_S = S - (L - last.L);
    return get_pipe_radius_at_S(last, local_S);
}

void create_pipe_contact_points(const Config &config, const vector<PipeComponent> &pipe_assembly,
                                const vector<scalar> &S_vec, vector<scalar> &Sc_vec, vector<scalar> &rc_vec,
                                int n_stab_cp = 1, int n_tool_cp = 1) {
    assert(config.dS_max_contact_target > 0.0);

    auto is_node = [&](scalar S) {
        return std::any_of(S_vec.begin(), S_vec.end(), [&](scalar s) { return std::abs(s - S) < SMALL_SCALAR; });
    };

    // 1. Add all computational nodes
    for (scalar S : S_vec)
        Sc_vec.push_back(S);

    // 2. Add start of each pipe component (unless already present)
    scalar L = 0.0;
    for (const auto &comp : pipe_assembly) {
        bool exists =
            std::any_of(Sc_vec.begin(), Sc_vec.end(), [&](scalar s) { return std::abs(s - L) < SMALL_SCALAR; });
        if (!exists)
            Sc_vec.push_back(L);
        L += comp.L;
    }

    // 3. Add extra contact points within each FEM element (including start and end)
    for (size_t ie = 0; ie < S_vec.size() - 1; ++ie) {
        scalar elem_start = S_vec[ie];
        scalar elem_end = S_vec[ie + 1];
        scalar elem_length = elem_end - elem_start;

        uint N_intervals = std::max(1u, static_cast<uint>(std::floor(elem_length / config.dS_max_contact_target)));
        scalar dS = elem_length / N_intervals;

        for (uint k = 1; k < N_intervals; ++k) {
            scalar S = elem_start + k * dS;
            bool exists =
                std::any_of(Sc_vec.begin(), Sc_vec.end(), [&](scalar s) { return std::abs(s - S) < SMALL_SCALAR; });
            if (!exists)
                Sc_vec.push_back(S);
        }
    }

    // 4. Add contact points for stabilizers and tool joints
    L = 0.0;
    for (const auto &comp : pipe_assembly) {
        // --- Stabilizer ---
        if (comp.has_stabilizer && n_stab_cp > 0) {
            scalar stab_center = L + comp.S_stabilizer;
            scalar stab_start = stab_center - 0.5 * comp.L_stabilizer;
            scalar stab_end = stab_center + 0.5 * comp.L_stabilizer;

            for (int i = 0; i < n_stab_cp; ++i) {
                scalar frac = (n_stab_cp == 1) ? 0.5 : static_cast<scalar>(i) / (n_stab_cp - 1);
                scalar stab_pos = stab_start + frac * (stab_end - stab_start);
                bool exists = std::any_of(Sc_vec.begin(), Sc_vec.end(),
                                          [&](scalar s) { return std::abs(s - stab_pos) < SMALL_SCALAR; });
                if (!exists)
                    Sc_vec.push_back(stab_pos);
            }

            // Remove any other contact points within the stabilizer span unless they are nodes or just added
            Sc_vec.erase(std::remove_if(Sc_vec.begin(), Sc_vec.end(),
                                        [&](scalar s) {
                                            bool in_span =
                                                (s > stab_start + SMALL_SCALAR && s < stab_end - SMALL_SCALAR);
                                            bool is_added = false;
                                            for (int i = 0; i < n_stab_cp; ++i) {
                                                scalar frac =
                                                    (n_stab_cp == 1) ? 0.5 : static_cast<scalar>(i) / (n_stab_cp - 1);
                                                scalar stab_pos = stab_start + frac * (stab_end - stab_start);
                                                if (std::abs(s - stab_pos) < SMALL_SCALAR) {
                                                    is_added = true;
                                                    break;
                                                }
                                            }
                                            return in_span && !is_node(s) && !is_added;
                                        }),
                         Sc_vec.end());
        }

        // --- Tool joints ---
        if (comp.has_tool_joints && n_tool_cp > 0) {
            // Start tool joint
            scalar tj_start = L;
            scalar tj_end = L + comp.L_tool;
            for (int i = 0; i < n_tool_cp; ++i) {
                scalar frac = (n_tool_cp == 1) ? 0.5 : static_cast<scalar>(i) / (n_tool_cp - 1);
                scalar tj_pos = tj_start + frac * (tj_end - tj_start);
                bool exists = std::any_of(Sc_vec.begin(), Sc_vec.end(),
                                          [&](scalar s) { return std::abs(s - tj_pos) < SMALL_SCALAR; });
                if (!exists)
                    Sc_vec.push_back(tj_pos);
            }
            // End tool joint
            tj_start = L + comp.L - comp.L_tool;
            tj_end = L + comp.L;
            for (int i = 0; i < n_tool_cp; ++i) {
                scalar frac = (n_tool_cp == 1) ? 0.5 : static_cast<scalar>(i) / (n_tool_cp - 1);
                scalar tj_pos = tj_start + frac * (tj_end - tj_start);
                bool exists = std::any_of(Sc_vec.begin(), Sc_vec.end(),
                                          [&](scalar s) { return std::abs(s - tj_pos) < SMALL_SCALAR; });
                if (!exists)
                    Sc_vec.push_back(tj_pos);
            }
        }
        L += comp.L;
    }

    // 5. Add contact point at the end of the string (unless already present)
    scalar L_tot = S_vec.back();
    bool exists =
        std::any_of(Sc_vec.begin(), Sc_vec.end(), [&](scalar s) { return std::abs(s - L_tot) < SMALL_SCALAR; });
    if (!exists)
        Sc_vec.push_back(L_tot);

    // 6. Sort Sc_vec in ascending order
    std::sort(Sc_vec.begin(), Sc_vec.end());

    // 7. Calculate rc_vec for each contact point
    rc_vec.clear();
    for (scalar S : Sc_vec) {
        rc_vec.push_back(get_max_contact_radius_at_S(pipe_assembly, S));
    }

    // Override last contact point radius with bit radius if applicable
    if (config.pipe_solver_type == PipeSolverType::CURVILINEAR &&
        config.conf_stat.string_type == StringType::DRILL_STRING &&
        config.conf_stat.bc_bit_rock_type != BC_BitRockType::NO_BIT) {
        rc_vec.back() = config.conf_stat.r_br; // Slightly smaller than bit radius
    }
}

void create_bow_spring_contact_points(const Config &config, const vector<PipeComponent> &pipe_assembly,
                                      const vector<Centralizer> &centralizers, vector<scalar> &Sbs_vec) {

    // Add centralizers in reverse order to ensure Sbs_vec is increasing
    for (int i = centralizers.size() - 1; i >= 0; i--) {
        Sbs_vec.push_back(centralizers[i].S_cent);

        for (uint j = 0; j < pipe_assembly.size(); j++) {
            const PipeComponent &component = pipe_assembly[j];
            scalar segment_start = 0.0;
            for (uint k = 0; k < j; k++) {
                segment_start += pipe_assembly[k].L;
            }
            scalar segment_end = segment_start + component.L;
            if (centralizers[i].S_cent >= segment_start && centralizers[i].S_cent <= segment_end) {
                scalar r_o = component.Do / 2.0;
                if (!(config.conf_stat.r_bs_outer > r_o)) {
                    std::cerr << "Error: Bow-spring radius r_bs (" << config.conf_stat.r_bs_outer
                              << ") is not greater than pipe outer radius r_o (" << r_o
                              << ") at position S = " << centralizers[i].S_cent << std::endl;
                    exit(EXIT_FAILURE);
                }
                break;
            }
        }
    }

    for (int i = 0; i < (int)centralizers.size() - 1; i++) {
        assert(Sbs_vec[i] < Sbs_vec[i + 1]);
    }
}

void create_eccentric_mass_distribution(const Config &config, const Pipe &pipe, const vector<scalar> &length_ec_vec,
                                        const vector<scalar> &Sec_vec, const vector<scalar> &r_ec_vec, byte *buf) {
    ArrayView<scalar> ec = pipe.get_field<PipeField::ec>(buf);
    ArrayView<scalar> S_pipe = pipe.get_field<PipeField::S>(buf);

    scalar S_min = S_pipe[0];
    scalar S_max = S_pipe[pipe.N - 1];

    for (uint i = 0; i < length_ec_vec.size(); i++) {
        scalar L_ec = length_ec_vec[i];
        scalar S_ec = Sec_vec[i];
        scalar r_ec = r_ec_vec[i];

        // Centered interval, clipped to string
        scalar S_ec_start = std::max(S_ec - 0.5 * L_ec, S_min);
        scalar S_ec_end = std::min(S_ec + 0.5 * L_ec, S_max);

        for (uint j = 0; j < pipe.N - 1; j++) {
            scalar elem_start = S_pipe[j];
            scalar elem_end = S_pipe[j + 1];

            // Compute overlap between [S_start, S_end] and [elem_start, elem_end]
            scalar overlap = std::max(0.0, std::min(S_ec_end, elem_end) - std::max(S_ec_start, elem_start));
            scalar dS = elem_end - elem_start;
            if (overlap > 0.0 && dS > 0.0) {
                ec[j] += r_ec * overlap / dS;
            }
        }
    }
}
void create_eccentric_mass_vectors(const Config &config, const vector<PipeComponent> &pipe_assembly,
                                   vector<scalar> &Sec_vec, vector<scalar> &L_ec_vec, vector<scalar> &r_ec_vec) {

    scalar L = 0.0;
    for (const auto &component : pipe_assembly) {
        for (const EccentricMass &em : component.eccentric_masses) {
            Sec_vec.push_back(L + em.S_ecc);
            L_ec_vec.push_back(em.L_ecc);
            r_ec_vec.push_back(em.r_ecc);
        }
        L += component.L;
    }
}
void create_pipe_renderer_components(const Config &config, const Pipe &pipe, vector<PipeComponent> &pipe_assembly,
                                     PipeRenderComponents *render_components, byte *buf) {
    scalar dS_max_target = config.dS_max_render_target;

    vector<scalar> &arr_S = render_components->arr_S;
    vector<uint> &arr_ie = render_components->arr_ie;
    vector<uint> &arr_ip = render_components->arr_ip;
    vector<Polygon> &arr_polygons = render_components->polygons;

    ArrayView<scalar> S_pipe = pipe.get_field<PipeField::S>(buf);
    uint i_poly_count = 0;
    scalar global_S = 0.0; // Current position along entire pipe
    arr_S.push_back(global_S);
    uint ip = 0;
    for (auto &component : pipe_assembly) {

        component.i_top_poly = i_poly_count;
        scalar component_pos = 0.0; // Position within current component
        scalar remaining_length = component.L;
        vector<scalar> S_splits;
        scalar fem_offset = 0.0; // Offset within current FEM element

        // Map component onto FEM elements
        for (uint ie = 0; ie < pipe.N - 1 && abs(remaining_length) > SMALL_SCALAR; ie++) {
            if (component_pos >= S_pipe[ie] - global_S && component_pos < S_pipe[ie + 1] - global_S) {
                scalar fem_start = S_pipe[ie] + fem_offset;
                scalar fem_length = S_pipe[ie + 1] - fem_start;
                scalar segment_length = min(fem_length, remaining_length);

                // Split segment into appropriate number of renderer components
                uint num_splits = max(1u, (uint)ceil(segment_length / dS_max_target));
                scalar sub_length = segment_length / num_splits;

                for (uint i = 0; i < num_splits; i++) {
                    S_splits.push_back(component_pos + (i + 1) * sub_length);
                }

                remaining_length -= segment_length;
                component_pos += segment_length;

                fem_offset = (segment_length < fem_length) ? fem_offset + segment_length : 0.0;
            }
        }

        // Create render components based on splits
        for (uint k = 0; k < S_splits.size(); k++) {
            arr_S.push_back(global_S + S_splits[k]);
            arr_ip.push_back(ip);
            arr_polygons.emplace_back(create_polygon(component, S_splits, k));
            i_poly_count++;
        }

        global_S += component.L;
        ip++;
    }

    // Second pass: Map S coordinates to FEM elements
    for (uint i = 0; i < arr_S.size(); i++) {
        scalar S_i = arr_S[i];
        for (uint ie = 0; ie < pipe.N - 1; ie++) {
            if (S_i >= S_pipe[ie] && S_i < S_pipe[ie + 1]) {
                arr_ie.push_back(ie);
                break;
            }
        }
    }
    assert(arr_ie.size() == arr_S.size() - 1);
}

struct ProfileDimensions {
    scalar L1 = 0, L2 = 0, L3 = 0, L4 = 0;
    scalar D1 = 0, D2 = 0, D3 = 0, D4 = 0;
    // Add more as needed
};

ProfileShape classify_profile_shape_and_dimensions(const PipeComponent &component, const vector<scalar> &S_splits,
                                                   uint k, ProfileDimensions &dims) {
    scalar start_pos = (k == 0) ? 0 : S_splits[k - 1];
    scalar end_pos = S_splits[k];
    scalar Lc = end_pos - start_pos;

    if (component.has_tool_joints) {
        bool has_start_tool = start_pos < component.L_tool;
        bool has_end_tool = end_pos > (component.L - component.L_tool);

        if (has_start_tool && has_end_tool) {
            // DOUBLE_STEP
            dims.L1 = component.L_tool;
            dims.L2 = component.L - component.L_tool;
            dims.L3 = component.L;
            dims.D1 = component.D_tool;
            dims.D2 = component.Do;
            dims.D3 = component.D_tool;
            return ProfileShape::DOUBLE_STEP;
        }
        if (has_start_tool) {
            // L_LEFT
            dims.L1 = component.L_tool;
            dims.L2 = Lc;
            dims.D1 = component.D_tool;
            dims.D2 = component.Do;
            return ProfileShape::L_LEFT;
        }
        if (has_end_tool) {
            // L_RIGHT
            dims.L1 = Lc - component.L_tool;
            dims.L2 = Lc;
            dims.D1 = component.Do;
            dims.D2 = component.D_tool;
            return ProfileShape::L_RIGHT;
        }
    } else if (component.has_stabilizer) {
        scalar stab_start = component.S_stabilizer - 0.5 * component.L_stabilizer;
        scalar stab_end = component.S_stabilizer + 0.5 * component.L_stabilizer;

        // Fully contains stabilizer
        if (start_pos <= stab_start + SMALL_SCALAR && end_pos >= stab_end - SMALL_SCALAR) {
            // MIDDLE_STEP (stabilizer fully inside polygon)
            dims.L1 = stab_start - start_pos;                          // Length before stabilizer
            dims.L2 = stab_start - start_pos + component.L_stabilizer; // End of stabilizer relative to start_pos
            dims.L3 = Lc;                                              // End of polygon
            dims.D1 = component.Do;
            dims.D2 = component.D_stabilizer;
            dims.D3 = component.Do;
            return ProfileShape::MIDDLE_STEP;
        }
        // Left split on stabilizer
        if (start_pos < stab_start && end_pos > stab_start && end_pos <= stab_end) {
            dims.L1 = stab_start - start_pos;
            dims.L2 = Lc;
            dims.D1 = component.Do;
            dims.D2 = component.D_stabilizer;
            return ProfileShape::L_LEFT;
        }
        // Right split on stabilizer
        if (start_pos >= stab_start && start_pos < stab_end && end_pos > stab_end) {
            dims.L1 = stab_end - start_pos;
            dims.L2 = Lc;
            dims.D1 = component.D_stabilizer;
            dims.D2 = component.Do;
            return ProfileShape::L_RIGHT;
        }
        // Polygon is entirely within stabilizer (rare, but possible)
        if (start_pos >= stab_start && end_pos <= stab_end) {
            dims.L1 = Lc;
            dims.D1 = component.D_stabilizer;
            return ProfileShape::FLAT;
        }
    }

    // Default: flat
    dims.L1 = Lc;
    dims.D1 = component.Do;
    return ProfileShape::FLAT;
}

Polygon create_polygon(const PipeComponent &component, const vector<scalar> &S_splits, uint k) {
    scalar start_pos = (k == 0) ? 0 : S_splits[k - 1];
    ProfileDimensions dims;
    ProfileShape shape = classify_profile_shape_and_dimensions(component, S_splits, k, dims);

    Polygon poly;
    poly.push_back(Vec2(0.0, component.Di / 2.0));

    switch (shape) {
    case ProfileShape::FLAT:
        poly.push_back(Vec2(0.0, dims.D1 / 2.0));
        poly.push_back(Vec2(dims.L1, dims.D1 / 2.0));
        poly.push_back(Vec2(dims.L1, component.Di / 2.0));
        break;
    case ProfileShape::L_LEFT:
        poly.push_back(Vec2(0.0, dims.D1 / 2.0));
        poly.push_back(Vec2(dims.L1, dims.D1 / 2.0));
        poly.push_back(Vec2(dims.L1, dims.D2 / 2.0));
        poly.push_back(Vec2(dims.L2, dims.D2 / 2.0));
        poly.push_back(Vec2(dims.L2, component.Di / 2.0));
        break;
    case ProfileShape::L_RIGHT:
        poly.push_back(Vec2(0.0, dims.D1 / 2.0));
        poly.push_back(Vec2(dims.L1, dims.D1 / 2.0));
        poly.push_back(Vec2(dims.L1, dims.D2 / 2.0));
        poly.push_back(Vec2(dims.L2, dims.D2 / 2.0));
        poly.push_back(Vec2(dims.L2, component.Di / 2.0));
        break;
    case ProfileShape::DOUBLE_STEP:
        poly.push_back(Vec2(0.0, dims.D1 / 2.0));
        poly.push_back(Vec2(dims.L1, dims.D1 / 2.0));
        poly.push_back(Vec2(dims.L1, dims.D2 / 2.0));
        poly.push_back(Vec2(dims.L2, dims.D2 / 2.0));
        poly.push_back(Vec2(dims.L2, dims.D3 / 2.0));
        poly.push_back(Vec2(dims.L3, dims.D3 / 2.0));
        poly.push_back(Vec2(dims.L3, component.Di / 2.0));
        break;
    case ProfileShape::MIDDLE_STEP:
        poly.push_back(Vec2(0.0, dims.D1 / 2.0));
        poly.push_back(Vec2(dims.L1, dims.D1 / 2.0));
        poly.push_back(Vec2(dims.L1, dims.D2 / 2.0));
        poly.push_back(Vec2(dims.L2, dims.D2 / 2.0));
        poly.push_back(Vec2(dims.L2, dims.D3 / 2.0));
        poly.push_back(Vec2(dims.L3, dims.D3 / 2.0));
        poly.push_back(Vec2(dims.L3, component.Di / 2.0));
        break;
    default:
        assert(false);
    }

    // Reverse order for correct winding if needed
    Polygon poly_correct_order;
    for (int i = poly.size() - 1; i >= 0; i--) {
        poly_correct_order.push_back(poly[i]);
    }
    assert(poly_correct_order.size() == poly.size());

    return poly_correct_order;
}

void save_pipe_csv(const Config &config, const Pipe &pipe, const byte *buf) {
    using namespace std;
    // const string filename = config.get_current_output_subdir() + "pipe.csv";
    const path file_path = config.output_dir / "pipe.csv";
    ofstream ost{file_path.c_str()};
    if (!ost) {
        throw runtime_error("Failed to open pipe geometry file: " + file_path.string() + "\n");
    }

    ArrayView<scalar> S = pipe.get_field<PipeField::S>(buf);
    ArrayView<scalar> ro = pipe.get_field<PipeField::ro>(buf);
    ArrayView<scalar> ri = pipe.get_field<PipeField::ri>(buf);
    ArrayView<scalar> ec = pipe.get_field<PipeField::ec>(buf);
    /*--------------------------------------------------------------------
    Create header
    --------------------------------------------------------------------*/
    ost << "N, Nc, L_tot, dS_min\n" << pipe.N << ", " << pipe.Nc << ", " << pipe.L_tot << ", " << pipe.dS_min << "\n";

    ost << "S, r_o, r_i, e_c, num_ir_to_ie, num_ic_to_ie\n";

    if (config.conf_stat.string_type == StringType::CASING_STRING) {
        ost << ", num_ibs_to_ie";
    }
    ost << "\n";

    ArrayView<uint> num_ic_to_ie = pipe.get_field<PipeField::num_ic_to_ie>(buf);
    ArrayView<uint> num_ibs_to_ie = pipe.get_field<PipeField::num_ibs_to_ie>(buf);
    ArrayView<uint> num_ir_to_ie = pipe.get_field<PipeField::num_ir_to_ie>(buf);

    for (uint i = 0; i < pipe.N - 1; i++) {
        ost << S[i] << ", " << ro[i] << ", " << ri[i] << ", " << ec[i] << ", " << num_ir_to_ie[i] << ", "
            << num_ic_to_ie[i];
        if (config.conf_stat.string_type == StringType::CASING_STRING) {
            ost << ", " << num_ibs_to_ie[i];
        }
        ost << "\n";
    }
    ost << S[pipe.N - 1] << ", "
        << "-1"
        << ", "
        << "-1"
        << ", "
        << "-1"
        << ", " << num_ic_to_ie[pipe.N - 1];
    if (config.conf_stat.string_type == StringType::CASING_STRING) {
        ost << ", " << num_ibs_to_ie[pipe.N - 1];
    }
    ost << "\n";

    // Printing contact points
    ost << "\n";
    ost << "Sc, rc\n";
    ArrayView<scalar> Sc = pipe.get_field<PipeField::Sc>(buf);
    ArrayView<scalar> rc = pipe.get_field<PipeField::rc>(buf);
    for (uint i = 0; i < pipe.Nc; i++) {
        ost << Sc[i] << ", " << rc[i] << "\n";
    }

    if (pipe.Nbs > 0) {
        ost << "\n";
        ost << "Sbs\n";
        ArrayView<scalar> Sbs = pipe.get_field<PipeField::Sbs>(buf);

        for (uint i = 0; i < pipe.Nbs; i++) {
            ost << Sbs[i] << "\n";
        }
    }
}
