
#include "pipe-renderer.hpp"
#include "sim-tools/math.hpp"

constexpr uint NUM_POINTS_CIRCUMF = 80;
constexpr scalar MAX_ANGLE_DEG_UNIQUE_VERTEX_NORMAL = 10;

inline glm::vec3 get_color_from_misc_graph_nodewise(ConfigGraphics &config_graphics, uint ie, scalar xi,
                                                    const MiscSpatialGraph &misc_graph) {

    const vector<scalar> &misc_vec = misc_graph.data;
    // linear interpolation for now

    assert(ie < misc_vec.size() - 1);
    const scalar value = misc_vec[ie] + xi * (misc_vec[ie + 1] - misc_vec[ie]);
    ColorMapType color_map_type = config_graphics.color_map_type;
    const glm::vec3 color =
        sito::get_color_from_colormap(value, config_graphics.contour_lim_min, config_graphics.contour_lim_max,
                                      color_map_type, config_graphics.grey_outside_limits);
    return color;
}

inline glm::vec3 get_color_from_fluid_graph_nodewise(ConfigGraphics &config_graphics, uint ie, scalar xi,
                                                     const FluidSpatialGraph &fluid_graph) {
    const vector<scalar> &fluid_vec = fluid_graph.data;
    // fluid_vec layout:
    //   fluid_vec[0]       = value at first node (node 0)
    //   fluid_vec[1..Ne]   = values at midpoints of elements 0..Ne-1
    //   fluid_vec[Ne+1]    = value at last node (node N-1)
    //
    // Build node-wise values by averaging adjacent element-midpoint values for interior nodes,
    // then linearly interpolate along the element using xi.
    assert(fluid_vec.size() >= 3);        // at least one element + two end node values
    const uint Ne = fluid_vec.size() - 2; // number of elements
    assert(ie < Ne);
    assert(xi >= -SMALL_SCALAR && xi <= 1.0 + SMALL_SCALAR);

    // Compute node values on the fly
    // node[0] = fluid_vec[0]
    // node[k] = 0.5*(fluid_vec[k] + fluid_vec[k+1]) for k = 1..Ne-1
    // node[Ne] = fluid_vec[Ne+1]
    scalar v_left;
    if (ie == 0) {
        v_left = fluid_vec[0];
    } else {
        // node ie = average of midpoint of element ie-1 (fluid_vec[ie]) and midpoint of element ie (fluid_vec[ie+1])
        v_left = 0.5 * (fluid_vec[ie] + fluid_vec[ie + 1]);
    }

    scalar v_right;
    if (ie + 1 == Ne) {
        v_right = fluid_vec[Ne + 1];
    } else {
        // node ie+1 = average of midpoint of element ie (fluid_vec[ie+1]) and midpoint of element ie+1
        // (fluid_vec[ie+2])
        v_right = 0.5 * (fluid_vec[ie + 1] + fluid_vec[ie + 2]);
    }

    const scalar value = v_left + xi * (v_right - v_left);

    ColorMapType color_map_type = config_graphics.color_map_type;
    const glm::vec3 color =
        sito::get_color_from_colormap(value, config_graphics.contour_lim_min, config_graphics.contour_lim_max,
                                      color_map_type, config_graphics.grey_outside_limits);
    return color;
}

inline glm::vec3 get_color_from_von_mises_stress(const Config &config, ConfigGraphics &config_graphics, uint ie,
                                                 scalar xi, const curvlin::PipeSolver *pipesolver_curvlin,
                                                 const Pipe &pipe, const Hole &hole, const byte *buf) {

    const scalar value = pipesolver_curvlin->compute_vm_stress_curvlin(ie, xi, pipe, hole, config, buf) / 1.0e6;
    ColorMapType color_map_type = config_graphics.color_map_type;
    const glm::vec3 color =
        sito::get_color_from_colormap(value, config_graphics.contour_lim_min, config_graphics.contour_lim_max,
                                      color_map_type, config_graphics.grey_outside_limits);
    config_graphics.contour_val_max = max(config_graphics.contour_val_max, (float)value);
    config_graphics.contour_val_min = min(config_graphics.contour_val_min, (float)value);
    return color;
}

PipeRenderer::PipeRenderer(const Config &config, const Pipe &pipe, const PipeRenderComponents &pipe_render_components,
                           const vector<PipeComponent> &pipe_assembly, const byte *buf) {
    arr_S_poly = pipe_render_components.arr_S;
    ptr_poly_to_ie = pipe_render_components.arr_ie;

    // Map the fine polygonal mesh to whole components
    scalar current_S = 0;
    uint current_poly = 0;

    component_info.reserve(component_info.size());

    for (const auto &comp : pipe_assembly) {
        ComponentInfo info;
        info.name = comp.name;
        info.L = comp.L;
        info.start_poly = current_poly;

        // Find the last polygon that belongs to this component
        scalar target_S = current_S + comp.L;
        while (current_poly < arr_S_poly.size() - 1 && arr_S_poly[current_poly] < target_S) {
            current_poly++;
        }
        info.end_poly = current_poly;
        current_S = target_S;

        component_info.push_back(info);
    }

    generate_mesh(config, pipe_assembly, pipe_render_components);

    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertex_buffer.size() * sizeof(Vertex), nullptr, GL_DYNAMIC_DRAW);
    Vertex::set_vertex_attrib_pointer(VBO);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, index_buffer.size() * sizeof(uint), index_buffer.data(), GL_STATIC_DRAW);

    glBindVertexArray(0);

    // Triad generation
    pipe_triads.resize(pipe.N);

    for (uint i = 0; i < pipe.N; i++) {
        pipe_triads[i].load();
    }
}

inline void add_polygon_ring_to_mesh(scalar S_begin, scalar S_comp_begin, Vec2 pA, Vec2 pB, Vec2 nA, Vec2 nB, uint ip,
                                     const PipeComponent &comp, vector<PipeRenderer::MeshVertex> &mesh,
                                     vector<uint> &index_buffer) {
    const uint prev = mesh.size();
    for (uint i = 0; i < NUM_POINTS_CIRCUMF; i++) {
        PipeRenderer::MeshVertex vA, vB;
        const scalar theta = i * 2.0 * M_PI / NUM_POINTS_CIRCUMF;
        const scalar c = cos(theta);
        const scalar s = sin(theta);

        vA.pos = {S_begin + pA.x(), c * pA.y(), s * pA.y()};
        vB.pos = {S_begin + pB.x(), c * pB.y(), s * pB.y()};
        vA.normal = {nA.x(), c * nA.y(), s * nA.y()};
        vB.normal = {nB.x(), c * nB.y(), s * nB.y()};
        assert(is_close(vA.normal.norm(), 1.0) && is_close(vB.normal.norm(), 1.0));

        // Determine if ip is odd or even
        bool ip_is_odd = (ip % 2) == 1;
        if (((theta >= 0 && theta < M_PI / 2) || (theta >= M_PI && theta < 3 * M_PI / 2))) {
            vA.color =
                ip_is_odd ? glm::vec3(0.0f, 0.0f, 0.0f) : glm::vec3(1.0f, 1.0f, 0.0f); // odd: black, even: yellow
        } else {
            vA.color =
                ip_is_odd ? glm::vec3(1.0f, 1.0f, 0.0f) : glm::vec3(0.0f, 0.0f, 0.0f); // odd: yellow, even: black
        }

        vB.color = vA.color;

        // SPØRRE ANDERS
        //  Highlight stabilizer region in blue if applicable
        // if (comp.type == ComponentType::STABILIZER && comp.has_stabilizer) {
        //     scalar S_stab = S_comp_begin + comp.S_stabilizer;
        //     scalar S_stab_start = S_stab - 0.5 * comp.L_stabilizer;
        //     scalar S_stab_end = S_stab + 0.5 * comp.L_stabilizer;
        // }
        //     if (vA.pos.x() >= S_stab_start && vA.pos.x() <= S_stab_end) {
        //         vA.color = glm::vec3(0.0f, 0.0f, 0.8f); // Blue for stabilizer region
        //     }
        //     if (vB.pos.x() >= S_stab_start && vB.pos.x() <= S_stab_end) {
        //         vB.color = glm::vec3(0.0f, 0.0f, 0.8f); // Blue for stabilizer region
        //     }
        // }

        mesh.push_back(vA);
        mesh.push_back(vB);
    }

    /*Add indices*/
    for (uint i = 0; i < NUM_POINTS_CIRCUMF - 1; i++) {
        index_buffer.push_back(prev + 2 * i);
        index_buffer.push_back(prev + 2 * i + 1);
        index_buffer.push_back(prev + 2 * i + 3);

        index_buffer.push_back(prev + 2 * i);
        index_buffer.push_back(prev + 2 * i + 3);
        index_buffer.push_back(prev + 2 * i + 2);
    }
    /*Close the segment*/
    index_buffer.push_back(prev + 2 * (NUM_POINTS_CIRCUMF - 1));
    index_buffer.push_back(prev + 2 * (NUM_POINTS_CIRCUMF - 1) + 1);
    index_buffer.push_back(prev + 1);

    index_buffer.push_back(prev + 2 * (NUM_POINTS_CIRCUMF - 1));
    index_buffer.push_back(prev + 1);
    index_buffer.push_back(prev);

    int _ = 1;
}

inline void get_polygon_point_and_compute_normals(uint iseg, const Polygon &polygon, Vec2 &pA, Vec2 &pB, Vec2 &nA,
                                                  Vec2 &nB) {
    const uint Nseg = polygon.size();
    assert(iseg < Nseg);

    uint im = (iseg == 0) ? Nseg - 1 : iseg - 1;
    uint i = iseg;
    uint ip = (iseg == Nseg - 1) ? 0 : i + 1;
    uint ipp = (ip == Nseg - 1) ? 0 : ip + 1;

    Vec2 p_im = polygon[im];
    Vec2 p_i = polygon[i];
    Vec2 p_ip = polygon[ip];
    Vec2 p_ipp = polygon[ipp];

    Vec2 t_im_seg = (p_i - p_im).normalized();
    Vec2 t_i_seg = (p_ip - p_i).normalized();
    Vec2 t_ip_seg = (p_ipp - p_ip).normalized();

    Vec2 n_im_seg = {t_im_seg.y(), -t_im_seg.x()};
    Vec2 n_i_seg = {t_i_seg.y(), -t_i_seg.x()};
    Vec2 n_ip_seg = {t_ip_seg.y(), -t_ip_seg.x()};

    scalar theta_i = acos(n_im_seg.dot(n_i_seg));
    scalar theta_ip = acos(n_i_seg.dot(n_ip_seg));
    assert(theta_i >= 0 && theta_i < M_PI);
    assert(theta_ip >= 0 && theta_ip < M_PI);

    if (theta_i < MAX_ANGLE_DEG_UNIQUE_VERTEX_NORMAL * M_PI / 180) {
        nA = (n_i_seg + n_im_seg).normalized();
    } else {
        nA = n_i_seg;
    }
    if (theta_ip < MAX_ANGLE_DEG_UNIQUE_VERTEX_NORMAL * M_PI / 180) {
        nB = (n_ip_seg + n_i_seg).normalized();
    } else {
        nB = n_i_seg;
    }
    pA = p_i;
    pB = p_ip;
    assert(is_close(nA.norm(), 1.0));
    assert(is_close(nB.norm(), 1.0));
}

inline void add_polygon_to_mesh(scalar S_begin, scalar S_comp_begin, const Polygon &polygon, uint ip,
                                const PipeComponent &comp, vector<PipeRenderer::MeshVertex> &mesh,
                                vector<uint> &vertices) {
    const uint Nseg = polygon.size();
    /*Check that the first and last point don't coincide*/
    assert((polygon[0] - polygon[Nseg - 1]).norm() > 100000 * SMALL_SCALAR);

    for (uint iseg = 0; iseg < polygon.size() - 1; iseg++) {
        Vec2 pA, pB, nA, nB;
        get_polygon_point_and_compute_normals(iseg, polygon, pA, pB, nA, nB);
        add_polygon_ring_to_mesh(S_begin, S_comp_begin, pA, pB, nA, nB, ip, comp, mesh, vertices);
    }
}

void PipeRenderer::generate_mesh(const Config &config, const vector<PipeComponent> &pipe_assembly,
                                 const PipeRenderComponents &pipe_render_components) {

    assert(index_buffer.size() == 0);
    assert(ptr_index.size() == 0 && ptr_vertex.size() == 0);
    assert(mesh.size() == 0);
    const uint Npoly = pipe_render_components.polygons.size();
    assert(pipe_render_components.arr_S.size() == Npoly + 1);
    assert(pipe_render_components.arr_ie.size() == Npoly);
    ptr_index.push_back(0);
    ptr_vertex.push_back(0);
    for (uint i = 0; i < Npoly; i++) {
        const Polygon &polygon = pipe_render_components.polygons[i];
        const scalar S_begin = pipe_render_components.arr_S[i];
        const scalar ip = pipe_render_components.arr_ip[i];
        const PipeComponent &comp = pipe_assembly[ip];
        const scalar S_comp_begin = comp.S_global;

        add_polygon_to_mesh(S_begin, S_comp_begin, polygon, ip, comp, mesh, index_buffer);
        ptr_index.push_back(index_buffer.size());
        ptr_vertex.push_back(mesh.size());
    }

    vertex_buffer.resize(mesh.size());

    assert(ptr_index.size() == Npoly + 1 && ptr_vertex.size() == Npoly + 1);
}

void PipeRenderer::draw(const Config &config, ConfigGraphics &config_graphics,
                        const curvlin::PipeSolver *solver_curvlin, const Pipe &pipe, const Hole &hole,
                        const vector<PipeComponent> &pipe_assembly, const vector<Vec3> &x_pipe,
                        const vector<Quaternion> &q_pipe, const array<MiscSpatialGraph, N_MG> &misc_plots,
                        const array<FluidSpatialGraph, N_FG> &fluid_plots, const byte *buf) {

    uint top_node = config.top_node;
    uint top_component = config.top_component;
    ipoly_top = pipe_assembly[top_component].i_top_poly;
    update_vertex_buffer(config, config_graphics, solver_curvlin, pipe, hole, x_pipe, q_pipe, misc_plots, fluid_plots,
                         buf);

    glBindVertexArray(VAO);

    const uint Npoly = get_Npoly();
    const uint start_index = ptr_index[ipoly_top];
    const uint num_index_active = ptr_index[Npoly] - ptr_index[ipoly_top];

    // glEnable(GL_CULL_FACE);

    if (config_graphics.wireframe_mode) {
        // shader_old.set_vec3("objectColor", glm::vec3(1.0f, 1.0f, 1.0f)); // White color
        glLineWidth(3.0f); // Increase line width for better visibility
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDrawElements(GL_TRIANGLES, num_index_active, GL_UNSIGNED_INT, (void *)(start_index * sizeof(uint)));
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glLineWidth(1.0f); // Reset line width to default
    } else {
        glDrawElements(GL_TRIANGLES, num_index_active, GL_UNSIGNED_INT, (void *)(start_index * sizeof(uint)));
    }

    // For some reason culling does not work on the hole, so disable when you finish drawing the pipe.
    // Probably since the hole is transparent so you actually need to see backfaces?
    glDisable(GL_CULL_FACE);

    glBindVertexArray(0);
}

void PipeRenderer::update_vertex_buffer(const Config &config, ConfigGraphics &config_graphics,
                                        const curvlin::PipeSolver *solver_curvlin, const Pipe &pipe, const Hole &hole,
                                        const vector<Vec3> &x_pipe, const vector<Quaternion> &q_pipe,
                                        const array<MiscSpatialGraph, N_MG> &misc_plots,
                                        const array<FluidSpatialGraph, N_FG> &fluid_plots, const byte *buf) {
    assert(vertex_buffer.size() == mesh.size());
    assert(ptr_index.size() == ptr_vertex.size() && ptr_index.size() == ptr_poly_to_ie.size() + 1 &&
           ptr_index.size() == arr_S_poly.size());
    const uint Npoly = ptr_vertex.size() - 1;
    const ArrayView<scalar> S_pipe = pipe.get_field<PipeField::S>(buf);

/*Loop over each polygon element and transform to global coordinates based on displacement*/
#pragma omp parallel for
    for (uint ipoly = ipoly_top; ipoly < Npoly; ipoly++) {
        const uint ie = ptr_poly_to_ie[ipoly];
        assert(ie + 1 < pipe.N);

        const scalar S_poly_i = arr_S_poly[ipoly];
        const scalar S_poly_ip = arr_S_poly[ipoly + 1];

        const scalar S_pipe_i = S_pipe[ie];
        const scalar S_pipe_ip = S_pipe[ie + 1];

        assert(S_pipe_ip > S_pipe_i + SMALL_SCALAR);
        assert(S_poly_i >= S_pipe_i && S_poly_i <= S_pipe_ip);

        const scalar xi_pipe_i = (S_poly_i - S_pipe_i) / (S_pipe_ip - S_pipe_i);
        const scalar xi_pipe_ip = (S_poly_ip - S_pipe_i) / (S_pipe_ip - S_pipe_i);

        assert(xi_pipe_i >= 0 && xi_pipe_i < 1);
        assert(xi_pipe_ip > 0 && xi_pipe_ip <= 1);

        /*Interpolate pipe properties at the start and end of the polygon*/
        const Vec3 x_pipe_begin =
            interp_spline_cubic_hermite(x_pipe[ie], x_pipe[ie + 1], q_pipe[ie], q_pipe[ie + 1], xi_pipe_i);
        const Vec3 x_pipe_end =
            interp_spline_cubic_hermite(x_pipe[ie], x_pipe[ie + 1], q_pipe[ie], q_pipe[ie + 1], xi_pipe_ip);

        const Quaternion q_pipe_begin = Quaternion::lerp(q_pipe[ie], q_pipe[ie + 1], xi_pipe_i);
        const Quaternion q_pipe_end = Quaternion::lerp(q_pipe[ie], q_pipe[ie + 1], xi_pipe_ip);

        const uint vert_begin = ptr_vertex[ipoly];
        const uint vert_end = ptr_vertex[ipoly + 1];

        scalar S_poly_old = std::numeric_limits<scalar>::max();

        scalar xi_poly;
        Quaternion q_xi;
        Vec3 x_centre;
        glm::vec3 contour_color;

        for (uint ivert = vert_begin; ivert < vert_end; ivert++) {
            const MeshVertex mesh_vertex = mesh[ivert];
            const Vec3 X_vert = mesh_vertex.pos;
            const scalar S_poly_new = X_vert.x();
            assert(S_poly_new - S_poly_i >= -SMALL_SCALAR && S_poly_new - S_poly_ip <= SMALL_SCALAR);

            // Only update the center calculation and contour color when we reach a new ring (new axial position)
            if (abs(S_poly_new - S_poly_old) > 0.01) {
                xi_poly = (S_poly_new - S_poly_i) / (S_poly_ip - S_poly_i);
                assert(xi_poly >= -SMALL_SCALAR && xi_poly <= 1 + SMALL_SCALAR);
                q_xi = Quaternion::lerp(q_pipe_begin, q_pipe_end, xi_poly);
                x_centre = x_pipe_begin + xi_poly * (x_pipe_end - x_pipe_begin);
                S_poly_old = S_poly_new;

                if (config_graphics.contour_variable != ContourPlotVariable::NONE) {
                    scalar xi_ie = (S_poly_new - S_pipe_i) / (S_pipe_ip - S_pipe_i);

                    if (config_graphics.contour_variable == ContourPlotVariable::VON_MISES_STRESS) {
                        contour_color = get_color_from_von_mises_stress(config, config_graphics, ie, xi_ie,
                                                                        solver_curvlin, pipe, hole, buf);
                    } else {
                        switch (config_graphics.contour_variable) {
                        case ContourPlotVariable::EFFECTIVE_TENSION: {
                            const MiscSpatialGraph &axial_tension_graph =
                                misc_plots[uint(MiscPlotSpatialVariable::EFFECTIVE_TENSION)];
                            contour_color =
                                get_color_from_misc_graph_nodewise(config_graphics, ie, xi_ie, axial_tension_graph);
                            break;
                        }
                        case ContourPlotVariable::EFFECTIVE_TORQUE: {
                            const MiscSpatialGraph &effective_torque_graph =
                                misc_plots[uint(MiscPlotSpatialVariable::EFFECTIVE_TORQUE)];
                            contour_color =
                                get_color_from_misc_graph_nodewise(config_graphics, ie, xi_ie, effective_torque_graph);
                            break;
                        }
                        case ContourPlotVariable::RADIAL_ACCELERATION: {
                            const MiscSpatialGraph &radial_acc_graph =
                                misc_plots[uint(MiscPlotSpatialVariable::RADIAL_ACCELERATION)];
                            contour_color =
                                get_color_from_misc_graph_nodewise(config_graphics, ie, xi_ie, radial_acc_graph);
                            break;
                        }
                        case ContourPlotVariable::TANGENTIAL_ACCELERATION: {
                            const MiscSpatialGraph &tang_acc_graph =
                                misc_plots[uint(MiscPlotSpatialVariable::TANGENTIAL_ACCELERATION)];
                            contour_color =
                                get_color_from_misc_graph_nodewise(config_graphics, ie, xi_ie, tang_acc_graph);
                            break;
                        }
                        case ContourPlotVariable::AXIAL_STRAIN: {
                            const MiscSpatialGraph &axial_strain_graph =
                                misc_plots[uint(MiscPlotSpatialVariable::AXIAL_STRAIN)];
                            contour_color =
                                get_color_from_misc_graph_nodewise(config_graphics, ie, xi_ie, axial_strain_graph);
                            break;
                        }
                        case ContourPlotVariable::LINEAR_CONTACT_FORCE: {
                            const MiscSpatialGraph &linear_contact_force_graph =
                                misc_plots[uint(MiscPlotSpatialVariable::LINEAR_CONTACT_FORCE)];
                            contour_color = get_color_from_misc_graph_nodewise(config_graphics, ie, xi_ie,
                                                                               linear_contact_force_graph);
                            break;
                        }
                        case ContourPlotVariable::TANGENTIAL_CONTACT_FORCE: {
                            const MiscSpatialGraph &tangential_contact_force_graph =
                                misc_plots[uint(MiscPlotSpatialVariable::TANGENTIAL_CONTACT_FORCE)];
                            contour_color = get_color_from_misc_graph_nodewise(config_graphics, ie, xi_ie,
                                                                               tangential_contact_force_graph);
                            break;
                        }
                        case ContourPlotVariable::STANDOFF_RATIO: {
                            const MiscSpatialGraph &so_graph =
                                misc_plots[uint(MiscPlotSpatialVariable::STANDOFF_RATIO)];
                            contour_color = get_color_from_misc_graph_nodewise(config_graphics, ie, xi_ie, so_graph);
                            break;
                        }
                        case ContourPlotVariable::PRESSURE_INNER: {
                            const FluidSpatialGraph &pressure_inner_graph =
                                fluid_plots[uint(FluidPlotSpatialVariable::PRESSURE_INNER)];
                            contour_color =
                                get_color_from_fluid_graph_nodewise(config_graphics, ie, xi_ie, pressure_inner_graph);
                            break;
                        }
                        case ContourPlotVariable::PRESSURE_OUTER: {
                            const FluidSpatialGraph &pressure_outer_graph =
                                fluid_plots[uint(FluidPlotSpatialVariable::PRESSURE_OUTER)];
                            contour_color =
                                get_color_from_fluid_graph_nodewise(config_graphics, ie, xi_ie, pressure_outer_graph);
                            break;
                        }
                        case ContourPlotVariable::VELOCITY_INNER: {
                            const FluidSpatialGraph &velocity_inner_graph =
                                fluid_plots[uint(FluidPlotSpatialVariable::VELOCITY_INNER)];
                            contour_color =
                                get_color_from_fluid_graph_nodewise(config_graphics, ie, xi_ie, velocity_inner_graph);
                            break;
                        }
                        case ContourPlotVariable::VELOCITY_OUTER: {
                            const FluidSpatialGraph &velocity_outer_graph =
                                fluid_plots[uint(FluidPlotSpatialVariable::VELOCITY_OUTER)];
                            contour_color =
                                get_color_from_fluid_graph_nodewise(config_graphics, ie, xi_ie, velocity_outer_graph);
                            break;
                        }
                        default: {
                            printf("Contour plot variable not implemented yet.");
                            config_graphics.contour_variable == ContourPlotVariable::NONE;
                        }
                        }
                    }
                }
            }

            const Vec3 X_vert_radial = config_graphics.radial_scale * Vec3{0, X_vert.y(), X_vert.z()};
            const Vec3 x_vert = x_centre + q_xi.rotate_vector(X_vert_radial);
            const Vec3 normal_current = q_xi.rotate_vector(mesh_vertex.normal);

            vertex_buffer[ivert].pos = eigen_vec3_to_glm_vec3(x_vert);
            vertex_buffer[ivert].normal = eigen_vec3_to_glm_vec3(normal_current);

            if (config_graphics.contour_variable == ContourPlotVariable::NONE) {
                vertex_buffer[ivert].color = mesh_vertex.color;
            } else {
                vertex_buffer[ivert].color = contour_color;
            }
        }
    }

    const uint ptr_top = ptr_vertex[ipoly_top];
    assert(vertex_buffer.size() == ptr_vertex[Npoly]);

    const size_t num_vertices_to_update = vertex_buffer.size() - ptr_top;
    if (num_vertices_to_update > 0) {
        /*Update the vertex buffer from all visible elements (from top node to Ne)*/
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, ptr_top * sizeof(Vertex), num_vertices_to_update * sizeof(Vertex),
                        &vertex_buffer[ptr_top]);
    }
    // glBufferSubData(GL_ARRAY_BUFFER, 0, num_vert_active * sizeof(Vertex), &vertex_buffer[ptr_top]);
}

void PipeRenderer::Vertex::set_vertex_attrib_pointer(uint VBO) {
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, pos));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, normal));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, color));
    glEnableVertexAttribArray(2);
}

void PipeRenderer::draw_triads(uint top_node, uint N, const vector<Vec3> &x_pipe, const vector<Quaternion> &q_pipe,
                               Shader &shader_triads, const glm::mat4 &view_matrix, const glm::mat4 &proj_matrix,
                               float L_char_triad_world) {
    shader_triads.use(); // Add this to ensure shader is bound

    shader_triads.set_mat4("view", view_matrix);
    shader_triads.set_mat4("projection", proj_matrix);

    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);

    constexpr float L_char_triad_model = 1.2;
    assert(L_char_triad_model > 1000 * SMALL_FLOAT);
    const float scale_factor_triads = L_char_triad_world / L_char_triad_model;

    for (uint i = top_node; i < N; i++) {

        glm::vec3 triad_position = eigen_vec3_to_glm_vec3(x_pipe[i]);
        const Mat3 &R = q_pipe[i].to_matrix();

        // Convert Eigen matrix to GLM matrix
        glm::mat4 triad_orientation = glm::mat4(1.0f);
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                triad_orientation[col][row] = R(row, col);
            }
        }

        // Build transformation matrix: Scale -> Rotate -> Translate
        glm::mat4 model_matrix_triads = glm::mat4(1.0f);
        model_matrix_triads = glm::translate(model_matrix_triads, triad_position);
        model_matrix_triads = model_matrix_triads * triad_orientation;
        model_matrix_triads = glm::scale(model_matrix_triads, glm::vec3(scale_factor_triads));

        shader_triads.set_mat4("model", model_matrix_triads);
        pipe_triads[i].draw(shader_triads);
    }
}

PipeRenderer::~PipeRenderer() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
}
