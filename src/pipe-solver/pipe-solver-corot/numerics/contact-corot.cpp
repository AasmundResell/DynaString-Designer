#include "contact-corot.hpp"

namespace corot {

inline scalar dS_e(uint ie, const vector<Vec3> &X) {
    return (X[ie + 1] - X[ie]).norm();
}
/*Distance between centroids of two elements*/
inline scalar dS_element_avg(uint i, const vector<Vec3> &X, const uint N) {
    assert(i < N);
    const uint Ne = N - 1;
    if (i == 0)
        return dS_e(0, X) / 2;
    else if (i == N - 1)
        return dS_e(Ne - 1, X) / 2;
    else
        return (dS_e(i - 1, X) + dS_e(i, X)) / 2;
}

inline int node_within_hole_segment(const uint ie_h, const uint N_hole, const ArrayView<Vec3> x_hole,
                                    const scalar r_e_hole, const Vec3 &x) {
    const uint Ne_hole = N_hole - 1;
    assert(ie_h < Ne_hole);

    const Vec3 &x_hole_A = x_hole[ie_h];
    const Vec3 &x_hole_B = x_hole[ie_h + 1];

    const Vec3 t = (x_hole_B - x_hole_A).normalized();
    const scalar l_hole_seg = (x_hole_B - x_hole_A).norm();
    const scalar dist = (x - x_hole_A).dot(t);

    const bool is_between = dist >= -SMALL_SCALAR && dist <= l_hole_seg + SMALL_SCALAR;

#define MAX_ALLOWABLE_HOLE_PAIR_ANGLE 10 * M_PI / 180
    if (is_between) {
        return 0;
    } else if (dist < 0) {
        if (ie_h == 0) {
            return -1;
        } else {
            assert(ie_h > 0);
            const Vec3 &x_hole_prev = x_hole[ie_h - 1];
            const Vec3 t_prev = (x_hole_A - x_hole_prev).normalized();
            const scalar product = min(1.0, t.dot(t_prev));
            const scalar theta = acos(product) + M_PI;
            assert(theta >= 0 && theta < MAX_ALLOWABLE_HOLE_PAIR_ANGLE + M_PI);
            const scalar extra_search_dist = r_e_hole * tan(theta);
            assert(extra_search_dist > -SMALL_SCALAR);
            return (extra_search_dist + SMALL_SCALAR > -dist) ? 0 : -1;
        }
    } else {
        assert(dist > 0);
        if (ie_h == Ne_hole - 1) {
            return 1;
        } else {
            const Vec3 &x_hole_next = x_hole[ie_h + 2];
            const Vec3 t_next = (x_hole_next - x_hole_B).normalized();
            const scalar product = min(1.0, t.dot(t_next));
            const scalar theta = acos(product);
            assert(theta >= 0 && theta < MAX_ALLOWABLE_HOLE_PAIR_ANGLE);
            const scalar extra_search_dist = r_e_hole * tan(theta);
            assert(extra_search_dist > -SMALL_SCALAR);
            return (extra_search_dist + SMALL_SCALAR > dist) ? 0 : 1;
        }
    }
#undef MAX_ALLOWABLE_HOLE_PAIR_ANGLE
}

void calc_hole_contact_forces_nodewise(const ConfigStatic &config_s, const uint top_node, const Pipe &pipe,
                                       const Hole &hole, BeamData &beam, byte *buf) {

    assert(config_s.contact_enabled);

    const uint N = pipe.N;

    const ArrayView<Vec3> x_hole = hole.get_field<HoleField::x>(buf);
    const ArrayView<scalar> s_hole = hole.get_field<HoleField::s>(buf);
    const ArrayView<scalar> r_hole = hole.get_field<HoleField::a>(buf);
    const ArrayView<scalar> ro = pipe.get_field<PipeField::ro>(buf);

    const vector<Vec3> X = beam.X;
    const vector<Vec3> &d_trans = beam.d_trans;
    const vector<Quaternion> &d_rot = beam.d_rot;
    const vector<Vec3> &v_trans = beam.v_trans;
    const vector<Vec3> &v_rot = beam.v_rot;
    vector<Vec3> &f_ext_trans = beam.f_ext_trans;
    vector<Vec3> &f_ext_rot = beam.f_ext_rot;
    vector<uint> &i_pipe_to_ie_hole = beam.i_pipe_to_ie_hole;
    update_hole_contact_indices<false>(N, hole, beam.i_pipe_to_ie_hole, X, d_trans, buf);

    const scalar mu_s = config_s.mu_static;
    const scalar mu_k = config_s.mu_kinetic;
    const scalar v_cs = config_s.critical_stribeck_velocity;
    const scalar K_c = config_s.K_normal;
    const scalar C_c = config_s.C_normal;
    scalar r_pipe;

#pragma omp parallel for
    for (uint i = top_node; i < N; i++) {
        const uint ie_h = i_pipe_to_ie_hole[i];
        assert(ie_h < hole.N_hole - 1);

        const Vec3 x = X[i] + d_trans[i];
        assert(node_within_hole_segment(ie_h, hole.N_hole, x_hole, (r_hole[ie_h] + r_hole[ie_h + 1]) / 2, x) == 0);
        const Vec3 &x_hole_A = x_hole[ie_h];
        const Vec3 &x_hole_B = x_hole[ie_h + 1];
        const Vec3 t = (x_hole_B - x_hole_A).normalized();
        const Vec3 x_center = x_hole_A + (x - x_hole_A).dot(t) * t;
        const Vec3 x_offset = x - x_center;
        const scalar d_center = (x_offset).norm();
        assert(is_orthogonal(t, x_offset));

        const scalar dS = dS_element_avg(i, X, N); // Distance between centroids of two elements

        scalar s_i = s_hole[ie_h] + (x_center - x_hole_A).norm();
        const Vec3 n = x_offset.normalized();
        const Vec3 omega = d_rot[i].rotate_vector(v_rot[i]); // angular velocity in global frame

        scalar r_h = hole.lerp_hole_property_from_pipe_node_position<scalar>(r_hole, s_i, ie_h, buf);

        scalar d_wall = r_h - d_center; // distance to wall from pipe center (positive if inside hole)

        if (i == 0) {
            r_pipe = ro[0];
        } else if (i == N - 1) {
            r_pipe = ro[N - 2]; // Last element radius
        } else {
            r_pipe = max(ro[i - 1], ro[i]);
        }

        Vec3 q_f;
        Vec3 q_m;

        calc_contact_force_3D(q_f, q_m, mu_s, mu_k, v_cs, K_c, C_c, r_pipe, d_wall, n, v_trans[i], omega);

        const Vec3 &fc = q_f * dS;
        const Vec3 &mc = q_m * dS;
        assert(fc.allFinite() && mc.allFinite()); // This cannot be in device code

        f_ext_trans[i] += fc;
        f_ext_rot[i] += mc;
    }
}

template <bool initial_search>
inline void update_hole_contact_indices(const uint N, const Hole &hole, vector<uint> &i_pipe_to_ie_hole,
                                        const vector<Vec3> &X, const vector<Vec3> &d_trans, byte *buf) {
    const uint N_hole = hole.N_hole + 1;
    const uint Ne_hole = N_hole - 1;
    const ArrayView<Vec3> x_hole = hole.get_field<HoleField::x>(buf);
    const ArrayView<scalar> r_hole = hole.get_field<HoleField::a>(buf);

    if constexpr (initial_search) { /*If its the initial search (all indices are zero from before),
                                                     the search is more involved*/
        for (uint i = 0; i < N; i++) {
            uint hi = i_pipe_to_ie_hole[i];

            const scalar r_e_hole = (r_hole[hi] + r_hole[hi + 1]) / 2; // Average for now

            int search_dir = node_within_hole_segment(hi, N_hole, x_hole, r_e_hole, X[i] + d_trans[i]);

            if (search_dir == 0) {
                continue;
            } else if (search_dir == -1) {
                /*Search backwards*/
                while (search_dir != 0) {
                    assert(search_dir == -1);
                    hi--;
                    assert(hi >= 0);
                    if (hi < 0) {
                        throw runtime_error("Hole index (" + to_string(hi) + ") < num hole elements (" +
                                            to_string(Ne_hole) + ") found for node i=" + to_string(i));
                    }
                    search_dir = node_within_hole_segment(hi, N_hole, x_hole, r_e_hole, X[i] + d_trans[i]);
                }
                i_pipe_to_ie_hole[i] = hi;
            } else {
                /*Search forwards*/
                while (search_dir != 0) {
                    assert(search_dir == 1);
                    hi++;
                    assert(hi < Ne_hole);
                    if (hi >= Ne_hole) {
                        throw runtime_error("Hole index (" + to_string(hi) + ") >= num hole elements (" +
                                            to_string(Ne_hole) + ") found for node i=" + to_string(i));
                    }
                    search_dir = node_within_hole_segment(hi, N_hole, x_hole, r_e_hole, X[i] + d_trans[i]);
                }
                i_pipe_to_ie_hole[i] = min(hi, Ne_hole - 1);
            }
        }
    } else { /*In later searches its assumed that the hole index is updated on the first try,
           since the node will cross no more than one hole element boundaries per timestep*/
#pragma omp parallel for
        for (uint i = 0; i < N; i++) {
            uint hi = i_pipe_to_ie_hole[i];
            assert(hi >= 0 && hi < Ne_hole);
            const scalar r_e_hole = (r_hole[hi] + r_hole[hi + 1]) / 2; // Average for now
            const int search_dir = node_within_hole_segment(hi, N_hole, x_hole, r_e_hole, X[i] + d_trans[i]);
            hi += search_dir;
            assert(hi >= 0 && hi < Ne_hole);
            hi = max((uint)0, min(hi, Ne_hole - 1)); /*Should ideally not happen, but this is done to increase
                                                       robustness and avoid segfaults etc. */
            i_pipe_to_ie_hole[i] = hi;
        }
    }
}

void initialize_hole_contact(const ConfigStatic &conf_stat, const Pipe &pipe, const Hole &hole, BeamData &beam,
                             byte *buf) {
    if (!conf_stat.contact_enabled)
        return;

    constexpr bool initial_search = true;
    try {
        printf("Starting initial hole contact detection. N hole nodes = %i, N string nodes = %i\n", hole.N_hole,
               pipe.N);

        update_hole_contact_indices<initial_search>(pipe.N, hole, beam.i_pipe_to_ie_hole, beam.X, beam.d_trans, buf);
    } catch (const exception &e) {
        throw runtime_error("Hole contact initialization failed\n" + string(e.what()));
    }
}

} // namespace corot
