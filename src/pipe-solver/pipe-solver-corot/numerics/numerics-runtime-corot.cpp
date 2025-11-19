#include "numerics-runtime-corot.hpp"
#include "contact-corot.hpp"

namespace corot {

template <uint i_first>
static void calc_inner_forces(const ConfigStatic &conf_stat, const Pipe &pipe, BeamData &beam, const byte *buf);

static void zero_internal(const uint N, vector<Vec3> &f_int_trans, vector<Vec3> &f_int_rot, vector<Vec3> &f_ext_trans,
                          vector<Vec3> &f_ext_rot);

static void add_mass_proportional_rayleigh_damping(uint N, scalar alpha, const vector<scalar> &M,
                                                   const vector<Vec3> &v_trans, vector<Vec3> &f_int_trans,
                                                   const vector<Vec3> &J_u, const vector<Quaternion> &d_rot,
                                                   const vector<Vec3> &v_rot, vector<Vec3> &f_int_rot);

void calc_forces(const ConfigStatic &conf_stat, const uint top_node, const Pipe &pipe, const Hole &hole, BeamData &beam,
                 ArenaBump &arena_h) {
    const uint N = pipe.N;

    zero_internal(N, beam.f_int_trans, beam.f_int_rot, beam.f_ext_trans, beam.f_ext_rot);

    // add_user_defined_external_forces(config, pipe, hole, beam.d_trans, beam.d_rot, beam.f_ext_trans, beam.f_ext_rot);

    calc_inner_forces<0>(conf_stat, pipe, beam, arena_h.buf);
    calc_inner_forces<1>(conf_stat, pipe, beam, arena_h.buf);

    if (conf_stat.contact_enabled) {
        calc_hole_contact_forces_nodewise(conf_stat, top_node, pipe, hole, beam, arena_h.buf);
    }

    if (conf_stat.rayleigh_damping_enabled) {
        add_mass_proportional_rayleigh_damping(N, conf_stat.alpha, beam.M, beam.v_trans, beam.f_int_trans, beam.J_u,
                                               beam.d_rot, beam.v_rot, beam.f_int_rot);
    }
}
/*--------------------------------------------------------------------
Explicit solution by the Simo-Wong algorithm for rotations and
non-staggered central differences for translations
--------------------------------------------------------------------*/

// /*Bending. Euler Bernoulli is used, symmetrical cross section
// The original matrix reads
// EI/L³ *[[ 12  6L  -12  6L ]
//         [ 6L  4L² -6L  2L²]
//         [-12 -6L   12 -6L ]
//         [ 6L  2L² -6L  4L²]]
// multiplied by [w1 theta1, w2 theta2],
// however, it can be simplified, since w1=w2=0,
// rows 1 and 3 can be skipped resulting in
// EI/L²**[[ 6   6 ]*[[theta1 ]
//         [ 4L  2L]  [theta2 ]]
//         [-6  -6 ]
//         [ 2L  4L]]
// */

// // const scalar Fy1 = E * I / (l0 * l0) * (6 * theta_2A + 6 * theta_z2);
// // const scalar Mz1 = E * I / (l0 * l0) * (4 * l0 * theta_z1 + 2 * l0 * theta_z2);
// // const scalar Fy2 = -Fy1;
// // const scalar Mz2 = E * I / (l0 * l0) * (2 * l0 * theta_z1 + 4 * l0 * theta_z2);

// const scalar F2 = E * I / (l0 * l0) * (6 * theta_2l + 6 * theta_5l);
// const scalar M2 = E * I / (l0 * l0) * (4 * l0 * theta_2l + 2 * l0 * theta_5l);
// const scalar F5 = -F2;
// const scalar M5 = E * I / (l0 * l0) * (2 * l0 * theta_2l + 4 * l0 * theta_5l);

// R_int_l[1] = F2;
// R_int_l[4] = M2;
// R_int_l[7] = F5;
// R_int_l[11] = M5;

// // const scalar Fz1 = E * I / (l0 * l0) * (6 * theta_3A + 6 * theta_3B);
// // const scalar My1 = E * I / (l0 * l0) * (4 * l0 * theta_3A + 2 * l0 * theta_3B);
// // const scalar Fz2 = -Fz1;
// // const scalar My2 = E * I / (l0 * l0) * (2 * l0 * theta_3A + 4 * l0 * theta_3B);

// const scalar F3 = E * I / (l0 * l0) * (6 * theta_3l + 6 * theta_6l);
// const scalar M3 = E * I / (l0 * l0) * (4 * l0 * theta_3l + 2 * l0 * theta_6l);
// const scalar F6 = -F3;
// const scalar M6 = E * I / (l0 * l0) * (2 * l0 * theta_3l + 4 * l0 * theta_6l);

// R_int_l[2] = F3;
// R_int_l[5] = M3;
// R_int_l[8] = F6;
// R_int_l[10] = M6;

// return R_int_l;
//}
//
static void zero_internal(const uint N, vector<Vec3> &f_int_trans, vector<Vec3> &f_int_rot, vector<Vec3> &f_ext_trans,
                          vector<Vec3> &f_ext_rot) {
#pragma omp parallel for
    for (uint i = 0; i < N; i++) {
        f_int_trans[i] = {0, 0, 0};
    }
#pragma omp parallel for
    for (uint i = 0; i < N; i++) {
        f_int_rot[i] = {0, 0, 0};
    }
#pragma omp parallel for
    for (uint i = 0; i < N; i++) {
        f_ext_trans[i] = {0, 0, 0};
    }
#pragma omp parallel for
    for (uint i = 0; i < N; i++) {
        f_ext_rot[i] = {0, 0, 0};
    }
}

template <uint i_first>
static void calc_inner_forces(const ConfigStatic &conf_stat, const Pipe &pipe, BeamData &beam, const byte *buf) {
    assert(i_first == 0 || i_first == 1);
    const uint N = pipe.N;
    const uint Ne = N - 1;
    const vector<Vec3> &X = beam.X;
    const scalar E = conf_stat.E;
    const scalar G = conf_stat.get_G();
    const scalar beta = conf_stat.beta;
    const CorotationalFormulation corotational_formulation = conf_stat.corotational_formulation;

#pragma omp parallel for
    for (uint ie = i_first; ie < Ne; ie += 2) {
        Vec12 R_int_e;

        scalar A = pipe.Ap_e(ie, buf);
        scalar I = pipe.Ip_e(ie, buf);
        scalar J = 2 * I;

        switch (corotational_formulation) {
        default: {
            assert(corotational_formulation == CorotationalFormulation::CRISFIELD);
            R_int_e = CrisfieldBeam::calc_element_inner_forces(ie, X, beam.d_trans, beam.d_rot, E, G, I, I, A, J, beta,
                                                               beam.v_trans, beam.v_rot);
            break;
        }
        case CorotationalFormulation::BATTINI: {
            R_int_e = BattiniBeam::global_internal_forces(ie, X, beam.d_trans, beam.d_rot, A, I, I, J, E, G);
            break;
        }
        }
        beam.f_int_trans[ie] += R_int_e.segment(0, 3);
        beam.f_int_rot[ie] += R_int_e.segment(3, 3);
        beam.f_int_trans[ie + 1] += R_int_e.segment(6, 3);
        beam.f_int_rot[ie + 1] += R_int_e.segment(9, 3);
    }

    /*------------------------------------------  # - R: [0,0,0, 0, 600000, 0]
    #   rel_loc: 1--------------------------
    To avoid race conditions the following pattern is used to compute and
    assemble the internal forces:

    Example shows a problem of 2 threads and 12 elements.

    first loop:
     e0,  e1,  e2,  e3,  e4,  e5,  e6,  e7,  e8,  e9, e10, e11
    [r0,  r0,  r0,  --,  --,  --,  r1,  r1,  r1,  --,  --, -- ]

    second loop
     e0,  e1,  e2,  e3,  e4,  e5,  e6,  e7,  e8,  e9, e10, e11
    [--,  --,  --,  r0,  r0,  r0,  --,  --,  --,  r1,  r1,  r1 ]

    ensuring that the memory region worked on by each thread is compact
    and, but that no two threads can update the same nodal force at the
    same time.
    --------------------------------------------------------------------*/
    // #pragma omp parallel
    //     {
    //         uint num_threads = omp_get_num_threads();
    //         uint bin_size = ceil((scalar)Ne / (2 * num_threads));
    //         // cout << "bin sz " << bin_size << endl;
    //         assert(2 * bin_size * num_threads >= Ne);
    //         uint r = omp_get_thread_num();
    //         uint a = 2 * r * bin_size;
    //         assert(a < Ne);
    //         uint b = min(a + bin_size, Ne);
    //         // printf("first a %i,b %i\n", a, b);

    //         for (uint ie = a; ie < b; ie++)
    //         {
    //             calc_element_inner_forces(ie, X.data(), beam.d_trans.data(), beam.d_rot.data(),
    //                                       beam.f_int_trans.data(), beam.f_int_rot.data(),
    //                                       pipe.ri_e(ie), pipe.ro_e(ie), E, G);
    //         }
    //     }
    // #pragma omp parallel
    //     {
    //         uint num_threads = omp_get_num_threads();
    //         uint bin_size = ceil((scalar)Ne / (2 * num_threads));
    //         assert(2 * bin_size * num_threads >= Ne);
    //         uint r = omp_get_thread_num();
    //         uint a = (2 * r + 1) * bin_size;
    //         assert(a < Ne);
    //         uint b = min(a + bin_size, Ne);
    //         // printf("second a %i,b %i\n", a, b);

    //         for (uint ie = a; ie < b; ie++)
    //         {
    //             calc_element_inner_forces(ie, X.data(), beam.d_trans.data(), beam.d_rot.data(),
    //                                       beam.f_int_trans.data(), beam.f_int_rot.data(),
    //                                       pipe.ri_e(ie), pipe.ro_e(ie), E, G);
    //         }
    //     }
}

static void velocity_update_partial_OLD(scalar dt, uint N, const scalar *__restrict__ M, const Vec3 *__restrict__ J_u,
                                        const Vec3 *__restrict__ f_int_trans, const Vec3 *__restrict__ f_int_rot,
                                        const Vec3 *__restrict__ f_ext_trans, const Vec3 *__restrict__ f_ext_rot,
                                        Vec3 *__restrict__ v_trans, Vec3 *__restrict__ v_rot) {
#pragma omp parallel
    {
#pragma omp for nowait
        for (uint i = 0; i < N; i++) {
            v_trans[i] += 0.5 * dt * (f_ext_trans[i] - f_int_trans[i]) / M[i];
        }
#pragma omp for
        for (uint i = 0; i < N; i++) {
            /*omega_u_dot = J_u^-1 * (R_rot_u - S(omega_u)*J_u*omega_u)*/
            const Vec3 R_rot_u = f_ext_rot[i] - f_int_rot[i];

            Vec3 &omega_u = v_rot[i];

            Mat3 JJu = J_u[i].asDiagonal();
            Vec3 Ju = J_u[i];
            Vec3 rot_term_orig = omega_u.cross(Vec3{J_u[i].array() * omega_u.array()});

            Vec3 rot_term_new = skew(omega_u) * JJu * omega_u;

            cout << "rot_term_orig " << rot_term_orig << endl;
            cout << "rot_term_new " << rot_term_new << endl;

            Vec3 omega_u_dot_new;
            omega_u_dot_new.x() = (R_rot_u.x() - (Ju.z() - Ju.y()) * omega_u.y() * omega_u.z()) / Ju.x();
            omega_u_dot_new.y() = (R_rot_u.y() - (Ju.x() - Ju.z()) * omega_u.x() * omega_u.z()) / Ju.y();
            omega_u_dot_new.z() = (R_rot_u.z() - (Ju.y() - Ju.x()) * omega_u.x() * omega_u.y()) / Ju.z();

            const Vec3 omega_u_dot = (R_rot_u - rot_term_orig).array() / J_u[i].array();

            cout << "omega_u_dot\n" << omega_u_dot << endl;
            cout << "omega_u_dot_new\n" << omega_u_dot_new << endl;

            omega_u += 0.5 * dt * omega_u_dot;
        }
    }
}

static void displacement_update(scalar dt, uint N, vector<Vec3> &v_trans, vector<Vec3> &v_rot, vector<Vec3> &d_trans,
                                vector<Quaternion> &d_rot) {
#pragma omp parallel for

    for (uint i = 0; i < N; i++) {
        d_trans[i] += dt * v_trans[i];
    }
#pragma omp parallel for
    for (uint i = 0; i < N; i++) {
        // Mat3 U = d_rot[i].to_matrix();
        // U = U * (Mat3::Identity() - 0.5 * dt * skew(v_rot[i])).inverse() *
        //     (Mat3::Identity() + 0.5 * dt * skew(v_rot[i]));
        // assert(U.allFinite());
        // assert(is_orthonormal(U));
        // d_rot[i].from_matrix(U);
        // assert(is_close(d_rot[i].norm(), 1.0));

        Quaternion &q = d_rot[i];
        const Vec3 &omega_u = v_rot[i];

        const Vec3 omega = q.rotate_vector(
            omega_u); // perhaps possible to simplify this and not having to first convert omega to the global frame

        // const Vec3 omega = q.rotate_vector_reversed(omega_u); // perhaps possible to simplify this and not having
        // to first convert omega to the global frame
        if (i == 1) {
            cout << "i " << i << endl;
            cout << "omega " << omega << endl;
            cout << "omega mag " << omega.norm() << endl;
        }
        // assert(is_close(omega_u.y(), 0, 1e-4) && is_close(omega_u.z(), 0, 1e-4));
        assert(is_close(q.norm(), 1.0));
    }
}

static void calc_delta_d(scalar dt, uint N, vector<Vec3> &delta_d_trans, vector<Vec3> &delta_d_rot,
                         const vector<Vec3> &v_trans, const vector<Vec3> &v_rot) {
#pragma omp parallel
    {
#pragma omp for nowait
        for (uint i = 0; i < N; i++) {
            delta_d_trans[i] = dt * v_trans[i];
        }
#pragma omp for
        for (uint i = 0; i < N; i++) {
            delta_d_rot[i] = dt * v_rot[i];
        }
    }
}

void work_update_partial(uint N, const uint top_node, const vector<Vec3> &delta_d_trans,
                         const vector<Vec3> &delta_d_rot, const vector<Vec3> &f_int_trans,
                         const vector<Vec3> &f_int_rot, const vector<Vec3> &f_ext_trans, const vector<Vec3> &f_ext_rot,
                         const vector<Vec3> &f_stat_trans, const vector<Vec3> &f_stat_rot, scalar &W_ext,
                         scalar &W_int) {
#pragma omp parallel for reduction(+ : W_int) reduction(+ : W_ext)
    for (uint i = top_node; i < N; i++) {
        /*The rotational dofs should have been rotated to the body frame allready*/
        W_int += 0.5 * (delta_d_trans[i].dot(f_int_trans[i]) + delta_d_rot[i].dot(f_int_rot[i]));
        W_ext += 0.5 * (delta_d_trans[i].dot(f_ext_trans[i]) + delta_d_rot[i].dot(f_ext_rot[i]) +
                        delta_d_trans[i].dot(f_stat_trans[i]) + delta_d_rot[i].dot(f_stat_rot[i]));
    }
}

void kinetic_energy_update(uint N, const vector<scalar> &M, const vector<Vec3> &J_u, const vector<Vec3> &v_trans,
                           const vector<Vec3> &v_rot, scalar &KE) {
    KE = 0;
#pragma omp parallel for reduction(+ : KE)
    for (uint i = 0; i < N; i++) {
        const Vec3 &vt = v_trans[i];
        const Vec3 &omega_u = v_rot[i];
        KE += 0.5 * M[i] * vt.squaredNorm() + 0.5 * omega_u.dot(Vec3{J_u[i].array() * omega_u.array()});
    }
}

static void rotate_moment_to_body_frame(uint N, const vector<Quaternion> &d_rot, vector<Vec3> &f_int_rot,
                                        vector<Vec3> &f_ext_rot) {
#pragma omp parallel for
    for (uint i = 0; i < N; i++) {
        if (i == 1) {
            cout << "f_ext_rot before\n" << f_ext_rot[i] << endl;
        }
        //     Mat3 U = d_rot[i].to_matrix();
        //     R_int[i].rot = U.transpose() * R_int[i].rot;
        //     R_ext[i].rot = U.transpose() * R_ext[i].rot;
        const Quaternion &q = d_rot[i];

        f_int_rot[i] = q.rotate_vector_reversed(f_int_rot[i]);
        f_ext_rot[i] = q.rotate_vector_reversed(f_ext_rot[i]);

        if (i == 1) {
            cout << "f_ext_rot after\n" << f_ext_rot[i] << endl;
        }
    }
}

static void add_mass_proportional_rayleigh_damping(uint N, scalar alpha, const vector<scalar> &M,
                                                   const vector<Vec3> &v_trans, vector<Vec3> &f_int_trans,
                                                   const vector<Vec3> &J_u, const vector<Quaternion> &d_rot,
                                                   const vector<Vec3> &v_rot, vector<Vec3> &f_int_rot) {
#pragma omp parallel for
    for (uint i = 0; i < N; i++) {
        f_int_trans[i] += alpha * M[i] * v_trans[i];
    }
// Test with rotational dofs also?
#pragma omp parallel for
    for (uint i = 0; i < N; i++) {
        Vec3 R_damp_rot = alpha * J_u[i].array() * v_rot[i].array();
        R_damp_rot = d_rot[i].rotate_vector(R_damp_rot);
        f_int_rot[i] += R_damp_rot;
    }
}

} // namespace corot