#include "pipe-solver-corot.hpp"
#include "numerics/contact-corot.hpp"
#include "numerics/numerics-runtime-corot.hpp"
#include "numerics/prepare-corot.hpp"
#include <iomanip>

namespace corot {

PipeSolver::PipeSolver(Config &config, ConfigDynamic &conf_dyn, const Pipe &pipe,
                       const vector<PipeComponent> &pipe_assembly, const Hole &hole, BCsData &bcs, ArenaBump &arena_h)
    : beam{config, pipe, arena_h.buf} {

    calc_dt_and_inertia_scaling(config, pipe, arena_h.buf);

    create_mass_vectors(beam.M, beam.J_u, config, pipe, arena_h.buf);

    set_pipe_feed_length_initial(config, conf_dyn, bcs, pipe, pipe_assembly, arena_h.buf);

    hole.initalize_pipe_nodes_to_hole_segments(pipe.N, config, beam.i_pipe_to_ie_hole.data(),
                                               pipe.get_field<PipeField::S>(arena_h.buf), arena_h.buf);

    print_beam(beam.i_pipe_to_ie_hole);

    set_initial_configuration(config, hole, pipe, beam.X, beam.d_trans, beam.d_rot, beam.i_pipe_to_ie_hole, bcs,
                              arena_h.buf);

    calc_static_loads(config.top_node, config.conf_stat, bcs, pipe, beam.f_stat_trans, beam.f_stat_rot, arena_h);

    calc_initial_accelerations(config.conf_stat, config.top_node, pipe, hole, beam, arena_h);

    initialize_hole_contact(config.conf_stat, pipe, hole, beam, arena_h.buf);
}

void PipeSolver::step(Config &config, ConfigDynamic &conf_dyn, const Pipe &pipe,
                      const vector<PipeComponent> &pipe_assembly, const Hole &hole, BCsData &bcs, ArenaBump &arena_h) {
    if (config.save_csv && config.n % config.n_write == 0) {
        save_csv(config, conf_dyn, pipe, hole, bcs, arena_h);
        printf("\n---------------------------------\n"
               "csv saved for: n = %lu, t = %.5f"
               "\n---------------------------------\n",
               config.n, config.t);
    }
    if (config.n % 1000 == 0) {
        check_energy_balance(config, conf_dyn);
    }
    config.use_gpu ? step_gpu(config, pipe, pipe_assembly, hole, bcs, arena_h)
                   : step_cpu(config, conf_dyn, pipe, pipe_assembly, hole, bcs, arena_h);
}

void PipeSolver::step_gpu(Config &config, const Pipe &pipe, const vector<PipeComponent> &pipe_assembly,
                          const Hole &hole, BCsData &bcs, ArenaBump &arena_d) {
    printf("GPU not implemented for corotational solver");
    assert(false);
}

void PipeSolver::step_cpu(Config &config, ConfigDynamic &conf_dyn, const Pipe &pipe,
                          const vector<PipeComponent> &pipe_assembly, const Hole &hole, BCsData &bcs,
                          ArenaBump &arena_h) {

    const scalar dt = config.conf_stat.dt;
    const uint N = pipe.N;
    ConfigStatic conf_stat = config.conf_stat;
    const bool check_energy_balance = conf_stat.check_energy_balance;

    vector<Vec3> &d_trans = beam.d_trans;
    vector<Quaternion> &d_rot = beam.d_rot;
    vector<Vec3> &v_trans = beam.v_trans;
    vector<Vec3> &v_rot = beam.v_rot;
    vector<Vec3> &a_trans = beam.a_trans;
    vector<Vec3> &a_rot = beam.a_rot;
    vector<Vec3> &L_rot = beam.L_rot;
    vector<Vec3> &m_rot = beam.m_rot;
    vector<Vec3> &f_int_trans = beam.f_int_trans;
    vector<Vec3> &f_int_rot = beam.f_int_rot;
    vector<Vec3> &f_ext_trans = beam.f_ext_trans;
    vector<Vec3> &f_ext_rot = beam.f_ext_rot;
    vector<Vec3> &f_stat_trans = beam.f_stat_trans;
    vector<Vec3> &f_stat_rot = beam.f_stat_rot;
    const vector<scalar> &M = beam.M;
    const vector<Vec3> &J_u = beam.J_u;
    scalar &W_int = conf_dyn.W_int;
    scalar &W_ext = conf_dyn.W_ext;
    scalar &KE = conf_dyn.KE;
    vector<Vec3> &delta_d_trans = beam.delta_d_trans; /*Only used if energy balance is checked*/
    vector<Vec3> &delta_d_rot = beam.delta_d_rot;     /*Only used if energy balance is checked*/

    BCs &bcs_field = bcs.get_bcs_field(arena_h.buf);

    if (conf_stat.bc_top_omega_type == BC_TopOmegaControllerType::PID) {
        assert(false);
        // scalar torque_top = beam.f[IX_rx(conf_dyn.top_node)]; // assume this stores inner forces for now
        // scalar J_top = beam.Jx[IX_rx(conf_dyn.top_node)];
        // scalar torque_top_drive = PID_control(conf_dyn, conf_stat);
        // conf_dyn.omega_top_target = (torque_top_drive - torque_top) / J_top;
    } else { // Fixed omega from input
        conf_dyn.omega_top_target = conf_stat.omega_top_input;
    }

    update_top_node(config, conf_dyn, pipe, pipe_assembly);
    update_top_kinematics(conf_stat.dt, bcs_field, conf_dyn.v_top_target, conf_dyn.omega_top_target);

    if (check_energy_balance) {
        assert(delta_d_trans.size() == N && delta_d_rot.size() == N);
    } else {
        assert(delta_d_trans.size() == 0 && delta_d_rot.size() == 0);
    }

#pragma omp parallel for
    for (uint i = config.top_node; i < N; i++) {
        const Vec3 delta_d = dt * v_trans[i] + 0.5 * dt * dt * a_trans[i];
        d_trans[i] += delta_d;
        if (check_energy_balance) {
            delta_d_trans[i] = delta_d;
        }
    }

    // rotations: Simo and Wong algorithm
#pragma omp parallel for
    for (uint i = config.top_node; i < N; i++) {
        Quaternion &q = d_rot[i];
        const Mat3 &J = J_u[i].asDiagonal();
        Vec3 &omega_u = v_rot[i];
        Vec3 &alpha_u = a_rot[i];
        L_rot[i] =
            q.rotate_vector(J * omega_u); // Storing angular momentum L = U*J_u*omega_u at t_n for velocity update
        const Vec3 theta_u = dt * omega_u + 0.5 * dt * dt * alpha_u;
        q.exponential_map_body_frame(theta_u); // Update the rotation as U_{n+1} = U_n * exp(S(theta_u))

        if (check_energy_balance) {
            delta_d_rot[i] = q.rotate_vector(theta_u); // delta_d is stored in inertial frame
        }
    }

    /*Enforcing boundary conditions*/
    bool first_call = true;
    set_bc_top(first_call, config.top_node, conf_stat, bcs.get_bcs_field(arena_h.buf), pipe, hole, beam, arena_h.buf);

    if (check_energy_balance) {
        work_update_partial(N, config.top_node, delta_d_trans, delta_d_rot, f_int_trans, f_int_rot, f_ext_trans,
                            f_ext_rot, f_stat_trans, f_stat_rot, W_ext, W_int);
    }

    /*Update internal and external forces*/
    calc_forces(conf_stat, config.top_node, pipe, hole, beam, arena_h);

    set_bc_force_dynamic(N, conf_stat, beam, hole, arena_h.buf); // Adds bottom forces

    if (check_energy_balance) {
        work_update_partial(N, config.top_node, delta_d_trans, delta_d_rot, f_int_trans, f_int_rot, f_ext_trans,
                            f_ext_rot, f_stat_trans, f_stat_rot, W_ext, W_int);
    }

    /*Update translation velocities and the translational accelerations */

#pragma omp parallel for
    for (uint i = config.top_node; i < N; i++) {
        const Vec3 a_trans_new = (f_ext_trans[i] + f_stat_trans[i] - f_int_trans[i]) / M[i];
        v_trans[i] += dt * 0.5 * (a_trans[i] + a_trans_new);
        a_trans[i] = a_trans_new;
    }

    // rotations: Simo and Wong algorithm

#pragma omp parallel for
    for (uint i = config.top_node; i < N; i++) {

        const Mat3 &J = J_u[i].asDiagonal();
        const Vec3 &L_n = L_rot[i];
        Vec3 &omega_u = v_rot[i];
        Vec3 &alpha_u = a_rot[i];
        const Mat3 U_np = d_rot[i].to_matrix();
        Vec3 &m = m_rot[i]; // Moment at t_{n+1/2}
        const Vec3 m_np = f_ext_rot[i] + f_ext_rot[i] - f_int_rot[i];
        /*Evaluate moment at t_{n+1/2} by trapezoidal rule, i.e m_{n+1/2} = 1/2*(m_{n} + m_{n+1}) */
        const Vec3 m_half = 0.5 * (m + m_np);
        m = m_np; // Update moment
        const Vec3 omega_u_old = omega_u;

        omega_u = J.inverse() * U_np.transpose() * (L_n + dt * m_half);

        // ADD IN BELOW
        //#ifndef NDEBUG
        //         const Vec3 L_np = U_np * J * omega_u;
        //         const Vec3 res = L_np - L_n - dt * m_half;
        //         assert(is_close(res.norm(), 0.0));
        //#endif

        // beta = 0.5 (not recommended by Simo, but seems to work better)
        alpha_u = 2 / dt * (omega_u - omega_u_old) - alpha_u;
    }
    /*Enforcing boundary conditions*/
    first_call = false;
    set_bc_top(first_call, config.top_node, conf_stat, bcs.get_bcs_field(arena_h.buf), pipe, hole, beam, arena_h.buf);

    if (check_energy_balance) {
        kinetic_energy_update(N, M, J_u, v_trans, v_rot, KE);
    }

    if (config.top_node_new != config.top_node) {
        update_pipe_feed(config, pipe, hole, bcs, arena_h);
    }
}

void PipeSolver::calc_static_loads(const uint top_node, const ConfigStatic &conf_stat, const BCsData &bcs,
                                   const Pipe &pipe, vector<Vec3> &f_stat_trans, vector<Vec3> &f_stat_rot,
                                   ArenaBump &arena_h) {
    const uint N = pipe.N;
    const byte *buf = arena_h.buf;

    /*Set to zero first*/
    assert(f_stat_trans.size() == N && f_stat_rot.size() == N);
    for (uint i = 0; i < N; i++) {
        f_stat_trans[i].setZero();
        f_stat_rot[i].setZero();
    }

    const bool gravity_enabled = conf_stat.gravity_enabled;
    const Vec3 g{0, 0, -STANDARD_GRAVITY};

    const scalar rho = conf_stat.rho_p;
    for (uint ie = top_node; ie < N - 1; ie++) {
        if (gravity_enabled) {
            const scalar m = pipe.dS_e(ie, buf) * pipe.Ap_e(ie, buf) * rho;

            f_stat_trans[ie] += 0.5 * m * g;
            f_stat_trans[ie + 1] += 0.5 * m * g;
            // R_static[ie].trans.y() += m * STANDARD_GRAVITY / 2;
            // R_static[ie + 1].trans.y() += m * STANDARD_GRAVITY / 2;
        }
    }

    /*Point loads*/
    /*If specified the point loads will be applied relative to the fixed orientation of the first node (typically
     * cantilever case)*/
    f_stat_trans[N - 1] += Vec3{0.0, 0.0, 0.0}; // * sin(config.t * 10);;
    f_stat_rot[N - 1] += Vec3{0.0, 0.0, 0.0};   // * sin(config.t * 10);
}

void PipeSolver::update_pipe_feed(Config &config, const Pipe &pipe, const Hole &hole, const BCsData &bcs,
                                  ArenaBump &arena_h) {
    /* Called every time a new pipe component is added or removed which
    extends or reduces the pipe domain.*/
    if (config.top_node_new < config.top_node) {
        initialize_beam_segment_solution(config, config.top_node_new, config.top_node, hole, pipe, bcs, arena_h.buf);
    }
    config.top_node = config.top_node_new;
}

void PipeSolver::initialize_beam_segment_solution(const Config &config, uint start_node, uint end_node,
                                                  const Hole &hole, const Pipe &pipe, const BCsData &bcs, byte *buf) {

    const BCs &bcs_field = bcs.get_bcs_field(buf);

    const scalar u_top = bcs_field.u_top_np;
    const scalar v_top = bcs_field.v_top_np;
    const scalar a_top = bcs_field.a_top_np;
    const scalar theta_top = bcs_field.theta_top_np;
    const scalar omega_top = bcs_field.omega_top_np;
    const scalar omega_dot_top = bcs_field.alpha_top_np;

    const scalar u_top_old = bcs_field.u_top_n;
    const scalar theta_top_old = bcs_field.theta_top_n;

    vector<Vec3> &d_trans = beam.d_trans;
    vector<Quaternion> &d_rot = beam.d_rot;
    vector<Vec3> &v_trans = beam.v_trans;
    vector<Vec3> &v_rot = beam.v_rot;
    vector<Vec3> &a_trans = beam.a_trans;
    vector<Vec3> &a_rot = beam.a_rot;
    vector<uint> &i_pipe_to_ie_hole = beam.i_pipe_to_ie_hole;
    vector<Vec3> &delta_d_trans = beam.delta_d_trans;
    vector<Vec3> &delta_d_rot = beam.delta_d_rot;

    const Mat3 &R_top = hole.get_field<HoleField::q>(buf)[0].to_matrix();
    const Vec3 &t = R_top.col(0);

    Quaternion orientation_top = hole.get_field<HoleField::q>(buf)[2];

    orientation_top.exponential_map_body_frame(Vec3{theta_top, 0.0, 0.0});
    if (is_close(theta_top, 0.0)) {
        assert(hole.get_field<HoleField::q>(buf)[2].to_matrix().isApprox(orientation_top.to_matrix()));
    }

    Vec3 t_top = hole.get_field<HoleField::q>(buf)[2].to_matrix().col(0);

    for (uint i = start_node; i < end_node; i++) {
        d_trans[i] = t_top * u_top;
        v_trans[i] = {v_top, 0.0, 0.0};
        a_trans[i] = {a_top, 0.0, 0.0};

        d_rot[i] = orientation_top;
        v_rot[i] = {omega_top, 0.0, 0.0};
        a_rot[i] = {omega_dot_top, 0.0, 0.0};

        if (config.conf_stat.check_energy_balance) {
            delta_d_rot[i] = t_top * (theta_top - theta_top_old);
            delta_d_trans[i] = t_top * (u_top - u_top_old);
        }

        const scalar s_i = pipe.calc_s(i, u_top, buf);
        const uint i_hole = hole.get_increment_index_pipe_node_to_hole_segment(0, s_i, buf);
        assert(i_hole == 0 || i_hole == 1);
        i_pipe_to_ie_hole[i] = i_hole;
    }
}

void PipeSolver::save_csv(Config &config, const ConfigDynamic &conf_dyn, const Pipe &pipe, const Hole &hole,
                          const BCsData &bcs, ArenaBump &arena_h) {
    if (config.output_global_frame) {
        save_csv_global(config, conf_dyn, pipe, bcs, arena_h);
    } else {
        save_csv_curvlin(config, conf_dyn, pipe, hole, bcs, arena_h);
    }
}

void PipeSolver::save_csv_global(Config &config, const ConfigDynamic &conf_dyn, const Pipe &pipe, const BCsData &bcs,
                                 ArenaBump &arena_h) {
    using namespace std;
    assert(config.save_csv && config.n % config.n_write == 0);
    assert(config.output_global_frame);
    const string output_subdir = config.create_output_subdir();
    save_csv_header_file(pipe.N, config, conf_dyn, output_subdir, bcs, arena_h.buf);
    /*--------------------------------------------------------------------
    Save beam output
    --------------------------------------------------------------------*/
    {
        const string beam_file = output_subdir + "/beam.csv";
        ofstream ost{beam_file};
        if (!ost) {
            throw runtime_error{"Failed to open output file: " + beam_file + "\n"};
        }
        const uint N = pipe.N;
        save_csv_global_header(config, ost);
        for (uint i = 0; i < N; i++) {
            const Vec3 &X = beam.X[i];
            const Vec3 &d = beam.d_trans[i];
            const Mat3 &U = beam.d_rot[i].to_matrix();
            const Vec3 &v = beam.v_trans[i];
            const Vec3 &omega_u = beam.v_rot[i];
            save_csv_global_line(ost, X, d, U, v, omega_u);
        }
    }
}

void PipeSolver::save_csv_curvlin(Config &config, const ConfigDynamic &conf_dyn, const Pipe &pipe, const Hole &hole,
                                  const BCsData &bcs, ArenaBump &arena_h) {
    using namespace std;
    assert(config.save_csv && config.n % config.n_write == 0);

    const string output_subdir = config.create_output_subdir();
    save_csv_header_file(pipe.N, config, conf_dyn, output_subdir, bcs, arena_h.buf);

    /*--------------------------------------------------------------------
    Save beam output
    --------------------------------------------------------------------*/
    {
        uint i;
        const uint top_node = config.top_node;
        const uint N = pipe.N;
        const string beam_file = output_subdir + "/beam.csv";
        ofstream ost{beam_file};
        if (!ost) {
            throw runtime_error{"Failed to open output file: " + beam_file + "\n"};
        }

        save_csv_curvlin_header(config, ost);
        const ArrayView<scalar> arr_s = hole.get_field<HoleField::s>(arena_h.buf);
        const ArrayView<Vec3> x_hole = hole.get_field<HoleField::x>(arena_h.buf);
        const ArrayView<Quaternion> q_hole = hole.get_field<HoleField::q>(arena_h.buf);
        const vector<Vec3> X = beam.X;
        const vector<Vec3> &d_trans = beam.d_trans;
        const vector<Quaternion> &d_rot = beam.d_rot;
        const ArrayView<scalar> S = pipe.get_field<PipeField::S>(arena_h.buf);
        const scalar Lp_tot = pipe.L_tot;
        const scalar L_feed = conf_dyn.feed_length;

        for_each_node_cpu(i) {
            const uint ie_h = beam.i_pipe_to_ie_hole[i];
            assert(ie_h < hole.N_hole - 1);

            const Vec3 x = X[i] + d_trans[i];
            const Mat3 R = d_rot[i].to_matrix();
            const Vec3 &x_hole_A = x_hole[ie_h];
            const Vec3 &x_hole_B = x_hole[ie_h + 1];
            const Vec3 t = (x_hole_B - x_hole_A).normalized();
            const Vec3 x_center = x_hole_A + (x - x_hole_A).dot(t) * t;
            const Vec3 u_lateral = x - x_center;
            assert(is_orthogonal(t, u_lateral));

            scalar s = arr_s[ie_h] + (x_center - x_hole_A).norm();

            // Interpolation of rotations using lerp
            scalar t_interp = (s - arr_s[ie_h]) / (arr_s[ie_h + 1] - arr_s[ie_h]);
            Quaternion ql = Quaternion::lerp(q_hole[ie_h], q_hole[ie_h + 1], t_interp);

            // Rotate the lateral deviation vector onto the curvlin frame of referance
            Vec3 u_proj = ql.rotate_vector_reversed(u_lateral);
            assert(is_close(u_proj[0], 0.0, 1.0E-2)); // Should be no x-component

            // Using that: s = -Lp_tot + S + ux;
            const scalar ux = s + Lp_tot - S[i];
            const scalar ux_rel = ux - L_feed; // Relative to the feed length for plotting

            const Mat3 R_0 = q_hole[ie_h].to_matrix();
            const Mat3 R_hat = R_0.transpose() * R;

            // Extract Euler angles (ZYX order)
            scalar theta_y = -asin(R_hat(2, 0)); // Rotation about Y-axis
            scalar theta_x, theta_z;

            if (std::abs(R_hat(2, 0)) < 1.0 - 1e-6) {       // Not in gimbal lock
                theta_x = atan2(R_hat(2, 1), R_hat(2, 2));  // Rotation about X-axis
                theta_z = atan2(R_hat(1, 0), R_hat(0, 0));  // Rotation about Z-axis
            } else {                                        // Gimbal lock case
                theta_x = 0;                                // Arbitrary
                theta_z = atan2(-R_hat(0, 1), R_hat(1, 1)); // Combine rotations into Z
            }

            const Vec3 f_int = R_0.transpose() * beam.f_int_trans[i];
            const Vec3 f_dyn = R_0.transpose() * beam.f_ext_trans[i];
            const Vec3 f_hyd = R_0.transpose() * beam.f_stat_trans[i];
            const Vec3 m_int = R_0.transpose() * beam.f_int_rot[i];
            const Vec3 m_ext = R_0.transpose() * beam.f_ext_rot[i];
            const Vec3 m_stat = R_0.transpose() * beam.f_stat_rot[i];

            constexpr uint w = 11;
            ost << setw(w) << s << ", " << setw(w) << ie_h << ", " << setw(w) << ux_rel << ", " << setw(w) << u_proj[1]
                << ", " << setw(w) << u_proj[2] << ", " << setw(w) << theta_x << ", " << setw(w) << theta_y << ", "
                << setw(w) << theta_z << ", " << setw(w) << f_int.x() << ", " << setw(w) << f_int.y() << ", " << setw(w)
                << f_int.z() << ", " << setw(w) << m_int.x() << ", " << setw(w) << m_int.y() << ", " << setw(w)
                << m_int.z() << ", " << setw(w) << f_dyn.x() << ", " << setw(w) << f_dyn.y() << ", " << setw(w)
                << f_dyn.z() << ", " << setw(w) << m_ext.x() << ", " << setw(w) << m_ext.y() << ", " << setw(w)
                << m_ext.z() << ", " << setw(w) << f_hyd.x() << ", " << setw(w) << f_hyd.y() << ", " << setw(w)
                << f_hyd.z() << ", " << setw(w) << m_stat.x() << ", " << setw(w) << m_stat.y() << ", " << setw(w)
                << m_stat.z();
            if (config.write_mass) {
                ost << setw(w) << ", " << setw(w) << beam.M[i] << ", " << setw(w) << beam.J_u[i][0] << ", " << setw(w)
                    << beam.J_u[i][1];
            }
            ost << endl;
        }
    }
    if (config.write_mass) {
        config.write_mass = false; // Only written when changed
    }
}

} // namespace corot
