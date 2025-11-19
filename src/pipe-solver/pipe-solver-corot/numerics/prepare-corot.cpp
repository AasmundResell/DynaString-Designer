#include "prepare-corot.hpp"
#include "numerics-runtime-corot.hpp"

namespace corot {

void check_energy_balance(const Config &config, const ConfigDynamic &conf_dyn) {
    if (!config.conf_stat.check_energy_balance) {
        return;
    }
    const scalar KE = conf_dyn.KE;
    const scalar W_int = conf_dyn.W_int;
    const scalar W_ext = conf_dyn.W_ext;
    const scalar tol = config.energy_balance_tol;
    const scalar E_residual = KE + W_int - W_ext;

    if (abs(E_residual) > tol * max(KE, max(W_int, W_ext))) {
        printf("Warning: Energy balance is not obeyed, energy residual = %f\n", E_residual);
    } else if ((E_residual)) {
        printf("Nan detected in energy residual\n");
    }
}
void set_initial_configuration(const Config &config, const Hole &hole, const Pipe &pipe, vector<Vec3> &X,
                               vector<Vec3> &d_trans, vector<Quaternion> &d_rot, const vector<uint> &indices,
                               const BCsData &bcs, const byte *buf) {

    assert(X.size() == d_trans.size() && X.size() == d_rot.size());
    const uint N = X.size();
    const uint top_node = config.top_node;
    const scalar L_feed = config.conf_stat.L_string_depth_initial;
    const scalar L_pipe = pipe.L_tot;
    const ArrayView<scalar> S = pipe.get_field<PipeField::S>(buf);

    assert(N == pipe.N);
    assert(L_pipe >= L_feed);

    Quaternion q_identity;
    q_identity.from_matrix(Mat3::Identity());

    /*Initialize the referance configuration X, according to the
    discretization vector S, and tangent to the top of the hole "t_top"*/
    Quaternion q_top = hole.get_field<HoleField::q>(buf)[0];
    const Vec3 t_top = q_top.to_matrix().col(0);
    for (uint i = 0; i < N; i++) {
        X[i] = t_top * (-L_pipe + S[i]);
        d_rot[i] = q_identity; // Value set for debug purpose

        // Check origo
        assert(i != N - 1 || (is_close(X[i][0], 0.0) && is_close(X[i][1], 0.0) && is_close(X[i][2], 0.0)));
    }

    const ArrayView<Quaternion> q_hole = hole.get_field<HoleField::q>(buf);
    const ArrayView<Vec3> x_hole = hole.get_field<HoleField::x>(buf);
    const ArrayView<scalar> s_hole = hole.get_field<HoleField::s>(buf);

    Quaternion ql;

    for (uint i = top_node; i < N; i++) {
        scalar s_pipe = pipe.calc_s(i, L_feed, buf);
        const uint i_hole = indices[i];
        assert(i_hole >= 0 && i_hole < hole.N_hole - 1);
        d_trans[i] = hole.lerp_hole_property_from_pipe_node_position<Vec3>(x_hole, s_pipe, i_hole, buf) - X[i];

        scalar t_interp = (s_pipe - s_hole[i_hole]) / (s_hole[i_hole + 1] - s_hole[i_hole]);
        ql = Quaternion::lerp(q_hole[i_hole], q_hole[i_hole + 1], t_interp);
        d_rot[i] = ql;
    }
}

void calc_initial_accelerations(const ConfigStatic &conf_stat, const uint top_node, const Pipe &pipe, const Hole &hole,
                                BeamData &beam, ArenaBump &arena_h) {
    calc_forces(conf_stat, top_node, pipe, hole, beam, arena_h);
    for (uint i = 0; i < pipe.N; i++) {
        const scalar M = beam.M[i];
        const Vec3 f = beam.f_ext_trans[i] - beam.f_int_trans[i];
        beam.a_trans[i] = f / M;

        const Mat3 U = beam.d_rot[i].to_matrix();
        const Mat3 &J_u = beam.J_u[i].asDiagonal();
        const Vec3 &m = beam.f_ext_rot[i] - beam.f_int_rot[i];
        const Vec3 &omega_u = beam.v_rot[i];
        beam.a_rot[i] = J_u.inverse() * (U.transpose() * m - omega_u.cross(J_u * omega_u));
    }
}

} // namespace corot