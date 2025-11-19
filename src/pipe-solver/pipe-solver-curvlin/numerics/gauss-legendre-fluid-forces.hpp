#pragma once
#include "pipe-solver/pipe-solver-curvlin/beam-curvlin.hpp"
#include "pipe-solver/pipe-solver-curvlin/fluid-curvlin.hpp"
//#include "pipe-solver/pipe-solver-curvlin/numerics/gauss-legendre-utils.hpp"

namespace curvlin {

// ...existing code...

DEVICE_FUNC inline Vec12 element_fluid_contribution_forces(const ConfigStatic &conf_stat, const Terms &terms,
                                                           const Variations &variations, const scalar kappa_y,
                                                           const scalar kappa_z, const scalar rho_fi, const scalar rho_fo,
                                                           const scalar Af, const scalar Ao, const scalar chi, 
                                                           const scalar v_i, const scalar v_o, 
                                                           const scalar p_i, const scalar p_o,
                                                           const scalar dp_i, const scalar dp_o) {
    const scalar u_s = terms.u_s;
    const scalar v_s = terms.v_s;
    const scalar w_s = terms.w_s;
    const scalar u_ts = terms.u_ts;
    const scalar v_ts = terms.v_ts;
    const scalar w_ts = terms.w_ts;
    const scalar v = terms.v;
    const scalar w = terms.w;

    const scalar u_t = terms.u_t;
    const scalar v_t = terms.v_t;
    const scalar w_t = terms.w_t;

    // compact variations
    const Vec2 &du = variations.du;
    const Vec4 &dv = variations.dv;
    const Vec4 &dw = variations.dw;
    const Vec2 &du_s = variations.du_s;
    const Vec4 &dv_s = variations.dv_s;
    const Vec4 &dw_s = variations.dw_s;

    Vec12 f = Vec12::Zero();

    // pressure curvature term scalar
    scalar s_pcurv = (-Af * p_i + Ao * p_o);
    add_contrib_du(f, du_s * (s_pcurv * u_s));
    add_contrib_dv(f, dv_s * (s_pcurv * v_s));
    add_contrib_dw(f, dw_s * (s_pcurv * w_s));

    // pressure coupling to curvature
    scalar s_p = (Af * p_i - Ao * p_o);
    add_contrib_du(f, du * (s_p * (kappa_y * w_s + kappa_z * v_s)));

    // pressure gradient
    scalar s_dp = (Af * dp_i - Ao * dp_o);
    add_contrib_du(f, du * (s_dp * (1 + u_s + kappa_y * w + kappa_z * v)));
    add_contrib_dv(f, dv * (s_dp * v_s));
    add_contrib_dw(f, dw * (s_dp * w_s));

    // damping / fluid inertia
    scalar s_inertia = (rho_fi * Af * v_i - chi * rho_fo * Ao * v_o);
    add_contrib_du(f, du * (s_inertia * (u_ts + kappa_y * w_t + kappa_z * v_t)));
    add_contrib_dv(f, dv * (s_inertia * (2 * v_ts - kappa_z * u_t)));
    add_contrib_dw(f, dw * (s_inertia * (2 * w_ts - kappa_y * u_t)));

    // centrifugal terms
    scalar s_cent = (rho_fi * Af * v_i * v_i + chi * rho_fo * Ao * v_o * v_o);
    add_contrib_dv(f, dv_s * (-s_cent * v_s));
    add_contrib_dv(f, dv * (-s_cent * kappa_z));
    add_contrib_dw(f, dw_s * (-s_cent * w_s));
    add_contrib_dw(f, dw * (-s_cent * kappa_y));

    return f;
}

inline void calc_fsi_torque(const Config &config, BeamData &beam, const Pipe &pipe, const Hole &hole) {}

inline void calc_fsi_force_fritz(const Config &config, BeamData &beam, const Pipe &pipe, const Hole &hole) {
    assert(false); // TODO: Implement Fritz force model in element loop
}

inline void calc_fsi_force_jansen(const Config &config, BeamData &beam, const Pipe &pipe, const Hole &hole) {
    assert(false); // TODO: Implement Jansen force model in element loop
}

inline void calc_fsi_force_resell(const Config &config, BeamData &beam, const Pipe &pipe, const Hole &hole) {
    assert(false); // TODO: Implement Resell force model in element loop
}
inline void calc_fsi_force(const Config &config, BeamData &beam, const Pipe &pipe, const Hole &hole) {

    switch (config.conf_stat.fsi_force_model) {
    case FSI_ForceModel::PAIDOUSSIS:
        assert(false); // TODO: Implement Paidoussis force model in element loop
        break;
    case FSI_ForceModel::FRITZ:
        calc_fsi_force_fritz(config, beam, pipe, hole);
        break;
    case FSI_ForceModel::JANSEN:
        calc_fsi_force_jansen(config, beam, pipe, hole);
        break;
    case FSI_ForceModel::RESELL:
        calc_fsi_force_resell(config, beam, pipe, hole);
        break;
    default:
        assert(false);
    }
}

} // namespace curvlin