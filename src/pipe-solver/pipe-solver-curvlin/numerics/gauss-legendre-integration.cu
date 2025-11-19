#include "pipe-solver/pipe-solver-curvlin/pipe-solver-interface-curvlin.hpp"
#include "pipe-solver/pipe-solver-curvlin/numerics/gauss-legendre-fluid-forces.hpp"
#include "pipe-solver/pipe-solver-curvlin/numerics/gauss-legendre-utils.hpp"

namespace curvlin {


DEVICE_FUNC Vec12 element_forces_strain_energy(const Terms &terms, const Variations &variations, const scalar I, const scalar Ap,
 const scalar E, const scalar G, const scalar k, const scalar kappa_y, const scalar kappa_z);

DEVICE_FUNC inline Vec12 element_forces_dissipation_energy(uint ie, const ConfigStatic &conf_stat,
                                                           const Terms &terms, const Variations &variations,
                                                           const scalar Ap, const scalar Af, const scalar I,
                                                           const scalar E, const scalar G, const scalar k, 
                                                           const scalar kappa_y, const scalar kappa_z);

DEVICE_FUNC Vec12 element_forces_kinetic_energy(uint ie, const ConfigStatic &conf_stat, const Terms &terms,
                                       const Variations &variations, const scalar I, const scalar Ap, const scalar r_ec, const scalar kappa_y, const scalar kappa_z);

constexpr scalar constexpr_sqrt(scalar x) {
    assert(x >= 0);
    
    if (x == 0) return 0;
    scalar curr = x;
    scalar prev = 0;
    // Usually converges in < 10 iterations
    for (int i = 0; i < 20; i++) {
        prev = curr;
        curr = 0.5 * (curr + x / curr);
        if (curr == prev) break;  // Converged
    }
    return curr;
}
// Gauss-Legendre quadrature points and weights for 1 to 5 point integration
__host__ __device__ constexpr scalar gauss_points[5][5] = {
    {0.0},
    {-1.0 / constexpr_sqrt(3.0), 1.0 / constexpr_sqrt(3.0)},
    {-constexpr_sqrt(3.0 / 5.0), 0.0, constexpr_sqrt(3.0 / 5.0)},
    {-constexpr_sqrt((3.0 + 2.0 * constexpr_sqrt(6.0 / 5.0)) / 7.0), -constexpr_sqrt((3.0 - 2.0 * constexpr_sqrt(6.0 / 5.0)) / 7.0),
     constexpr_sqrt((3.0 - 2.0 * constexpr_sqrt(6.0 / 5.0)) / 7.0), constexpr_sqrt((3.0 + 2.0 * constexpr_sqrt(6.0 / 5.0)) / 7.0)},
    {-constexpr_sqrt(5.0 + 2.0 * constexpr_sqrt(10.0 / 7.0)) / 3.0, -constexpr_sqrt(5.0 - 2.0 * constexpr_sqrt(10.0 / 7.0)) / 3.0, 0.0,
     constexpr_sqrt(5.0 - 2.0 * constexpr_sqrt(10.0 / 7.0)) / 3.0, constexpr_sqrt(5.0 + 2.0 * constexpr_sqrt(10.0 / 7.0)) / 3.0}};

__host__ __device__ constexpr scalar gauss_weights[5][5] = {
    {2.0},
    {1.0, 1.0},
    {5.0 / 9.0, 8.0 / 9.0, 5.0 / 9.0},
    {(18.0 - constexpr_sqrt(30.0)) / 36.0, (18.0 + constexpr_sqrt(30.0)) / 36.0, (18.0 + constexpr_sqrt(30.0)) / 36.0, (18.0 - constexpr_sqrt(30.0)) / 36.0},
    {(322.0 - 13.0 * constexpr_sqrt(70.0)) / 900.0, (322.0 + 13.0 * constexpr_sqrt(70.0)) / 900.0, 128.0 / 225.0,
     (322.0 + 13.0 * constexpr_sqrt(70.0)) / 900.0, (322.0 - 13.0 * constexpr_sqrt(70.0)) / 900.0}};


DEVICE_FUNC void calc_element_forces_gauss_legendre(uint ie, const ConfigStatic &conf_stat, const BCsData &bcs,
                                                BeamData &beam, const FluidData &fluid, const Pipe &pipe, 
                                                const Hole &hole, byte *buf) {    
    const scalar dS = pipe.dS_e(ie, buf);
    const scalar Ap = pipe.Ap_e(ie, buf);
    const scalar Af = pipe.Af_e(ie, buf);
    const scalar I = pipe.Ip_e(ie, buf);
    const scalar ec = pipe.get_field<PipeField::ec>(buf)[ie] * conf_stat.ec_amplifier; 
    const scalar E = conf_stat.E;
    const scalar G = conf_stat.get_G();
    const scalar rho_fi = conf_stat.rho_fi;
    const scalar rho_fo = conf_stat.rho_fo;
    const scalar k = pipe.k_s(ie, buf);
    const scalar alpha_ts = 12.0 * E * I/ (k * G * Ap * dS * dS);

    ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);
    ArrayView<Vec3> v = beam.get_field<BeamField::v>(buf);
    ArrayView<Vec3> a = beam.get_field<BeamField::a>(buf);
    ArrayView<Vec3> theta = beam.get_field<BeamField::theta>(buf);
    ArrayView<Vec3> omega = beam.get_field<BeamField::omega>(buf);
    ArrayView<Vec3> alpha = beam.get_field<BeamField::alpha>(buf);
    
    ArrayView<Vec3> f_int = beam.get_field<BeamField::f_int>(buf);
    ArrayView<Vec3> m_int = beam.get_field<BeamField::m_int>(buf);
    ArrayView<uint> i_pipe_to_ie_hole = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);
    
    const scalar u_top = bcs.get_bcs_field(buf).u_top_np;
    const scalar s_i = pipe.calc_s(ie, u[ie].x(), buf);
    const scalar s_ip = pipe.calc_s(ie + 1, u[ie + 1].x(), buf);

    uint i_hole = i_pipe_to_ie_hole[ie];
    uint ip_hole = i_pipe_to_ie_hole[ie + 1];


    ArrayView<Vec2> kappas = hole.get_field<HoleField::kappa>(buf);
    const Vec2 curv_i = hole.lerp_hole_property_from_pipe_node_position<Vec2>(kappas, s_i, i_hole, buf);
    const Vec2 curv_ip = hole.lerp_hole_property_from_pipe_node_position<Vec2>(kappas, s_ip, ip_hole, buf);
    const Vec2 curvatures = (curv_i + curv_ip) / 2; // Elemental curvature
    scalar kappa_y = curvatures.x();
    scalar kappa_z = curvatures.y();

    scalar v_i, v_o, p_i, p_o, dp_i, dp_o, chi; //Added mass coefficient (Paidoussis)
    // Fluid values are defined in each drill string node (subject to change)
    ArrayView<scalar> arr_p_i = fluid.get_field<FluidField::p_i>(buf);
    ArrayView<scalar> arr_p_o = fluid.get_field<FluidField::p_o>(buf);
    ArrayView<scalar> arr_dp_i = fluid.get_field<FluidField::dp_i>(buf);
    ArrayView<scalar> arr_dp_o = fluid.get_field<FluidField::dp_o>(buf);
    v_i = fluid.get_field<FluidField::v_i>(buf)[ie + 1];
    v_o = fluid.get_field<FluidField::v_o>(buf)[ie + 1];
    p_i = arr_p_i[ie + 1];
    p_o = arr_p_o[ie + 1];
    dp_i = arr_dp_i[ie + 1];
    dp_o = arr_dp_o[ie + 1];
    chi = 0.0;
    if (conf_stat.fluid_dynamics_enabled) {
        scalar r_bh = hole.calc_pipe_element_hole_radius(s_i, s_ip, i_hole, ip_hole, buf);

        scalar r_o = pipe.get_field<PipeField::ro>(buf)[ie];
        chi = ((r_bh / r_o) * 2 * 2 + 1) / ((r_bh / r_o) * 2 * 2 - 1);
    }

    Vec12 U;
    U << u[ie].x() - u_top, u[ie].y(), u[ie].z(), theta[ie].x(),
        theta[ie].y(), theta[ie].z(), u[ie + 1].x() - u_top, u[ie + 1].y(),
        u[ie + 1].z(), theta[ie + 1].x(), theta[ie + 1].y(), theta[ie + 1].z();

    Vec12 V;
    V << v[ie].x(), v[ie].y(), v[ie].z(), omega[ie].x(),
        omega[ie].y(), omega[ie].z(), v[ie + 1].x(), v[ie + 1].y(),
        v[ie + 1].z(), omega[ie + 1].x(), omega[ie + 1].y(), omega[ie + 1].z();
    
    Vec12 A;
    A << a[ie].x(), a[ie].y(), a[ie].z(), alpha[ie].x(),
        alpha[ie].y(), alpha[ie].z(), a[ie + 1].x(), a[ie + 1].y(),
        a[ie + 1].z(), alpha[ie + 1].x(), alpha[ie + 1].y(), alpha[ie + 1].z();

    uint N_integration_points = conf_stat.N_points_gauss_legendre;
    assert(N_integration_points >= 1 && N_integration_points <= 5);

    const scalar *points = gauss_points[N_integration_points - 1];
    const scalar *weights = gauss_weights[N_integration_points - 1];

    Vec12 element_forces = Vec12::Zero();
    for (uint i = 0; i < N_integration_points; ++i) {
        scalar xi = (points[i] + 1) / 2.0; // Transform to 0-1 domain
        scalar weight = weights[i] / 2.0;  // Transform to 0-1 domain
        
        Variations variations;
        Terms terms;

        update_terms_and_variations(xi, dS, alpha_ts, U, V, A, terms, variations);
        
        Vec12 f = Vec12::Zero();

        f += element_forces_strain_energy(terms, variations, I, Ap, E, G, k, kappa_y, kappa_z);
        
        f += element_forces_kinetic_energy(ie, conf_stat, terms, variations, I, Ap, ec, kappa_y, kappa_z);
        
        if (conf_stat.rayleigh_damping_enabled) {
            f += element_forces_dissipation_energy(ie, conf_stat, terms, variations, Ap, Af, I, E, G, k, kappa_y, kappa_z);
        }

        if (conf_stat.fluid_dynamics_enabled) {
            f += element_fluid_contribution_forces(conf_stat, terms, variations, kappa_y, kappa_z,
              rho_fi, rho_fo, Af, Ap + Af, chi, v_i, v_o, p_i, p_o, dp_i, dp_o);
        }

        element_forces += weight * f * dS;
    }

    // Assign the calculated forces to the global force vector
    f_int[ie].x() += element_forces[0];
    f_int[ie].y() += element_forces[1];
    f_int[ie].z() += element_forces[2];
    m_int[ie].x() += element_forces[3];
    m_int[ie].y() += element_forces[4];
    m_int[ie].z() += element_forces[5];
    f_int[ie + 1].x() += element_forces[6];
    f_int[ie + 1].y() += element_forces[7];
    f_int[ie + 1].z() += element_forces[8];
    m_int[ie + 1].x() += element_forces[9];
    m_int[ie + 1].y() += element_forces[10];
    m_int[ie + 1].z() += element_forces[11];

}



DEVICE_FUNC inline Vec12 element_forces_strain_energy(const Terms &terms, const Variations &variations, const scalar I,
    const scalar Ap, const scalar E, const scalar G, const scalar k, 
    const scalar kappa_y, const scalar kappa_z) {

const scalar v = terms.v;
const scalar w = terms.w;
const scalar ty = terms.ty;
const scalar tz = terms.tz;

const scalar u_s = terms.u_s;
const scalar v_s = terms.v_s;
const scalar w_s = terms.w_s;
const scalar tx_s = terms.tx_s;
const scalar ty_s = terms.ty_s;
const scalar tz_s = terms.tz_s;


const scalar gamma_1 = u_s + 0.5 * powi(u_s, 2) + 0.5 * powi(v_s, 2) + 0.5 * powi(w_s, 2) + kappa_y * w + kappa_z * v;
const scalar gamma_2 = v_s - tz;
const scalar gamma_3 = w_s + ty;
const Vec3 gamma = Vec3(gamma_1, gamma_2, gamma_3);

const scalar kappa_1 = tx_s - ty_s * tz;
const scalar kappa_2 = ty_s + kappa_y + tx_s * tz;
const scalar kappa_3 = tz_s - kappa_z - tx_s * ty;
const Vec3 kappa = Vec3(kappa_1, kappa_2, kappa_3);


// clang-format off
Mat3 C_F;
    C_F << Ap * E, 0.0,         0.0,
           0.0,    Ap * G * k,  0.0,
           0.0,    0.0,         Ap * G * k;

Mat3 C_M;
    C_M << 2 * G * I, 0.0,   0.0,
           0.0,       E * I, 0.0,
           0.0,       0.0,   E * I;


//Mat3 C_F;
//C_F << Ap * (E - G * k * (ty*ty + tz*tz)),                Ap * (E - G * k) * tz, -Ap * (E - G * k) * ty,
//       Ap * (E - G * k) * tz, Ap * (G * k - E * tz*tz) ,             Ap * E * ty * tz,
//      -Ap * (E - G * k) * ty, Ap * E * ty * tz,                      Ap * (G * k - E * ty*ty);
//
//Mat3 C_M;
//C_M << I * (2 * G - E * (ty*ty + tz*tz)),          I*(2 * G - E) * tz, -I*(2 * G - E) * ty,
//       I*(2 * G - E) * tz, I * (E - 2 * G * tz*tz),                 2 * G * I * ty * tz,
//      -I*(2 * G - E) * ty, 2 * G * I * ty * tz,                   I * (E - 2 * G * ty*ty);

// clang-format on
    
const Vec3 f_inner = C_F * gamma;
const Vec3 m_inner = C_M * kappa;

Vec12 f = Vec12::Zero();
    
    // dgamma_1;
    Vec2 contrib_du_s = variations.du_s * ((1 + u_s) * f_inner.x());
    Vec4 contrib_dv_s = variations.dv_s * (v_s * f_inner.x());
    Vec4 contrib_dw_s = variations.dw_s * (w_s * f_inner.x());
    Vec4 contrib_dv = variations.dv * (kappa_z * f_inner.x());
    Vec4 contrib_dw = variations.dw * (kappa_y * f_inner.x());
    add_contrib_du(f, contrib_du_s);
    add_contrib_dv(f, contrib_dv_s);
    add_contrib_dw(f, contrib_dw_s);
    add_contrib_dv(f, contrib_dv);
    add_contrib_dw(f, contrib_dw);

    // dgamma_2;
    add_contrib_dv(f, variations.dv_s * f_inner.y());
    add_contrib_dtz(f, variations.dtz * (-f_inner.y()));
    

    // dgamma_3;
    add_contrib_dw(f, variations.dw_s * f_inner.z());
    add_contrib_dty(f, variations.dty * f_inner.z());

    // dkappa_1
    add_contrib_dtx(f, variations.dtx_s * m_inner.x());
    add_contrib_dty(f, variations.dty_s * (-tz * m_inner.x()));
    add_contrib_dtz(f, variations.dtz * (-ty_s * m_inner.x()));

    // dkappa_2
    add_contrib_dty(f, variations.dty_s * m_inner.y());
    add_contrib_dtx(f, variations.dtx_s * (tz * m_inner.y()));
    add_contrib_dtz(f, variations.dtz * (tx_s * m_inner.y()));

    // dkappa_3
    add_contrib_dtz(f, variations.dtz_s * m_inner.z());
    add_contrib_dtx(f, variations.dtx_s * (-ty * m_inner.z()));
    add_contrib_dty(f, variations.dty * (-tx_s * m_inner.z()));

return f;
}

DEVICE_FUNC inline Vec12 element_forces_kinetic_energy(uint ie, const ConfigStatic &conf_stat, const Terms &terms,
                                                                const Variations &variations, const scalar I, 
                                                                const scalar Ap, const scalar ec,
                                                                const scalar kappa_y, const scalar kappa_z) {
    const scalar rho_p = conf_stat.rho_p;

    
    const scalar tx = terms.tx;
    const scalar ty = terms.ty;
    const scalar tz = terms.tz;
    const scalar u_t = terms.u_t;
    const scalar tx_t = terms.tx_t;
    const scalar ty_t = terms.ty_t;
    const scalar tz_t = terms.tz_t;
    const scalar u_tt = terms.u_tt;
    const scalar v_tt = terms.v_tt;
    const scalar w_tt = terms.w_tt;
    const scalar tx_tt = terms.tx_tt;
    const scalar ty_tt = terms.ty_tt;
    const scalar tz_tt = terms.tz_tt;

    Vec12 f = Vec12::Zero();

    // Mass imbalance contributions (f -= Ap * dv * rho_p * ec * expr)
    scalar f_dv = -Ap * rho_p * ec * (powi(tx_t, 2) * std::cos(tx) + tx_tt * std::sin(tx));
    scalar f_dw = -Ap * rho_p * ec * (powi(tx_t, 2) * std::sin(tx) - tx_tt * std::cos(tx));
    scalar f_dtx = -Ap * rho_p * ec * (v_tt * std::sin(tx) - w_tt * std::cos(tx) - ec * tx_tt);
    add_contrib_dv(f, variations.dv * f_dv);  
    add_contrib_dw(f, variations.dw * f_dw);
    add_contrib_dtx(f, variations.dtx * f_dtx);
    
    // Rotational kinetic energy contributions
    scalar f_du = -rho_p * I * (-powi(kappa_y, 2)*u_tt - kappa_y * tx_t * tz_t - kappa_y*tx_tt*tz - kappa_y*ty_tt - powi(kappa_z, 2)*u_tt - kappa_z*tx_t*ty_t - kappa_z*tx_tt*ty + kappa_z*tz_tt);
    scalar f_dtx_rot = -rho_p * I * (-kappa_y*tz*u_tt - kappa_y*tz_t*u_t - kappa_z*ty*u_tt - kappa_z*ty_t*u_t - 2*tx_t*ty*ty_t - 2*tx_t*tz*tz_t - tx_tt*powi(ty, 2) - tx_tt*powi(tz, 2) + ty*tz_tt + 2*ty_t*tz_t + ty_tt*tz);
    scalar f_dty_rot = -rho_p * I * (-kappa_y*u_tt + kappa_z*tx_t*u_t + powi(tx_t, 2)*ty + tx_tt*tz - 4*ty_t*tz*tz_t - 2*ty_tt*powi(tz, 2));
    scalar f_dtz_rot = -rho_p * I * (kappa_y*tx_t*u_t + kappa_z*u_tt + powi(tx_t, 2)*tz + tx_tt*ty + 2*powi(ty_t, 2)*tz);
    add_contrib_du(f, variations.du * f_du);
    add_contrib_dtx(f, variations.dtx * f_dtx_rot);
    add_contrib_dty(f, variations.dty * f_dty_rot);
    add_contrib_dtz(f, variations.dtz * f_dtz_rot);

    
    return f;
}



DEVICE_FUNC inline Vec12 element_forces_dissipation_energy(uint ie, const ConfigStatic &conf_stat,
                                                           const Terms &terms, const Variations &variations,
                                                           const scalar Ap, const scalar Af, const scalar I,
                                                           const scalar E, const scalar G, const scalar k, 
                                                           const scalar kappa_y, const scalar kappa_z) {
    //NOTE! This function currently only implements stiffness-proportional Rayleigh damping (β term).
    //Mass-proportional damping (α term) is added in the time integration directly
    
    const scalar beta = conf_stat.beta;  // stiffness-proportional 
    const scalar u_ts  = terms.u_ts;
    const scalar v_ts  = terms.v_ts;
    const scalar w_ts  = terms.w_ts;
    const scalar tx_ts = terms.tx_ts;
    const scalar ty_ts = terms.ty_ts;
    const scalar tz_ts = terms.tz_ts;

    Vec12 f = Vec12::Zero();

    // (β) Elastic
    add_contrib_du(f, variations.du_s * (beta * (E * Ap) * u_ts));
    add_contrib_dty(f, variations.dty_s * (beta * (E * I) * ty_ts));
    add_contrib_dtz(f, variations.dtz_s * (beta * (E * I) * tz_ts));

    // (β) Shear + torsion
    add_contrib_dtx(f, variations.dtx_s * (beta * (2.0 * G * I) * tx_ts));
    add_contrib_dv(f, variations.dv_s * (beta * (G * k * Ap) * v_ts));
    add_contrib_dtz(f, variations.dtz * (-beta * (G * k * Ap) * v_ts));
    add_contrib_dw(f, variations.dw_s * (beta * (G * k * Ap) * w_ts));
    add_contrib_dty(f, variations.dty * (beta * (G * k * Ap) * w_ts));
    return f;
}


} // namespace curvlin
