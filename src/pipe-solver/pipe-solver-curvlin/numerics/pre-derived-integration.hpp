#pragma once
#include "pipe-solver/pipe-solver-curvlin/beam-curvlin.hpp"
#include "pipe-solver/pipe-solver-curvlin/fluid-curvlin.hpp"

namespace curvlin {

DEVICE_FUNC inline void calc_element_inner_forces_prederived(uint ie, const ConfigStatic &conf_stat, const BCsData &bc,
                                                             BeamData &beam, const Pipe &pipe, const Hole &hole,
                                                             byte *buf) {
    const scalar dx = pipe.dS_e(ie, buf);
    assert(dx > SMALL_SCALAR);
    const scalar Ap = pipe.Ap_e(ie, buf);
    const scalar Ip = pipe.Ip_e(ie, buf);
    const scalar k = pipe.k_s(ie, buf);
    const scalar E = conf_stat.E;
    const scalar G = conf_stat.get_G();
    const scalar alpha = 12.0 * E * Ip / (k * G * Ap * dx * dx);

    const scalar beta = 1.0 / (1.0 + alpha);
    const scalar u_top = bc.get_bcs_field(buf).u_top_np;

    ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);
    ArrayView<Vec3> v = beam.get_field<BeamField::v>(buf);
    ArrayView<Vec3> theta = beam.get_field<BeamField::theta>(buf);
    ArrayView<Vec3> omega = beam.get_field<BeamField::omega>(buf);
    ArrayView<Vec3> f_int = beam.get_field<BeamField::f_int>(buf);
    ArrayView<Vec3> m_int = beam.get_field<BeamField::m_int>(buf);

    const scalar ux_i = u[ie].x() - u_top;
    const scalar ux_ip = u[ie + 1].x() - u_top;
    scalar kappa_y, kappa_z;

//---Axial and torsional forces/torques with rayleigh beta damping ---
const scalar EA_dx = Ap * E / dx;
const scalar GIp2_dx = 2.0 * G * Ip / dx;
// Axial
const scalar du = (u[ie].x() - u_top) - (u[ie + 1].x() - u_top); // = u[ie].x() - u[ie+1].x()
const scalar dv = v[ie].x() - v[ie + 1].x();
const scalar fac_fx = EA_dx * (du + beta * dv);
f_int[ie].x()     += fac_fx;
f_int[ie + 1].x() += -fac_fx;

// Torsion
const scalar dtheta = theta[ie].x() - theta[ie + 1].x();
const scalar domega = omega[ie].x() - omega[ie + 1].x();
const scalar fac_mx = GIp2_dx * (dtheta + beta * domega);
m_int[ie].x()     += fac_mx;
m_int[ie + 1].x() += -fac_mx;

//---Lateral stiffness contributions (no rayleigh beta damping) ---

    // clang-format off
const scalar K_00 = -12*powi(beta, 2);
const scalar K_01 = 6*powi(beta, 2)*dx;
const scalar K_02 = 3*powi(beta, 2)*powi(dx, 2) - powi(dx, 2);
const scalar K_03 = 3*powi(beta, 2)*powi(dx, 2) + powi(dx, 2);
const scalar K_04 = -6*powi(beta, 2) + 12*beta - 6;
const scalar K_05 = K_00/dx + 12*beta/dx;
const scalar K_06 = -3*powi(beta, 2)*dx + 6*beta*dx - 3*dx;
const scalar K_07 = 6*powi(beta, 2) - 6*beta;
const scalar K_08 = 12*beta - 12;
const scalar K_09 = -6*beta*dx + 6*dx;



const scalar fac_01 = E*Ip/powi(dx, 3);
f_int[ie].y() += fac_01*(-K_00*u[ie].y() + K_00*u[ie + 1].y() + K_01*theta[ie + 1].z() + K_01*theta[ie].z());
f_int[ie].z() += fac_01*(-K_00*u[ie].z() + K_00*u[ie + 1].z() - K_01*theta[ie + 1].y() - K_01*theta[ie].y());
m_int[ie].y() += fac_01*(-K_01*u[ie].z() + K_01*u[ie + 1].z() + K_02*theta[ie + 1].y() + K_03*theta[ie].y());
m_int[ie].z() += fac_01*(K_01*u[ie].y() - K_01*u[ie + 1].y() + K_02*theta[ie + 1].z() + K_03*theta[ie].z());
f_int[ie + 1].y() += fac_01*(K_00*u[ie].y() - K_00*u[ie + 1].y() - K_01*theta[ie + 1].z() - K_01*theta[ie].z());
f_int[ie + 1].z() += fac_01*(K_00*u[ie].z() - K_00*u[ie + 1].z() + K_01*theta[ie + 1].y() + K_01*theta[ie].y());
m_int[ie + 1].y() += fac_01*(-K_01*u[ie].z() + K_01*u[ie + 1].z() + K_02*theta[ie].y() + K_03*theta[ie + 1].y());
m_int[ie + 1].z() += fac_01*(K_01*u[ie].y() - K_01*u[ie + 1].y() + K_02*theta[ie].z() + K_03*theta[ie + 1].z());

const scalar fac_03 = E*Ip*beta/(beta*powi(dx, 2) - powi(dx, 2));
f_int[ie].y() += fac_03*(K_04*theta[ie + 1].z() + K_04*theta[ie].z() + K_05*u[ie].y() - K_05*u[ie + 1].y());
f_int[ie].z() += fac_03*(-K_04*theta[ie + 1].y() - K_04*theta[ie].y() + K_05*u[ie].z() - K_05*u[ie + 1].z());
m_int[ie].y() += fac_03*(K_06*theta[ie + 1].y() + K_06*theta[ie].y() + K_07*u[ie].z() - K_07*u[ie + 1].z());
m_int[ie].z() += fac_03*(K_06*theta[ie + 1].z() + K_06*theta[ie].z() - K_07*u[ie].y() + K_07*u[ie + 1].y());
f_int[ie + 1].y() += fac_03*(-K_04*theta[ie + 1].z() - K_04*theta[ie].z() - K_05*u[ie].y() + K_05*u[ie + 1].y());
f_int[ie + 1].z() += fac_03*(K_04*theta[ie + 1].y() + K_04*theta[ie].y() - K_05*u[ie].z() + K_05*u[ie + 1].z());
m_int[ie + 1].y() += fac_03*(K_06*theta[ie + 1].y() + K_06*theta[ie].y() + K_07*u[ie].z() - K_07*u[ie + 1].z());
m_int[ie + 1].z() += fac_03*(K_06*theta[ie + 1].z() + K_06*theta[ie].z() - K_07*u[ie].y() + K_07*u[ie + 1].y());

const scalar fac_04 = E*Ip*beta/(beta*powi(dx, 3) - powi(dx, 3));
f_int[ie].y() += fac_04*(K_08*u[ie].y() - K_08*u[ie + 1].y());
f_int[ie].z() += fac_04*(K_08*u[ie].z() - K_08*u[ie + 1].z());
m_int[ie].y() += fac_04*(K_09*u[ie].z() - K_09*u[ie + 1].z());
m_int[ie].z() += fac_04*(-K_09*u[ie].y() + K_09*u[ie + 1].y());
f_int[ie + 1].y() += fac_04*(-K_08*u[ie].y() + K_08*u[ie + 1].y());
f_int[ie + 1].z() += fac_04*(-K_08*u[ie].z() + K_08*u[ie + 1].z());
m_int[ie + 1].y() += fac_04*(K_09*u[ie].z() - K_09*u[ie + 1].z());
m_int[ie + 1].z() += fac_04*(-K_09*u[ie].y() + K_09*u[ie + 1].y());

    // clang-format on

    if (conf_stat.curvature_enabled) { // Linear curvature terms

        const scalar s_i = pipe.calc_s(ie, u[ie].x(), buf);
        const scalar s_ip = pipe.calc_s(ie + 1, u[ie + 1].x(), buf);
        ArrayView<uint> i_pipe_to_ie_hole = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);

        uint i_hole = i_pipe_to_ie_hole[ie];
        uint ip_hole = i_pipe_to_ie_hole[ie + 1];
        ArrayView<Vec2> curvatures_arr = hole.get_field<HoleField::kappa>(buf);
        const Vec2 curv_i = hole.lerp_hole_property_from_pipe_node_position<Vec2>(curvatures_arr, s_i, i_hole, buf);
        const Vec2 curv_ip = hole.lerp_hole_property_from_pipe_node_position<Vec2>(curvatures_arr, s_ip, ip_hole, buf);
        const Vec2 curvatures = (curv_i + curv_ip) / 2; // Elemental curvature
        kappa_y = curvatures.x();
        kappa_z = curvatures.y();

        // clang-format off
        const scalar K_10 = -1.0/420.0*powi(beta, 2)*dx - 1.0/120.0*beta*dx + (1.0/24.0)*dx;
        const scalar K_11 = (1.0/210.0)*powi(beta, 2) + (1.0/30.0)*beta + 1.0/3.0;
        const scalar K_12 = -1.0/420.0*powi(beta, 2)*dx - 1.0/120.0*beta*dx - 1.0/24.0*dx;
        //const scalar K_13 = -1.0/210.0*powi(beta, 2) - 1.0/30.0*beta + 1.0/6.0;
        const scalar K_14 = (1.0/840.0)*powi(beta, 2)*powi(dx, 2) - 1.0/120.0*powi(dx, 2);
        const scalar K_15 = (1.0/840.0)*powi(beta, 2)*powi(dx, 2) + (1.0/120.0)*powi(dx, 2);
        const scalar K_16 = -1.0/2.0;
        const scalar K_17 = (1.0/12.0)*dx;

        const scalar fac_10 = Ap*E*dx*powi(kappa_y, 2);
        f_int[ie].z() += fac_10*(K_10*theta[ie + 1].y() + K_11*u[ie].z() + K_12*theta[ie].y() + u[ie + 1].z()*(-K_11 - K_16));
        m_int[ie].y() += fac_10*(-K_10*u[ie + 1].z() + K_12*u[ie].z() + K_14*theta[ie + 1].y() + K_15*theta[ie].y());
        f_int[ie + 1].z() += fac_10*(-K_10*theta[ie].y() + K_11*u[ie + 1].z() - K_12*theta[ie + 1].y() + u[ie].z()*(-K_11 - K_16));
        m_int[ie + 1].y() += fac_10*(K_10*u[ie].z() - K_12*u[ie + 1].z() + K_14*theta[ie].y() + K_15*theta[ie + 1].y());

        const scalar fac_11 = Ap*E*dx*kappa_y*kappa_z;
        f_int[ie].y() += fac_11*(-K_10*theta[ie + 1].y() - K_11*u[ie].z() - K_12*theta[ie].y() + u[ie + 1].z()*(K_11 + K_16));
        f_int[ie].z() += fac_11*(K_10*theta[ie + 1].z() - K_11*u[ie].y() + K_12*theta[ie].z() + u[ie + 1].y()*(K_11 + K_16));
        m_int[ie].y() += fac_11*(K_10*u[ie + 1].y() - K_12*u[ie].y() + K_14*theta[ie + 1].z() + K_15*theta[ie].z());
        f_int[ie + 1].y() += fac_11*(K_10*theta[ie].y() - K_11*u[ie + 1].z() + K_12*theta[ie + 1].y() + u[ie].z()*(K_11 + K_16));
        f_int[ie + 1].z() += fac_11*(-K_10*theta[ie].z() - K_11*u[ie + 1].y() - K_12*theta[ie + 1].z() + u[ie].y()*(K_11 + K_16));
        m_int[ie + 1].y() += fac_11*(-K_10*u[ie].y() + K_12*u[ie + 1].y() + K_14*theta[ie].z() + K_15*theta[ie + 1].z());
        m_int[ie + 1].z() += fac_11*(K_10*u[ie].z() - K_12*u[ie + 1].z() + K_14*theta[ie].y() + K_15*theta[ie + 1].y());

        const scalar fac_12 = (1.0/6.0)*Ap*E*dx*kappa_y*theta[ie + 1].y() - 1.0/6.0*Ap*E*dx*kappa_y*theta[ie].y() + Ap*E*kappa_y*u[ie].z() + Ap*E*kappa_y*u[ie + 1].z();
        f_int[ie].x() += fac_12*(K_16);
        f_int[ie].x() += fac_12*(-K_16);

        const scalar fac_13 = Ap*E*kappa_y*ux_i - Ap*E*kappa_y*ux_ip;
        f_int[ie].z() += fac_13*(K_16);
        m_int[ie].y() += fac_13*(K_17);
        f_int[ie + 1].z() += fac_13*(K_16);
        m_int[ie + 1].y() += fac_13*(-K_17);

        const scalar fac_14 = Ap*E*dx*powi(kappa_z, 2);
        f_int[ie].y() += fac_14*(-K_10*theta[ie + 1].z() + K_11*u[ie].y() - K_12*theta[ie].z() + u[ie + 1].y()*(-K_11 - K_16));
        m_int[ie].z() += fac_14*(K_10*u[ie + 1].y() - K_12*u[ie].y() + K_14*theta[ie + 1].z() + K_15*theta[ie].z());
        f_int[ie + 1].y() += fac_14*(K_10*theta[ie].z() + K_11*u[ie + 1].y() + K_12*theta[ie + 1].z() + u[ie].y()*(-K_11 - K_16));
        m_int[ie + 1].z() += fac_14*(-K_10*u[ie].y() + K_12*u[ie + 1].y() + K_14*theta[ie].z() + K_15*theta[ie + 1].z());

        const scalar fac_15 = -1.0/6.0*Ap*E*dx*kappa_z*theta[ie + 1].z() + (1.0/6.0)*Ap*E*dx*kappa_z*theta[ie].z() + Ap*E*kappa_z*u[ie].y() + Ap*E*kappa_z*u[ie + 1].y();
        f_int[ie].x() += fac_15*(-K_16);
        f_int[ie].x() += fac_15*(K_16);

        const scalar fac_16 = Ap*E*kappa_z*ux_i - Ap*E*kappa_z*ux_ip;
        f_int[ie].y() += fac_16*(-K_16);
        m_int[ie].z() += fac_16*(K_17);
        f_int[ie + 1].y() += fac_16*(-K_16);
        m_int[ie + 1].z() += fac_16*(-K_17);
        // clang-format on
    }
    if (conf_stat.beam_stiffness_2nd_order_enabled) { // Non-linear "geometric" stiffness terms
        // clang-format off
        const scalar K_21 = -1.0/5.0*powi(beta, 2) - 1;
        const scalar K_22 = -1.0/10.0*powi(beta, 2)*dx;
        //const scalar K_23 = -1.0/20.0*powi(beta, 2)*powi(dx, 2) + (1.0/12.0)*powi(dx, 2);
        //const scalar K_24 = -1.0/20.0*powi(beta, 2)*powi(dx, 2) - 1.0/12.0*powi(dx, 2);
            
        const scalar fac_22 = (Ap*E*ux_i - Ap*E*ux_ip)/powi(dx, 2);
        f_int[ie].y() += fac_22*(K_21*u[ie].y() - K_21*u[ie + 1].y() + K_22*theta[ie + 1].z() + K_22*theta[ie].z());
        f_int[ie].z() += fac_22*(K_21*u[ie].z() - K_21*u[ie + 1].z() - K_22*theta[ie + 1].y() - K_22*theta[ie].y());
        m_int[ie].y() += fac_22*(-K_22*u[ie].z() + K_22*u[ie + 1].z() + theta[ie + 1].y()*((1.0/2.0)*K_22*dx + (1.0/12.0)*powi(dx, 2)) + theta[ie].y()*((1.0/2.0)*K_22*dx - 1.0/12.0*powi(dx, 2)));
        m_int[ie].z() += fac_22*(K_22*u[ie].y() - K_22*u[ie + 1].y() + theta[ie + 1].z()*((1.0/2.0)*K_22*dx + (1.0/12.0)*powi(dx, 2)) + theta[ie].z()*((1.0/2.0)*K_22*dx - 1.0/12.0*powi(dx, 2)));
        f_int[ie + 1].y() += fac_22*(-K_21*u[ie].y() + K_21*u[ie + 1].y() - K_22*theta[ie + 1].z() - K_22*theta[ie].z());
        f_int[ie + 1].z() += fac_22*(-K_21*u[ie].z() + K_21*u[ie + 1].z() + K_22*theta[ie + 1].y() + K_22*theta[ie].y());
        m_int[ie + 1].y() += fac_22*(-K_22*u[ie].z() + K_22*u[ie + 1].z() + theta[ie + 1].y()*((1.0/2.0)*K_22*dx - 1.0/12.0*powi(dx, 2)) + theta[ie].y()*((1.0/2.0)*K_22*dx + (1.0/12.0)*powi(dx, 2)));
        m_int[ie + 1].z() += fac_22*(K_22*u[ie].y() - K_22*u[ie + 1].y() + theta[ie + 1].z()*((1.0/2.0)*K_22*dx - 1.0/12.0*powi(dx, 2)) + theta[ie].z()*((1.0/2.0)*K_22*dx + (1.0/12.0)*powi(dx, 2)));
            // clang-format on
    }
}

/*

DEVICE_FUNC inline void calc_element_fluid_forces_prederived-spiral(uint ie, const ConfigStatic &conf_stat,
                                                              BeamData &beam,
                                                              const FluidData &fluid, const Pipe &pipe,
                                                              const Hole &hole, byte *buf) {

    /*--------------------------------------------------------------------
    Calculate drill string forces that originates from the fluid and are dependant on the
    dynamic motion and shape of the drill string structure.
    Summary of contributions:
    Ritto: https://hal.science/hal-00692826/document
    Reddy: https://www.researchgate.net/publication/228949667_Dynamics_of_fluid-conveying_beams
    --------------------------------------------------------------------*/
/*
assert(false); // New expressions must be derived!
const uint N = pipe.N;
ArrayView<int> i_pipe_to_ie_hole = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);
ArrayView<scalar> u = beam.get_field<BeamField::u>(buf);

uint i_hole = i_pipe_to_ie_hole[ie];

assert(false); // Chi must be calculated on a per-element basis!
const scalar s_i = pipe.calc_s(ie, u[[ie].x()], buf);
ArrayView<scalar> r_h = hole.get_field<HoleField::a>(buf);
ArrayView<scalar> r_o = pipe.get_field<PipeField::ro>(buf);

const scalar r_bh = hole.lerp_hole_property_from_pipe_node_position<scalar>(r_h, s_i, i_hole, buf);
const scalar chi = ((r_bh / r_o[ie]) * 2 * 2 + 1) / ((r_bh / r_o[ie]) * 2 * 2 - 1);
const scalar rho_f = conf_stat.rho_f;

const scalar ds = pipe.dS_e(ie, buf);
const scalar Af = pipe.Af_e(ie, buf);
const scalar Ap = pipe.Ap_e(ie, buf);
const scalar Ao = Ap + Af;
const scalar Ip = pipe.Ip_e(ie, buf);
const scalar k = pipe.k_s(ie, buf);
const scalar E = conf_stat.E;
const scalar G = conf_stat.get_G();
const scalar alpha = 12.0 * E * Ip / (k * G * Ap * ds * ds);

ArrayView<scalar> arr_v_i = fluid.get_field<FluidField::v_i>(buf);
ArrayView<scalar> arr_v_o = fluid.get_field<FluidField::v_o>(buf);
ArrayView<scalar> arr_p_i = fluid.get_field<FluidField::p_i>(buf);
ArrayView<scalar> arr_p_o = fluid.get_field<FluidField::p_o>(buf);

// Fluid values are defined in each drill string node
const scalar ve_i = (arr_v_i[ie + 1] + arr_v_i[ie]) / 2.0;
const scalar ve_o = (arr_v_o[ie + 1] + arr_v_o[ie]) / 2.0;
const scalar pe_i = (arr_p_i[ie + 1] + arr_p_i[ie]) / 2.0;
const scalar pe_o = (arr_p_o[ie + 1] + arr_p_o[ie]) / 2.0;

const scalar dp_i = (arr_p_i[ie + 1] - arr_p_i[ie]) / pipe.dS_e(ie, buf);
const scalar dp_o = (arr_p_o[ie + 1] - arr_p_o[ie]) / pipe.dS_e(ie, buf);

const scalar m_f = Af * rho_f;
const scalar beta = 1.0 / (1.0 + alpha);

*/
/*--------------------------------------------------------------------
//Centripetal force and pressure
Symmetric stiffness contributions by: Nw'T * Nw' shape functions
--------------------------------------------------------------------*/
/*
const scalar fs = beta * beta * (-m_f * ve_i * ve_i - Af * pe_i + Af * pe_o - chi * rho_f * Af * ve_o * ve_o);
const scalar ks00 = fs * (alpha * alpha + 2 * alpha + 6.0 / 5) / ds;
const scalar ks01 = fs * 1.0 / 10;
const scalar ks02 = -ks00;
const scalar ks03 = ks01;
const scalar ks11 = fs * (alpha * alpha / 12 + alpha / 6 + 2.0 / 15) * ds;
const scalar ks12 = -ks01;
const scalar ks13 = -fs * (alpha * alpha / 12 + alpha / 6 + 1.0 / 30) * ds;
const scalar ks22 = ks00;
const scalar ks23 = -ks01;
const scalar ks33 = ks11;
uint DOF = 2 * ie;
Ry[DOF] -= (ks00 * uy[DOF] + ks01 * uy[DOF + 1] + ks02 * uy[DOF + 2] + ks03 * uy[DOF + 3]);
Ry[DOF + 1] -= (ks01 * uy[DOF] + ks11 * uy[DOF + 1] + ks12 * uy[DOF + 2] + ks13 * uy[DOF + 3]);
Ry[DOF + 2] -= (ks02 * uy[DOF] + ks12 * uy[DOF + 1] + ks22 * uy[DOF + 2] + ks23 * uy[DOF + 3]);
Ry[DOF + 3] -= (ks03 * uy[DOF] + ks13 * uy[DOF + 1] + ks23 * uy[DOF + 2] + ks33 * uy[DOF + 3]);

Rz[DOF] -= (ks00 * uz[DOF] - ks01 * uz[DOF + 1] + ks02 * uz[DOF + 2] - ks03 * uz[DOF + 3]);
Rz[DOF + 1] -= (-ks01 * uz[DOF] + ks11 * uz[DOF + 1] - ks12 * uz[DOF + 2] + ks13 * uz[DOF + 3]);
Rz[DOF + 2] -= (ks02 * uz[DOF] - ks12 * uz[DOF + 1] + ks22 * uz[DOF + 2] - ks23 * uz[DOF + 3]);
Rz[DOF + 3] -= (-ks03 * uz[DOF] + ks13 * uz[DOF + 1] - ks23 * uz[DOF + 2] + ks33 * uz[DOF + 3]);
*/

/*--------------------------------------------------------------------
//Tangential force from fluid acceleration
Semi-antisymmetric contribution by: NwT * Nw' shape functions

    Ka  = (m_f * a_i - chi * A_p * rho_p * a_o ) *
                      [[ ka00,  ka01,  ka02, ka03],
                       [-ka01,  0.0 ,  ka12, ka13],
                       [-ka02, -ka12,  ka22, ka23],
                       [-ka03, -ka13, -ka23, 0.0 ]];
--------------------------------------------------------------------*/
/*
const scalar fa = beta * beta * (m_f * ae_i - chi * rho_f * Ap * ae_o); // Opposite direction, should make sense?
const scalar ka00 = -fa * (alpha * alpha / 2 + alpha + 1.0 / 2);
const scalar ka01 = fa * (alpha * alpha / 12 + 11 * alpha / 60 + 1.0 / 10) * dx;
const scalar ka02 = -ka00;
const scalar ka03 = -ka01;
const scalar ka12 = ka12;
const scalar ka13 = -fa * (alpha / 60 + 1.0 / 60) * dx * dx;
const scalar ka22 = -ka00;
const scalar ka23 = ka01;

Ry[DOF] -= (ka00 * uy[DOF] + ka01 * uy[DOF + 1] + ka02 * uy[DOF + 2] + ka03 * uy[DOF + 3]);
Ry[DOF + 1] -= (-ka01 * uy[DOF] + ka12 * uy[DOF + 2] + ka13 * uy[DOF + 3]);
Ry[DOF + 2] -= (-ka02 * uy[DOF] - ka12 * uy[DOF + 1] + ka22 * uy[DOF + 2] + ka23 * uy[DOF + 3]);
Ry[DOF + 3] -= (-ka03 * uy[DOF] - ka13 * uy[DOF + 1] - ka23 * uy[DOF + 2]);

Rz[DOF] -= (ka00 * uz[DOF] - ka01 * uz[DOF + 1] + ka02 * uz[DOF + 2] - ka03 * uz[DOF + 3]);
Rz[DOF + 1] -= (ka01 * uz[DOF] - ka12 * uz[DOF + 2] + ka13 * uz[DOF + 3]);
Rz[DOF + 2] -= (-ka02 * uz[DOF] + ka12 * uz[DOF + 1] + ka22 * uz[DOF + 2] - ka23 * uz[DOF + 3]);
Rz[DOF + 3] -= (ka03 * uz[DOF] - ka13 * uz[DOF + 1] + ka23 * uz[DOF + 2]);
*/

/*--------------------------------------------------------------------
//Fluid pressure-gradient contribution
Semi-antisymmetric contribution by NwT * Nw' - Nw'T * Nw shape functions

    Kdp  = ( - m_f * v_i + chi * rho_f * A_o * v_o ) *
                      [[ 0.0,   cv01,  cv02, cv03],
                       [-cv01,  0.0,   cv12, cv13],
                       [-cv02, -cv12,  0.0,  cv23],
                       [-cv03, -cv13, -cv23,  0.0]];
--------------------------------------------------------------------*/
/*
const scalar dp = beta * beta * (Af * dp_i - Af * dp_o) / 2;
const scalar kp01 = dp * (alpha * alpha * 1.0 / 6 + alpha * 11.0 / 30 + 1.0 / 5) * ds;
const scalar kp02 = dp * alpha * alpha + 2.0 * alpha + 1.0;
const scalar kp03 = -kp01;
const scalar kp12 = kp01;
const scalar kp13 = -dp * (alpha / 30 + 1.0 / 30) * ds * ds;
const scalar kp23 = kp01;

Ry[DOF] -= (kp01 * vy[DOF + 1] + kp02 * vy[DOF + 2] + kp03 * vy[DOF + 3]);
Ry[DOF + 1] -= (-kp01 * vy[DOF] + kp12 * vy[DOF + 2] + kp13 * vy[DOF + 3]);
Ry[DOF + 2] -= (-kp02 * vy[DOF] - kp12 * vy[DOF + 1] + kp23 * vy[DOF + 3]);
Ry[DOF + 3] -= (-kp03 * vy[DOF] - kp13 * vy[DOF + 1] - kp23 * vy[DOF + 2]);

Rz[DOF] -= (-kp01 * vz[DOF + 1] + kp02 * vz[DOF + 2] - kp03 * vz[DOF + 3]);
Rz[DOF + 1] -= (kp01 * vz[DOF] - kp12 * vz[DOF + 2] + kp13 * vz[DOF + 3]);
Rz[DOF + 2] -= (-kp02 * vz[DOF] + kp12 * vz[DOF + 1] - kp23 * vz[DOF + 3]);
Rz[DOF + 3] -= (kp03 * vz[DOF] - kp13 * vz[DOF + 1] + kp23 * vz[DOF + 2]);
*/

/*--------------------------------------------------------------------
//Coriolis acceleration damping from fluid flow
Semi-antisymmetric contribution by NwT * Nw' - Nw'T * Nw shape functions

    Cv  = ( - m_f * v_i + chi * rho_f * A_o * v_o ) *
                      [[ 0.0,   cv01,  cv02, cv03],
                       [-cv01,  0.0,   cv12, cv13],
                       [-cv02, -cv12,  0.0,  cv23],
                       [-cv03, -cv13, -cv23,  0.0]];
--------------------------------------------------------------------*/
/*
const scalar c = beta * beta * (m_f * ve_i - chi * rho_f * Ao * ve_o);
const scalar cv01 = c * (alpha * alpha * 1.0 / 6 + alpha * 11.0 / 30 + 1.0 / 5) * ds;
const scalar cv02 = c * (alpha * alpha + 2.0 * alpha + 1.0);
const scalar cv03 = -cv01;
const scalar cv12 = cv01;
const scalar cv13 = -c * (alpha / 30 + 1.0 / 30) * ds * ds;
const scalar cv23 = cv01;
Ry[DOF] -= (cv01 * vy[DOF + 1] + cv02 * vy[DOF + 2] + cv03 * vy[DOF + 3]);
Ry[DOF + 1] -= (-cv01 * vy[DOF] + cv12 * vy[DOF + 2] + cv13 * vy[DOF + 3]);
Ry[DOF + 2] -= (-cv02 * vy[DOF] - cv12 * vy[DOF + 1] + cv23 * vy[DOF + 3]);
Ry[DOF + 3] -= (-cv03 * vy[DOF] - cv13 * vy[DOF + 1] - cv23 * vy[DOF + 2]);

Rz[DOF] -= (-cv01 * vz[DOF + 1] + cv02 * vz[DOF + 2] - cv03 * vz[DOF + 3]);
Rz[DOF + 1] -= (cv01 * vz[DOF] - cv12 * vz[DOF + 2] + cv13 * vz[DOF + 3]);
Rz[DOF + 2] -= (-cv02 * vz[DOF] + cv12 * vz[DOF + 1] - cv23 * vz[DOF + 3]);
Rz[DOF + 3] -= (cv03 * vz[DOF] - cv13 * vz[DOF + 1] + cv23 * vz[DOF + 2]);
}
*/

} // namespace curvlin