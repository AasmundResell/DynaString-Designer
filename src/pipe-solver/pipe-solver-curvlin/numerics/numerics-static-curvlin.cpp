#include "pipe-solver/bcs.hpp"
#include "pipe-solver/pipe-solver-curvlin/beam-curvlin.hpp"
#include "pipe-solver/pipe-solver-curvlin/fluid-curvlin.hpp"
#include "pipe-solver/pipe-solver-curvlin/numerics/constant-data.hpp"
#include "pipe-solver/pipe-solver-curvlin/numerics/contact-curvlin.hpp"

namespace curvlin {

inline scalar calc_friction_factor_haaland(scalar Re, scalar roughness_ratio);
inline scalar calc_friction_factor_pipe(const scalar Re, const scalar roughness_ratio);
inline scalar calc_friction_factor_annulus(scalar Re, scalar roughness_ratio, scalar diameter_ratio);
inline scalar calc_dp_friction_pipe(const scalar rho, const scalar mu, const scalar velocity, const scalar r_i);
inline scalar calc_dp_friction_annulus(const scalar rho, const scalar mu, const scalar velocity, const scalar r_bh,
                                       const scalar r_o);
inline scalar calc_friction_factor_haaland(scalar Re, scalar roughness_ratio) {
    // Haaland equation - explicit approximation to Colebrook-White
    // Accurate within 1.5% of Colebrook-White
    return pow(-1.8 * log10(pow(roughness_ratio / 3.7, 1.11) + 6.9 / Re), -2);
}

void calc_fluid_newtonian(const ConfigStatic &conf_stat, FluidData &fluid, const Hole &hole, const Pipe &pipe,
                          const BeamData &beam, const uint top_node, ArenaBump &arena_h) {
    byte *buf = arena_h.buf;

    const scalar rho_fi = conf_stat.rho_fi;
    const scalar rho_fo = conf_stat.rho_fo;
    const uint N = pipe.N;
    const uint Nf = fluid.Nf;

    assert(Nf == N + 1); // Fluid nodes are at pipe nodes + 1 extra at bottom

    ArrayView<scalar> p_o = fluid.get_field<FluidField::p_o>(buf);
    ArrayView<scalar> p_i = fluid.get_field<FluidField::p_i>(buf);
    ArrayView<scalar> v_i = fluid.get_field<FluidField::v_i>(buf);
    ArrayView<scalar> v_o = fluid.get_field<FluidField::v_o>(buf);
    ArrayView<scalar> dp_i = fluid.get_field<FluidField::dp_i>(buf);
    ArrayView<scalar> dp_o = fluid.get_field<FluidField::dp_o>(buf);

    ArrayView<uint> hole_indices = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);
    const ArrayView<Vec3> x = hole.get_field<HoleField::x>(arena_h.buf);
    ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);

    // Set pressures at top element start
    p_i[top_node] = 0.0;
    p_o[top_node] = 0.0; // assume open air at top

    // Simple steady-flow integration: hydrostatic + frictional losses (Darcy-like)
    const scalar mu_f = conf_stat.mu_f;
    const scalar Q_f = conf_stat.Q_f;
    ArrayView<scalar> arr_ri = pipe.get_field<PipeField::ri>(buf);
    ArrayView<scalar> arr_ro = pipe.get_field<PipeField::ro>(buf);

#pragma omp parallel for
    for (uint i = top_node; i < N - 1; ++i) {
        // element start / end arc-lengths
        scalar s_i = pipe.calc_s(i, u[i].x(), buf);
        scalar s_ip = pipe.calc_s(i + 1, u[i + 1].x(), buf);

        // geometry positions at element nodes
        Vec3 x_start = hole.lerp_hole_property_from_pipe_node_position(x, s_i, hole_indices[i], buf);
        Vec3 x_end = hole.lerp_hole_property_from_pipe_node_position(x, s_ip, hole_indices[i + 1], buf);

        // element length and mid geometry
        scalar dS = pipe.dS_e(i, buf);

        // pipe/annulus geometry for friction
        scalar r_i = arr_ri[i];
        scalar A_f_e = pipe.Af_e(i, buf);
        scalar A_p_e = pipe.Ap_e(i, buf);
        scalar r_bh = hole.calc_pipe_element_hole_radius(s_i, s_ip, hole_indices[i], hole_indices[i + 1], buf);
        scalar A_annulus = max(SMALL_SCALAR, (scalar)(M_PI * r_bh * r_bh - (A_p_e + A_f_e)));

        // steady velocities (constant Q_f)
        v_i[i + 1] = (A_f_e > SMALL_SCALAR) ? Q_f / A_f_e : 0.0;
        v_o[i + 1] = (A_annulus > SMALL_SCALAR) ? Q_f / A_annulus : 0.0;

        // frictional pressure drop per element (using your helpers returning dp/ds)
        scalar dp_fric_i = 0.0;
        scalar dp_fric_o = 0.0;
        if (A_f_e > SMALL_SCALAR) {
            dp_fric_i = calc_dp_friction_pipe(rho_fi, mu_f, v_i[i + 1], r_i) * dS;
        }
        if (A_annulus > SMALL_SCALAR) {
            dp_fric_o = calc_dp_friction_annulus(rho_fo, mu_f, v_o[i + 1], r_bh, /*r_o*/ arr_ro[i]) * dS;
        }

        // hydrostatic increment between start and end node (z coordinates)
        scalar dz = x_end.z() - x_start.z();
        scalar dp_hydro_i = -rho_fi * STANDARD_GRAVITY * dz;
        scalar dp_hydro_o = -rho_fo * STANDARD_GRAVITY * dz;

        // integrate: p_next = p_curr + hydrostatic_increment - friction_loss
        p_i[i + 1] = p_i[i] + dp_hydro_i - dp_fric_i;
        p_o[i + 1] = p_o[i] + dp_hydro_o - dp_fric_o;
    }

    // ----Calculate pressure and velocity at bottom node----
    scalar s_bottom = pipe.calc_s(N - 1, u[N - 1].x(), buf);
    int ih = hole_indices[N - 1];
    scalar r_bh = hole.calc_pipe_element_hole_radius(s_bottom, s_bottom, ih, ih, buf);

    Vec3 x_bottom = hole.lerp_hole_property_from_pipe_node_position(x, s_bottom, ih, buf);
    const uint prev_pipe_idx = (N >= 2) ? (N - 2) : 0; // safe fallback
    scalar s_prev = pipe.calc_s(prev_pipe_idx, u[prev_pipe_idx].x(), buf);
    Vec3 x_prev = hole.lerp_hole_property_from_pipe_node_position(x, s_prev, hole_indices[prev_pipe_idx], buf);
    scalar dz_bot_prev = x_bottom.z() - x_prev.z();

    // NOTE! Should also add the frictional losses in the last segment!
    //  hydrostatic increment from previous fluid node to bottom
    p_i[Nf - 1] = p_i[Nf - 2] + (-rho_fi * STANDARD_GRAVITY * dz_bot_prev);
    p_o[Nf - 1] = p_o[Nf - 2] + (-rho_fo * STANDARD_GRAVITY * dz_bot_prev);

    scalar dp_nozzle;
    if (conf_stat.string_type == StringType::DRILL_STRING) {
        // nozzle area: use inner flow area at last element (or dedicated nozzle area if available)
        const scalar A_nozzle_tot = 2.49 * 10e-4; //(m^2) From HJ kompendium
        const scalar Cd = 0.95;                   // discharge coefficient (tune as appropriate)
        dp_nozzle = (rho_fi * Q_f * Q_f) / (2.0 * Cd * Cd * A_nozzle_tot * A_nozzle_tot);

        // annulus area: hole area - bit-rock radius
        scalar A_annulus = max(SMALL_SCALAR, (scalar)(M_PI * r_bh * r_bh - conf_stat.r_br * conf_stat.r_br));

        // steady velocities (constant Q_f)
        v_i[Nf - 1] = (A_nozzle_tot > SMALL_SCALAR) ? Q_f / A_nozzle_tot : 0.0;
        v_o[Nf - 1] = (A_annulus > SMALL_SCALAR) ? Q_f / A_annulus : 0.0;

    } else {
        dp_nozzle = 0.0;
        // annulus area: hole area - bottom pipe radius
        scalar A_annulus = max(SMALL_SCALAR, (scalar)(M_PI * r_bh * r_bh - arr_ro[N - 2] * arr_ro[N - 2]));
        v_i[Nf - 1] = v_i[Nf - 2];
        v_o[Nf - 1] = (A_annulus > SMALL_SCALAR) ? Q_f / A_annulus : 0.0;
    }
    v_i[top_node] = v_i[top_node + 1]; // Switch to pump flow area later!
    v_o[top_node] = v_o[top_node + 1];

    // ----Calculate pump pressure and add to the inner pressure field----
    const scalar p_pump = dp_nozzle + p_o[Nf - 1] - p_i[Nf - 1];
#pragma omp parallel for
    for (uint k = top_node; k < Nf; ++k) {
        p_i[k] += p_pump;
    }

    // ----Calculate pressure gradients----
    scalar dS = pipe.dS_e(top_node, buf);
    dp_i[top_node + 1] = ((p_i[top_node + 1] + p_i[top_node + 2]) / 2.0 - p_i[top_node]) / dS;
    dp_o[top_node + 1] = ((p_o[top_node + 1] + p_o[top_node + 2]) / 2.0 - p_o[top_node]) / dS;

#pragma omp parallel for
    for (uint ie = top_node + 1; ie < N - 1; ++ie) {
        dS = pipe.dS_e(ie, buf);
        uint ie_f = ie + 1;
        // The pressure field is defined at element centers.
        // Central difference to compute the gradient over element:
        // dp/dS ≈ (p_i[i+1] - p_i[i-1]) / (2*dS)
        dp_i[ie_f] = (p_i[ie_f + 1] - p_i[ie_f - 1]) / (2.0 * dS);
        dp_o[ie_f] = (p_o[ie_f + 1] - p_o[ie_f - 1]) / (2.0 * dS);
    }

    // Average of the last node between the two last elements
    dp_o[Nf - 2] = (p_o[Nf - 1] - (p_o[Nf - 2] + p_o[Nf - 3]) / 2.0) / dS;
    dp_i[Nf - 2] = (p_i[Nf - 1] - (p_i[Nf - 2] + p_i[Nf - 3]) / 2.0) / dS;

    // Boundary value take the same value as the first computed gradient
    dp_i[top_node] = dp_i[top_node + 1];
    dp_o[top_node] = dp_o[top_node + 1];
    dp_i[Nf - 1] = dp_i[Nf - 2];
    dp_o[Nf - 1] = dp_o[Nf - 2];
}

void calc_static_loads(ConfigStatic &conf_stat, BeamData &beam, const FluidData &fluid, const Hole &hole,
                       const Pipe &pipe, const uint top_node, ArenaBump &arena_h) {

    byte *buf = arena_h.buf;
    const uint Ne = pipe.get_Ne();
    const uint N = pipe.N;
    const scalar rho_p = conf_stat.rho_p;
    const scalar rho_fi = conf_stat.rho_fi; // Inner fluid density
    const scalar rho_fo = conf_stat.rho_fo; // Outer fluid density
    const scalar Q_f = conf_stat.Q_f;       // Total fluid flow rate

    const bool gravity_enabled = conf_stat.gravity_enabled;

    ArrayView<Vec3> f_hyd = beam.get_field<BeamField::f_hyd>(buf);
    ArrayView<Vec3> m_hyd = beam.get_field<BeamField::m_hyd>(buf);

    // Set all static force vectors to zero
    beam_fill(N, top_node, N, f_hyd, BeamFieldArrayDOF::x, 0.0);
    beam_fill(N, top_node, N, f_hyd, BeamFieldArrayDOF::y, 0.0);
    beam_fill(N, top_node, N, f_hyd, BeamFieldArrayDOF::z, 0.0);
    beam_fill(N, top_node, N, m_hyd, BeamFieldArrayDOF::x, 0.0);
    beam_fill(N, top_node, N, m_hyd, BeamFieldArrayDOF::y, 0.0);
    beam_fill(N, top_node, N, m_hyd, BeamFieldArrayDOF::z, 0.0);

    const ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);
    scalar s_i = pipe.calc_s(top_node, u[top_node].x(), buf);

    /*--------------------------------------------------------------------
    Loop over all elements. Calculate either consistent or lumped loads
    and add them to the load vectors appropriately.
    --------------------------------------------------------------------*/
    const ArrayView<uint> hole_indices = beam.get_field<BeamField::i_pipe_to_ie_hole>(arena_h.buf);
    const ArrayView<Vec3> g_local = hole.get_field<HoleField::g>(buf);

    if (top_node == Ne) { // Return when
        return;
    }

#pragma omp parallel for
    for (uint ie = top_node; ie < Ne; ie++) {
        const scalar ux_p = u[ie + 1].x();
        scalar s_ip = pipe.calc_s(ie + 1, ux_p, buf);
        assert(s_ip >= s_i);
        assert(abs(s_ip - SMALL_SCALAR) < hole.get_L(buf));
        uint i_hole = hole_indices[ie];
        uint ip_hole = hole_indices[ie + 1];

        scalar dS_e = pipe.dS_e(ie, buf);

        if (gravity_enabled) {

            // Effective weight: gravity - buoyancy
            scalar f_eff = rho_p * pipe.Ap_e(ie, buf) * dS_e;

            // Add bouancy and inner fluid weight
            scalar Ao_e = pipe.Ap_e(ie, buf) + pipe.Af_e(ie, buf);
            scalar Af_e = pipe.Af_e(ie, buf);
            f_eff += (rho_fi * Af_e - rho_fo * Ao_e) * pipe.dS_e(ie, buf);

            /*Interpolate between gravity vectors defined in two hole nodes*/
            const Vec3 g_loc_i = hole.lerp_hole_property_from_pipe_node_position<Vec3>(g_local, s_i, i_hole, buf);
            const Vec3 g_loc_ip = hole.lerp_hole_property_from_pipe_node_position<Vec3>(g_local, s_ip, ip_hole, buf);
            Vec3 g_loc_ie = (g_loc_i + g_loc_ip) / 2;
            g_loc_ie = g_loc_ie.normalized() * STANDARD_GRAVITY;

            f_hyd[ie].x() += f_eff / 2.0 * g_loc_ie.x();
            f_hyd[ie + 1].x() += f_eff / 2.0 * g_loc_ie.x();

            f_hyd[ie].y() += f_eff / 2.0 * g_loc_ie.y();
            f_hyd[ie + 1].y() += f_eff / 2.0 * g_loc_ie.y();
            m_hyd[ie].z() += f_eff * dS_e / 12.0 * g_loc_ie.y();
            m_hyd[ie + 1].z() += -f_eff * dS_e / 12.0 * g_loc_ie.y();

            f_hyd[ie].z() += f_eff / 2 * g_loc_ie.z();
            f_hyd[ie + 1].z() += f_eff / 2 * g_loc_ie.z();
            m_hyd[ie].y() += -f_eff * dS_e / 12 * g_loc_ie.z();
            m_hyd[ie + 1].y() += f_eff * dS_e / 12 * g_loc_ie.z();
        }

        s_i = s_ip;
    }

    if (conf_stat.fluid_dynamics_enabled == true) {
        // ------------Area forces from pressure------------------
        // Explanation: The buoyancy forces added here, are corrected in the FEM integration when
        // fluid dynamics calculations are enabled the pressure differential over a single element
        // (as if the element is "cut"). However, at each end face exposed to fluid, we need to
        // add the actual pressure force which has been "removed" from the buoyancy contribution.

        ArrayView<scalar> p_i = fluid.get_field<FluidField::p_i>(buf);
        ArrayView<scalar> p_o = fluid.get_field<FluidField::p_o>(buf);
        ArrayView<scalar> v_i = fluid.get_field<FluidField::v_i>(buf);
        // ArrayView<scalar> v_o = fluid.get_field<FluidField::v_o>(buf);

#pragma omp parallel for
        for (uint k = top_node + 1; k < N - 1; ++k) {
            // previous element (k-1) and next element (k)
            scalar Ao_prev = pipe.Ap_e(k - 1, buf) + pipe.Af_e(k - 1, buf);
            scalar Af_prev = pipe.Af_e(k - 1, buf);
            scalar Ao_next = pipe.Ap_e(k, buf) + pipe.Af_e(k, buf);
            scalar Af_next = pipe.Af_e(k, buf);
            // area differences (signed) give correct force direction
            scalar dAo = Ao_next - Ao_prev;
            scalar dAf = Af_next - Af_prev;
            // Nodal fluid pressures as average of surrounding elemental fluid pressures
            scalar p_i_nodal = (p_i[k + 1] - p_i[k]) / 2.0;
            scalar p_o_nodal = (p_o[k + 1] - p_o[k]) / 2.0;
            // axial nodal thrust fromp_i_top area change: outer pressure on dAo minus inner pressure on dAf
            scalar f_p_node = p_o_nodal * dAo + p_i_nodal * dAf;
            // accumulate to static axial force for node k
            f_hyd[k].x() += f_p_node;
        }

        const uint Nf = fluid.Nf;

        scalar p_i_bot = p_i[Nf - 1]; // Inner nozzle pressure
        scalar p_o_bot = p_o[Nf - 1]; // Outer bottom pressure

        scalar p_i_top = p_i[top_node];
        scalar Af_top = pipe.Af_e(top_node, buf);
        assert(p_o[top_node] == 0.0); // Should be zero if open to atmosphere

        // Full inner flow area and outer projected area
        scalar Af_bottom = pipe.Af_e(N - 2, buf);
        scalar Ao_bottom = pipe.Ap_e(N - 2, buf) + pipe.Af_e(N - 2, buf);

        // Bottom pressure force: outer pressure on full outer area minus inner pressure on full inner area + momentum
        // flux
        scalar f_p_bottom = p_o_bot * Ao_bottom - p_i_bot * Af_bottom + rho_fi * Q_f * (v_i[Nf - 1] - v_i[Nf - 2]);
        scalar f_p_top = p_i_top * Af_top;

        // Apply (current convention subtracts WOB, so subtract F_end to add downward if F_end < 0)
        f_hyd[N - 1].x() -= f_p_bottom;
        f_hyd[top_node + 1].x() += f_p_top; // Apply to top node + 1 since top_node is overwritten by BC
    }
}

void calc_softstring_axial_tension(const Pipe &pipe, BeamData &beam, const ConfigStatic &conf_stat, byte *buf) {
    ArrayView<Vec3> f_hyd = beam.get_field<BeamField::f_hyd>(buf);
    ArrayView<scalar> f_axial = beam.get_field<BeamField::f_axial>(buf);

    const uint N = pipe.N;

    // You may optionally include WOB at the bit:
    scalar cumulative = 0.0; // or conf_stat.WOB if you want to add bit force

    for (int i = int(N) - 1; i >= 0; --i) {
        cumulative += f_hyd[i].x(); // axial gravity contribution at node i
        f_axial[i] = cumulative;    // the soft-string axial tension profile
    }
}

void update_fluid_mass(ConfigStatic &conf_stat, const uint top_node, const Pipe &pipe, const Hole &hole, BeamData &beam,
                       ArenaBump &arena_h) {

    byte *buf = arena_h.buf;
    scalar rho_fi = conf_stat.rho_fi;
    scalar rho_fo = conf_stat.rho_fo;
    assert(conf_stat.fluid_dynamics_enabled);

    const uint N = pipe.N;
    const uint Ne = N - 1;

    ArrayView<scalar> mfy = beam.get_field<BeamField::mfy>(buf);
    ArrayView<scalar> mfz = beam.get_field<BeamField::mfz>(buf);
    ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);
    ArrayView<scalar> ro = pipe.get_field<PipeField::ro>(buf);
    ArrayView<uint> i_pipe_to_ie_hole = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);

#pragma omp parallel for // Set all fluid mass vectors to zero
    for (uint i = 0; i < N; i++) {
        mfy[i] = 0.0;
        mfz[i] = 0.0;
    }

#pragma omp parallel for
    for (uint ie = top_node; ie < Ne; ie++) {
        scalar dS = pipe.dS_e(ie, buf);
        scalar A_p = pipe.Ap_e(ie, buf);
        scalar A_f = pipe.Af_e(ie, buf);

        // Inner fluid mass is added for all fsi models
        scalar m_f = rho_fi * A_f * dS;

        if (conf_stat.fsi_force_model == FSI_ForceModel::PAIDOUSSIS) {
            /*--------------------------------------------------------------------
            Fluid added mass for paidoussis force model
            --------------------------------------------------------------------*/
            scalar chi;
            scalar r_bh;
            const scalar s_i = pipe.calc_s(ie, u[ie].x(), buf);
            const scalar s_ip = pipe.calc_s(ie + 1, u[ie + 1].x(), buf);
            const scalar r_o_ie = ro[ie];

            int i_hole = i_pipe_to_ie_hole[ie];
            int ip_hole = i_pipe_to_ie_hole[ie + 1];

            r_bh = hole.calc_pipe_element_hole_radius(s_i, s_ip, i_hole, ip_hole, buf);

            // Limiting chi value selected for gamma=0.99)
            chi = min(100.0, ((r_bh / r_o_ie) * 2 * 2 + 1) / ((r_bh / r_o_ie) * 2 * 2 - 1));

            m_f += chi * rho_fo * (A_p + A_f) * dS;
        } else if (conf_stat.fsi_force_model == FSI_ForceModel::JANSEN) {
            /*--------------------------------------------------------------------
            Fluid added mass for fritz and jansen force model
            --------------------------------------------------------------------*/
            constexpr scalar Ca = 1.7; // YIGIT & CHRISTOFOROU, 1999
            m_f += Ca * rho_fo * (A_p + A_f) * dS;
        }

        mfy[ie] += m_f / 2;
        mfy[ie + 1] += m_f / 2;
        mfz[ie] += m_f / 2;
        mfz[ie + 1] += m_f / 2;
    }
}

void calc_fluid_field_bernoulli(ConfigStatic &conf_stat, const uint top_node, FluidData &fluid, const BeamData &beam,
                                const Pipe &pipe, const Hole &hole, ArenaBump &arena_h) {

    return; // TODO: Implement function

    byte *buf = arena_h.buf;
    /*--------------------------------------------------------------------
    Calculate fluid field variables based on Bernoulli's equation
    For now, fluid field variables are defined per pipe element (in the middle of the element)
    --------------------------------------------------------------------*/
    const scalar rho_fi = conf_stat.rho_fi;
    const scalar rho_fo = conf_stat.rho_fo;

    const scalar mu_f = conf_stat.mu_f;
    const scalar Q_f = conf_stat.Q_f;
    const uint Nf = fluid.Nf;
    const uint N = pipe.N;

    ArrayView<Quaternion> q_hole = hole.get_field<HoleField::q>(buf);
    ArrayView<scalar> s = hole.get_field<HoleField::s>(buf);

    ArrayView<uint> hole_indices = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);
    ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);

    ArrayView<scalar> arr_ri = pipe.get_field<PipeField::ri>(buf);
    ArrayView<scalar> arr_ro = pipe.get_field<PipeField::ro>(buf);

    ArrayView<scalar> p_i = fluid.get_field<FluidField::p_i>(buf);
    ArrayView<scalar> p_o = fluid.get_field<FluidField::p_o>(buf);
    ArrayView<scalar> v_i = fluid.get_field<FluidField::v_i>(buf);
    ArrayView<scalar> v_o = fluid.get_field<FluidField::v_o>(buf);

    fluid_fill(Nf, top_node, p_i, 0.0);
    fluid_fill(Nf, top_node, p_o, 0.0);
    fluid_fill(Nf, top_node, v_i, 0.0);
    fluid_fill(Nf, top_node, v_o, 0.0);

    const Vec3 g_glob{0, 0, -STANDARD_GRAVITY};

    scalar p_di = 0.0; // Dynamic pressure at the inner pipe
    scalar p_do = 0.0; // Dynamic pressure at the outer pipe
    scalar p_h = 0.0;  // Hydrostatic pressure

#pragma omp parallel for
    for (uint i = top_node + 1; i < Nf - 2; i++) {
        const scalar s_i = pipe.calc_s(i - 1, u[i - 1].x(), buf);
        const scalar s_ip = pipe.calc_s(i, u[i].x(), buf);
        const scalar s_ie = (s_i + s_ip) / 2.0;

        int ie_h = hole_indices[i];
        int ie_hp = hole_indices[i + 1];

        // Geometry parameters
        scalar r_bh = hole.calc_pipe_element_hole_radius(s_i, s_ip, ie_h, ie_hp, buf);
        scalar r_i = arr_ri[i];
        scalar r_o = arr_ro[i];
        scalar dS = pipe.dS_e(i, buf);
        scalar A_f = pipe.Af_e(i, buf);
        scalar A_a = M_PI * r_bh * r_bh - (pipe.Ap_e(i, buf) + A_f);

        // Update velocities
        v_i[i] = Q_f / A_f;
        v_o[i] = Q_f / A_a;

        // Interpolation of rotations using lerp
        scalar t_interp = (s_ie - s[ie_h]) / (s[ie_hp] - s[ie_h]);

        Quaternion q_ie = Quaternion::lerp(q_hole[ie_h], q_hole[ie_hp], t_interp);

        // Use a gradient method to calculate dp instead of absolute depth
        // This is to ensure
        const Vec3 t_ie = q_ie.to_matrix().col(0).normalized();
        const scalar g_hole = t_ie.dot(g_glob);

        // Pressure changes (note: going down the pipe)
        scalar dp_friction_i = calc_dp_friction_pipe(rho_fi, mu_f, v_i[i], r_i) * dS;
        scalar dp_hydrostatic = rho_fi * g_hole * dS;

        // Pressure changes (note: going up the annulus)
        scalar dp_friction_o = calc_dp_friction_annulus(rho_fo, mu_f, v_o[i], r_bh, r_o) * dS;

        p_h += dp_hydrostatic;
        p_do += dp_friction_o;
        p_di += -dp_friction_i;

        p_o[i] = p_do; // + p_h;
        p_i[i] = p_di; // + p_h;
    }

    // Nozzle pressure
    scalar p_nozzle = p_di;
    p_i[Nf - 1] = p_nozzle;
    p_o[Nf - 1] = p_nozzle;

    // Pump pressure
    scalar p_pump = p_do - p_di;

// 5. Adjust pressures to have p_inlet = p_pump
#pragma omp parallel for
    for (uint i = 0; i < Nf; i++) {
        p_i[i] += p_pump;
    }
}

void calc_fluid_field_compressible(ConfigStatic &conf_stat, const uint top_node, FluidData &fluid, const BeamData &beam,
                                   const Pipe &pipe, const Hole &hole, ArenaBump &arena_h) {
    byte *buf = arena_h.buf;

    // Fluid properties
    const scalar rho_fi = conf_stat.rho_fi;         // Reference density inner
    const scalar rho_fo = conf_stat.rho_fo;         // Reference density outer
    const scalar mu_f = conf_stat.mu_f;             // Viscosity
    const scalar Q_f = conf_stat.Q_f;               // Flow rate
    const scalar c_f = 1500.0;                      // Speed of sound in fluid (m/s)
    const scalar bulk_modulus = rho_fi * c_f * c_f; // Fluid bulk modulus

    const uint Nf = fluid.Nf;
    const uint N = pipe.N;

    // Get required field arrays
    ArrayView<Quaternion> q_hole = hole.get_field<HoleField::q>(buf);
    ArrayView<scalar> s = hole.get_field<HoleField::s>(buf);
    ArrayView<uint> hole_indices = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);
    ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);
    ArrayView<scalar> arr_ri = pipe.get_field<PipeField::ri>(buf);
    ArrayView<scalar> arr_ro = pipe.get_field<PipeField::ro>(buf);

    // Fluid field variables
    ArrayView<scalar> p_i = fluid.get_field<FluidField::p_i>(buf);
    ArrayView<scalar> p_o = fluid.get_field<FluidField::p_o>(buf);
    ArrayView<scalar> v_i = fluid.get_field<FluidField::v_i>(buf);
    ArrayView<scalar> v_o = fluid.get_field<FluidField::v_o>(buf);

    // Initialize fields
    fluid_fill(Nf, top_node, p_i, 0.0);
    fluid_fill(Nf, top_node, p_o, 0.0);
    fluid_fill(Nf, top_node, v_i, 0.0);
    fluid_fill(Nf, top_node, v_o, 0.0);

    const Vec3 g_glob{0, 0, -STANDARD_GRAVITY};

    // Calculate nozzle pressure drop coefficient
    // Typical discharge coefficient for drilling nozzles
    const scalar Cd = 0.95;
    const scalar A_nozzle = pipe.Af_e(Nf - 1, buf); // Area of nozzle
    // Pressure drop through nozzle: dp = (rho*Q^2)/(2*Cd^2*A^2)
    const scalar dp_nozzle = (rho_fi * Q_f * Q_f) / (2.0 * Cd * Cd * A_nozzle * A_nozzle);

    // Time stepping parameters
    const scalar CFL = 0.5;     // CFL number for stability
    const scalar dt_max = 1e-3; // Maximum allowable timestep
    const scalar t_final = 1.0; // Time to reach steady state
    scalar t = 0.0;

    // Main time stepping loop
    while (t < t_final) {
        scalar dt = dt_max;

// Calculate local timestep based on CFL condition
#pragma omp parallel for reduction(min : dt)
        for (uint i = top_node + 1; i < Nf - 2; i++) {
            scalar dS = pipe.dS_e(i, buf);

            // Local wave speed
            scalar a_i = sqrt(bulk_modulus / rho_fi);
            scalar a_o = sqrt(bulk_modulus / rho_fo);

            // Local timestep based on CFL condition
            scalar dt_local = CFL * dS / max(a_i + abs(v_i[i]), a_o + abs(v_o[i]));
            dt = min(dt, dt_local);
        }

// Update flow variables
#pragma omp parallel for
        for (uint i = top_node + 1; i < Nf - 2; i++) {
            const scalar s_i = pipe.calc_s(i - 1, u[i - 1].x(), buf);
            const scalar s_ip = pipe.calc_s(i, u[i].x(), buf);
            const scalar s_ie = (s_i + s_ip) / 2.0;

            int ie_h = hole_indices[i];
            int ie_hp = hole_indices[i + 1];

            // Geometry
            scalar r_bh = hole.calc_pipe_element_hole_radius(s_i, s_ip, ie_h, ie_hp, buf);
            scalar r_i = arr_ri[i];
            scalar r_o = arr_ro[i];
            scalar dS = pipe.dS_e(i, buf);

            // Calculate gravity component
            scalar t_interp = (s_ie - s[ie_h]) / (s[ie_hp] - s[ie_h]);
            Quaternion q_ie = Quaternion::lerp(q_hole[ie_h], q_hole[ie_hp], t_interp);
            const Vec3 t_ie = q_ie.to_matrix().col(0).normalized();
            const scalar g_hole = t_ie.dot(g_glob);

            // Friction terms
            scalar f_i = calc_dp_friction_pipe(rho_fi, mu_f, v_i[i], r_i);
            scalar f_o = calc_dp_friction_annulus(rho_fo, mu_f, v_o[i], r_bh, r_o);

            // Update density (mass conservation)
            // if (i > top_node + 1 && i < Nf - 3) {
            //    rho_f -= dt/dS * (rho_f*v_i[i] - rho_f*v_i[i-1]);
            //}

            // Update velocity (momentum conservation)
            if (i > top_node + 1 && i < Nf - 3) {
                v_i[i] -= dt / dS *
                          (v_i[i] * v_i[i] / 2.0 - v_i[i - 1] * v_i[i - 1] / 2.0 +
                           (p_i[i + 1] - p_i[i - 1]) / (2.0 * rho_fi) - g_hole - f_i);

                v_o[i] -= dt / dS *
                          (v_o[i] * v_o[i] / 2.0 - v_o[i - 1] * v_o[i - 1] / 2.0 +
                           (p_o[i + 1] - p_o[i - 1]) / (2.0 * rho_fo) - g_hole - f_o);
            }

            // Update pressure (equation of state)
            p_i[i] = bulk_modulus * log(rho_fi / rho_fi);
            p_o[i] = bulk_modulus * log(rho_fo / rho_fo);
        }

        // Handle pressure at bottom (bit)
        // Pressure in annulus at bottom = Pressure in pipe at bottom - nozzle pressure drop
        p_i[Nf - 1] = p_i[Nf - 2];             // Extrapolate pipe pressure to bit
        p_o[Nf - 1] = p_i[Nf - 1] - dp_nozzle; // Account for nozzle pressure drop

        // Calculate required pump pressure based on total system pressure drop
        scalar p_pump = (p_i[Nf - 1] - p_i[top_node]) + // Pressure drop in pipe
                        dp_nozzle +                     // Pressure drop through nozzle
                        (p_o[top_node] - p_o[Nf - 1]);  // Pressure drop in annulus

        // Set inlet conditions
        p_i[top_node] = p_pump; // Set inlet pressure to required pump pressure
        p_o[top_node] = 0.0;    // Reference pressure at top of annulus

        // Set velocities at boundaries
        v_i[top_node] = Q_f / pipe.Af_e(top_node, buf);
        v_o[Nf - 1] =
            -Q_f / (M_PI * pow(hole.calc_pipe_element_hole_radius(pipe.calc_s(Nf - 2, u[Nf - 2].x(), buf),
                                                                  pipe.calc_s(Nf - 1, u[Nf - 1].x(), buf),
                                                                  hole_indices[Nf - 2], hole_indices[Nf - 1], buf),
                               2) -
                    (pipe.Ap_e(Nf - 1, buf) + pipe.Af_e(Nf - 1, buf)));

        t += dt;
    }
}

inline scalar calc_friction_factor_annulus(scalar Re, scalar roughness_ratio, scalar diameter_ratio) {
    // diameter_ratio = D_i/D_o
    if (Re < SMALL_SCALAR) {
        return 0.0;
    } else if (Re < 2300) {
        // Laminar flow - theoretical solution for annulus
        scalar f_lam = 96.0 / Re * (1.0 / (1.0 - diameter_ratio));
        return max(SMALL_SCALAR, f_lam);
    } else if (Re < 4000) {
        // Transition - interpolate between laminar and turbulent
        scalar f_lam = 96.0 / Re * (1.0 / (1.0 - diameter_ratio));
        scalar f_turb = calc_friction_factor_haaland(4000, roughness_ratio);

        scalar x = (Re - 2300) / (4000 - 2300);
        return f_lam + x * (f_turb - f_lam);
    } else {
        // Turbulent - can use same equations as pipe flow
        // but with hydraulic diameter
        return calc_friction_factor_haaland(Re, roughness_ratio);
    }
}

// Calculate pressure gradient
inline scalar calc_dp_friction_pipe(const scalar rho, const scalar mu, const scalar velocity, const scalar r_i) {

    constexpr scalar roughness = 0.05 * 1e-3;
    // Safety: If viscosity is zero or very small, treat as frictionless (no friction loss)
    if (mu < SMALL_SCALAR) {
        return 0.0;
    }
    const scalar Re = abs(velocity) * 2 * r_i * rho / mu;
    const scalar roughness_ratio = roughness / (2 * r_i);
    const scalar f = calc_friction_factor_pipe(Re, roughness_ratio);

    // Darcy-Weisbach equation
    scalar dp_ds = f * (rho * powi(velocity, 2)) / (4 * r_i);
    return dp_ds;
}

inline scalar calc_dp_friction_annulus(const scalar rho, const scalar mu, const scalar velocity, const scalar r_bh,
                                       const scalar r_o) {

    constexpr scalar roughness = 0.05 * 1e-3;
    const scalar d_h = 2 * r_bh - 2 * r_o;
    if (mu < SMALL_SCALAR) {
        return 0.0;
    }
    const scalar Re = abs(velocity) * d_h * rho / mu;
    const scalar roughness_ratio = roughness / d_h;
    const scalar f = calc_friction_factor_annulus(Re, roughness_ratio, r_bh / r_o);

    // Darcy-Weisbach equation
    scalar dp_ds = f * (rho * powi(velocity, 2)) / (4 * r_bh);
    return dp_ds;
}

inline scalar calc_effective_viscosity_pl(const scalar shear_rate, const scalar K, const scalar n) {
    // Power Law model
    // K = consistency index
    // n = flow behavior index (n < 1 for shear thinning)
    return K * pow(shear_rate, n - 1);
}

inline scalar calc_effective_viscosity_hb(const scalar shear_rate,
                                          const scalar tau_y, // Yield stress
                                          const scalar K,     // Consistency index
                                          const scalar n) {   // Flow behavior index
    if (shear_rate < SMALL_SCALAR) {
        // Return a large but finite viscosity for very low shear rates
        return 1000.0 * (tau_y + K * pow(SMALL_SCALAR, n)) / SMALL_SCALAR;
    }

    // Effective viscosity = shear stress / shear rate
    // For HB fluid: tau = tau_y + K * gamma_dot^n
    const scalar shear_stress = tau_y + K * pow(shear_rate, n);
    return shear_stress / shear_rate;
}

inline scalar calc_friction_factor_pipe(const scalar Re, const scalar roughness_ratio) {
    // Roughness ratio = pipe roughness / diameter
    if (Re < SMALL_SCALAR) {
        return 0.0;
    } else if (Re < 2300) {
        // Laminar flow
        return max(SMALL_SCALAR, 64.0 / Re);
    } else if (Re < 4000) {
        // Transitional flow - interpolate between laminar and turbulent
        scalar f_lam = 64.0 / Re;

        // Turbulent friction factor at Re=4000 (using Colebrook)
        scalar f_turb = calc_friction_factor_haaland(4000, roughness_ratio);

        // Linear interpolation
        scalar x = (Re - 2300) / (4000 - 2300);
        return f_lam + x * (f_turb - f_lam);
    } else {
        // Fully turbulent - use Colebrook-White
        return calc_friction_factor_haaland(Re, roughness_ratio);
    }
}

inline scalar calc_shear_rate_annulus(const scalar velocity, const scalar r_bh, const scalar r_o) {
    // Approximate shear rate in annulus
    const scalar gap = r_bh - r_o;
    return abs(velocity) / gap;
}

inline scalar calc_dp_friction_annulus_non_newtonian(const scalar rho, const scalar mu_ref, const scalar velocity,
                                                     const scalar r_bh, const scalar r_o, const scalar K,
                                                     const scalar n) {
    const scalar shear_rate = calc_shear_rate_annulus(velocity, r_bh, r_o);
    const scalar mu_eff = calc_effective_viscosity_pl(shear_rate, K, n);

    const scalar d_h = 2 * r_bh - 2 * r_o;
    // Modified Reynolds number using effective viscosity
    const scalar Re = abs(velocity) * d_h * rho / mu_eff;

    // Rest of the calculation remains the same
    constexpr scalar roughness = 0.05 * 1e-3;
    const scalar roughness_ratio = roughness / d_h;
    const scalar f = calc_friction_factor_annulus(Re, roughness_ratio, r_bh / r_o);

    scalar dp_ds = f * (rho * powi(velocity, 2)) / (4 * r_bh);
    return dp_ds;
}

} // namespace curvlin
