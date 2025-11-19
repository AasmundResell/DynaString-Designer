#include "beam-common.hpp"
#include <iomanip>

void calc_dt_and_inertia_scaling(Config &config, const Pipe &pipe, const byte *buf) {

    ConfigStatic &conf_stat = config.conf_stat;

    // material properties
    const scalar rho_p = config.conf_stat.rho_p;
    const scalar rho_fi = config.conf_stat.rho_fi;
    const scalar E = conf_stat.E;
    const scalar G = conf_stat.get_G();

    scalar dS_min = std::numeric_limits<scalar>::max();
    conf_stat.inertia_scaling = std::numeric_limits<scalar>::max();
    const scalar alpha = conf_stat.alpha;
    const scalar beta = conf_stat.beta;

    // track per-mode maxima
    scalar omega_axial_max = 0.0;
    scalar omega_bend_max = 0.0;
    scalar omega_torsion_max = 0.0;

    for (uint ie = 0; ie < pipe.get_Ne(); ie++) {

        scalar dS = pipe.dS_e(ie, buf);
        assert(config.pipe_solver_type == PipeSolverType::CURVILINEAR ||
               is_close(rho_fi, 0.0)); // fluid not valid for corot yet
        const scalar Ip = pipe.Ip_e(ie, buf);
        const scalar If = pipe.If_e(ie, buf);
        const scalar Ap = pipe.Ap_e(ie, buf);
        const scalar Af = pipe.Af_e(ie, buf);

        const scalar I_tot = 2.0 * (rho_p * Ip + rho_fi * If);
        const scalar M_tot = rho_p * Ap + rho_fi * Af;
        const scalar r_g = powi(I_tot / M_tot, 2); // Radius of gyration
        const scalar J = 2.0 * Ip;

        dS_min = min(dS_min, dS);
        scalar inertia_scaling = 17.5 * (1 + 12 * (r_g / (dS_min * dS_min))) / 420; // cf. BELYTSCHKO AND MINDLE (1980)
        conf_stat.inertia_scaling = min(inertia_scaling, conf_stat.inertia_scaling);

        // -----------------------------
        // Conservative analytical element frequency bounds
        // -----------------------------
        const scalar omega_axial = M_PI * sqrt((E * Ap) / std::max(M_tot, (scalar)1e-12)) / dS;
        const scalar omega_bend = (M_PI * M_PI) * sqrt((E * Ip) / std::max(M_tot, (scalar)1e-12)) / (dS * dS);
        const scalar omega_torsion = M_PI * sqrt((G * J) / std::max(I_tot, (scalar)1e-12)) / dS;

        // element maximum frequency (take conservative maximum)
        omega_axial_max = std::max(omega_axial_max, omega_axial);
        omega_bend_max = std::max(omega_bend_max, omega_bend);
        omega_torsion_max = std::max(omega_torsion_max, omega_torsion);
    }
    assert(dS_min > 0);
    assert(config.conf_stat.CFL < 1 && config.conf_stat.CFL > 0);

    // Critical timestep for each mode WITHOUT damping (alpha=beta=0)
    scalar dt_axial_no_damping =
        (omega_axial_max > SMALL_SCALAR) ? 2.0 / omega_axial_max : std::numeric_limits<scalar>::infinity();
    scalar dt_bend_no_damping =
        (omega_bend_max > SMALL_SCALAR) ? 2.0 / omega_bend_max : std::numeric_limits<scalar>::infinity();
    scalar dt_torsion_no_damping =
        (omega_torsion_max > SMALL_SCALAR) ? 2.0 / omega_torsion_max : std::numeric_limits<scalar>::infinity();

    // Minimal critical timestep without damping
    scalar dt_no_damping = std::min({dt_axial_no_damping, dt_bend_no_damping, dt_torsion_no_damping});

    assert(dS_min > 0); // just a check
    assert(config.conf_stat.CFL < 1 && config.conf_stat.CFL > 0);

    scalar omega_use;
    scalar dt_crit = std::numeric_limits<scalar>::infinity();
    if (conf_stat.riemann_solver) {
        // --- Add Riemann cell CFL check ---
        scalar c_max = std::max({sqrt(E / rho_p), sqrt(G / rho_p)}); // Use max of axial/torsional wave speeds
        scalar dt_riemann = 0.99 * config.dS_min_rc_target / c_max;
        dt_crit = dt_riemann;
        omega_use = std::max({omega_axial_max, omega_bend_max, omega_torsion_max});
    } else if (conf_stat.enable_ax_tor_subcycle) {
        omega_use = omega_bend_max;

        // omega_use = std::max({omega_axial_max, omega_bend_max, omega_torsion_max});
    } else {
        omega_use = std::max({omega_axial_max, omega_bend_max, omega_torsion_max});
    }

    // Belytschko stability adjustment for Rayleigh damping (per-element dt_crit)
    omega_use = std::max(omega_use, (scalar)1e-12); // guard
    const scalar psi = alpha / (2.0 * omega_use) + beta * omega_use / 2.0;
    dt_crit = min(dt_crit, (2.0 / omega_use) * (std::sqrt(1.0 + psi * psi) - psi));

    // accumulate global minimum critical timestep across elements
    conf_stat.dt = conf_stat.CFL * dt_crit;
    // config.conf_stat.dt = config.conf_stat.CFL * conf_stat.dt_min;
    cout << "----------------------- Choosing dt ----------------------\n"
         << "dt from Belytcho stability criteria (with damping): " << dt_crit << "\n"
         << "dt axial (mode-based, NO damping): " << dt_axial_no_damping << "\n"
         << "dt bending (mode-based, NO damping): " << dt_bend_no_damping << "\n"
         << "dt torsion (mode-based, NO damping): " << dt_torsion_no_damping << "\n"
         << "dt is chosen as CFL * dt_min, where CFL = " << config.conf_stat.CFL << "\n"
         << "Final dt (min of all criteria): " << config.conf_stat.dt << "\n";
    cout << "----------------------------------------------------------\n";

    // Enforce "fast" solver policy: disallow alpha/beta that reduce critical dt below 30% of undamped value
    if (dt_no_damping < std::numeric_limits<scalar>::infinity() && conf_stat.dt < 0.3 * conf_stat.CFL * dt_no_damping &&
        !conf_stat.riemann_solver) {
        throw std::runtime_error("Rayleigh damping (alpha/beta) reduces critical timestep below 30% of undamped "
                                 "value - aborting to enforce fast solver settings.");
    }

    // ----------------------------------------------
    // OPTIONAL AXIAL/TORSIONAL SUBCYCLING
    // ----------------------------------------------
    if (config.conf_stat.enable_ax_tor_subcycle) {

        // 1) Compute dt_slow using existing logic:
        // config.conf_stat.dt *= 20; // store temporarily

        // 2) Compute an axial/torsional dt limit
        // Use the axial/torsional NO-DAMPING bounds you already calculate:
        scalar dt_fast_limit = std::min(dt_axial_no_damping, dt_torsion_no_damping);

        // 3) Scale by user CFL_fast
        scalar dt_fast = config.conf_stat.CFL_at * dt_fast_limit;

        // 4) Ensure dt_fast divides dt_slow evenly
        int n = static_cast<int>(std::ceil(config.conf_stat.dt / dt_fast));
        n = std::max(n, 1);

        dt_fast = config.conf_stat.dt / (scalar)n;

        // Store results
        config.conf_stat.n_at = n;
        config.conf_stat.dt_at = dt_fast;

        std::cout << "Subcycling enabled:\n";
        std::cout << "  Slow dt        = " << config.conf_stat.dt << "\n";
        std::cout << "  Fast dt target = " << dt_fast_limit << "\n";
        std::cout << "  Using dt_fast  = " << dt_fast << "\n";
        std::cout << "  n_subcycles    = " << n << "\n";
    }

    if (config.simulation_mode != SimulationMode::INFINITE_LOOP) {
        /*Update number of timesteps and t_max, so that they are consistent*/
        if (config.n_max == 0 && config.t_max > 0) {
            config.t_max = conf_stat.dt * config.n_max;
        } else if ((config.n_max > 0) & (config.t_max == 0)) {
            config.n_max = static_cast<size_t>(config.t_max / conf_stat.dt);
            config.t_max = config.n_max * conf_stat.dt;
        } else if (config.n_max > 0 && config.t_max > 0) {
            const scalar t_max_min = min(config.n_max * conf_stat.dt, config.t_max);
            config.n_max = static_cast<size_t>(t_max_min / conf_stat.dt);
            config.t_max = conf_stat.dt * config.n_max;
        } else {
            assert(false);
        }
    }
}

void calc_contact_stiffness_and_damping(Config &config, const uint N, const ArrayView<scalar> &mass, const byte *buf) {
    // =========================================================
    // 1. Extract minimum nodal mass
    // =========================================================
    scalar m_min = std::numeric_limits<scalar>::max();
    for (uint i = 0; i < N; ++i) {
        m_min = std::min(m_min, mass[i]);
    }

    // =========================================================
    // 2. Pull timestep
    // =========================================================
    const scalar dt = config.conf_stat.dt;

    // =========================================================
    // 3. Critical stiffness for explicit stability
    //    Kcrit ~ 2*m / dt^2    (derived from ω = sqrt(K/m), CFL ~ 2/ω)
    // =========================================================
    const scalar Kcrit = (2.0 * m_min) / (dt * dt);

    // =========================================================
    // 4. NORMAL CHANNEL  (promote lateral dynamics!)
    // =========================================================
    // Keep normal stiffness small fraction of critical. Too stiff → kills lateral vibration.
    // Target: 5–10% of Kcrit is a sweet spot.
    const scalar K_normal = 0.002 * Kcrit;

    // Light damping preserves energy exchange but controls overlap ringing.
    // ζ_n in [0.05, 0.15]
    const scalar zeta_n = 0.10;
    const scalar C_normal = 2.0 * zeta_n * std::sqrt(K_normal * m_min);

    // =========================================================
    // 5. TANGENTIAL STATIC CHANNEL (stick spring)
    // =========================================================
    // Tangential stick is slightly softer than normal contact confinement,
    // so that torsion can build gradually.
    const scalar K_tangent = 10.0 * K_normal;

    // Damping ratio moderate (suppresses chatter in swept torsion),
    // ζ_t in [0.4, 0.7]
    const scalar zeta_t = 0.6;
    const scalar C_tangent = 2.0 * zeta_t * std::sqrt(K_tangent * m_min);

    // =========================================================
    // 6. v0 weighting scale (Yamane–Nakamura Eq. 6–7)
    // =========================================================
    // Tied to dt so that as dt→0 you approach Coulomb friction cleanly.
    const scalar cw = 1000; // 0.2–0.5 typical
    const scalar v0 = cw * dt;

    // =========================================================
    // 7. Store in config
    // =========================================================
    config.conf_stat.K_normal = K_normal;
    config.conf_stat.C_normal = C_normal;
    // config.conf_stat.K_tangent = K_tangent;
    // config.conf_stat.C_tangent = C_tangent;
    // config.conf_stat.critical_stribeck_velocity = v0;

    // =========================================================
    // 8. Diagnostics
    // =========================================================
    std::cout << "=== Auto contact parameters ===\n";
    std::cout << "dt         = " << dt << "\n";
    std::cout << "m_min      = " << m_min << "\n";
    std::cout << "Kcrit      = " << Kcrit << "\n";
    std::cout << "K_normal   = " << K_normal << "  (7% of Kcrit)\n";
    std::cout << "C_normal   = " << C_normal << "  (zeta_n=" << zeta_n << ")\n";
    std::cout << "K_tangent  = " << K_tangent << "  (50% K_normal)\n";
    std::cout << "C_tangent  = " << C_tangent << "  (zeta_t=" << zeta_t << ")\n";
    std::cout << "v0_contact = " << v0 << "  (cw=" << cw << ")\n";
}

void create_mass_vectors(vector<scalar> &m, vector<Vec3> &J_u, Config &config, const Pipe &pipe, byte *buf) {

    config.write_mass = true;
    const uint Ne = pipe.N - 1;
    scalar rho_p = config.conf_stat.rho_p;
    const ArrayView<scalar> ec_vec = pipe.get_field<PipeField::ec>(buf);

    for (uint ie = 0; ie < Ne; ie++) { // SJEKK Ne - 1
        scalar dS = pipe.dS_e(ie, buf);
        scalar A_p = pipe.Ap_e(ie, buf);
        scalar Ip_e = pipe.Ip_e(ie, buf);

        // ec is the radial eccentricity of the element mass center, it is calculated
        // as the eccentricity length span ec_dist (non-dimensional) times a user defined fraction of the outer radius
        scalar ec = config.conf_stat.ec_amplifier * ec_vec[ie];

        /*--------------------------------------------------------------------
        Lumped mass for axial and lateral forces
        --------------------------------------------------------------------*/
        scalar m_ie = rho_p * A_p * dS;
        m[ie] += m_ie / 2;
        m[ie + 1] += m_ie / 2;

        /*--------------------------------------------------------------------
        Torsion inertia
        --------------------------------------------------------------------*/
        scalar Jx_ie = rho_p * (2 * Ip_e) * dS + rho_p * A_p * ec * ec * dS;

        /*--------------------------------------------------------------------
        Lateral (radial) inertia
        see: https://quickfem.com/wp-content/uploads/IFEM.Ch31.pdf (eqn. 31.19)
        --------------------------------------------------------------------*/
        assert(config.conf_stat.inertia_scaling <= 1.0);
        scalar a = config.conf_stat.inertia_scaling;
        scalar Jr_ie = m_ie * a * dS * dS;
        const Vec3 Je = {Jx_ie, Jr_ie, Jr_ie};

        J_u[ie] += 0.5 * Je;
        J_u[ie + 1] += 0.5 * Je;
    }
}

void save_csv_header_file(const uint N, const Config &config, const ConfigDynamic &conf_dyn, const string output_subdir,
                          const BCsData &bcs, byte *buf) {
    using namespace std;
    /*--------------------------------------------------------------------
    Save common info
    --------------------------------------------------------------------*/
    const uint top_node = config.top_node;
    assert(top_node < N);
    const scalar dt = config.conf_stat.dt;
    const scalar t = config.t;
    const uint n_w = config.n_write;
    const uint n_tot = config.n_max;
    const scalar L_feed = bcs.get_bcs_field(buf).u_top_np;
    const scalar L_pipe_tot = config.conf_stat.L;

    const scalar hook_load = conf_dyn.hook_load;
    const scalar top_torque = conf_dyn.top_drive_torque;
    const scalar WOB = conf_dyn.WOB;
    const scalar TOB = conf_dyn.TOB;
    const scalar ROP = conf_dyn.ROP;
    const scalar omega_bit = conf_dyn.omega_bit;
    const scalar v_top = conf_dyn.v_top;
    const scalar omega_top = conf_dyn.omega_top;

    const uint top_component = config.top_component;

    const bool global_frame = config.output_global_frame;
    uint solver_type_int;

    switch (config.pipe_solver_type) {
    case PipeSolverType::CURVILINEAR:
        solver_type_int = 0;
        break;
    case PipeSolverType::COROTATIONAL:
        solver_type_int = 1;
        break;
    default:
        break;
    }

    const string header_file = output_subdir + "/header.csv";
    ofstream ost{header_file};
    if (!ost) {
        throw runtime_error{"Failed to open output file: " + header_file + "\n"};
    }

    ost << "N, t, dt, n_tot, L_feed, L_string_tot, v_top, omega_top, hook_load, top_torque, WOB, TOB, ROP, omega_bit, "
           "top_node, "
           "top_component, "
           "global_frame, solver_type";
    /*Write energy header*/
    if (config.conf_stat.check_energy_balance && config.pipe_solver_type == PipeSolverType::COROTATIONAL) {
        ost << ", KE, W_int, W_ext, E_res\n";
    } else {
        ost << "\n";
    } /*Write energy*/

    ost << N << ", " << t << ", " << dt << ", " << n_tot << ", " << L_feed << ", " << L_pipe_tot << ", " << v_top
        << ", " << omega_top << ", " << hook_load << ", " << top_torque << ", " << WOB << ", " << TOB << ", " << ROP
        << ", " << omega_bit << ", " << top_node << ", " << top_component << ", " << global_frame << ", "
        << solver_type_int;

    if (config.conf_stat.check_energy_balance && config.pipe_solver_type == PipeSolverType::COROTATIONAL) {
        const scalar KE = conf_dyn.KE;
        const scalar W_int = conf_dyn.W_int;
        const scalar W_ext = conf_dyn.W_ext;

        const scalar E_res = KE + W_int - W_ext;
        ost << ", " << KE << ", " << W_int << ", " << W_ext << ", " << E_res << "\n ";
    } else {
        ost << "\n";
    }
}

void save_csv_curvlin_header(const Config &config, ofstream &ost) {
    using namespace ::std;
    constexpr uint w = 11;

    // clang-format off

    ost << setw(w) << "s," << setw(w) << "i_h," << setw(w) 
        << "u_x," << setw(w) << "u_y," << setw(w) << "u_z," << setw(w) 
        << "theta_x," << setw(w) << "theta_y," << setw(w) << "theta_z," << setw(w) 
        << "v_x," << setw(w) << "v_y," << setw(w) << "v_z," << setw(w) 
        << "omega_x," << setw(w) << "omega_y," << setw(w) << "omega_z," << setw(w)
        << "a_x," << setw(w) << "a_y," << setw(w) << "a_z," << setw(w) 
        << "alpha_x," << setw(w) << "alpha_y," << setw(w) << "alpha_z,"  << setw(w) 
        << "fux_int," << setw(w) << "fuy_int," << setw(w) << "fuz_int," << setw(w)
        << "frx_int," << setw(w) << "fry_int," << setw(w) << "frz_int," << setw(w) 
        << "fux_ext," << setw(w) << "fuy_ext," << setw(w) << "fuz_ext," << setw(w) 
        << "frx_ext," << setw(w) << "fry_ext," << setw(w) << "frz_ext," << setw(w) 
        << "fux_stat," << setw(w) << "fuy_stat," << setw(w) << "fuz_stat," << setw(w)
        << "frx_stat," << setw(w) << "fry_stat," << setw(w) << "frz_stat";

    if (config.write_mass) {
        ost << setw(w) << ",m," << setw(w) << "Jx," << setw(w) << "Jr";
    }
    if (config.write_misc_quantities) {
        ost << setw(w) << ",f_ax," << setw(w) << "a_r," << setw(w) << "a_t," << setw(w) << "so," << setw(w)
            << "sigma_vm," << setw(w) << "q_c";
    }
    ost << endl;
}

void save_csv_global_header(const Config &config, ofstream &ost) {
    using namespace ::std;
    /*Write solution*/
    constexpr uint w = 11;
    ost << setw(w) << "X1," << setw(w) << "X2," << setw(w) << "X3," << setw(w) << "d1," << setw(w) << "d2," << setw(w)
        << "d3," << setw(w) << "U11," << setw(w) << "U12," << setw(w) << "U13," << setw(w) << "U21," << setw(w)
        << "U22," << setw(w) << "U23," << setw(w) << "U31," << setw(w) << "U32," << setw(w) << "U33 " << setw(w)
        << "v1," << setw(w) << "v2," << setw(w) << "v3," << setw(w) << "omega_u_1," << setw(w) << "omega_u_2,"
        << setw(w) << "omega_u_3"
        << "\n";
}

void save_csv_global_line(ofstream &ost, const Vec3 &X, const Vec3 &d, const Mat3 &U, const Vec3 &v,
                          const Vec3 &omega_u) {
    using namespace ::std;
    constexpr uint w = 11;
    ost << setw(w) << X.x() << "," << setw(w) << X.y() << "," << setw(w) << X.z() << "," << setw(w) << d.x() << ","
        << setw(w) << d.y() << "," << setw(w) << d.z() << "," << setw(w) << U(0, 0) << "," << setw(w) << U(0, 1) << ","
        << setw(w) << U(0, 2) << "," << setw(w) << U(1, 0) << "," << setw(w) << U(1, 1) << "," << setw(w) << U(1, 2)
        << "," << setw(w) << U(2, 0) << "," << setw(w) << U(2, 1) << "," << setw(w) << U(2, 2) << "," << setw(w)
        << v.x() << "," << setw(w) << v.y() << "," << setw(w) << v.z() << "," << setw(w) << omega_u.x() << ","
        << setw(w) << omega_u.y() << "," << setw(w) << omega_u.z() << "\n";
}

void save_csv_fluid_header(const Config &config, ofstream &ost) {
    using namespace ::std;
    constexpr uint w = 11;
    ost << setw(w) << "s," << setw(w) << "p_i," << setw(w) << "p_o," << setw(w) << "v_i," << setw(w) << "v_o" << setw(w)
        << "dp_i," << setw(w) << "dp_o";
    ost << endl;
}