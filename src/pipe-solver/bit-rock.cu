#include "bit-rock.hpp"
#include "misc/includes.hpp"
#include "misc/utils.hpp"



BitRockData create_bit_rock(Config &config, ArenaBump &arena_h) {
    BitRockData bit_rock{};
    // 1. Allocate BitRock struct
    arena_h.allocate<BitRock>(&bit_rock.bit_rock_field, 1);
    BitRock &br = bit_rock.get_bit_rock_field(arena_h.buf);

    // 2. Compute N_cells at runtime
    const scalar rpm_max     = 200;
    const scalar omega   = rpm_max * 2.0 * M_PI / 60.0;
    const scalar dt      = config.conf_stat.dt;
    const uint   n_blades= config.conf_stat.n_blades;
    const scalar CFL_target = 0.8;
    scalar cells_per_blade  = omega * dt / (2.0 * M_PI) / CFL_target;
    uint N_cells_per_blade  = (uint)ceil(std::max(8.0, cells_per_blade * 360.0));
    uint N_cells = n_blades * N_cells_per_blade; // Increased resolution for better accuracy
    br.N_cells = N_cells;

    // 3. Allocate cut_depths array (separate from BitRock struct)
    arena_h.allocate<scalar>(&bit_rock.cut_depths_offset, N_cells);

    printf("Bit rock cells: %u (≈ %.2f per blade)\n", br.N_cells, (scalar)N_cells_per_blade);

    // 4. Initialize geometry
    br.s_blade = config.conf_stat.L_string_depth_initial;
    ArrayView<scalar> cut_depths = bit_rock.get_cut_depths(arena_h.buf);
    for (uint i = 0; i < br.N_cells; i++) {
        cut_depths[i] = config.L_hole_depth_initial;
    }
    br.Z0 = cut_depths[br.ptr];
    br.s_blade_old = br.s_blade;
    br.theta_old   = br.theta;
    return bit_rock;
}

DEVICE_FUNC inline scalar delta_theta_periodic(scalar theta, scalar theta_old, scalar L) {
    theta = fmod_pos(theta, L);
    assert(theta_old >= 0 && theta_old <= L);
    scalar dtheta;

    const scalar max_dist_travelled = L / 3;

    /*Normal situation*/
    if (abs(theta - theta_old) < max_dist_travelled) {
        dtheta = theta - theta_old;
    } else { /*Assuming boundary is crossed*/
        if (theta > theta_old) {
            dtheta = (theta - L) - theta_old;
        } else if (theta < theta_old) {
            dtheta = (theta + L) - theta_old;
        } else {
            assert(false);
        }
    }
    assert(abs(dtheta) < max_dist_travelled);
    return dtheta;
}

DEVICE_FUNC inline void set_bit_rock_bc(BitRockData &bit_rock_data, const scalar theta_i, const scalar s_blade_i, const uint N_cells,
                            const scalar dx, byte *buf) {
    assert(theta_i >= 0.0 && theta_i <= dx * N_cells);
    const uint ptr = (uint)(theta_i / dx);
    assert(ptr < N_cells);
    ArrayView<scalar> cut_depths = bit_rock_data.get_cut_depths(buf);
    
    
    // Calculate fraction of cell traversed
    scalar cell_fraction = (theta_i - ptr * dx) / dx;
    
    // Interpolate the new cut depth based on traversed fraction
    scalar current_depth = cut_depths[ptr];
    cut_depths[ptr] = max(current_depth, 
                            current_depth + (s_blade_i - current_depth) * cell_fraction);
}

DEVICE_FUNC void update_bit_kinematics(const ConfigStatic &conf_stat, BitRockData &bit_rock_data, byte *buf) {
    /*--------------------------------------------------------------------
    Update kinematics and calculate current depth of cut
    --------------------------------------------------------------------*/
    ArrayView<scalar> cut_depths = bit_rock_data.get_cut_depths(buf);
    BitRock &br = bit_rock_data.get_bit_rock_field(buf);


    const uint n_blades = conf_stat.n_blades;
    const uint N_cells = br.N_cells;
    const scalar L = 2 * M_PI / n_blades;
    const scalar dx = br.cell_size_rad(n_blades);

    
    /*Update pointer to current position*/
    const scalar theta_new_mod = fmod_pos(br.theta, L);
    assert(theta_new_mod >= 0 && theta_new_mod <= L);
    const uint ptr = (uint)(theta_new_mod / dx);
    assert(ptr < N_cells);
    br.ptr = ptr;
    
    /*Calculate instantaneous depth of cut*/
    br.d_cut = br.s_blade - cut_depths[br.ptr];
    
    // Update stored kinematics
    br.theta = theta_new_mod;
    br.s_blade_old = br.s_blade;
}

DEVICE_FUNC void cut_rock_surface(const ConfigStatic &conf_stat, BitRockData &bit_rock_data, byte *buf) {
    /*--------------------------------------------------------------------
    Determine if rock should be cut based on MSE criterion and update surface
    --------------------------------------------------------------------*/
    BitRock &br = bit_rock_data.get_bit_rock_field(buf);

    const uint n_blades = conf_stat.n_blades;
    const uint N_cells = br.N_cells;
    const scalar L = 2 * M_PI / n_blades;
    const scalar dx = br.cell_size_rad(n_blades);
    
    const scalar dtheta = delta_theta_periodic(br.theta, br.theta_old, L);
    const scalar ds_blade = br.s_blade - br.s_blade_old;
    const uint n_iter = abs(dtheta) / dx + 1;
    const scalar omega = br.omega;
    /*Update cut depths only if MSE threshold is exceeded*/
    if (omega > 0.01 ){ // 0.5 rad/s (4.8 rpm) threshold for cutting
        for (uint i = 0; i < n_iter; i++) {
            const scalar theta_i = fmod_pos(br.theta_old + (i + 1) * dtheta / n_iter, L);
            assert(theta_i >= 0 && theta_i <= L);
            const scalar s_blade_i = br.s_blade_old + (i + 1) * ds_blade / n_iter;
            set_bit_rock_bc(bit_rock_data, theta_i, s_blade_i, N_cells, dx, buf);
        }
    }
}

DEVICE_FUNC void cut_rock_surface_PDE(const ConfigStatic &conf_stat, BitRockData &bit_rock, byte *buf)
{
    ArrayView<scalar> cut = bit_rock.get_cut_depths(buf);
    BitRock &br = bit_rock.get_bit_rock_field(buf);

    const uint   N   = br.N_cells;                 // cells per sector
    const uint   Nb  = conf_stat.n_blades;
    const scalar dt  = conf_stat.dt;
    assert(conf_stat.enable_ax_tor_subcycle == false); // this function assumes no subcycling

    // ---- kinematics and grid ----
    const scalar L  = 2.0 * M_PI / (scalar)Nb;     // radians per sector
    const scalar dx = br.cell_size_rad(Nb);        // radians per cell in a sector

    // blade angle advance in this step (radians)
    const scalar dtheta = delta_theta_periodic(br.theta, br.theta_old, L);
    // blade angle at time n+1
    const scalar theta_next = fmod_pos(br.theta_old + dtheta, L);

    // axial advance (source term): ds = V_b * dt, baseline at time n is s_blade_old
    const scalar ds = br.s_blade - br.s_blade_old;

    // off-bottom? freeze surface and return (WOB=0 via your Detournay path)
    const bool on_bottom = (cut[br.ptr] - br.s_blade) <= SMALL_SCALAR;
    if (!on_bottom) { br.d_cut = 0.0; br.s_blade_old = br.s_blade; br.theta_old = br.theta; return; }

    // ---- 1) TRUE upwind update on fixed Θ-grid (PDE discretization) ----
    // Courant number with normalized Θ (ΔΘ = 1/N)
    const scalar uTheta = br.omega * ((scalar)Nb / (2.0 * M_PI)); // [1/s]
    const scalar dTheta = 1.0 / (scalar)N;
    const scalar C      = uTheta * dt / dTheta;                   // require |C| ≤ 1

    // λ^n_i = cut[i] - s_blade_old
    // λ^{n+1}_i = λ^n_i - C(λ^n_i - λ^n_{i-1}) + ds
    for (uint i = 0; i < N; ++i) {
        const uint im1 = (i == 0 ? N-1 : i-1);
        const scalar lam_i   = cut[i]   - br.s_blade_old;
        const scalar lam_im1 = cut[im1] - br.s_blade_old;
        const scalar lam_new = lam_i - C * (lam_i - lam_im1) + ((ds > 0.0) ? ds : 0.0);
        cut[i] = br.s_blade + lam_new;    // write absolute back
    }

    // ---- 2) Enforce boundary λ(t^{n+1}, Θ=0)=0 at the EXACT blade angle (time n+1) ----
    // Use your sub-cell interpolation helper to place s_blade at theta_next
    set_bit_rock_bc(bit_rock, theta_next, br.s_blade, N, dx, buf);

    // Update pointer to the blade’s cell at time n+1 (for callers that rely on ptr)
    const uint ptr_next = (uint)(theta_next / dx);  // floor
    br.ptr = (ptr_next < N) ? ptr_next : (N-1);

    // ---- 3) Depth of cut d = λ(Θ=1) sampled with the SAME sub-cell fraction ----
    // fractional offset within the blade cell
    const scalar frac = (theta_next - (scalar)br.ptr * dx) / dx;   // in [0,1)

    // the angle one sector behind is theta_next - L, which has the same 'frac'
    // indices just behind the blade, with the same fractional offset:
    int k0 = (int)br.ptr - 1; if (k0 < 0) k0 += (int)N;            // cell just behind
    int k1 = k0 - 1;            if (k1 < 0) k1 += (int)N;          // next upstream

    // linear interpolation at the same offset to get surface at Θ=1:
    const scalar surf_back = (1.0 - frac) * cut[(uint)k0] + frac * cut[(uint)k1];

    //scalar d = surf_back - br.s_blade;                              // λ(Θ=1)
    //br.d_cut = d;

    // ---- 4) book-keeping for next step ----
    br.s_blade_old = br.s_blade;
    br.theta_old   = br.theta;
}



DEVICE_FUNC void calculate_bit_rock_Detournay(const ConfigStatic &conf_stat, BitRock &br, const scalar v_bit) {
    /*--------------------------------------------------------------------
    Get model parameters from config
    ------------------- -------------------------------------------------*/
    scalar xi = conf_stat.xi_br;      // cutter inclination coefficient
    scalar epsilon = conf_stat.epsilon_br; // rock_specific strength
    scalar r = conf_stat.r_br;       // radius drill bit
    scalar sigma = conf_stat.sigma_br;   // rock contact stress
    scalar l = conf_stat.l_br;           // wear flat length
    scalar mu = conf_stat.mu_br;          // friction coefficient
    scalar gamma = conf_stat.gamma_br; // bit geometry parameter
    uint N_br = conf_stat.n_blades;

    /*--------------------------------------------------------------------
    Updating normal forces and torques
    --------------------------------------------------------------------*/
    //Regularization parameters
    const scalar e_heaviside = conf_stat.e_heaviside_br;
    const scalar e_ramp = conf_stat.e_ramp_br;
    const scalar e_sign = conf_stat.e_sign_br;
    const scalar tol_ramp = conf_stat.tol_ramp_br;
    const scalar tol_heaviside = conf_stat.tol_heaviside_br;


    const scalar d = br.d_cut;
    const scalar d_ramp = ramp_reg(d, e_ramp, tol_ramp);
    const scalar d_heaviside = heaviside_reg(d, e_heaviside, tol_heaviside);
    const scalar v_heaviside = heaviside_reg(v_bit, e_heaviside, tol_heaviside);
    const scalar omega_sign = sign_reg(br.omega, e_sign);

    scalar Fc = xi * epsilon * r * N_br * d_ramp;
    scalar Ff = sigma * r * l * v_heaviside * d_heaviside;
    scalar Tc = epsilon * r * r / 2 * N_br * d_ramp * omega_sign;
    scalar Tf = mu * r * gamma * Ff / 2 * omega_sign;

    br.WOB = Fc + Ff;
    br.TOB = Tc + Tf;
}

DEVICE_FUNC void calculate_bit_rock_Detournay_w_reaction(const ConfigStatic &conf_stat,
                                              BitRock &br,
                                              const scalar v_bit, const scalar omega_bit,
                                              const scalar F_reaction, const scalar T_reaction)
{
    // --- Parameters ---
    const scalar xi      = conf_stat.xi_br;        // cutter inclination coeff
    const scalar epsilon = conf_stat.epsilon_br;   // rock specific strength
    const scalar r       = conf_stat.r_br;         // bit radius
    const scalar sigma   = conf_stat.sigma_br;     // contact stress
    const scalar l       = conf_stat.l_br;         // wear-flat length
    const scalar mu      = conf_stat.mu_br;        // friction coeff
    const scalar gamma   = conf_stat.gamma_br;     // geometry param
    const uint   N_br    = conf_stat.n_blades;

    // Regularization parameters
    const scalar e_heaviside = conf_stat.e_heaviside_br;
    const scalar e_ramp      = conf_stat.e_ramp_br;
    const scalar e_sign      = conf_stat.e_sign_br;
    const scalar tol_ramp    = conf_stat.tol_ramp_br;
    const scalar tol_heaviside = conf_stat.tol_heaviside_br;


    // Local quantities
    const scalar d = br.d_cut;
    const scalar d_ramp = ramp_reg(d, e_ramp, tol_ramp);
    const scalar d_heaviside = heaviside_reg(d, e_heaviside, tol_heaviside);
    const scalar v_heaviside = heaviside_reg(v_bit, e_heaviside, tol_heaviside);
    
    if (d <= -1e-6) {
        br.WOB = 0.0;
        br.TOB = 0.0;
        return;
    }
    
    scalar Fc = xi * epsilon * r * N_br * d_ramp;
    scalar Ff = sigma * r * l;
    scalar Tc = epsilon * r * r / 2 * N_br * d_ramp ;
    scalar Tf = mu * r * gamma * Ff / 2 ;
    
    
    // --- Compute g_a, g_t (Eq.29 adapted) ---
    scalar ga, gt;
    scalar eps_v = 1e-3; // velocity tolerance 
    scalar eps_omega = 1e-3; // angular velocity tolerance

    scalar sign_omega = sign_reg(omega_bit, e_sign); // smooth sign function

    if (v_bit < -eps_v) {
        ga = 0;
    }
    else if (fabs(v_bit) <= eps_v) {
        if (F_reaction <= Fc) { 
            ga=0;
        }
        else if (F_reaction >= Fc+Ff) {
            ga=1;   
            
        } else {
            ga=(F_reaction-Fc)/Ff;
        }
    } else { 
        ga=1; 
    }


    if (omega_bit < -eps_omega) {
        gt = 0;
    } else if (fabs(omega_bit) <= eps_omega) {
        if (T_reaction <= Tc) {
            gt = 0;
        } else if (T_reaction >= Tc + Tf) {
            gt = 1;
        } else {
            gt = (T_reaction - Tc) / Tf;
        }
    } else {
        gt = 1;
    }
  
    br.WOB = Fc + ga * Ff;
    br.TOB = sign_omega * (Tc + gt * Tf);   // ensure opposing torque
    
}





DEVICE_FUNC void calculate_bit_rock_Detournay_regularized(const ConfigStatic &conf_stat, BitRock &br, const scalar v_bit) {
    /*--------------------------------------------------------------------
    Get model parameters from config
    --------------------------------------------------------------------*/
    scalar xi = conf_stat.xi_br;      // cutter inclination coefficient
    scalar epsilon = conf_stat.epsilon_br; // rock_specific strength
    scalar r = conf_stat.r_br;       // radius drill bit
    scalar sigma = conf_stat.sigma_br;   // rock contact stress
    scalar l = conf_stat.l_br;           // wear flat length
    scalar mu = conf_stat.mu_br;          // friction coefficient
    scalar gamma = conf_stat.gamma_br; // bit geometry parameter
    uint N_br = conf_stat.n_blades;

    /*--------------------------------------------------------------------
    Updating normal forces and torques
    --------------------------------------------------------------------*/
    const scalar omega = br.omega;
    
    scalar e_reg = conf_stat.e_reg;
    
    //const scalar omega_heaviside = heaviside_reg(br.omega, e);
    //const scalar v_heaviside = heaviside_reg(v_bit, e_heaviside, tol_heaviside);
    //const scalar omega_heaviside = heaviside_reg(omega, e_heaviside, tol_heaviside);
    //const scalar d_heaviside = heaviside_reg(br.d_cut, e_heaviside, tol_heaviside);


    // Regularize depth of cut and use it to gate cutting terms
    const scalar tol_ramp = conf_stat.tol_ramp_br;
    const scalar tol_heaviside = conf_stat.tol_heaviside_br;
    const scalar e_heaviside = conf_stat.e_heaviside_br;
    const scalar e_ramp = conf_stat.e_ramp_br;

    // Cutting contributions scaled by regularized depth and gated by H_cut
    scalar Ff = sigma * r * l ;
    
    scalar Fc = epsilon * r * xi * 2 * M_PI * v_bit 
                / sqrt(omega * omega + e_reg * e_reg);
    scalar Tc = 0.5 * epsilon * r * r * 2 * M_PI * v_bit * omega 
                / (omega * omega + e_reg * e_reg);
    scalar Tf = 0.5 * mu * gamma * r * r * sigma * l * omega / sqrt(omega * omega + e_reg * e_reg);

    br.WOB = (Fc + Ff); 
    br.TOB = (Tc + Tf); 
}


DEVICE_FUNC void calculate_bit_rock_Tucker_Wang(const ConfigStatic &conf_stat, BitRock &br, const scalar v_bit) {
    /*--------------------------------------------------------------------
    Get model parameters from config
    --------------------------------------------------------------------*/
    scalar a0 = conf_stat.a0_br;
    scalar a1 = conf_stat.a1_br;
    scalar a2 = conf_stat.a2_br;
    scalar a3 = conf_stat.a3_br;
    scalar a4 = conf_stat.a4_br;
    
    /*--------------------------------------------------------------------
    Updating normal forces and torques
    --------------------------------------------------------------------*/
    br.WOB = (v_bit - a2 * br.omega + a0)/a1;
    br.TOB = a3 * br.d_cut + a4;
}

DEVICE_FUNC void calculate_bit_rock_Tucker_Wang_torsional(const ConfigStatic &conf_stat, BitRock &br, const scalar WOB_reaction) {
    /*--------------------------------------------------------------------
    Get model parameters from config
    --------------------------------------------------------------------*/
    scalar r = conf_stat.r_br;       // radius drill bit
    scalar a0 = conf_stat.a0_br;
    scalar a1 = conf_stat.a1_br;
    scalar a2 = conf_stat.a2_br;
    scalar a3 = conf_stat.a3_br;
    
    /*--------------------------------------------------------------------
    Updating normal forces and torques
    --------------------------------------------------------------------*/
    if (WOB_reaction > 0.0){
        br.TOB = WOB_reaction * r * a0 * tanh(a1 * br.omega) + a2 * br.omega / (1 + a3 * br.omega);
    }
    else
        br.TOB = 0.0;
}
    