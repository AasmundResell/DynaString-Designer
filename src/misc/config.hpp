#pragma once
#include "includes.hpp"
#include "utils.hpp"

struct ConfigStatic {
    scalar L;                      /*Pipe undeformed total length*/
    scalar L_string_depth_initial; /*Initial string feed depth*/
    scalar dt;
    scalar dt_at;           // time step for axial/torsional subcycling
    scalar CFL;             // CFL number
    scalar CFL_at;          // CFL number for axial/torsional time integration
    scalar inertia_scaling; // inertia scaling
    scalar E;
    scalar nu;
    scalar rho_p;
    scalar rho_fi;             // density of inner fluid
    scalar rho_fo;             // density of outer fluid
    scalar standpipe_pressure; // pressure at standpipe (top_node)
    scalar Q_f;
    scalar mu_f;
    scalar mu_kinetic, mu_static;
    scalar critical_stribeck_velocity;
    scalar K_normal;                     /*Wall normal contact stiffness*/
    scalar C_normal;                     /*Wall normal contact damping coefficient*/
    scalar K_tangent;                    /*Wall tangential slip contact stiffness*/
    scalar C_tangent;                    /*Wall tangential slip contact damping coefficient*/
    scalar K_bs;                         /*Centralizer bow spring stiffness*/
    scalar C_bs;                         /*Centralizer bow spring damping coefficient*/
    scalar p_bs;                         /*Bow spring power-law exponent*/
    scalar r_bs_outer;                   /*Outer radius of the bow springs*/
    scalar r_bs_inner;                   /*Inner radius of the bow springs*/
    scalar alpha;                        /*Rayleigh mass proportional damping*/
    scalar beta;                         /*Rayleigh stiffness proportional damping*/
    scalar WOB_constant, TOB_constant;   /*Constant applied weight on bit and torque on bit*/
    scalar v_top_input, omega_top_input; /*Top input velocity and angular velocity*/
    scalar ec_amplifier;                 // amplifier of the eccentricity outer radius

    // Top drive parameters
    scalar J_td;                   // Rotary inertia of inertia top drive
    scalar PID_Kp, PID_Ki, PID_Kd; // PID controller variables

    // Bit-rock parameters
    scalar r_br; // radius drill bit
    scalar m_br; // mass drill bit
    scalar J_br; // Rotary inertia of drill bit

    /*--------Detournay bit-rock parameters -------------*/
    scalar xi_br;            // cutter inclination coefficient for bit rock
    scalar epsilon_br;       // rock_specific strength
    scalar sigma_br;         // rock contact stress
    scalar l_br;             // wear flat length for bit rock
    scalar mu_br;            // friction coefficient for bit rock
    scalar gamma_br;         // bit geometry parameter
    scalar e_reg;            // regularization parameter for bit rock regularized ROP
    scalar e_heaviside_br;   // regularization parameter for heaviside functions in bit rock
    scalar e_ramp_br;        // regularization parameter for ramp function in bit rock
    scalar e_sign_br;        // regularization parameter for sign function in bit rock
    scalar tol_heaviside_br; // tolerance parameter for heaviside functions in bit rock
    scalar tol_ramp_br;      // tolerance parameter for ramp function in bit rock
    /*--------Tucker-Wang bit-rock parameters -------------*/
    scalar a0_br;
    scalar a1_br;
    scalar a2_br;
    scalar a3_br;
    scalar a4_br;
    uint n_blades; // number of PDC bit-blades (constant for each simulation)

    uint n_at;                             // number of subcycles for axial/torsional integration
    uint N_points_gauss_legendre;          // number of points in gauss legendre quadrature
    uint Nc_iter;                          // number of iterations for the contact algorithm
    uint N_bs;                             /*Number of bow springs*/
    bool contact_enabled;                  // contact enabled
    bool gravity_enabled;                  // gravity enabled
    bool fluid_dynamics_enabled;           // fluid flow dynamics and damping enabled
    bool curvature_enabled;                // curvature enabled
    bool beam_stiffness_2nd_order_enabled; // beam stiffness 2nd order enabled
    bool rayleigh_damping_enabled;         // rayleigh damping enabled
    bool check_energy_balance;             // check energy balance
    bool linear_strain_energy_enabled;     // use linear strain energy formulation
    bool riemann_solver;
    bool enable_ax_tor_subcycle;
    bool bc_rock_cutting_analytical; // use analytical rock cutting update instead of PDE

    StringType string_type;
    CurvilinearIntegrationType curvilinear_integration_type;
    CorotationalFormulation corotational_formulation;
    BC_BottomAxialKinematics bc_bottom_axial_kinematics;
    BC_TopKinematicsType bc_top_kinematics_type;
    BC_TopOmegaControllerType bc_top_omega_type;
    BC_BitRockType bc_bit_rock_type;
    FSI_ForceModel fsi_force_model;
    FSI_TorqueModel fsi_torque_model;

    DEVICE_FUNC scalar get_G() const { return E / (2 * (1 + nu)); }
    DEVICE_FUNC scalar get_sound_speed() const { return sqrt(E / rho_p); }
};

// Lives on the host, manages most dynamic variables changed host-side
struct ConfigDynamic {
    scalar W_int{0.0}, W_ext{0.0}, KE{0.0};          /*Variables for energy balance check*/
    scalar v_top_target{0.0}, omega_top_target{0.0}; /*Target feed velocity and top angular velocity*/
    scalar block_position{0.0};                      /*Position of the traveling block, positivt direction "up"*/
    scalar feed_length{0.0};                         /*Current total length of feed pipe*/
    scalar hook_load{0.0};                           /*Hook load*/
    scalar top_drive_torque{0.0};                    /*Top drive torque*/
    scalar WOB{0.0};                                 /*Weight on bit*/
    scalar TOB{0.0};                                 /*Torque on bit*/
    scalar ROP{0.0};                                 /*Rate of penetration*/
    scalar v_top{0.0};                               /*Top velocity*/
    scalar omega_bit{0.0};                           /*Bit angular velocity*/
    scalar omega_top{0.0};                           /*Top drive angular velocity*/
    scalar MSE{0.0};                                 /*Mechanical Specific Energy*/
    scalar s_max{0.0};                               /* highest s reached by bit so far */
    scalar t_ROP{0.0};                               /* simulation time when max_bit_s_reached was last sampled */
};

struct Config {
    ConfigStatic conf_stat;
    string simulation_name;
    uint num_threads_omp{0};
    uint top_node{0};      /*Top node index*/
    uint top_node_new{0};  /*New top node index*/
    uint top_component{0}; /* Index of component being inserted */

    size_t n_max{0};
    scalar t_max{0.0};
    scalar t{0.0};
    size_t n{0}; // Current time step
    scalar t_init{0.0};
    scalar dS_min_node_target{0.0};    // Minimum distance between fem nodes
    scalar dS_max_render_target{0.0};  // Maximum distance between render points
    scalar dS_max_contact_target{0.0}; // Maximum distance between contact points
    scalar dS_min_rc_target{0.0};      // Minimum length for riemann cells
    scalar L_hole_depth_initial{0.0};  // Initial hole depth
    uint N_tj{0}, N_stab{1};           // Number of tool-joint and stabilizer contact points
    scalar energy_balance_tol{0.0};
    scalar radial_unit_conversion{1.0}; // Conversion factor for radial units

    path base_dir;
    path output_dir;
    uint n_write{0};
    SimulationMode simulation_mode;
    PipeSolverType pipe_solver_type;
    bool gui_enabled{false};
    bool use_gpu;
    bool close_graphics_window{false};
    bool save_csv{false};
    bool output_global_frame{false};
    bool initialization{false};
    bool reassemble_mass_vectors{false};
    bool write_mass{true};
    bool write_misc_quantities{false};
    bool pipe_fully_fed{false};
    bool pipe_fully_pulled{false};
    uint static_load_update_interval{0};
    uint random_seed{0}; // Seed for random number generation, used for reproducibility

    // BC_Top bc_top;

    Timer timer;

    // Config();
    // Config(const Config &config) = delete;
    // Config &operator=(const Config &config) = delete;

    path get_current_output_subdir() const;

    const path create_output_subdir() const;
};

struct ConfigGUI {

    bool gui_enabled;
    float gui_update_interval_sec;
    // (Real) time (in sec) between each time the simulation output is refreshed in the GUI
    float time_since_last_gui_update_sec = 0.0f;

    double target_time = 0;
    static constexpr uint framerate_target = 45; // fps
    uint n_steps_progressed_since_last_frame;
};