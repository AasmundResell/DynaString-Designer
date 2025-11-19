#include "input-parser.hpp"
#include "sim-tools/yaml-parser.hpp"
#include "utils.hpp"
#include <algorithm>
#include <filesystem>

using namespace sito;
#include "all-options.inl"

YamlParser *yaml_parser = nullptr;

InputParser::InputParser(const string &input_filename) : input_filename_{input_filename} {
    assert(yaml_parser == nullptr);
    yaml_parser = new YamlParser{input_filename, all_options};
}
InputParser::~InputParser() {
    assert(yaml_parser != nullptr);
    delete yaml_parser;
    yaml_parser = nullptr;
}

Config InputParser::create_config(bool gui_enabled) const {
    Config config;
    try {
        config = parse_yaml_config_options(gui_enabled);
    } catch (exception &e) {
        THROW_RUNTIME_ERROR("Error parsing config options:\n" + string(e.what()));
    }
    cout << "Config options parsed without errors\n";
    return config;
}

Config InputParser::parse_yaml_config_options(bool gui_enabled) const {
    assert(yaml_parser != nullptr);
    YamlParser &yp = *yaml_parser;
    Config config{};
    auto filename = std::filesystem::path(input_filename_).filename().string();
    const string yml_extensions[2] = {".yml", ".yaml"};

    // Remove known YAML extensions if present
    for (const auto &ext : yml_extensions) {
        if (filename.size() > ext.size() && filename.substr(filename.size() - ext.size()) == ext) {
            filename = filename.substr(0, filename.size() - ext.size());
            break;
        }
    }
    config.simulation_name = filename;
    config.base_dir = std::filesystem::current_path();
    cout << "Simulation directory: " << config.base_dir << endl;

    ConfigStatic &conf_stat = config.conf_stat;

    /*-------------------------------------------------
    ------------------Solver settings------------------
    -------------------------------------------------*/

    conf_stat.CFL = yp.read_required_option<scalar>("setup/CFL", SMALL_SCALAR, 1.0 - SMALL_SCALAR,
                                                    "CFL must be larger than 0 and smaller than 1");

    config.dS_min_node_target = yp.read_required_option<scalar>("setup/dS_min_node", SMALL_SCALAR, nullopt,
                                                                "dS_min_node must be a positive number");

    config.conf_stat.riemann_solver = yp.read_optional_option<bool>("setup/riemann_solver", false);

    if (config.conf_stat.riemann_solver) {
        config.dS_min_rc_target = yp.read_required_option<scalar>("setup/dS_min_riemann_cell", SMALL_SCALAR, nullopt,
                                                                  "dS_min_riemann_cell must be a positive number");
        if (config.dS_min_rc_target > config.dS_min_node_target) {
            THROW_RUNTIME_ERROR("dS_min_riemann_cell must be smaller than dS_min_node");
        }
    }

    config.conf_stat.enable_ax_tor_subcycle = yp.read_optional_option<bool>("setup/at_subcycling", false);

    if (config.conf_stat.enable_ax_tor_subcycle) {
        config.conf_stat.CFL_at = yp.read_required_option<scalar>("setup/CFL_at", SMALL_SCALAR, 1.0,
                                                                  "CFL_at must be larger than 0 and max equal to 1");
    }

    if (gui_enabled) {
        config.gui_enabled = true;
        config.simulation_mode = SimulationMode::INFINITE_LOOP;
        config.dS_max_render_target = yp.read_optional_option<scalar>("setup/dS_max_render", config.dS_min_node_target);
        if (config.dS_max_render_target > config.dS_min_node_target) {
            THROW_RUNTIME_ERROR("dS_max_render must be smaller than dS_min_node");
        }
    } else {
        /*Must specify a stopping criterion if renderer is not active*/
        if (yp.option_exists("setup/n_max") && yp.option_exists("setup/t_max")) {
            THROW_RUNTIME_ERROR("Either specify n_max or t_max, not both\n");
        } else if (yp.option_exists("setup/n_max")) {
            config.n_max = yp.read_required_option<size_t>("setup/n_max", nullopt, nullopt,
                                                           "n_max must be specified because t_max is unspecified");
            config.t_max = std::numeric_limits<scalar>::max();
            config.simulation_mode = SimulationMode::MAX_STEPS;
        } else if (yp.option_exists("setup/t_max")) {
            config.t_max = yp.read_required_option<scalar>("setup/t_max", SMALL_SCALAR, nullopt,
                                                           "t_max must be specified because n_max is unspecified");
            config.n_max = std::numeric_limits<size_t>::max();
            config.simulation_mode = SimulationMode::MAX_TIME;
        } else {
            THROW_RUNTIME_ERROR("Either n_max or t_max must be specified when renderer is inactive, but neither are");
        }
    }

    config.save_csv = yp.read_optional_option<bool>("setup/save_csv", false);
    config.n_write = yp.read_optional_option<uint>("setup/n_write", 1000);
    config.write_misc_quantities = yp.read_optional_option<bool>("setup/write_misc_quantities", false);

    // Only applies for radial length inputs!
    LengthUnit radial_length_unit =
        yp.read_optional_enum_option<LengthUnit>("setup/radial_unit", length_unit_from_string, LengthUnit::METER);

    if (radial_length_unit == LengthUnit::INCH) {
        printf("Inches specified for the radial length unit, all radial inputs must be specified in inches!\n");
        config.radial_unit_conversion = 0.0254;
    }

    if (!yp.option_exists("setup/n_threads")) {
        printf("Warning: num_threads is unspecified. Setting default value of 1\n");
        config.num_threads_omp = 1;
    } else {
        config.num_threads_omp = yp.read_required_option<uint>("setup/n_threads", 1, 100);
    }
    config.use_gpu = yp.read_optional_option<bool>("setup/use_gpu", false);
    if (config.use_gpu && !CUDA_ENABLED) {
        THROW_RUNTIME_ERROR("GPU acceleration requested but program was not built with CUDA support");
    }

    // Initialization step of the solution with artifical damping
    config.initialization = yp.read_optional_option<bool>("setup/initialization", false);

    if (config.initialization) {
        config.t_init = yp.read_required_option<scalar>("setup/t_init", 0.0, nullopt, "t_init must be specified");
    }

    /*--------------- Choose solver type --------------*/

    config.pipe_solver_type =
        yp.read_required_enum_option<PipeSolverType>("setup/pipe_solver_type", pipe_solver_type_from_string);

    if (config.pipe_solver_type == PipeSolverType::CURVILINEAR) {

        /*---------------------------------------------------
        ---------------- Curvilinear solver settings---------
        ---------------------------------------------------*/
        conf_stat.curvilinear_integration_type = yp.read_required_enum_option<CurvilinearIntegrationType>(
            "curvilinear/curvilinear_integration_type", curvilinear_integration_type_from_string);

        conf_stat.curvature_enabled = yp.read_optional_option<bool>("curvilinear/curvature_enabled", false);

        conf_stat.beam_stiffness_2nd_order_enabled =
            yp.read_optional_option<bool>("curvilinear/beam_stiffness_2nd_order", false);

        conf_stat.linear_strain_energy_enabled =
            yp.read_optional_option<bool>("curvilinear/linear_strain_energy_enabled", false);

        if (conf_stat.curvilinear_integration_type == CurvilinearIntegrationType::GAUSS_LEGENDRE) {
            conf_stat.N_points_gauss_legendre =
                yp.read_optional_option<uint>("curvilinear/gauss-legendre_integration_order", 3); //
        }

        config.static_load_update_interval =
            yp.read_optional_option<uint>("curvilinear/static_load_update_interval", 100);

        config.output_global_frame = yp.read_optional_option<bool>("curvilinear/output_global_frame", false);
    }

    if (config.pipe_solver_type == PipeSolverType::COROTATIONAL) {

        /*---------------------------------------------------
        --------------- Corotational solver settings---------
        ---------------------------------------------------*/
        conf_stat.check_energy_balance = yp.read_optional_option<bool>("corotational/check_energy_balance", false);
        if (conf_stat.check_energy_balance) {
            config.energy_balance_tol = yp.read_optional_option<scalar>("corotational/energy_balance_tol", 0.01);
        }

        conf_stat.corotational_formulation = yp.read_optional_enum_option<CorotationalFormulation>(
            "corotational/corotational_formulation", corotational_formulation_from_string,
            CorotationalFormulation::CRISFIELD);

        if (yp.read_optional_option<bool>("corotational/output_curvlin_frame", false) == true) {
            config.output_global_frame = false;
        } else {
            config.output_global_frame = true;
        }
    }

    /*---------------------------------------------------
    ------------------Physics settings ------------------
    ---------------------------------------------------*/

    /*--------------Governing physics -----------------*/

    conf_stat.gravity_enabled = yp.read_optional_option<bool>("properties/gravity_enabled", true);

    conf_stat.contact_enabled = yp.read_optional_option<bool>("properties/contact_properties/contact_enabled", true);
    conf_stat.Nc_iter = yp.read_optional_option<scalar>("properties/contact_properties/Nc_iter",
                                                        10); // Note: only used for ellipsoid contact

    // Set to zero default for now as it can cause stability issues
    conf_stat.ec_amplifier = yp.read_optional_option<scalar>("properties/eccentricity_control", 0.0, 0.0, 10.0);

    /*--------------- Contact properties --------------*/
    config.dS_max_contact_target = yp.read_required_option<scalar>("setup/dS_max_contact");
    config.N_tj = yp.read_optional_option<uint>("setup/N_tj", 0);
    config.N_stab = yp.read_optional_option<uint>("setup/N_stab", 1);

    // TODO -> Find a set of default contact values that give stable behaviour
    conf_stat.K_normal = yp.read_required_option<scalar>("properties/contact_properties/K_normal", 0, nullopt);
    conf_stat.C_normal = yp.read_required_option<scalar>("properties/contact_properties/C_normal", 0, nullopt);
    conf_stat.K_tangent = yp.read_optional_option<scalar>("properties/contact_properties/K_tangent", 0.0, 0, nullopt);
    conf_stat.C_tangent = yp.read_optional_option<scalar>("properties/contact_properties/C_tangent", 0.0, 0, nullopt);
    conf_stat.critical_stribeck_velocity = yp.read_required_option<scalar>(
        "properties/contact_properties/stribeck_velocity", 0.01, 5.0); // Below 1.0 seems to create instabilities!

    conf_stat.mu_static = yp.read_required_option<scalar>("properties/contact_properties/mu_static", 0, nullopt);
    conf_stat.mu_kinetic = yp.read_required_option<scalar>("properties/contact_properties/mu_kinetic", 0, nullopt);

    /*--------------- Fluid-properties and FSI interaction --------------*/

    conf_stat.standpipe_pressure =
        yp.read_optional_option<scalar>("properties/fluid_properties/standpipe_pressure", 0.0);

    conf_stat.fluid_dynamics_enabled =
        yp.read_optional_option<bool>("properties/fluid_properties/fluid_dynamics_enabled", false);

    if (conf_stat.fluid_dynamics_enabled) {
        conf_stat.Q_f = yp.read_optional_option<scalar>("properties/fluid_properties/flow_rate", 0.0);

        conf_stat.fsi_force_model = yp.read_optional_enum_option<FSI_ForceModel>(
            "properties/fluid_properties/fsi_force_model", fsi_force_model_from_string, FSI_ForceModel::NONE);

        conf_stat.fsi_torque_model = yp.read_optional_enum_option<FSI_TorqueModel>(
            "properties/fluid_properties/fsi_torque_model", fsi_torque_model_from_string, FSI_TorqueModel::NONE);
    }

    /*--------------- Choose string analysis type --------------*/
    config.conf_stat.string_type =
        yp.read_required_enum_option<StringType>("properties/string_type", string_type_from_string);

    /*--------------------------------------------------
    ------------ Boundary Conditions settings ----------
    --------------------------------------------------*/

    /*------------Axial Kinematic Configuration -------------
    Main setting determining the "class" of axial bcs for the pipe-in-hole

    Settings are:
    1. free
        - Simplest option, pipe is free at the end
        - Suitable for tripping type analysis
        - Note: The hole has no end-cap surface
        - Note: Bit rock model is disabled
    2. bottom-contact (only for drill string or coiled tubing)
        - defines a physical end surface which the bit is pushed against
        - The rock surface is cutted and tracked accordingly
        - The initial surface depth must be specified
    3. fixed
        - End is fixed at the end of the pipe
        - Pipe is fixed at the shortest depth of the hole or the pipe length
        - Feeding may be possible if the pipe is longer than the hole
        - Note: Bit rock may be enabled, but no cutting occurs
    ---------------------------------------------------*/

    if (config.conf_stat.string_type == StringType::CASING_STRING) {
        conf_stat.bc_bottom_axial_kinematics = BC_BottomAxialKinematics::FREE;
    } else if (config.conf_stat.string_type == StringType::DRILL_STRING) {
        conf_stat.bc_bottom_axial_kinematics = BC_BottomAxialKinematics::BOTTOM_CONTACT;
    }

    /*------------Top drive system and boundary conditions -------------*/
    conf_stat.bc_top_kinematics_type = yp.read_optional_enum_option<BC_TopKinematicsType>(
        "properties/top_drive/kinematics", bc_top_kinematics_from_string, BC_TopKinematicsType::DIRICHLET);

    conf_stat.v_top_input = yp.read_optional_option<scalar>("properties/top_drive/v_top", 0.0) / 3600.0;

    conf_stat.omega_top_input = yp.read_optional_option<scalar>("properties/top_drive/omega_top", 0.0) * M_PI / 30.0;

    if (conf_stat.bc_top_kinematics_type == BC_TopKinematicsType::NEUMANN) {
        //----- Top-drive/rotary table properties------
        conf_stat.J_td = yp.read_required_option<scalar>("properties/top_drive/inertia");
        conf_stat.PID_Kp = yp.read_required_option<scalar>("properties/top_drive/PID_Kp");
        conf_stat.PID_Ki = yp.read_optional_option<scalar>("properties/top_drive/PID_Ki", 0.0);
        conf_stat.PID_Kd = yp.read_optional_option<scalar>("properties/top_drive/PID_Kd", 0.0);

        conf_stat.bc_top_omega_type = yp.read_optional_enum_option<BC_TopOmegaControllerType>(
            "properties/top_drive/torque_controller", bc_top_omega_type_from_string, BC_TopOmegaControllerType::PID);
    }

    /*------------Bit-rock properties and bottom boundary conditions -------------*/
    conf_stat.WOB_constant = yp.read_optional_option<scalar>("properties/WOB", 0.0);
    conf_stat.TOB_constant = yp.read_optional_option<scalar>("properties/TOB", 0.0);

    if (config.pipe_solver_type == PipeSolverType::CURVILINEAR) {
        if (conf_stat.bc_bottom_axial_kinematics == BC_BottomAxialKinematics::BOTTOM_CONTACT ||
            conf_stat.bc_bottom_axial_kinematics == BC_BottomAxialKinematics::FIXED) {
            conf_stat.bc_bit_rock_type = yp.read_optional_enum_option<BC_BitRockType>(
                "properties/bit_rock/model", bc_bit_rock_type_from_string, BC_BitRockType::NO_BIT);
        } else if (conf_stat.bc_bottom_axial_kinematics == BC_BottomAxialKinematics::FREE) {
            // Ensures that the bit-rock object is not generated
            conf_stat.bc_bit_rock_type = BC_BitRockType::NO_BIT;
        }

        if (conf_stat.bc_bit_rock_type != BC_BitRockType::NO_BIT) {
            conf_stat.bc_rock_cutting_analytical =
                yp.read_optional_option<bool>("properties/bit_rock/rock_cutting_analytical", false);
            conf_stat.n_blades = yp.read_required_option<uint>("properties/bit_rock/n_blades", 1, 10);
            conf_stat.m_br = yp.read_required_option<scalar>("properties/bit_rock/mass", 10.0); // Atleast 10 kg
            conf_stat.J_br = yp.read_optional_option<scalar>("properties/bit_rock/inertia", 0.0);

            if (conf_stat.bc_bit_rock_type == BC_BitRockType::DETOURNAY ||
                conf_stat.bc_bit_rock_type == BC_BitRockType::DETOURNAY_REGULARIZED) {
                conf_stat.epsilon_br = yp.read_required_option<scalar>("properties/bit_rock/epsilon");
                conf_stat.sigma_br = yp.read_required_option<scalar>("properties/bit_rock/sigma");
                conf_stat.l_br = yp.read_required_option<scalar>("properties/bit_rock/l");
                conf_stat.xi_br = yp.read_required_option<scalar>("properties/bit_rock/xi");
                conf_stat.gamma_br = yp.read_required_option<scalar>("properties/bit_rock/gamma");
                conf_stat.mu_br = yp.read_required_option<scalar>("properties/bit_rock/mu");
                conf_stat.e_heaviside_br = yp.read_optional_option<scalar>("properties/bit_rock/e_heaviside", 0.0);
                conf_stat.e_ramp_br = yp.read_optional_option<scalar>("properties/bit_rock/e_ramp", 0.0);
                conf_stat.e_sign_br = yp.read_optional_option<scalar>("properties/bit_rock/e_sign", 0.0);
                conf_stat.tol_heaviside_br = yp.read_optional_option<scalar>("properties/bit_rock/tol_heaviside", 0.1);
                conf_stat.tol_ramp_br =
                    max(yp.read_optional_option<scalar>("properties/bit_rock/tol_ramp", 0.001), SMALL_SCALAR);

                if (conf_stat.bc_bit_rock_type == BC_BitRockType::DETOURNAY_REGULARIZED)
                    conf_stat.e_reg = yp.read_required_option<scalar>("properties/bit_rock/e_reg");

            } else if (conf_stat.bc_bit_rock_type == BC_BitRockType::TUCKER_WANG ||
                       conf_stat.bc_bit_rock_type == BC_BitRockType::TUCKER_WANG_TORSIONAL) {
                conf_stat.a0_br = yp.read_required_option<scalar>("properties/bit_rock/a0");
                conf_stat.a1_br = yp.read_required_option<scalar>("properties/bit_rock/a1");
                conf_stat.a2_br = yp.read_required_option<scalar>("properties/bit_rock/a2");
                conf_stat.a3_br = yp.read_required_option<scalar>("properties/bit_rock/a3");
                if (conf_stat.bc_bit_rock_type == BC_BitRockType::TUCKER_WANG) {
                    conf_stat.a4_br = yp.read_required_option<scalar>("properties/bit_rock/a4");
                }
            }
        }
    } else { // Bit-rock model not supported for corotational solver yet
        assert(config.pipe_solver_type == PipeSolverType::COROTATIONAL);
        config.conf_stat.bc_bit_rock_type = BC_BitRockType::NO_BIT;
    }

    /*-------------------------------------------------
    ----------------Material properties----------------
    -------------------------------------------------*/

    conf_stat.E = yp.read_required_option<scalar>("properties/young_modulus");

    conf_stat.nu = yp.read_required_option<scalar>("properties/poisson_ratio");

    conf_stat.rho_p = yp.read_required_option<scalar>("properties/pipe_density");
    conf_stat.rho_fi = yp.read_required_option<scalar>("properties/fluid_properties/fluid_density_inner");
    conf_stat.rho_fo = yp.read_required_option<scalar>("properties/fluid_properties/fluid_density_outer");

    if (conf_stat.fluid_dynamics_enabled) {
        conf_stat.mu_f = yp.read_required_option<scalar>("properties/fluid_properties/fluid_viscosity");
    }

    conf_stat.rayleigh_damping_enabled =
        yp.read_optional_option<bool>("properties/damping_properties/rayleigh_damping_enabled", true);

    conf_stat.alpha = yp.read_optional_option<scalar>("properties/damping_properties/alpha", 0.0);
    conf_stat.beta = yp.read_optional_option<scalar>("properties/damping_properties/beta", 0.0);

    return config;
}

Hole InputParser::create_hole(Config &config, ArenaBump &arena_h) const {
    assert(yaml_parser != nullptr);
    YamlParser &yp = *yaml_parser;
    const scalar radial_conv = config.radial_unit_conversion;
    Hole hole{};
    vector<HoleSection> hole_sections;
    HoleProfileType profile_type;
    HoleTrajectoryType trajectory_type;

    uint N_hole{0};      // Number of hole nodes
    vector<Mat3> Q_list; /*List local basis (Q is an orthogonal matrix describing the orientation of a straight hole
                            segment)*/
    vector<scalar> R;    /*List of radii for each hole node*/
    vector<Vec3> nodes;  /*Coordinates of centre of drillhole. Defines the hole discretization*/
    /*Survey points vectors, same format as the industry standard, angles in degrees */
    vector<scalar> MD;
    vector<scalar> inclinations;
    vector<scalar> azimuths;
    vector<scalar> diameters;       /*Input diameter vector*/
    vector<scalar> diameter_depths; /*Input depths diameter vector*/

    // Options for "linear" hole generation
    uint n_interpolate{0};    // Number of spline points for spline interpolation
    scalar inclination_top;   /*Top inclination angle, zero by default*/
    scalar inclination_build; /*Inclination angle to build up to in the build section*/
    scalar azimuth_top;       /*Initial azimuthal angle at top, zero by default*/
    scalar azimuth_build;     /*Azimuthal angle to build up to in the build section*/
    scalar L_total{0.0};      // Total length of hole
    scalar L_build;           // For build section
    scalar L_front;           // For straight section in the front
    scalar L_end;             // For straight section extension at the back

    /*--------------------------------------------------------------------
    Parsing hole properties
    --------------------------------------------------------------------*/
    try {
        /*--------------------------------------------------------------------
        Hole trajectory and profile type
        --------------------------------------------------------------------*/

        trajectory_type = yp.read_optional_enum_option<HoleTrajectoryType>("hole/trajectory_build_type",
                                                                           hole_trajectory_build_type_from_string,
                                                                           HoleTrajectoryType::MINIMUM_CURVATURE);
        profile_type =
            yp.read_required_enum_option<HoleProfileType>("hole/input_build_type", hole_profile_type_from_string);

        if (profile_type == HoleProfileType::LINEAR) {
            n_interpolate = yp.read_required_option<scalar>("hole/n_interpolate");
            MD.resize(n_interpolate);

            inclination_top = yp.read_optional_option<scalar>("hole/inclination_top", 0.0);
            azimuth_top = yp.read_optional_option<scalar>("hole/azimuth_top", 0.0);

            L_build = yp.read_optional_option<scalar>("hole/L_build", 0.0);

            if (L_build > 0.0) {
                inclination_build = yp.read_required_option<scalar>("hole/inclination_build");
                azimuth_build = yp.read_optional_option<scalar>("hole/azimuth_build", 0.0);
            } else {
                inclination_build = inclination_top;
            }
            L_front = yp.read_optional_option<scalar>("hole/L_front", 0.0);
            L_end = yp.read_optional_option<scalar>("hole/L_end", 0.0);
            L_total = L_front + L_build + L_end;
            if (L_total <= 0.0)
                throw runtime_error{"Length of hole must be a positive number"};

        } else if (profile_type == HoleProfileType::VECTOR) {

            MD = yp.read_required_option_vector<scalar>("hole/MD");
            inclinations = yp.read_required_option_vector<scalar>("hole/inclinations");
            azimuths = yp.read_required_option_vector<scalar>("hole/azimuths");
            L_total = MD.back();
            n_interpolate = MD.size();

            if (n_interpolate != inclinations.size() || n_interpolate != azimuths.size())
                throw runtime_error{"length of survey point vectors must match"};

            if (MD[0] != 0.0)
                throw runtime_error{"MD[0] must be zero"};
        } else {
            assert(false);
        }

        /*Must specify a stopping criterion if renderer is not active*/
        if (yp.option_exists("hole/ds_target") && yp.option_exists("hole/n_hole")) {
            THROW_RUNTIME_ERROR("Either specify ds_target or n_hole, not both\n");
        } else if (yp.option_exists("hole/ds_target")) {
            scalar ds_target = yp.read_required_option<scalar>("hole/ds_target", SMALL_SCALAR, L_total,
                                                               "ds_target must be  shorter than total hole length");
            N_hole = uint(L_total / ds_target);
        } else if (yp.option_exists("hole/n_hole")) {
            N_hole = yp.read_required_option<uint>("hole/n_hole", 0, nullopt, "n_hole must be positive");
        } else {
            THROW_RUNTIME_ERROR("Either ds_target or n_hole must be specified for hole discretization");
        }

        /*--------------------------------------------------------------------
        Parsing hole sections
        --------------------------------------------------------------------*/
        vector<YAML::Node> hole_sections_yml = yp.read_required_yaml_node_vector("hole/sections");
        hole_sections.reserve(hole_sections_yml.size());

        for (const auto &section : hole_sections_yml) {
            HoleSection hole_section;

            if (section["name"]) {
                hole_section.name = section["name"].as<string>();
            } else {
                hole_section.name = "UNDEFINED";
            }

            string surface_type = section["surface_type"] ? section["surface_type"].as<string>() : "circular";
            hole_section.surface_type = hole_surface_type_from_string.at(surface_type);

            if (!hole_sections.empty()) { // Only read when it is not the first section
                hole_section.depth = section["depth"].as<scalar>();
                if (hole_section.depth < 0.0) {
                    THROW_RUNTIME_ERROR("Hole section depth must be a positive number");
                } else if (hole_section.depth <= hole_sections.back().depth) {
                    THROW_RUNTIME_ERROR("Each hole section's depth must be greater than the previous section's depth");
                }
            }

            if (!section["diameter"]) {
                THROW_RUNTIME_ERROR("hole_section: scalar 'diameter' must be specified for all surface types");
            }
            hole_section.D = section["diameter"].as<scalar>() * radial_conv;

            if (hole_section.surface_type == HoleSurfaceType::CIRCULAR) {
                // For circular, diameter is required and is the mean diameter
                hole_section.a_mean = hole_section.D / 2.0;
                hole_section.b_mean = hole_section.D / 2.0;
            } else if (hole_section.surface_type == HoleSurfaceType::ELLIPTICAL) {
                if (!section["a_mean"] || !section["b_mean"]) {
                    THROW_RUNTIME_ERROR(
                        "hole_section: 'a_mean' and 'b_mean' must be specified for elliptical surface type");
                }
                hole_section.a_mean = section["a_mean"].as<scalar>() * radial_conv;
                hole_section.b_mean = section["b_mean"].as<scalar>() * radial_conv;
                // Enforce a_mean and b_mean >= D/2
                if (hole_section.a_mean < hole_section.D / 2.0 || hole_section.b_mean < hole_section.D / 2.0) {
                    THROW_RUNTIME_ERROR("hole_section: 'a_mean' and 'b_mean' must be >= diameter/2");
                }
                if (section["dalpha"]) {
                    hole_section.dalpha = section["dalpha"].as<scalar>();
                }
            } else {
                THROW_RUNTIME_ERROR("Invalid hole surface type specified");
            }

            if (section["cross_section_build_type"]) {
                // Need functionality for this!!
                string cross_section_build_type =
                    section["cross_section_build_type"] ? section["cross_section_build_type"].as<string>() : "none";
                auto it = hole_cross_section_build_type_from_string.find(cross_section_build_type);
                if (it != hole_cross_section_build_type_from_string.end()) {
                    hole_section.cross_section_build_type = it->second;
                } else {
                    THROW_RUNTIME_ERROR("Invalid hole cross section build type specified: " + cross_section_build_type);
                }

                // Read optional parameters for random field
                if (hole_section.cross_section_build_type == HoleCrossSectionBuildType::RANDOM_FIELD) {
                    // For circular: a_mean is the mean random radius
                    if (hole_section.surface_type == HoleSurfaceType::CIRCULAR) {
                        if (section["a_mean"]) {
                            hole_section.a_mean = section["a_mean"].as<scalar>() * radial_conv;
                            if (hole_section.a_mean < hole_section.D / 2.0) {
                                THROW_RUNTIME_ERROR(
                                    "hole_section: 'a_mean' must be >= diameter/2 for circular random field");
                            }
                        } else {
                            THROW_RUNTIME_ERROR("hole_section: 'a_mean' must be specified for circular random field");
                        }
                    }

                    if (section["std_radial"]) {
                        hole_section.std_pr_radial = section["std_radial"].as<scalar>();
                    }
                    if (section["std_ellipse_angle"]) {
                        hole_section.std_pr_dalpha = section["std_ellipse_angle"].as<scalar>();
                    }
                    if (section["cor_radial"]) {
                        hole_section.cor_radial = section["cor_radial"].as<scalar>();
                    }
                    if (section["cor_ellipse_angle"]) {
                        hole_section.cor_dalpha = section["cor_ellipse_angle"].as<scalar>();
                    }
                }
            }

            if (section["offset_build_type"]) {
                string offset_build_type =
                    section["offset_build_type"] ? section["offset_build_type"].as<string>() : "none";
                auto it = hole_offset_build_type_from_string.find(offset_build_type);
                if (it != hole_offset_build_type_from_string.end()) {
                    hole_section.offset_build_type = it->second;
                } else {
                    THROW_RUNTIME_ERROR("Invalid hole offset build type specified: " + offset_build_type);
                }

                if (hole_section.offset_build_type == HoleOffsetBuildType::NONE) {

                } else if (hole_section.offset_build_type == HoleOffsetBuildType::ANALYTICAL_SPIRAL ||
                           hole_section.offset_build_type == HoleOffsetBuildType::RANDOM_FIELD) {
                    if (!section["pitch"] || !section["diameter_spiral"]) {
                        THROW_RUNTIME_ERROR("hole_section: scalars 'pitch' and 'diameter_spiral' must be specified for "
                                            "analytical-spiral or random-field hole offset type");
                    }

                    hole_section.pitch = section["pitch"].as<scalar>();
                    hole_section.diameter_spiral = section["diameter_spiral"].as<scalar>() * radial_conv;
                    if (hole_section.pitch < 0.0 || hole_section.diameter_spiral < 0.0) {
                        THROW_RUNTIME_ERROR(
                            "hole_section: 'pitch' and 'diameter_spiral' must be zero or positive numbers");
                    }

                    // Read optional parameters for random field
                    if (hole_section.offset_build_type == HoleOffsetBuildType::RANDOM_FIELD) {
                        if (section["std_pitch"]) {
                            hole_section.std_pr_pitch = section["std_pitch"].as<scalar>();
                        }
                        if (section["std_diameter_spiral"]) {
                            hole_section.std_pr_diameter_spiral =
                                section["std_diameter_spiral"].as<scalar>() * radial_conv;
                        }
                        if (section["std_offset_noise"]) {
                            hole_section.std_pr_off_noise = section["std_offset_noise"].as<scalar>();
                        }
                        if (section["cor_pitch"]) {
                            hole_section.cor_pitch = section["cor_pitch"].as<scalar>();
                        }
                        if (section["cor_diameter_spiral"]) {
                            hole_section.cor_diameter_spiral = section["cor_diameter_spiral"].as<scalar>();
                        }
                        if (section["cor_offset_noise"]) {
                            hole_section.cor_off_noise = section["cor_offset_noise"].as<scalar>();
                        }
                    }

                    if (section["diameter_drift"]) {
                        hole_section.diameter_drift = section["diameter_drift"].as<scalar>() * radial_conv;
                    }

                    if (section["max_curvature"]) {
                        hole_section.max_curvature = section["max_curvature"].as<scalar>();
                    }
                } else {
                    THROW_RUNTIME_ERROR("Invalid hole offset build type specified");
                }
            }

            hole_sections.push_back(hole_section);
        }

    } catch (exception &e) {
        throw runtime_error("Error parsing hole properties:\n" + string(e.what()));
        cout << "Hole properties parsed without errors\n";
    }

    // If a bit-rock model is enabled, set the bit radius based on the last (open) hole section:
    if (config.conf_stat.bc_bit_rock_type != BC_BitRockType::NO_BIT) {
        const HoleSection &last_section = hole_sections.back();
        config.conf_stat.r_br = last_section.D / 2.0;
    }
    /*--------------------------------------------------------------------
    Constructing hole object from parsed paramters
    --------------------------------------------------------------------*/
    try {
        if (profile_type == HoleProfileType::LINEAR)
            hole = hole_create_from_linear(trajectory_type, N_hole, MD, inclination_top, inclination_build, azimuth_top,
                                           azimuth_build, L_total, L_front, L_end, arena_h);
        else if (profile_type == HoleProfileType::VECTOR)
            hole = hole_create_from_vector(trajectory_type, N_hole, MD, inclinations, azimuths, arena_h);

    } catch (exception &e) {
        THROW_RUNTIME_ERROR("Failed to create hole object\n" + string(e.what()));
    }

    if (config.conf_stat.bc_bottom_axial_kinematics == BC_BottomAxialKinematics::BOTTOM_CONTACT ||
        config.conf_stat.bc_bottom_axial_kinematics == BC_BottomAxialKinematics::FIXED) {
        /*---------------------Only set this when digging--------------------------*/
        config.L_hole_depth_initial = yp.read_optional_option<scalar>(
            "properties/L_hole_depth_initial", 0.0, 0.0, hole.get_L(arena_h.buf) - 0.001,
            "L_hole_depth_initial must be positive and less than the total hole depth");
    } else {
        config.L_hole_depth_initial = hole.get_L(arena_h.buf) - 0.001;
    }

    /*--------------------------------------------------------------------
    Creating hole surface
    --------------------------------------------------------------------*/

    try {
        create_hole_surface(config, hole, hole_sections, arena_h);
    } catch (exception &e) {
        THROW_RUNTIME_ERROR("Failed to create hole surface:\n" + string(e.what()));
    }

    /*--------------------------------------------------------------------
    Save hole geometry (only saves the first timestep for now, modify
    if needed)
    --------------------------------------------------------------------*/
    try {
        save_hole_csv(config, hole, arena_h);
    } catch (exception &e) {
        THROW_RUNTIME_ERROR("Failed to write hole object to csv\n" + string(e.what()));
    }

    return hole;
}

Pipe InputParser::create_pipe(Config &config, vector<PipeComponent> &pipe_assembly,
                              PipeRenderComponents *pipe_render_components, ArenaBump &arena_h) const {
    assert(yaml_parser != nullptr);
    YamlParser &yp = *yaml_parser;
    scalar radial_conv = config.radial_unit_conversion;
    scalar L_string;
    ConfigStatic &conf_stat = config.conf_stat;
    vector<Centralizer> centralizers = {};
    vector<EccentricMass> eccentric_masses = {};

    /*--------------------------------------------------------------------
    Parsing pipe properties
    --------------------------------------------------------------------*/
    try {
        vector<uint> component_id = yp.read_required_option_vector<uint>("pipe/component_id");
        vector<uint> component_count = yp.read_required_option_vector<uint>("pipe/component_count");
        if (component_id.size() != component_count.size()) {
            THROW_RUNTIME_ERROR("component id and component count doesn't match");
        }
        // Read unique components first
        vector<YAML::Node> unique_components = yp.read_required_yaml_node_vector("pipe/unique_components");
        vector<PipeComponent> component_templates;
        component_templates.reserve(unique_components.size());

        bool steering_already_defined = false;

        // Parse unique component templates
        for (const auto &component : unique_components) {
            PipeComponent pipe_component;

            //--------------------------------------------------------------
            // Parse common parameters for all components
            //--------------------------------------------------------------
            pipe_component.L = component["L"].as<scalar>();
            pipe_component.Do = component["D_outer"].as<scalar>() * radial_conv;
            pipe_component.Di = component["D_inner"].as<scalar>() * radial_conv;
            if (component["name"]) {
                pipe_component.name = component["name"].as<string>();
            }
            //--------------------------------------------------------------
            // Parse type specific parameters for the various components
            //--------------------------------------------------------------
            if (component["type"]) {
                try {
                    pipe_component.type = component_type_from_string_user_input.at(component["type"].as<string>());
                } catch (const std::out_of_range &) {
                    THROW_RUNTIME_ERROR("Unknown pipe component type: " + component["type"].as<string>());
                }

                switch (pipe_component.type) {
                case (ComponentType::DRILL_PIPE):
                    parse_drill_pipe_component(config, component, pipe_component);
                    break;
                case (ComponentType::MWD):
                    parse_mwd_component(config, component, pipe_component);
                    break;
                case (ComponentType::STABILIZER):
                    parse_stabilizer_component(config, component, pipe_component);
                    break;
                case (ComponentType::SUB):
                    parse_sub_component(config, component, pipe_component);
                    break;
                case (ComponentType::STEERING):
                    parse_steering_component(config, component, pipe_component, steering_already_defined);
                    break;
                case (ComponentType::CASING):
                    parse_casing_component(config, component, pipe_component);
                    break;
                }
            } else {
                pipe_component.type = ComponentType::NONE;
            }
            component_templates.push_back(pipe_component);
        }

        // Calculate total number of components
        size_t total_components = 0;
        for (size_t i = 0; i < component_id.size(); i++) {
            total_components += component_count[i];
        }

        // Expand components into sequential vector
        pipe_assembly.reserve(total_components);

        for (size_t i = 0; i < component_id.size(); i++) {
            uint id = component_id[i];
            uint count = component_count[i];
            if (id >= component_templates.size()) {
                THROW_RUNTIME_ERROR("Component ID out of range");
            }
            for (uint j = 0; j < count; j++) {
                pipe_assembly.push_back(component_templates[id]);
            }
        }

        if (pipe_assembly.empty()) {
            throw runtime_error("There are zero components in total");
        }

        // Calculate component global positions
        L_string = 0.0;
        for (uint i = 0; i < pipe_assembly.size(); i++) {
            pipe_assembly[i].S_global = L_string;
            L_string += pipe_assembly[i].L;
        }

        conf_stat.L = L_string;

        scalar L_feed_default = min(config.L_hole_depth_initial, L_string);

        if (conf_stat.bc_bottom_axial_kinematics == BC_BottomAxialKinematics::BOTTOM_CONTACT ||
            conf_stat.bc_bottom_axial_kinematics == BC_BottomAxialKinematics::FREE) {
            conf_stat.L_string_depth_initial =
                yp.read_optional_option<scalar>("properties/L_string_depth_initial", L_feed_default);
            if (conf_stat.L_string_depth_initial < 0.0 || conf_stat.L_string_depth_initial > L_string) {
                throw runtime_error(
                    "Invalid value for the initial feed length. The length must be positive and less than "
                    "the total string length.");
            } else if (conf_stat.L_string_depth_initial > config.L_hole_depth_initial) {
                throw runtime_error("Invalid value for the initial feed length. The length must be less than "
                                    "the initial hole depth.");
            }
        } else if (conf_stat.bc_bottom_axial_kinematics == BC_BottomAxialKinematics::FIXED) {
            conf_stat.L_string_depth_initial = L_feed_default;
        } else {
            assert(false);
        }

        if (conf_stat.string_type == StringType::CASING_STRING) {
            parse_centralizers(config, pipe_assembly, centralizers, L_string);
        }
    } catch (std::exception &e) {
        throw runtime_error("Error parsing pipe properties:\n" + string(e.what()));
    }

    cout << "Pipe properties parsed without errors\n";

    /*--------------------------------------------------------------------
    Constructing pipe object from parsed paramters
    --------------------------------------------------------------------*/
    Pipe pipe{};
    try {
        pipe = pipe_create(config, pipe_assembly, pipe_render_components, centralizers, arena_h);
    } catch (exception &e) {
        THROW_RUNTIME_ERROR("Failed to create pipe object\n" + string(e.what()));
    }
    try {
        /*--------------------------------------------------------------------
        Save pipe
        --------------------------------------------------------------------*/
        save_pipe_csv(config, pipe, arena_h.buf);
        cout << "Pipe object saved to csv\n";
    } catch (exception &e) {
        throw runtime_error("Failed to write pipe object to csv\n" + string(e.what()));
    }
    return pipe;
}

void InputParser::parse_drill_pipe_component(const Config &config, const YAML::Node &component_node,
                                             PipeComponent &pipe_component) const {

    validate_pipe_component_keys(component_node, component_drill_pipe_keys);

    scalar radial_conv = config.radial_unit_conversion;

    if (component_node["D_tooljoint"] && component_node["L_tooljoint"]) {
        pipe_component.has_tool_joints = true;
        pipe_component.D_tool = component_node["D_tooljoint"].as<scalar>() * radial_conv;
        pipe_component.L_tool = component_node["L_tooljoint"].as<scalar>();
        if (pipe_component.D_tool < pipe_component.Do) {
            THROW_RUNTIME_ERROR("Tool joint diameter cannot be less than the pipe outer diameter");

            if (pipe_component.L_tool < 0.0 || pipe_component.L_tool > pipe_component.L / 2.0) {
                THROW_RUNTIME_ERROR("Tool joint length must be positive and less than half the pipe length");
            }
        }
        if (component_node["mass_imbalance"]) {
            pipe_component.has_mass_imbalance = true;
            scalar dist_frac = component_node["mass_imbalance"].as<scalar>();
            if (dist_frac < 0.0 || dist_frac > 1.0) {
                THROW_RUNTIME_ERROR("Mass imbalance distance fraction must be between 0.0 and 1.0");
            }
            EccentricMass ecc_mass_tj_begin; // Eccentric mass at the first tool joint
            EccentricMass ecc_mass_tj_end;   // Eccentric mass at the last tool joint
            ecc_mass_tj_begin.S_ecc = pipe_component.L_tool / 2.0;
            ecc_mass_tj_end.S_ecc = pipe_component.L - pipe_component.L_tool / 2.0;
            ecc_mass_tj_begin.L_ecc = pipe_component.L_tool;
            ecc_mass_tj_end.L_ecc = pipe_component.L_tool;
            // D_tool already converted to the correct units
            ecc_mass_tj_begin.r_ecc = dist_frac * pipe_component.D_tool / 2.0;
            ecc_mass_tj_end.r_ecc = dist_frac * pipe_component.D_tool / 2.0;
            pipe_component.eccentric_masses.push_back(ecc_mass_tj_begin);
            pipe_component.eccentric_masses.push_back(ecc_mass_tj_end);
        }
    } else if (!component_node["L_tooljoint"] && component_node["D_tooljoint"] ||
               component_node["L_tooljoint"] && !component_node["D_tooljoint"]) {
        THROW_RUNTIME_ERROR("Both tool joint diameter and length must be specified at the same time");
    }

    if (component_node["mass_imbalance"] && !pipe_component.has_tool_joints) {
        THROW_RUNTIME_ERROR("Mass imbalance for drill pipe components is only supported when tool joints are "
                            "specified.");
    }
}

void InputParser::parse_mwd_component(const Config &config, const YAML::Node &component_node,
                                      PipeComponent &pipe_component) const {
    validate_pipe_component_keys(component_node, component_mwd_keys);

    if (component_node["S_sensor"] && component_node["L_sensor"]) {
        pipe_component.has_sensors = true;
        pipe_component.S_sensor = component_node["S_sensor"].as<scalar>();
        pipe_component.L_sensor = component_node["L_sensor"].as<scalar>();
        if (pipe_component.S_sensor < pipe_component.L_sensor / 2.0 ||
            pipe_component.S_sensor > pipe_component.L - pipe_component.L_sensor / 2.0) {
            THROW_RUNTIME_ERROR("Sensor position plus half the sensor length must be within the span of "
                                "the component length");
        }

        if (component_node["mass_imbalance"]) {
            pipe_component.has_mass_imbalance = true;
            scalar dist_frac = component_node["mass_imbalance"].as<scalar>();
            if (dist_frac < 0.0 || dist_frac > 1.0) {
                THROW_RUNTIME_ERROR("Mass imbalance distance fraction must be between 0.0 and 1.0");
            }

            EccentricMass ecc_mass_measurement; // Eccentric mass at the first tool joint
            ecc_mass_measurement.S_ecc = pipe_component.S_sensor;
            ecc_mass_measurement.L_ecc = pipe_component.L_sensor;
            // Do already converted to the correct units
            ecc_mass_measurement.r_ecc = dist_frac * pipe_component.Do / 2.0;
            pipe_component.eccentric_masses.push_back(ecc_mass_measurement);
        }

    } else if (!component_node["S_sensor"] && component_node["L_sensor"] ||
               component_node["S_sensor"] && !component_node["L_sensor"]) {
        THROW_RUNTIME_ERROR("Both sensor position and sensor length must be specified at the same time");
    }

    if (component_node["mass_imbalance"] && !pipe_component.has_sensors) {
        THROW_RUNTIME_ERROR(
            "Sensor mass imbalance for mwd type components is only supported when sensors are specified.");
    }
}

void InputParser::parse_stabilizer_component(const Config &config, const YAML::Node &component_node,
                                             PipeComponent &pipe_component) const {

    validate_pipe_component_keys(component_node, component_stabilizer_keys);

    scalar radial_conv = config.radial_unit_conversion;

    if (component_node["D_stabilizer"] && component_node["L_stabilizer"]) {
        pipe_component.has_stabilizer = true;
        pipe_component.L_stabilizer = component_node["L_stabilizer"].as<scalar>();

        pipe_component.D_stabilizer = component_node["D_stabilizer"].as<scalar>() * radial_conv;
        if (pipe_component.D_stabilizer < pipe_component.Do) {
            THROW_RUNTIME_ERROR("Stabilizer diameter cannot be less than the pipe outer diameter");
        }

        if (component_node["S_stabilizer"]) {
            pipe_component.S_stabilizer = component_node["S_stabilizer"].as<scalar>();

            if (pipe_component.S_stabilizer < 0.0 || pipe_component.S_stabilizer > pipe_component.L) {
                THROW_RUNTIME_ERROR("Stabilizer position must be positive and less than the component length");
            }
        } else {
            pipe_component.S_stabilizer = pipe_component.L / 2.0; // Default to the middle of the component
        }

        if (component_node["mass_imbalance"]) {
            pipe_component.has_mass_imbalance = true;
            scalar dist_frac = component_node["mass_imbalance"].as<scalar>();
            if (dist_frac < 0.0 || dist_frac > 1.0) {
                THROW_RUNTIME_ERROR("Mass imbalance distance fraction must be between 0.0 and 1.0");
            }
            EccentricMass ecc_stabilizer; // Eccentric mass of the stabilizer
            ecc_stabilizer.S_ecc = pipe_component.S_stabilizer;
            ecc_stabilizer.L_ecc = pipe_component.L_stabilizer;
            // D_stabilizer already converted to the correct units
            ecc_stabilizer.r_ecc = dist_frac * pipe_component.D_stabilizer / 2.0;
            pipe_component.eccentric_masses.push_back(ecc_stabilizer);
        }

    } else if ((!component_node["D_stabilizer"] && component_node["L_stabilizer"]) ||
               (component_node["D_stabilizer"] && !component_node["L_stabilizer"])) {
        THROW_RUNTIME_ERROR("Both stabilizer diameter and stabilizer length must be specified at the same time");
    }

    if (component_node["mass_imbalance"] && !pipe_component.has_stabilizer) {
        THROW_RUNTIME_ERROR("Stabilizer mass imbalance is only supported when stabilizer is specified.");
    }
}

void InputParser::parse_sub_component(const Config &config, const YAML::Node &component_node,
                                      PipeComponent &pipe_component) const {
    validate_pipe_component_keys(component_node, component_sub_keys);
    scalar radial_conv = config.radial_unit_conversion;
}

void InputParser::parse_casing_component(const Config &config, const YAML::Node &component_node,
                                         PipeComponent &pipe_component) const {
    if (config.conf_stat.string_type != StringType::CASING_STRING) {
        THROW_RUNTIME_ERROR("Casing components can only be used with casing strings");
    }
    validate_pipe_component_keys(component_node, component_casing_keys);
}

void InputParser::parse_steering_component(const Config &config, const YAML::Node &component_node,
                                           PipeComponent &pipe_component, bool &steering_already_defined) const {
    if (steering_already_defined) {
        THROW_RUNTIME_ERROR("Only one steering component can be specified");
    }
    steering_already_defined = true;
    pipe_component.has_steering = true;

    validate_pipe_component_keys(component_node, component_steering_keys);
    if (config.conf_stat.string_type == StringType::CASING_STRING) {
        THROW_RUNTIME_ERROR("Steering components can not be used with casing strings");
    }
    if (component_node["steering_mode"]) {
        pipe_component.steering_mode = steering_mode_from_string.at(component_node["steering_mode"].as<string>());
    } else {
        printf("Steering mode not specified for steering component");
    }
}

void InputParser::parse_centralizers(Config &config, const vector<PipeComponent> &pipe_assembly,
                                     vector<Centralizer> &centralizers, scalar L_pipe) const {
    assert(yaml_parser != nullptr);
    scalar radial_conv = config.radial_unit_conversion;
    YamlParser &yp = *yaml_parser;

    ConfigStatic &conf_stat = config.conf_stat;

    vector<scalar> centralizers_dS = yp.read_optional_option_vector<scalar>("pipe/centralizer_dS", {});
    vector<uint> centralizers_counts = yp.read_optional_option_vector<uint>("pipe/centralizer_count", {});
    vector<uint> centralizers_types = yp.read_optional_option_vector<uint>("pipe/centralizer_types", {});

    if (centralizers_dS.size() != centralizers_counts.size() || centralizers_dS.size() != centralizers_types.size()) {
        throw runtime_error("Invalid length for the centralizer properties. The vector lengths must match.");
    } else if (centralizers_dS.size() > 0) {
        config.conf_stat.r_bs_outer = yp.read_required_option<scalar>("pipe/bow-spring/D_bs_outer") / 2.0 * radial_conv;
        config.conf_stat.r_bs_inner =
            yp.read_optional_option<scalar>("pipe/bow-spring/D_bs_inner", 0.0) / 2.0 * radial_conv;
        conf_stat.K_bs = yp.read_optional_option<scalar>("pipe/bow-spring/K_bs", 1e6);
        conf_stat.C_bs = yp.read_optional_option<scalar>("pipe/bow-spring/C_bs", 1e3);
        conf_stat.N_bs = yp.read_optional_option<uint>("pipe/bow-spring/N_bs", 4);
        conf_stat.p_bs = yp.read_optional_option<scalar>("pipe/bow-spring/p_bs", 1.0, 1.0, 3.0);

        scalar group_start = L_pipe - centralizers_dS[0] / 2.0; // Start from the bottom
        for (size_t i = 0; i < centralizers_dS.size(); i++) {
            for (size_t j = 0; j < centralizers_counts[i]; j++) {
                Centralizer centralizer;
                centralizer.S_cent = group_start - centralizers_dS[i] * j;
                if (centralizer.S_cent < 0.0 || centralizer.S_cent > L_pipe) {
                    std::cout << "Total length of the casing string: " << L_pipe << " meters" << std::endl;
                    throw std::runtime_error("The position of the centralizer (S_cent) is set outside of the "
                                             "span of casing string.");
                }
                centralizer.type = static_cast<CentralizerType>(centralizers_types[i]);
                centralizers.push_back(centralizer);
            }
            group_start -= centralizers_dS[i] * centralizers_counts[i];
        }
    }

    for (const auto &centralizer : centralizers) {
        scalar pos = centralizer.S_cent;
        scalar L = 0.0;
        bool found = false;
        for (const auto &comp : pipe_assembly) {
            if (pos >= L && pos < L + comp.L) {
                found = true;
                if (comp.type != ComponentType::CASING) {
                    throw std::runtime_error("Centralizer at position " + std::to_string(pos) +
                                             " is not placed on a casing component.");
                }
                break;
            }
            L += comp.L;
        }
        if (!found) {
            throw std::runtime_error("Centralizer at position " + std::to_string(pos) + " is outside the pipe string.");
        }
    }
}

// Validation function
void InputParser::validate_pipe_component_keys(const YAML::Node &component_node, const set<string> &type_keys) const {
    for (auto it = component_node.begin(); it != component_node.end(); ++it) {
        std::string key = it->first.as<std::string>();
        if (component_common_keys.find(key) == component_common_keys.end() && type_keys.find(key) == type_keys.end()) {
            THROW_RUNTIME_ERROR("Invalid key '" + key + "' in pipe component.");
        }
    }
}