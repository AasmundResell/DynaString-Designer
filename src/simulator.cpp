#include "simulator.hpp"
#include "misc/double-buffer-gui.hpp"
#include "misc/input-parser.hpp"
#include <chrono>
#include <filesystem>
#include <thread>

constexpr size_t arena_size = 32 * ONE_MB; // Increased to 32 MB

Simulator::Simulator(string &input_file, ConfigGUI &config_gui, PipeRenderComponents *pipe_render_components) {

    InputParser input_parser{input_file};
    config = input_parser.create_config(config_gui.gui_enabled);
    create_output_dir();

    arena_h.create(MemType::HOST, arena_size);
    if (config.use_gpu && config.pipe_solver_type == PipeSolverType::CURVILINEAR) {
        arena_d.create(MemType::DEVICE, arena_size);
    } else {
        arena_d = ArenaBump{}; // Want internal pointers, etc to be set to default values
    }

    bc_data = create_bcs(arena_h);
    hole = input_parser.create_hole(config, arena_h);
    pipe = input_parser.create_pipe(config, pipe_assembly, pipe_render_components, arena_h);
    bit_rock_data = create_bit_rock(config, arena_h);

    /*----Set number of omp threads -----------------------
    -----------------------------------------------------*/
    omp_set_num_threads(config.num_threads_omp);
#pragma omp parallel
    {
        if (omp_get_thread_num() == 0) {
            printf("Num omp threads: %i\n", omp_get_num_threads());
        }
        printf("Hello from omp thread %i\n", omp_get_thread_num());
    }
    switch (config.pipe_solver_type) {
    case PipeSolverType::CURVILINEAR:
        pipe_solver_curvlin = make_unique<curvlin::PipeSolver>(config, config_dynamic, pipe, pipe_assembly, hole,
                                                               bc_data, bit_rock_data, arena_h, arena_d);
        printf("Running with curvilinear pipe solver\n");
        break;
    case PipeSolverType::COROTATIONAL:
        pipe_solver_corot =
            make_unique<corot::PipeSolver>(config, config_dynamic, pipe, pipe_assembly, hole, bc_data, arena_h);
        printf("Running with corotational pipe solver\n");
        break;
    }

    if (config.initialization) {
        damping_initialization(config_gui);
    }

    print_setup_info();

    if (config_gui.gui_enabled) {
        double_buffer_gui = make_unique<DoubleBufferGUI>(arena_h, config, config_dynamic);
    }

    config.timer.start_counter();
    input_file.clear();
}

bool inline still_running(const Config &config) {
    switch (config.simulation_mode) {
    case SimulationMode::INFINITE_LOOP:
        return true;
    case SimulationMode::MAX_STEPS:
        assert(config.n_max > 0);
        return config.n < config.n_max - 1;
    default:
        assert(config.simulation_mode == SimulationMode::MAX_TIME);
        return config.t <= config.t_max;
    }
}

void Simulator::progress_solver(ConfigGUI &config_gui, std::atomic<bool> &stop_program) {

    config_gui.n_steps_progressed_since_last_frame = 0;
    const double target_hz = 30.0;
    const uint min_steps = std::max(1u, static_cast<uint>(std::ceil(1.0 / (target_hz * config.conf_stat.dt))));

    config_gui.target_time += 1.0 / config_gui.framerate_target;

    while (1) {
        config_gui.n_steps_progressed_since_last_frame++;

        /*================================================================================
         Step the solvers depending on the solver type
        ==================================================================================*/
        step_solver();

        /*================================================================================
         The renderer will attempt to follow the target framerate, and at the same time
         compute as many timesteps as possible. If it lags behind fever solver timesteps
         will be computed. A minimum number of solver steps are specified, if it gets
         below this threshold, the framerate will drop.
        ==================================================================================*/
        bool break_ = false;
        const double elapsed = config.timer.get_elapsed_time_sec();

        const bool elapsed_exceeded = elapsed >= config_gui.target_time;
        if (!still_running(config)) {
            stop_program.store(true);
            break_ = true;
        } else if ((config_gui.n_steps_progressed_since_last_frame >= min_steps) && elapsed_exceeded) {
            break_ = true;
        }
        if (break_) {
            break;
        }
    }

    /*================================================================================
    Update the data in the double buffer gui if it is enabled
    ==================================================================================*/
    if (config_gui.gui_enabled) {
        assert(double_buffer_gui != nullptr);
        const ArenaBump &arena_src = config.use_gpu ? arena_d : arena_h;
        double_buffer_gui->update_from_solver(pipe_solver_curvlin.get(), pipe_solver_corot.get(), pipe, hole,
                                              bit_rock_data, arena_src, config_dynamic, config);
        double_buffer_gui->increment_buffer_version();
    }
}

void Simulator::step_solver() {

    if (config.n % 1000 == 0) {
        // printf("---------------n = %lu---------------\n", config.conf_dyn.n);
        printf("----- n = %lu, t_sim = %6.2f, t_real = %6.2f -----\n", config.n, config.t,
               config.timer.get_elapsed_time_sec());
    }

    switch (config.pipe_solver_type) {
    case PipeSolverType::CURVILINEAR:
        pipe_solver_curvlin->step(config, config_dynamic, pipe, pipe_assembly, hole, bc_data, bit_rock_data, arena_h,
                                  arena_d);
        break;
    case PipeSolverType::COROTATIONAL:
        pipe_solver_corot->step(config, config_dynamic, pipe, pipe_assembly, hole, bc_data, arena_h);
        break;
    default:
        assert(false);
    }

    config.n++;
    config.t = config.n * config.conf_stat.dt;
}

Simulator::~Simulator() {
    arena_h.destroy();
    arena_d.destroy();
    pipe_assembly.clear();
}

void Simulator::restart_solver() {
    assert(false); // Not fully implemented yet, need to parse input file again
    arena_h.destroy();
    arena_d.destroy();

    arena_h.create(MemType::HOST, arena_size);
    if (config.use_gpu && config.pipe_solver_type == PipeSolverType::CURVILINEAR) {
        arena_d.create(MemType::DEVICE, arena_size);
    } else {
        arena_d = ArenaBump{}; // Want internal pointers, etc to be set to default values
    }

    // bc_data = create_bcs(arena_h);
    // hole = input_parser.create_hole(config, arena_h);
    // pipe = input_parser.create_pipe(config, pipe_assembly, pipe_render_components, arena_h);
    // bit_rock_data = create_bit_rock(config, arena_h);

    /*----Set number of omp threads -----------------------
    -----------------------------------------------------*/
    omp_set_num_threads(config.num_threads_omp);
#pragma omp parallel
    {
        if (omp_get_thread_num() == 0) {
            printf("Num omp threads: %i\n", omp_get_num_threads());
        }
        printf("Hello from omp thread %i\n", omp_get_thread_num());
    }
    switch (config.pipe_solver_type) {
    case PipeSolverType::CURVILINEAR:
        pipe_solver_curvlin = make_unique<curvlin::PipeSolver>(config, config_dynamic, pipe, pipe_assembly, hole,
                                                               bc_data, bit_rock_data, arena_h, arena_d);
        printf("Running with curvilinear pipe solver\n");
        break;
    case PipeSolverType::COROTATIONAL:
        pipe_solver_corot =
            make_unique<corot::PipeSolver>(config, config_dynamic, pipe, pipe_assembly, hole, bc_data, arena_h);
        printf("Running with corotational pipe solver\n");
        break;
    }

    print_setup_info();

    config.timer.start_counter();
}

void Simulator::update_constant_data_gpu() {
    if (config.use_gpu) {
        switch (config.pipe_solver_type) {
        case PipeSolverType::CURVILINEAR:
            pipe_solver_curvlin->update_constant_data_gpu(config, pipe, hole, bc_data, bit_rock_data, arena_d);
            break;
        case PipeSolverType::COROTATIONAL:
            assert(false);
            break;
        }
    }
}

void Simulator::damping_initialization(ConfigGUI &config_gui) {
    ConfigDynamic &conf_dyn = config_dynamic;
    ConfigStatic &conf_stat = config.conf_stat;

    SimulationMode sim_mode_tmp = config.simulation_mode;
    scalar t_max_tmp = config.t_max;
    config.simulation_mode = SimulationMode::MAX_TIME; // Run for a fixed time
    config.t_max = config.t_init;                      // Run for the initialization time
    config.t = 0.0;
    config.n = 0;

    bool gui_enabled_tmp = config_gui.gui_enabled;
    config_gui.gui_enabled = false; // Disable gui during initialization

    // Store relevant input values as temporary values
    scalar alpha_tmp = conf_stat.alpha;
    scalar amp_ec_tmp = conf_stat.ec_amplifier;

    scalar v_top_input_tmp = conf_stat.v_top_input;
    scalar omega_top_input_tmp = conf_stat.omega_top_input;
    bool save_csv_tmp = config.save_csv;

    // Intialization values: Set to gurantee damping and stability
    conf_stat.alpha = 10.0;
    conf_stat.ec_amplifier = 0.0;
    conf_stat.v_top_input = 0.0;
    // conf_stat.omega_top_input = 0.0;
    config.save_csv = false;

    BC_BottomAxialKinematics bc_axial_kin_tmp = conf_stat.bc_bottom_axial_kinematics;
    if (bc_axial_kin_tmp == BC_BottomAxialKinematics::FIXED &&
        config.pipe_solver_type == PipeSolverType::CURVILINEAR) { // Calculate static steady state in the bottom
        conf_stat.bc_bottom_axial_kinematics = BC_BottomAxialKinematics::FREE;
    }

    bool bc_detournay_regularized = (conf_stat.bc_bit_rock_type == BC_BitRockType::DETOURNAY_REGULARIZED);
    if (bc_detournay_regularized) {
        conf_stat.bc_bit_rock_type = BC_BitRockType::NO_BIT;
    }

    update_constant_data_gpu();

    cout << "\nRunning solver initialization ...\n";
    std::atomic<bool> dummy_stop{false};

    progress_solver(config_gui, dummy_stop);

    // Reset relevant config values
    config.simulation_mode = sim_mode_tmp;
    config.t_max = t_max_tmp;
    config.t = 0.0;
    config.n = 0;
    config.save_csv = save_csv_tmp;
    config_gui.gui_enabled = gui_enabled_tmp;
    conf_stat.alpha = alpha_tmp;
    conf_stat.ec_amplifier = amp_ec_tmp;
    conf_stat.v_top_input = v_top_input_tmp;
    conf_stat.omega_top_input = omega_top_input_tmp;
    conf_dyn.W_ext = 0.0;
    conf_dyn.W_int = 0.0;
    conf_dyn.KE = 0.0;

    if (bc_detournay_regularized) {
        conf_stat.bc_bit_rock_type = BC_BitRockType::DETOURNAY_REGULARIZED;
    }

    if (bc_axial_kin_tmp == BC_BottomAxialKinematics::FIXED &&
        config.pipe_solver_type == PipeSolverType::CURVILINEAR) { // Calculate static steady state in the bottom
        conf_stat.bc_bottom_axial_kinematics = BC_BottomAxialKinematics::FIXED;
        const ArrayView<Vec3> &u = pipe_solver_curvlin->beam.get_field<curvlin::BeamField::u>(arena_h.buf);
        conf_stat.L_string_depth_initial = u[pipe.N - 1].x();
    }

    update_constant_data_gpu();
}

void Simulator::sleep_solver(bool enforce_real_time) {

    if (!enforce_real_time) { // When running without a GUI
        return;
    }

    double elapsed = config.timer.get_elapsed_time_sec();
    double simulated_time = config.t; // current simulation time
    double sleep_sec = simulated_time - elapsed;
    if (sleep_sec > 0.0) { // Enforce maximum real-time:
        auto dur = std::chrono::duration<double>(sleep_sec);
        std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(dur));
    } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

const ConfigDynamic &Simulator::get_config_dynamic_gui() const {
    return double_buffer_gui->get_config_dynamic();
}

void Simulator::stop_timer() {
    config.timer.stop_counter();

    config.timer.print_elapsed_time();
    const scalar elapsed_time_sec = config.timer.get_elapsed_time_sec();
    assert(config.t > 0.0);
    const scalar time_ratio = config.t / elapsed_time_sec;
    cout << "Simulation was " << time_ratio << " times faster than real time\n";
}

void Simulator::print_setup_info() {

    if (config.use_gpu) {
        cout << "\n-----------------Starting GPU simulation----------------\n";
    } else {
        cout << "\n-----------------Starting CPU simulation----------------\n";
    }

    switch (config.pipe_solver_type) {
    case PipeSolverType::CURVILINEAR:
        printf("Pipe solver type: Curvilinear\n");
        break;
    case PipeSolverType::COROTATIONAL:
        printf("Pipe solver type: Corotational\n");
        break;
    default:
        assert(false);
    }

    switch (config.simulation_mode) {
    case SimulationMode::INFINITE_LOOP:
        printf("Simulation will run indefinitely until it's terminated by the user\n");
        break;
    case SimulationMode::MAX_STEPS:
        printf("Simulation will run for %lu timesteps\n", config.n_max);
        break;
    case SimulationMode::MAX_TIME:
        printf("Simulation will run for a total time of %.2f seconds\n", config.t_max);
        break;
    default:
        assert(false);
    }
    printf("Delta time = %f s\n", config.conf_stat.dt);
    printf("--------------------------------------------------------\n");
}

void Simulator::create_output_dir() {
    using namespace std;

    path &base_dir = config.base_dir;
    string output_dir_name = config.simulation_name;
    config.output_dir = base_dir / output_dir_name;

    if (filesystem::exists(config.output_dir))
        if (!filesystem::remove_all(config.output_dir)) {
            throw runtime_error{"Couldn't remove old output directory: " + config.output_dir.string()};
        }
    if (!filesystem::create_directory(config.output_dir)) {
        throw runtime_error{"Failed to create output directory: " + config.output_dir.string() + "\n"};
    }
}
