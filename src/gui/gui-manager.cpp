#include "gui-manager.hpp"
#include "frame-data.hpp"

constexpr uint SCREEN_WIDTH_DEFAULT = 1920;
constexpr uint SCREEN_HEIGHT_DEFAULT = 1080;
uint screen_width = SCREEN_WIDTH_DEFAULT;
uint screen_height = SCREEN_HEIGHT_DEFAULT;
constexpr float scale_factor = 1.1f;
bool wireframe_mode = false;
float radial_scale = 1.0f;

void framebuffer_size_callback(GLFWwindow *window, int width, int height);
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);
void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods);
// void mouse_button_callback(GLFWwindow *window, int button, int action, int mods);
void cursor_pos_callback(GLFWwindow *window, double cursor_x, double cursor_y);

constexpr bool check_misc_spatial_graphs_order(const std::array<MiscSpatialGraph, N_MG> &arr) {
    for (size_t i = 0; i < arr.size(); ++i) {
        if (static_cast<size_t>(arr[i].variable) != i) {
            return false;
        }
    }
    return true;
}
GUI_Manager::GUI_Manager() : spatial_graphs{
                         SpatialGraph(PlotSpatialVariable::DISPLACEMENT_TRANS, Direction::X, 0.001f, "u", "Displacement", "[m]"),
                         SpatialGraph(PlotSpatialVariable::DISPLACEMENT_TRANS, Direction::Y, 0.001f, "u", "Displacement", "[m]"),
                         SpatialGraph(PlotSpatialVariable::DISPLACEMENT_TRANS, Direction::Z, 0.001f, "u", "Displacement", "[m]"),

                         SpatialGraph(PlotSpatialVariable::DISPLACEMENT_ROT, Direction::X, 0.001f, "theta", "Rotation", "[rad]"),
                         SpatialGraph(PlotSpatialVariable::DISPLACEMENT_ROT, Direction::Y, 0.001f, "theta", "Rotation", "[rad]"),
                         SpatialGraph(PlotSpatialVariable::DISPLACEMENT_ROT, Direction::Z, 0.001f, "theta", "Rotation", "[rad]"),

                         SpatialGraph(PlotSpatialVariable::VELOCITY_TRANS, Direction::X, 0.001f, "v", "Translational velocity", "[m/s]"),
                         SpatialGraph(PlotSpatialVariable::VELOCITY_TRANS, Direction::Y, 0.001f, "v", "Translational velocity", "[m/s]"),
                         SpatialGraph(PlotSpatialVariable::VELOCITY_TRANS, Direction::Z, 0.001f, "v", "Translational velocity", "[m/s]"),

                         SpatialGraph(PlotSpatialVariable::VELOCITY_ROT, Direction::X, 0.001f, "omega", "Rotational velocity", "[rad/s]"),
                         SpatialGraph(PlotSpatialVariable::VELOCITY_ROT, Direction::Y, 0.001f, "omega", "Rotational velocity", "[rad/s]"),
                         SpatialGraph(PlotSpatialVariable::VELOCITY_ROT, Direction::Z, 0.001f, "omega", "Rotational velocity", "[rad/s]"),

                         SpatialGraph(PlotSpatialVariable::ACCELERATION_TRANS, Direction::X, 0.001f, "a", "Translational acceleration", "[m/s^2]"),
                         SpatialGraph(PlotSpatialVariable::ACCELERATION_TRANS, Direction::Y, 0.001f, "a", "Translational acceleration", "[m/s^2]"),
                         SpatialGraph(PlotSpatialVariable::ACCELERATION_TRANS, Direction::Z, 0.001f, "a", "Translational acceleration", "[m/s^2]"),

                         SpatialGraph(PlotSpatialVariable::ACCELERATION_ROT, Direction::X, 0.001f, "alpha", "Rotational acceleration", "[rad/s^2]"),
                         SpatialGraph(PlotSpatialVariable::ACCELERATION_ROT, Direction::Y, 0.001f, "alpha", "Rotational acceleration", "[rad/s^2]"),
                         SpatialGraph(PlotSpatialVariable::ACCELERATION_ROT, Direction::Z, 0.001f, "alpha", "Rotational acceleration", "[rad/s^2]"),

                         SpatialGraph(PlotSpatialVariable::FORCE_INT, Direction::X, 1.0f, "f_inner", "Force inner", "[N]"),
                         SpatialGraph(PlotSpatialVariable::FORCE_INT, Direction::Y, 1.0f, "f_inner", "Force inner", "[N]"),
                         SpatialGraph(PlotSpatialVariable::FORCE_INT, Direction::Z, 1.0f, "f_inner", "Force inner", "[N]"),

                         SpatialGraph(PlotSpatialVariable::MOMENT_INT, Direction::X, 0.0001f, "m_inner", "Moment inner", "[Nm]"),
                         SpatialGraph(PlotSpatialVariable::MOMENT_INT, Direction::Y, 0.0001f, "m_inner", "Moment inner", "[Nm]"),
                         SpatialGraph(PlotSpatialVariable::MOMENT_INT, Direction::Z, 0.0001f, "m_inner", "Moment inner", "[Nm]"),

                         SpatialGraph(PlotSpatialVariable::FORCE_EXT_DYN, Direction::X, 1.0f, "f_dynamic", "Force dynamic", "[N]"),
                         SpatialGraph(PlotSpatialVariable::FORCE_EXT_DYN, Direction::Y, 1.0f, "f_dynamic", "Force dynamic", "[N]"),
                         SpatialGraph(PlotSpatialVariable::FORCE_EXT_DYN, Direction::Z, 1.0f, "f_dynamic", "Force dynamic", "[N]"),

                         SpatialGraph(PlotSpatialVariable::MOMENT_EXT_DYN, Direction::X, 1.0f, "m_dynamic", "Moment dynamic", "[Nm]"),
                         SpatialGraph(PlotSpatialVariable::MOMENT_EXT_DYN, Direction::Y, 1.0f, "m_dynamic", "Moment dynamic", "[Nm]"),
                         SpatialGraph(PlotSpatialVariable::MOMENT_EXT_DYN, Direction::Z, 1.0f, "m_dynamic", "Moment dynamic", "[Nm]"),

                         SpatialGraph(PlotSpatialVariable::FORCE_EXT_STAT, Direction::X, 1.0f, "f_static", "Force static", "[N]"),
                         SpatialGraph(PlotSpatialVariable::FORCE_EXT_STAT, Direction::Y, 1.0f, "f_static", "Force static", "[N]"),
                         SpatialGraph(PlotSpatialVariable::FORCE_EXT_STAT, Direction::Z, 1.0f, "f_static", "Force static", "[N]"),

                         SpatialGraph(PlotSpatialVariable::MOMENT_EXT_STAT, Direction::X, 1.0f, "m_static", "Moment static", "[Nm]"),
                         SpatialGraph(PlotSpatialVariable::MOMENT_EXT_STAT, Direction::Y, 1.0f, "m_static", "Moment static", "[Nm]"),
                         SpatialGraph(PlotSpatialVariable::MOMENT_EXT_STAT, Direction::Z, 1.0f, "m_static", "Moment static", "[Nm]"),
                     
                     },
                     misc_spatial_graphs{
                        MiscSpatialGraph{MiscPlotSpatialVariable::AXIAL_TENSION, ContourPlotVariable::AXIAL_TENSION, false, "True axial tension", "Axial tension", "[tonnes]"},
                        MiscSpatialGraph{MiscPlotSpatialVariable::EFFECTIVE_TENSION, ContourPlotVariable::EFFECTIVE_TENSION, false, "Effective axial tension", "Axial tension", "[tonnes]"},
                        MiscSpatialGraph{MiscPlotSpatialVariable::EFFECTIVE_TORQUE, ContourPlotVariable::EFFECTIVE_TORQUE, false, "Effective torque", "Torque", "[kNm]"},
                        MiscSpatialGraph{MiscPlotSpatialVariable::RADIAL_ACCELERATION, ContourPlotVariable::RADIAL_ACCELERATION, true, "Radial acceleration", "Radial acceleration", "[g]"},
                        MiscSpatialGraph{MiscPlotSpatialVariable::TANGENTIAL_ACCELERATION, ContourPlotVariable::TANGENTIAL_ACCELERATION, true, "Tangential acceleration", "Tangential acceleration", "[g]"},
                        MiscSpatialGraph{MiscPlotSpatialVariable::AXIAL_STRAIN, ContourPlotVariable::AXIAL_STRAIN, true, "Axial strain", "Axial strain", "[-]"},
                        MiscSpatialGraph{MiscPlotSpatialVariable::STANDOFF_RATIO, ContourPlotVariable::STANDOFF_RATIO, false, "Standoff ratio", "Standoff ratio", "[%]", 0.0f, 100.0f},
                        MiscSpatialGraph{MiscPlotSpatialVariable::VON_MISES_STRESS, ContourPlotVariable::VON_MISES_STRESS, true, "Von mises stress ", "Stress", "[MPa]"},
                        MiscSpatialGraph{MiscPlotSpatialVariable::LINEAR_CONTACT_FORCE, ContourPlotVariable::LINEAR_CONTACT_FORCE, false, "Linear contact force", "Contact force", "[N/m]"},
                        MiscSpatialGraph{MiscPlotSpatialVariable::TANGENTIAL_CONTACT_FORCE, ContourPlotVariable::TANGENTIAL_CONTACT_FORCE, false, "Tangential contact force", "Contact force", "[N/m]"}
                    },
                    fluid_spatial_graphs{
                        FluidSpatialGraph{FluidPlotSpatialVariable::PRESSURE_INNER, ContourPlotVariable::PRESSURE_INNER, "Inner pressure", "Pressure", "[MPa]"},
                        FluidSpatialGraph{FluidPlotSpatialVariable::PRESSURE_OUTER, ContourPlotVariable::PRESSURE_OUTER, "Outer pressure", "Pressure", "[MPa]"},
                        FluidSpatialGraph{FluidPlotSpatialVariable::VELOCITY_INNER, ContourPlotVariable::VELOCITY_INNER, "Inner velocity", "Velocity", "[m/s]"},
                        FluidSpatialGraph{FluidPlotSpatialVariable::VELOCITY_OUTER, ContourPlotVariable::VELOCITY_OUTER, "Outer velocity", "Velocity", "[m/s]"},
                        FluidSpatialGraph{FluidPlotSpatialVariable::dPRESSURE_INNER, ContourPlotVariable::NONE , "Inner pressure gradient", "Pressure grad", "[MPa/m]"},
                        FluidSpatialGraph{FluidPlotSpatialVariable::dPRESSURE_OUTER, ContourPlotVariable::NONE, "Outer pressure gradient", "Pressure grad", "[MPa/m]"},
                    } {

    assert(check_misc_spatial_graphs_order(misc_spatial_graphs) &&
           "misc_spatial_graphs order does not match MiscPlotSpatialVariable enum order!");

    if (!glfwInit()) {
        THROW_RUNTIME_ERROR("Failed to initialize GLFW");
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4); // Enable 4x MSAA
#ifndef NDEBUG
    /*Request debug context*/
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, true);
#endif

    {
        GLFWmonitor *primaryMonitor = glfwGetPrimaryMonitor();
        const GLFWvidmode *mode = glfwGetVideoMode(primaryMonitor);
        screen_width = mode->width;
        screen_height = mode->height;
    }

    // Create a fullscreen window
    window = glfwCreateWindow(screen_width, screen_height, "DynaString", nullptr, nullptr);

    if (!window) {
        glfwTerminate();
        THROW_RUNTIME_ERROR("Failed to create GLFW window");
    }

    glfwMaximizeWindow(window);
    glfwMakeContextCurrent(window);

    // Set callback functions
    // glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_pos_callback);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetKeyCallback(window, key_callback);
    glfwSetWindowUserPointer(window, this);

    // Enable V-Sync (Reenabled after multithreading was added). This should ensure 60 fps if the monitor supports it
    glfwSwapInterval(1);

    // Initialize GLEW
    if (glewInit() != GLEW_OK) {
        THROW_RUNTIME_ERROR("Failed to initialize GLEW");
    }

/*Setup debug callback. */
#ifndef NDEBUG
    int flags;
    glGetIntegerv(GL_CONTEXT_FLAGS, &flags);
    if (flags & GL_CONTEXT_FLAG_DEBUG_BIT) {
        glEnable(GL_DEBUG_OUTPUT);
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        glDebugMessageCallback(gl_debug_output, nullptr);
        glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE);
    } else {
        printf("DEBUG CONTEXT SETUP FAILED\n");
        assert(false);
    }
#endif

    frame_data.width = screen_width;
    frame_data.height = screen_height;

    glfwSetWindowUserPointer(window, this);

    glEnable(GL_MULTISAMPLE);

    // After glewInit()
    printf("OpenGL version: %s\n", glGetString(GL_VERSION));
    printf("GLSL version: %s\n", glGetString(GL_SHADING_LANGUAGE_VERSION));

    ui_manager = make_unique<UI_Manager>(window, screen_width, screen_height);

    ui_manager->call_load_sidebar_textures(
        {"../resources/category_button_labels/home.png", "../resources/category_button_labels/settings.png",
         "../resources/category_button_labels/graphics.png", "../resources/category_button_labels/graphs.png",
         "../resources/category_button_labels/well_design.png"});
}

void GUI_Manager::init_simulation(Simulator *simulator, PipeRenderComponents pipe_render_components) {

    const Pipe &pipe = simulator->pipe;
    const Hole &hole = simulator->hole;
    Config &config = simulator->config;

    const uint N = pipe.N;
    const uint Nf = N + 1;
    x_pipe.resize(N);
    q_pipe.resize(N);
    for (uint i = 0; i < N; ++i) {
        x_pipe[i] = Vec3::Zero();
        q_pipe[i].q0 = 1.0;
        q_pipe[i].q = Vec3::Zero();
    }

    for (SpatialGraph &graph : spatial_graphs) {
        graph.data.resize(N);
    }
    for (MiscSpatialGraph &misc_graph : misc_spatial_graphs) {
        if (misc_graph.elementwise) { // N - 1 elements + 2 boundary nodes
            misc_graph.data.resize(N + 1);
        } else {
            misc_graph.data.resize(N);
        }
    }
    for (FluidSpatialGraph &fluid_graph : fluid_spatial_graphs) {
        fluid_graph.data.resize(Nf);
    }

    ui_manager->init_simulation(simulator, spatial_graphs, misc_spatial_graphs, fluid_spatial_graphs);

    graphics_manager =
        make_unique<GraphicsManager>(config, config_graphics, hole, pipe, pipe_render_components,
                                     simulator->pipe_assembly, simulator->double_buffer_gui->get_arena_h().buf);
    printf("GUI created\n");
}

void GUI_Manager::render(Simulator *simulator, string *input_file, std::atomic<bool> &stop_program,
                         std::atomic<bool> &start_simulator, std::atomic<bool> &restart_solver) {
    assert(start_simulator == false);
    if (glfwWindowShouldClose(window)) {
        stop_program.store(true);
        glfwDestroyWindow(window);
        glfwTerminate();
        return;
    }

    update_frame_data_polling(window);

    ui_manager->new_frame();
    if (simulator != nullptr) {
        uint current_version = simulator->double_buffer_gui->get_buffer_version();
        if (current_version != last_processed_version) {
            process_simulator(simulator);
            last_processed_version = current_version;
        }
    }

    render_graphics(simulator);
    render_graphs(simulator);
    render_well_design(simulator);

    ui_manager->call_imgui_popup_message_with_cooldown("The entire pipe assembly has been fed into the hole",
                                                       &ui_manager->ui_settings.pipe_fully_fed_message,
                                                       &simulator->config.pipe_fully_fed);
    ui_manager->call_imgui_popup_message_with_cooldown("The entire pipe assembly has been pulled out of the hole",
                                                       &ui_manager->ui_settings.pipe_fully_pulled_message,
                                                       &simulator->config.pipe_fully_pulled);

    ImGui::End(); // End dockspace
    ui_manager->render_side_menu_bar(simulator, config_graphics, input_file, start_simulator, restart_solver,
                                     spatial_graphs, misc_spatial_graphs, fluid_spatial_graphs);
    ui_manager->render_imgui();

    glfwSwapBuffers(window);
    glfwPollEvents();

    if (simulator != nullptr) {
        simulator->update_constant_data_gpu();
    }
}

void GUI_Manager::process_simulator(Simulator *simulator) {
    // ================================================
    // Process the simulator data for visualization and
    // update the UI graphs accordingly
    // ================================================

    Config &config = simulator->config;
    const Hole &hole = simulator->hole;
    const Pipe &pipe = simulator->pipe;
    const curvlin::PipeSolver *solver_curvlin = simulator->pipe_solver_curvlin.get();
    const corot::PipeSolver *solver_corot = simulator->pipe_solver_corot.get();
    const ArenaBump &arena_h_gui = simulator->double_buffer_gui->get_arena_h();

    if (config.pipe_solver_type == PipeSolverType::CURVILINEAR) {
        const BitRockData &bit_rock_data = simulator->bit_rock_data;
        transform_solution_to_global_frame_curvlin(hole, pipe, *solver_curvlin, config.top_node,
                                                   config_graphics.radial_scale, arena_h_gui);
    } else {
        assert(config.pipe_solver_type == PipeSolverType::COROTATIONAL && solver_corot != nullptr);
        assert(!config.use_gpu);

        transform_solution_to_global_frame_corot(hole, pipe, *solver_corot, config.top_node,
                                                 config_graphics.radial_scale, arena_h_gui);
    }

    update_spatial_graphs(simulator);
}

void GUI_Manager::render_well_design(Simulator *simulator) {
    ImGui::Begin("Well Design Information");
    if (simulator != nullptr) {
        ui_manager->call_imgui_hole_quantities_control_and_plot_window();
        ui_manager->call_imgui_display_pipe_data(simulator->pipe_assembly);
    }
    ImGui::End();
}

void GUI_Manager::render_graphics(Simulator *simulator) {

    ImGui::Begin("World View"); // consider moving this to gui manager

    if (simulator != nullptr) {
        graphics_manager->render(window, simulator->config, config_graphics, simulator->get_config_dynamic_gui(),
                                 simulator->pipe_solver_curvlin.get(), simulator->hole, simulator->pipe,
                                 simulator->pipe_assembly, x_pipe, q_pipe, misc_spatial_graphs, fluid_spatial_graphs,
                                 simulator->double_buffer_gui->get_arena_h().buf);
    }
    ImGui::End();
}

void GUI_Manager::render_graphs(Simulator *simulator) {
    ImGui::Begin("Graph plots", nullptr, ImGuiWindowFlags_NoCollapse);
    if (simulator != nullptr && simulator->pipe_solver_curvlin != nullptr) {
        ui_manager->call_graph_plot_window(simulator->config, simulator->get_config_dynamic_gui(),
                                           *simulator->pipe_solver_curvlin, spatial_graphs, misc_spatial_graphs,
                                           fluid_spatial_graphs, simulator->pipe, simulator->hole,
                                           simulator->double_buffer_gui->get_arena_h().buf, simulator->bit_rock_data);
    }
    ImGui::End();
}

GUI_Manager::~GUI_Manager() {}

void GUI_Manager::transform_solution_to_global_frame_curvlin(const Hole &hole, const Pipe &pipe,
                                                             const curvlin::PipeSolver &solver_curvlin, uint top_node,
                                                             float r_scale, const ArenaBump &arena_h) {

    byte *buf = arena_h.buf;
    const curvlin::BeamData &beam = solver_curvlin.beam;
    ArrayView<Vec3> u = beam.get_field<curvlin::BeamField::u>(buf);
    ArrayView<Vec3> theta = beam.get_field<curvlin::BeamField::theta>(buf);
    ArrayView<uint> i_pipe_to_ie_hole = beam.get_field<curvlin::BeamField::i_pipe_to_ie_hole>(buf);
    ArrayView<scalar> s = hole.get_field<HoleField::s>(buf);
    ArrayView<Quaternion> q = hole.get_field<HoleField::q>(buf);
    ArrayView<Vec3> x = hole.get_field<HoleField::x>(buf);

    const uint N = pipe.N;

#pragma omp parallel for
    for (uint i = top_node; i < N; i++) {
        const uint ie_h = i_pipe_to_ie_hole[i];
        const scalar ux = u[i].x();
        const scalar uy = u[i].y();
        const scalar uz = u[i].z();
        const scalar thetax = theta[i].x();
        const scalar thetay = theta[i].y();
        const scalar thetaz = theta[i].z();

        const scalar s_i = pipe.calc_s(i, ux, buf);
        const scalar s_hole_i = s[ie_h];
        const scalar s_hole_ip = s[ie_h + 1];

        // Temp, should not have to do this
        const scalar t = max(0.0, min(1.0, (s_i - s_hole_i) / (s_hole_ip - s_hole_i)));

        const Quaternion &q_hole_ih = q[ie_h];
        const Quaternion &q_hole_ihp = q[ie_h + 1];
        const Quaternion q_i = Quaternion::lerp(q_hole_ih, q_hole_ihp, t);

        const Vec3 u_l = Vec3{0.0, uy * r_scale, uz * r_scale};

        // Global position vector of hole centerline at pipe node
        Vec3 x_pipe_holecenter =
            hole.lerp_hole_property_from_pipe_node_position<Vec3>(x, s_i, ie_h, buf); // Negative sign out of the hole
        const Vec3 x_p = x_pipe_holecenter + q_i.rotate_vector(u_l);                  // Displacements global frame

        // This operation should be optimized with quaternions
        const Mat3 R_i = q_i.to_matrix();
        // clang-format off
        Mat3 Rz;
        Rz << cos(thetaz), -sin(thetaz), 0,
              sin(thetaz),  cos(thetaz), 0,
              0,            0,           1;
            
        Mat3 Ry;
        Ry << cos(thetay), 0, sin(thetay),
              0,      1,       0,
             -sin(thetay), 0, cos(thetay);
        
        Mat3 Rx;
        Rx << 1, 0,            0, 
              0, cos(thetax), -sin(thetax),
              0, sin(thetax),  cos(thetax);
        // clang-format on

        Mat3 R_pipe = Rz * Ry * Rx;

        Mat3 U = R_i * R_pipe;
        Quaternion q_p;
        q_p.from_matrix(U);

        x_pipe[i] = x_p;
        q_pipe[i] = q_p;
    }
}

void GUI_Manager::transform_solution_to_global_frame_corot(const Hole &hole, const Pipe &pipe,
                                                           const corot::PipeSolver &solver_corot, uint top_node,
                                                           float r_scale, const ArenaBump &arena_h) {
    const vector<Vec3> &X = solver_corot.beam.X;
    const vector<Vec3> &d_trans = solver_corot.beam.d_trans;
    const vector<Quaternion> &d_rot = solver_corot.beam.d_rot;
    const vector<uint> &i_pipe_to_ie_hole = solver_corot.beam.i_pipe_to_ie_hole;
    const ArrayView<Vec3> x_hole = hole.get_field<HoleField::x>(arena_h.buf);
    const uint N = pipe.N;
    assert(N == pipe.N);
    if (is_close(r_scale, 1.0)) {
        // In the case that radial scale is 1, the solution needs no special treatment, just copy it over directly
#pragma omp parallel for
        for (uint i = top_node; i < N; i++) {
            x_pipe[i] = X[i] + d_trans[i];
            q_pipe[i] = d_rot[i];
        }
    } else {
#pragma omp parallel for
        for (uint i = top_node; i < N; i++) {
            const uint ie_h = i_pipe_to_ie_hole[i];
            assert(ie_h < hole.N_hole - 1);
            const Vec3 x_p = X[i] + d_trans[i];
            const Vec3 &x_hole_A = x_hole[ie_h];
            const Vec3 &x_hole_B = x_hole[ie_h + 1];
            const Vec3 t = (x_hole_B - x_hole_A).normalized();
            const Vec3 x_center = x_hole_A + (x_p - x_hole_A).dot(t) * t;
            Vec3 u_lateral = (x_p - x_center);

            u_lateral *= r_scale;
            x_pipe[i] = x_center + u_lateral;
            q_pipe[i] = d_rot[i];
        }
    }
}

void cursor_pos_callback(GLFWwindow *window, double cursor_x, double cursor_y) {

    if (frame_data.first_mouse) {
        frame_data.cursor_x_old = (float)cursor_x;
        frame_data.cursor_y_old = (float)cursor_y;
        frame_data.first_mouse = false;
    } else if (frame_data.mouse_is_over_graphics_window) {
        float delta_cursor_x = (float)cursor_x - frame_data.cursor_x_old;
        float delta_cursor_y = frame_data.cursor_y_old - (float)cursor_y;
        frame_data.cursor_x_old = (float)cursor_x;
        frame_data.cursor_y_old = (float)cursor_y;
        frame_data.delta_cursor_x_frame += delta_cursor_x;
        frame_data.delta_cursor_y_frame += delta_cursor_y;
    }
}

static int last_monitor_width = 0;
static int last_monitor_height = 0;

void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
    glViewport(0, 0, width, height);

    // Get current monitor resolution
    GLFWmonitor *monitor = glfwGetWindowMonitor(window);
    if (!monitor)
        monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode *mode = glfwGetVideoMode(monitor);

    // Only rescale if monitor resolution changed
    if (mode->width != last_monitor_width || mode->height != last_monitor_height) {
        last_monitor_width = mode->width;
        last_monitor_height = mode->height;

        GUI_Manager *gui_manager = static_cast<GUI_Manager *>(glfwGetWindowUserPointer(window));
        if (gui_manager && gui_manager->ui_manager) {
            gui_manager->ui_manager->ui_utils.set_scale(mode->width, mode->height);
            gui_manager->ui_manager->imgui_manager->apply_imgui_screen_scaling(mode->width, mode->height);
        }
    }

    frame_data.width = width;
    frame_data.height = height;
}

// Add this function definition at the end of the file
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
    if (!frame_data.mouse_is_over_graphics_window)
        return;
    frame_data.scroll_y_offset = yoffset;
}

void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods) {

    /*================================================================================
     //ÅSMUND: Fjern hele denne funksjonen. Alle vanlige knapper oppdateres i update_frame_data_polling
     (legg til knapper etter behov)
     Fjern globale variable som wireframe_mode og radial_scale, og flytt dem til feks config_graphics
     oppdater dem fra dataen i frame_data
    ==================================================================================*/
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) {
        case GLFW_KEY_F:
            frame_data.fps_mode = !frame_data.fps_mode;
            break;
        case GLFW_KEY_X:
            // TODO: this does not work, now wireframe mode is set using the config object, maybe it should be part of
            // frame_data
            wireframe_mode = !wireframe_mode;
            break;
        case GLFW_KEY_Q: // '+' key
            radial_scale *= scale_factor;
            break;
        case GLFW_KEY_E: // '-' key
            radial_scale *= 1.0f / scale_factor;
            radial_scale = max(1.0f, radial_scale); // Ensure we don't scale below 1.0
            break;
        case GLFW_KEY_F11: // Add this case
        {
            static bool is_fullscreen = false;
            is_fullscreen = !is_fullscreen;
            GLFWmonitor *primary = glfwGetPrimaryMonitor();
            const GLFWvidmode *mode = glfwGetVideoMode(primary);
            if (is_fullscreen) {
                // Instead of true fullscreen, create a borderless window that covers the screen
                glfwSetWindowAttrib(window, GLFW_DECORATED, GLFW_FALSE);
                glfwSetWindowPos(window, 0, 0);
                glfwSetWindowSize(window, mode->width, mode->height);
            } else {
                // Return to normal windowed mode
                glfwSetWindowAttrib(window, GLFW_DECORATED, GLFW_TRUE);
                glfwSetWindowPos(window, 50, 50);
                glfwSetWindowSize(window, screen_width, screen_height);
            }
        } break;
        }
    }
}

void update_frame_data_polling(GLFWwindow *window) {
    /*Check if window should close*/
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }

    {
        /*Update button pressing. Only valid if mouse is over graphics window*/

        frame_data.is_pressed[(uint)Button::LEFT_MOUSE] =
            glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS &&
            frame_data.mouse_is_over_graphics_window;
        frame_data.is_pressed[(uint)Button::MIDDLE_MOUSE] =
            glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS &&
            frame_data.mouse_is_over_graphics_window;
        frame_data.is_pressed[(uint)Button::RIGHT_MOUSE] =
            glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS &&
            frame_data.mouse_is_over_graphics_window;

        frame_data.is_pressed[(uint)Button::W] = glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS;
        frame_data.is_pressed[(uint)Button::S] = glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS;
        frame_data.is_pressed[(uint)Button::A] = glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS;
        frame_data.is_pressed[(uint)Button::D] = glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS;
        frame_data.is_pressed[(uint)Button::E] = glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS;
        frame_data.is_pressed[(uint)Button::Q] = glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS;
        frame_data.is_pressed[(uint)Button::F] = glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS;
        frame_data.is_pressed[(uint)Button::SPACE] = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
        frame_data.is_pressed[(uint)Button::LEFT_SHIFT] = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
    }

    {
        /*Update delta time*/
        const double current_time = (double)glfwGetTime();
        frame_data.delta_time = current_time - frame_data.last_frame;
        frame_data.last_frame = current_time;
    }
}

void GUI_Manager::update_spatial_graphs_curvlin(const Pipe &pipe, const curvlin::PipeSolver &solver_curvlin,
                                                const uint top_node, const byte *buf) {
    using namespace curvlin;

    const BeamData &beam = solver_curvlin.beam;
    const FluidData &fluid = solver_curvlin.fluid;

    const ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);
    const ArrayView<Vec3> v = beam.get_field<BeamField::v>(buf);
    const ArrayView<Vec3> a = beam.get_field<BeamField::a>(buf);
    const ArrayView<Vec3> theta = beam.get_field<BeamField::theta>(buf);
    const ArrayView<Vec3> omega = beam.get_field<BeamField::omega>(buf);
    const ArrayView<Vec3> alpha = beam.get_field<BeamField::alpha>(buf);
    const ArrayView<Vec3> f_int = beam.get_field<BeamField::f_int>(buf);
    const ArrayView<Vec3> f_dyn = beam.get_field<BeamField::f_dyn>(buf);
    const ArrayView<Vec3> f_hyd = beam.get_field<BeamField::f_hyd>(buf);
    const ArrayView<Vec3> m_int = beam.get_field<BeamField::m_int>(buf);
    const ArrayView<Vec3> m_dyn = beam.get_field<BeamField::m_dyn>(buf);
    const ArrayView<Vec3> m_hyd = beam.get_field<BeamField::m_hyd>(buf);
    const ArrayView<scalar> p_i = fluid.get_field<FluidField::p_i>(buf);
    const ArrayView<scalar> p_o = fluid.get_field<FluidField::p_o>(buf);
    const ArrayView<scalar> v_i = fluid.get_field<FluidField::v_i>(buf);
    const ArrayView<scalar> v_o = fluid.get_field<FluidField::v_o>(buf);
    const ArrayView<scalar> dp_i = fluid.get_field<FluidField::dp_i>(buf);
    const ArrayView<scalar> dp_o = fluid.get_field<FluidField::dp_o>(buf);
    const ArrayView<scalar> S = pipe.get_field<PipeField::S>(buf);

    uint i;
    const uint N = pipe.N;

    vector<scalar> &s_nw = ui_manager->s_nodewise;
    vector<scalar> &s_el = ui_manager->s_elementwise;

    vector<scalar> &u_x = spatial_graphs[(uint)PlotSpatialVariable::DISPLACEMENT_TRANS * 3 + (uint)Direction::X].data;
    vector<scalar> &u_y = spatial_graphs[(uint)PlotSpatialVariable::DISPLACEMENT_TRANS * 3 + (uint)Direction::Y].data;
    vector<scalar> &u_z = spatial_graphs[(uint)PlotSpatialVariable::DISPLACEMENT_TRANS * 3 + (uint)Direction::Z].data;
    vector<scalar> &theta_x = spatial_graphs[(uint)PlotSpatialVariable::DISPLACEMENT_ROT * 3 + (uint)Direction::X].data;
    vector<scalar> &theta_y = spatial_graphs[(uint)PlotSpatialVariable::DISPLACEMENT_ROT * 3 + (uint)Direction::Y].data;
    vector<scalar> &theta_z = spatial_graphs[(uint)PlotSpatialVariable::DISPLACEMENT_ROT * 3 + (uint)Direction::Z].data;

    vector<scalar> &v_x = spatial_graphs[(uint)PlotSpatialVariable::VELOCITY_TRANS * 3 + (uint)Direction::X].data;
    vector<scalar> &v_y = spatial_graphs[(uint)PlotSpatialVariable::VELOCITY_TRANS * 3 + (uint)Direction::Y].data;
    vector<scalar> &v_z = spatial_graphs[(uint)PlotSpatialVariable::VELOCITY_TRANS * 3 + (uint)Direction::Z].data;
    vector<scalar> &omega_x = spatial_graphs[(uint)PlotSpatialVariable::VELOCITY_ROT * 3 + (uint)Direction::X].data;
    vector<scalar> &omega_y = spatial_graphs[(uint)PlotSpatialVariable::VELOCITY_ROT * 3 + (uint)Direction::Y].data;
    vector<scalar> &omega_z = spatial_graphs[(uint)PlotSpatialVariable::VELOCITY_ROT * 3 + (uint)Direction::Z].data;

    vector<scalar> &a_x = spatial_graphs[(uint)PlotSpatialVariable::ACCELERATION_TRANS * 3 + (uint)Direction::X].data;
    vector<scalar> &a_y = spatial_graphs[(uint)PlotSpatialVariable::ACCELERATION_TRANS * 3 + (uint)Direction::Y].data;
    vector<scalar> &a_z = spatial_graphs[(uint)PlotSpatialVariable::ACCELERATION_TRANS * 3 + (uint)Direction::Z].data;
    vector<scalar> &alpha_x = spatial_graphs[(uint)PlotSpatialVariable::ACCELERATION_ROT * 3 + (uint)Direction::X].data;
    vector<scalar> &alpha_y = spatial_graphs[(uint)PlotSpatialVariable::ACCELERATION_ROT * 3 + (uint)Direction::Y].data;
    vector<scalar> &alpha_z = spatial_graphs[(uint)PlotSpatialVariable::ACCELERATION_ROT * 3 + (uint)Direction::Z].data;

    vector<scalar> &f_int_x = spatial_graphs[(uint)PlotSpatialVariable::FORCE_INT * 3 + (uint)Direction::X].data;
    vector<scalar> &f_int_y = spatial_graphs[(uint)PlotSpatialVariable::FORCE_INT * 3 + (uint)Direction::Y].data;
    vector<scalar> &f_int_z = spatial_graphs[(uint)PlotSpatialVariable::FORCE_INT * 3 + (uint)Direction::Z].data;
    vector<scalar> &m_int_x = spatial_graphs[(uint)PlotSpatialVariable::MOMENT_INT * 3 + (uint)Direction::X].data;
    vector<scalar> &m_int_y = spatial_graphs[(uint)PlotSpatialVariable::MOMENT_INT * 3 + (uint)Direction::Y].data;
    vector<scalar> &m_int_z = spatial_graphs[(uint)PlotSpatialVariable::MOMENT_INT * 3 + (uint)Direction::Z].data;

    vector<scalar> &f_ext_x = spatial_graphs[(uint)PlotSpatialVariable::FORCE_EXT_DYN * 3 + (uint)Direction::X].data;
    vector<scalar> &f_ext_y = spatial_graphs[(uint)PlotSpatialVariable::FORCE_EXT_DYN * 3 + (uint)Direction::Y].data;
    vector<scalar> &f_ext_z = spatial_graphs[(uint)PlotSpatialVariable::FORCE_EXT_DYN * 3 + (uint)Direction::Z].data;
    vector<scalar> &m_ext_x = spatial_graphs[(uint)PlotSpatialVariable::MOMENT_EXT_DYN * 3 + (uint)Direction::X].data;
    vector<scalar> &m_ext_y = spatial_graphs[(uint)PlotSpatialVariable::MOMENT_EXT_DYN * 3 + (uint)Direction::Y].data;
    vector<scalar> &m_ext_z = spatial_graphs[(uint)PlotSpatialVariable::MOMENT_EXT_DYN * 3 + (uint)Direction::Z].data;

    vector<scalar> &f_stat_x = spatial_graphs[(uint)PlotSpatialVariable::FORCE_EXT_STAT * 3 + (uint)Direction::X].data;
    vector<scalar> &f_stat_y = spatial_graphs[(uint)PlotSpatialVariable::FORCE_EXT_STAT * 3 + (uint)Direction::Y].data;
    vector<scalar> &f_stat_z = spatial_graphs[(uint)PlotSpatialVariable::FORCE_EXT_STAT * 3 + (uint)Direction::Z].data;
    vector<scalar> &m_stat_x = spatial_graphs[(uint)PlotSpatialVariable::MOMENT_EXT_STAT * 3 + (uint)Direction::X].data;
    vector<scalar> &m_stat_y = spatial_graphs[(uint)PlotSpatialVariable::MOMENT_EXT_STAT * 3 + (uint)Direction::Y].data;
    vector<scalar> &m_stat_z = spatial_graphs[(uint)PlotSpatialVariable::MOMENT_EXT_STAT * 3 + (uint)Direction::Z].data;

    vector<scalar> &p_inner = fluid_spatial_graphs[(uint)FluidPlotSpatialVariable::PRESSURE_INNER].data;
    vector<scalar> &p_outer = fluid_spatial_graphs[(uint)FluidPlotSpatialVariable::PRESSURE_OUTER].data;
    vector<scalar> &dp_inner = fluid_spatial_graphs[(uint)FluidPlotSpatialVariable::dPRESSURE_INNER].data;
    vector<scalar> &dp_outer = fluid_spatial_graphs[(uint)FluidPlotSpatialVariable::dPRESSURE_OUTER].data;
    vector<scalar> &v_inner = fluid_spatial_graphs[(uint)FluidPlotSpatialVariable::VELOCITY_INNER].data;
    vector<scalar> &v_outer = fluid_spatial_graphs[(uint)FluidPlotSpatialVariable::VELOCITY_OUTER].data;

    for_each_node_cpu(i) {

        s_nw[i] = -pipe.L_tot + S[i] + u[i].x();
        if (i == top_node) {
            s_el[i] = s_nw[i];
        } else {
            s_el[i] = 0.5 * (s_nw[i] + s_nw[i - 1]);
        }

        p_inner[i] = p_i[i] / 1e6;   // Convert to MPa
        p_outer[i] = p_o[i] / 1e6;   // Convert to MPa
        dp_outer[i] = dp_o[i] / 1e6; // Convert to MPa
        dp_inner[i] = dp_i[i] / 1e6; // Convert to MPa

        v_inner[i] = v_i[i];
        v_outer[i] = v_o[i];

        u_x[i] = u[i].x();
        u_y[i] = u[i].y();
        u_z[i] = u[i].z();

        theta_x[i] = theta[i].x();
        theta_y[i] = theta[i].y();
        theta_z[i] = theta[i].z();

        v_x[i] = v[i].x();
        v_y[i] = v[i].y();
        v_z[i] = v[i].z();

        omega_x[i] = omega[i].x();
        omega_y[i] = omega[i].y();
        omega_z[i] = omega[i].z();

        a_x[i] = a[i].x();
        a_y[i] = a[i].y();
        a_z[i] = a[i].z();

        alpha_x[i] = alpha[i].x();
        alpha_y[i] = alpha[i].y();
        alpha_z[i] = alpha[i].z();

        f_int_x[i] = f_int[i].x();
        f_int_y[i] = f_int[i].y();
        f_int_z[i] = f_int[i].z();

        m_int_x[i] = m_int[i].x();
        m_int_y[i] = m_int[i].y();
        m_int_z[i] = m_int[i].z();

        f_stat_x[i] = f_hyd[i].x();
        f_stat_y[i] = f_hyd[i].y();
        f_stat_z[i] = f_hyd[i].z();

        m_stat_x[i] = m_hyd[i].x();
        m_stat_y[i] = m_hyd[i].y();
        m_stat_z[i] = m_hyd[i].z();

        f_ext_x[i] = f_dyn[i].x();
        f_ext_y[i] = f_dyn[i].y();
        f_ext_z[i] = f_dyn[i].z();

        m_ext_x[i] = m_dyn[i].x();
        m_ext_y[i] = m_dyn[i].y();
        m_ext_z[i] = m_dyn[i].z();

        for (SpatialGraph &graph : spatial_graphs) {
            if (graph.auto_size) {
                graph.update_auto_scale(i);
            }
        }
        for (FluidSpatialGraph &graph : fluid_spatial_graphs) {
            if (graph.auto_size) {
                graph.update_auto_scale(i);
            }
        }
    }

    // Bottom fluid node
    s_el[N] = s_nw[N - 1];
    p_inner[N] = p_i[N] / 1e6;
    p_outer[N] = p_o[N] / 1e6;
    v_inner[N] = v_i[N];
    v_outer[N] = v_o[N];
    dp_outer[N] = dp_o[N] / 1e6; // Convert to MPa
    dp_inner[N] = dp_i[N] / 1e6; // Convert to MPa

    for (SpatialGraph &graph : spatial_graphs) {
        if (graph.auto_size) {
            graph.s_min = s_nw[top_node];
            graph.s_max = s_nw[N - 1];
        }
    }
    for (FluidSpatialGraph &graph : fluid_spatial_graphs) {
        if (graph.auto_size) {
            graph.update_auto_scale(N);
            graph.s_min = s_el[top_node];
            graph.s_max = s_el[N];
        }

        // Update contour plot limits if auto limits is enabled
        if (config_graphics.contour_variable == graph.contour_variable && config_graphics.contour_auto_limits) {
            config_graphics.contour_val_max = graph.data_max;
            config_graphics.contour_val_min = graph.data_min;
            config_graphics.contour_lim_max = graph.data_max;
            config_graphics.contour_lim_min = graph.data_min;
        }
    }
}

void GUI_Manager::update_spatial_graphs_corot(const Pipe &pipe, const Hole &hole, const corot::PipeSolver &solver_corot,
                                              const uint top_node, const byte *buf) {
    using namespace corot;

    const corot::BeamData &beam = solver_corot.beam;
    uint i;
    const uint N = pipe.N;

    const ArrayView<scalar> arr_s = hole.get_field<HoleField::s>(buf);
    const ArrayView<Vec3> x_hole = hole.get_field<HoleField::x>(buf);
    const ArrayView<Quaternion> q_hole = hole.get_field<HoleField::q>(buf);
    const vector<Vec3> X = beam.X;
    const vector<Vec3> &d_trans = beam.d_trans;
    const vector<Quaternion> &d_rot = beam.d_rot;
    const vector<Vec3> &v_glob = beam.v_trans;
    const vector<Vec3> &omega_glob = beam.v_trans;
    const vector<Vec3> &a_glob = beam.a_trans;
    const vector<Vec3> &alpha_glob = beam.a_rot;

    const ArrayView<scalar> S = pipe.get_field<PipeField::S>(buf);
    const scalar Lp_tot = pipe.L_tot;

    vector<scalar> &s = ui_manager->s_nodewise;
    vector<scalar> &u_x = spatial_graphs[(uint)PlotSpatialVariable::DISPLACEMENT_TRANS * 3 + (uint)Direction::X].data;
    vector<scalar> &u_y = spatial_graphs[(uint)PlotSpatialVariable::DISPLACEMENT_TRANS * 3 + (uint)Direction::Y].data;
    vector<scalar> &u_z = spatial_graphs[(uint)PlotSpatialVariable::DISPLACEMENT_TRANS * 3 + (uint)Direction::Z].data;
    vector<scalar> &theta_x = spatial_graphs[(uint)PlotSpatialVariable::DISPLACEMENT_ROT * 3 + (uint)Direction::X].data;
    vector<scalar> &theta_y = spatial_graphs[(uint)PlotSpatialVariable::DISPLACEMENT_ROT * 3 + (uint)Direction::Y].data;
    vector<scalar> &theta_z = spatial_graphs[(uint)PlotSpatialVariable::DISPLACEMENT_ROT * 3 + (uint)Direction::Z].data;

    vector<scalar> &v_x = spatial_graphs[(uint)PlotSpatialVariable::VELOCITY_TRANS * 3 + (uint)Direction::X].data;
    vector<scalar> &v_y = spatial_graphs[(uint)PlotSpatialVariable::VELOCITY_TRANS * 3 + (uint)Direction::Y].data;
    vector<scalar> &v_z = spatial_graphs[(uint)PlotSpatialVariable::VELOCITY_TRANS * 3 + (uint)Direction::Z].data;
    vector<scalar> &omega_x = spatial_graphs[(uint)PlotSpatialVariable::VELOCITY_ROT * 3 + (uint)Direction::X].data;
    vector<scalar> &omega_y = spatial_graphs[(uint)PlotSpatialVariable::VELOCITY_ROT * 3 + (uint)Direction::Y].data;
    vector<scalar> &omega_z = spatial_graphs[(uint)PlotSpatialVariable::VELOCITY_ROT * 3 + (uint)Direction::Z].data;

    vector<scalar> &a_x = spatial_graphs[(uint)PlotSpatialVariable::ACCELERATION_TRANS * 3 + (uint)Direction::X].data;
    vector<scalar> &a_y = spatial_graphs[(uint)PlotSpatialVariable::ACCELERATION_TRANS * 3 + (uint)Direction::Y].data;
    vector<scalar> &a_z = spatial_graphs[(uint)PlotSpatialVariable::ACCELERATION_TRANS * 3 + (uint)Direction::Z].data;
    vector<scalar> &alpha_x = spatial_graphs[(uint)PlotSpatialVariable::ACCELERATION_ROT * 3 + (uint)Direction::X].data;
    vector<scalar> &alpha_y = spatial_graphs[(uint)PlotSpatialVariable::ACCELERATION_ROT * 3 + (uint)Direction::Y].data;
    vector<scalar> &alpha_z = spatial_graphs[(uint)PlotSpatialVariable::ACCELERATION_ROT * 3 + (uint)Direction::Z].data;

    vector<scalar> &f_int_x = spatial_graphs[(uint)PlotSpatialVariable::FORCE_INT * 3 + (uint)Direction::X].data;
    vector<scalar> &f_int_y = spatial_graphs[(uint)PlotSpatialVariable::FORCE_INT * 3 + (uint)Direction::Y].data;
    vector<scalar> &f_int_z = spatial_graphs[(uint)PlotSpatialVariable::FORCE_INT * 3 + (uint)Direction::Z].data;
    vector<scalar> &m_int_x = spatial_graphs[(uint)PlotSpatialVariable::MOMENT_INT * 3 + (uint)Direction::X].data;
    vector<scalar> &m_int_y = spatial_graphs[(uint)PlotSpatialVariable::MOMENT_INT * 3 + (uint)Direction::Y].data;
    vector<scalar> &m_int_z = spatial_graphs[(uint)PlotSpatialVariable::MOMENT_INT * 3 + (uint)Direction::Z].data;

    vector<scalar> &f_ext_x = spatial_graphs[(uint)PlotSpatialVariable::FORCE_EXT_DYN * 3 + (uint)Direction::X].data;
    vector<scalar> &f_ext_y = spatial_graphs[(uint)PlotSpatialVariable::FORCE_EXT_DYN * 3 + (uint)Direction::Y].data;
    vector<scalar> &f_ext_z = spatial_graphs[(uint)PlotSpatialVariable::FORCE_EXT_DYN * 3 + (uint)Direction::Z].data;
    vector<scalar> &m_ext_x = spatial_graphs[(uint)PlotSpatialVariable::MOMENT_EXT_DYN * 3 + (uint)Direction::X].data;
    vector<scalar> &m_ext_y = spatial_graphs[(uint)PlotSpatialVariable::MOMENT_EXT_DYN * 3 + (uint)Direction::Y].data;
    vector<scalar> &m_ext_z = spatial_graphs[(uint)PlotSpatialVariable::MOMENT_EXT_DYN * 3 + (uint)Direction::Z].data;

    vector<scalar> &f_stat_x = spatial_graphs[(uint)PlotSpatialVariable::FORCE_EXT_STAT * 3 + (uint)Direction::X].data;
    vector<scalar> &f_stat_y = spatial_graphs[(uint)PlotSpatialVariable::FORCE_EXT_STAT * 3 + (uint)Direction::Y].data;
    vector<scalar> &f_stat_z = spatial_graphs[(uint)PlotSpatialVariable::FORCE_EXT_STAT * 3 + (uint)Direction::Z].data;
    vector<scalar> &m_stat_x = spatial_graphs[(uint)PlotSpatialVariable::MOMENT_EXT_STAT * 3 + (uint)Direction::X].data;
    vector<scalar> &m_stat_y = spatial_graphs[(uint)PlotSpatialVariable::MOMENT_EXT_STAT * 3 + (uint)Direction::Y].data;
    vector<scalar> &m_stat_z = spatial_graphs[(uint)PlotSpatialVariable::MOMENT_EXT_STAT * 3 + (uint)Direction::Z].data;

    for_each_node_cpu(i) {
        const uint ie_h = beam.i_pipe_to_ie_hole[i];
        assert(ie_h < hole.N_hole - 1);

        const Vec3 x = X[i] + d_trans[i];
        const Mat3 R = d_rot[i].to_matrix();
        const Vec3 &x_hole_A = x_hole[ie_h];
        const Vec3 &x_hole_B = x_hole[ie_h + 1];
        const Vec3 t = (x_hole_B - x_hole_A).normalized();
        const Vec3 x_center = x_hole_A + (x - x_hole_A).dot(t) * t;
        const Vec3 u_lateral = x - x_center;
        assert(is_orthogonal(t, u_lateral));

        s[i] = arr_s[ie_h] + (x_center - x_hole_A).norm();

        // Interpolation of rotations using lerp
        scalar t_interp = (s[i] - arr_s[ie_h]) / (arr_s[ie_h + 1] - arr_s[ie_h]);
        Quaternion ql = Quaternion::lerp(q_hole[ie_h], q_hole[ie_h + 1], t_interp);

        // Rotate the lateral deviation vector onto the curvlin frame of referance
        Vec3 u_proj = ql.rotate_vector_reversed(u_lateral);
        assert(is_close(u_proj[0], 0.0, 1.0E-2)); // Should be no x-component

        // Using that: s = -Lp_tot + S + ux;
        u_x[i] = s[i] + Lp_tot - S[i];

        u_y[i] = u_proj[1];
        u_z[i] = u_proj[2];

        const Mat3 R_0 = q_hole[ie_h].to_matrix();
        const Mat3 R_hat = R_0.transpose() * R;

        // Extract Euler angles (ZYX order)
        theta_y[i] = -asin(R_hat(2, 0)); // Rotation about Y-axis

        if (std::abs(R_hat(2, 0)) < 1.0 - 1e-6) {          // Not in gimbal lock
            theta_x[i] = atan2(R_hat(2, 1), R_hat(2, 2));  // Rotation about X-axis
            theta_z[i] = atan2(R_hat(1, 0), R_hat(0, 0));  // Rotation about Z-axis
        } else {                                           // Gimbal lock case
            theta_x[i] = 0;                                // Arbitrary
            theta_z[i] = atan2(-R_hat(0, 1), R_hat(1, 1)); // Combine rotations into Z
        }

        const Vec3 v = R_0.transpose() * v_glob[i];
        const Vec3 omega = R_0.transpose() * omega_glob[i];
        const Vec3 a = R_0.transpose() * a_glob[i];
        const Vec3 alpha = R_0.transpose() * alpha_glob[i];
        const Vec3 f_int = R_0.transpose() * beam.f_int_trans[i];
        const Vec3 m_int = R_0.transpose() * beam.f_int_rot[i];
        const Vec3 f_dyn = R_0.transpose() * beam.f_ext_trans[i];
        const Vec3 m_ext = R_0.transpose() * beam.f_ext_rot[i];
        const Vec3 f_hyd = R_0.transpose() * beam.f_stat_trans[i];
        const Vec3 m_stat = R_0.transpose() * beam.f_stat_rot[i];

        v_x[i] = v.x();
        v_y[i] = v.y();
        v_z[i] = v.z();
        omega_x[i] = omega.x();
        omega_y[i] = omega.y();
        omega_z[i] = omega.z();
        a_x[i] = a.x();
        a_y[i] = a.y();
        a_z[i] = a.z();
        alpha_x[i] = alpha.x();
        alpha_y[i] = alpha.y();
        alpha_z[i] = alpha.z();

        f_int_x[i] = f_int.x();
        f_int_y[i] = f_int.y();
        f_int_z[i] = f_int.z();
        m_int_x[i] = m_int.x();
        m_int_y[i] = m_int.y();
        m_int_z[i] = m_int.z();

        f_ext_x[i] = f_dyn.x();
        f_ext_y[i] = f_dyn.y();
        f_ext_z[i] = f_dyn.z();
        m_ext_x[i] = m_ext.x();
        m_ext_y[i] = m_ext.y();
        m_ext_z[i] = m_ext.z();

        f_stat_x[i] = f_hyd.x();
        f_stat_y[i] = f_hyd.y();
        f_stat_z[i] = f_hyd.z();
        m_stat_x[i] = m_stat.x();
        m_stat_y[i] = m_stat.y();
        m_stat_z[i] = m_stat.z();

        if (i > top_node && i < N - 1) { // Ignore end BC values for the automatic scaling
            for (SpatialGraph &graph : spatial_graphs) {
                if (graph.auto_size && graph.show_graph_plot) {
                    graph.update_auto_scale(i);
                }
            }
        }
    }

    for (SpatialGraph &graph : spatial_graphs) {
        if (graph.auto_size) {
            graph.s_min = s[top_node];
            graph.s_max = s[N - 1];
        }
    }
}

void GUI_Manager::update_spatial_graphs(Simulator *simulator) {

    Config &config = simulator->config;
    const Hole &hole = simulator->hole;
    const Pipe &pipe = simulator->pipe;
    curvlin::PipeSolver *solver_curvlin = simulator->pipe_solver_curvlin.get();
    const corot::PipeSolver *solver_corot = simulator->pipe_solver_corot.get();
    byte *buf = simulator->double_buffer_gui->get_arena_h().buf;

    uint N = pipe.N;
    uint top_node = config.top_node;
    if (config.pipe_solver_type == PipeSolverType::CURVILINEAR) {
        update_spatial_graphs_curvlin(pipe, *solver_curvlin, top_node, buf);
    } else if (config.pipe_solver_type == PipeSolverType::COROTATIONAL) {
        update_spatial_graphs_corot(pipe, hole, *solver_corot, top_node, buf);
    }

    for (MiscSpatialGraph &misc_graph : misc_spatial_graphs) {
        if (misc_graph.show_graph_plot || config_graphics.contour_variable == misc_graph.contour_variable) {
            if (misc_graph.variable == MiscPlotSpatialVariable::EFFECTIVE_TENSION) {
                update_effective_tension(top_node, N, misc_graph, pipe, solver_curvlin->fluid, buf);
            } else if (misc_graph.variable == MiscPlotSpatialVariable::AXIAL_TENSION) {
                update_axial_tension(top_node, N, misc_graph);
            } else if (misc_graph.variable == MiscPlotSpatialVariable::EFFECTIVE_TORQUE) {
                update_effective_torque(top_node, N, misc_graph);
            } else if (misc_graph.variable == MiscPlotSpatialVariable::RADIAL_ACCELERATION) {
                update_radial_acceleration(top_node, N, misc_graph, *solver_curvlin, pipe, hole, config, buf);
            } else if (misc_graph.variable == MiscPlotSpatialVariable::TANGENTIAL_ACCELERATION) {
                update_tangential_acceleration(top_node, N, misc_graph, *solver_curvlin, pipe, hole, config, buf);
            } else if (misc_graph.variable == MiscPlotSpatialVariable::AXIAL_STRAIN) {
                update_axial_strain(top_node, N, misc_graph, pipe, buf);
            } else if (misc_graph.variable ==
                       MiscPlotSpatialVariable::VON_MISES_STRESS) { // Only update if shown in graph window
                update_von_mises_stress(top_node, N, config, *solver_curvlin, misc_graph, pipe, hole, buf);
            } else if (misc_graph.variable == MiscPlotSpatialVariable::LINEAR_CONTACT_FORCE) {
                update_linear_contact_force(top_node, N, misc_graph, pipe, buf);
            } else if (misc_graph.variable == MiscPlotSpatialVariable::TANGENTIAL_CONTACT_FORCE) {
                update_frictional_contact_force(top_node, N, misc_graph, pipe, buf);
            } else if (misc_graph.variable == MiscPlotSpatialVariable::STANDOFF_RATIO) {
                update_standoff_ratio(top_node, N, *solver_curvlin, misc_graph, pipe, hole, buf);
            }
            misc_graph.s_min = ui_manager->s_nodewise[top_node];
            misc_graph.s_max = ui_manager->s_nodewise[N - 1];

            float range = misc_graph.data_max - misc_graph.data_min;
            misc_graph.y_min = misc_graph.data_min - (range * display_margin);
            misc_graph.y_max = misc_graph.data_max + (range * display_margin);

            // Update contour plot limits if auto limits is enabled
            if (config_graphics.contour_variable == misc_graph.contour_variable &&
                config_graphics.contour_auto_limits) {
                config_graphics.contour_val_max = misc_graph.data_max;
                config_graphics.contour_val_min = misc_graph.data_min;
                config_graphics.contour_lim_max = misc_graph.data_max;
                config_graphics.contour_lim_min = misc_graph.data_min;
            }
        }
    }
}

void GUI_Manager::update_effective_tension(uint top_node, uint N, MiscSpatialGraph &misc_graph, const Pipe &pipe,
                                           const curvlin::FluidData &fluid, const byte *buf) {
    assert(MiscPlotSpatialVariable::EFFECTIVE_TENSION == misc_graph.variable);
    using namespace curvlin;

    ArrayView<scalar> p_i = fluid.get_field<FluidField::p_i>(buf);
    ArrayView<scalar> p_o = fluid.get_field<FluidField::p_o>(buf);

    misc_graph.data_min = std::numeric_limits<float>::max();
    misc_graph.data_max = -std::numeric_limits<float>::max();

    scalar sum_axial = 0.0;
    uint i;
    vector<scalar> &f_eff = misc_graph.data;
    const vector<scalar> &fx_inner = spatial_graphs[(uint)PlotSpatialVariable::FORCE_INT * 3 + (uint)Direction::X].data;
    for_each_node_cpu(i) {
        // Calculate cumulated axial tension along the string
        sum_axial += fx_inner[i];
        f_eff[i] = -sum_axial;
    }

    scalar p_i_n, p_o_n, A_i, A_o; // Nodewise internal and external pressure and areas
    for_each_node_cpu(i) {
        if (i == top_node) {
            p_i_n = p_i[top_node];
            p_o_n = p_o[top_node];
            A_i = pipe.Af_e(top_node, buf);
            A_o = pipe.Ap_e(top_node, buf) + pipe.Af_e(top_node, buf);

        } else if (i == N - 1) {
            p_i_n = p_i[N];
            p_o_n = p_o[N];
            A_i = pipe.Af_e(N - 2, buf);
            A_o = pipe.Ap_e(N - 2, buf) + pipe.Af_e(N - 2, buf);
        } else { // Remember, pressures have a ghost node at both ends, Nf = N + 1
            p_i_n = 0.5 * (p_i[i] + p_i[i + 1]);
            p_o_n = 0.5 * (p_o[i] + p_o[i + 1]);
            A_i = 0.5 * (pipe.Af_e(i - 1, buf) + pipe.Af_e(i, buf));
            A_o = 0.5 * (pipe.Ap_e(i - 1, buf) + pipe.Af_e(i - 1, buf) + pipe.Ap_e(i, buf) + pipe.Af_e(i, buf));
        }

        // Calculate effective tension along the string
        f_eff[i] = (f_eff[i] - p_i_n * A_i + p_o_n * A_o) / 1000.0 / 9.81;
        misc_graph.data_min = min(misc_graph.data_min, (float)f_eff[i]);
        misc_graph.data_max = max(misc_graph.data_max, (float)f_eff[i]);
    }
}

void GUI_Manager::update_axial_tension(uint top_node, uint N, MiscSpatialGraph &misc_graph) {
    assert(MiscPlotSpatialVariable::AXIAL_TENSION == misc_graph.variable);
    scalar sum_axial = 0.0;
    uint i;
    vector<scalar> &fx_tension = misc_graph.data;
    const vector<scalar> &fx_inner = spatial_graphs[(uint)PlotSpatialVariable::FORCE_INT * 3 + (uint)Direction::X].data;
    for_each_node_cpu(i) {
        // Calculate cumulated axial tension along the string
        sum_axial += fx_inner[i];
        fx_tension[i] = -sum_axial / 1000.0 / 9.81;
        misc_graph.data_min = min(misc_graph.data_min, (float)fx_tension[i]);
        misc_graph.data_max = max(misc_graph.data_max, (float)fx_tension[i]);
    }
}

void GUI_Manager::update_effective_torque(uint top_node, uint N, MiscSpatialGraph &misc_graph) {
    assert(MiscPlotSpatialVariable::EFFECTIVE_TORQUE == misc_graph.variable);
    scalar sum_torque = 0.0;
    uint i;
    vector<scalar> &torque_cum = misc_graph.data;
    const vector<scalar> &mx_inner =
        spatial_graphs[(uint)PlotSpatialVariable::MOMENT_INT * 3 + (uint)Direction::X].data;
    for_each_node_cpu(i) {
        // Calculate cumulated torque along the string
        sum_torque += mx_inner[i];
        torque_cum[i] = sum_torque / 1000.0; // Convert to kNm, sign convention: negative for transmitted torque
        misc_graph.data_min = min(misc_graph.data_min, (float)torque_cum[i]);
        misc_graph.data_max = max(misc_graph.data_max, (float)torque_cum[i]);
    }
}

void GUI_Manager::update_radial_acceleration(uint top_node, uint N, MiscSpatialGraph &misc_graph,
                                             const curvlin::PipeSolver &solver_curvlin, const Pipe &pipe,
                                             const Hole &hole, const Config &config, const byte *buf) {
    assert(MiscPlotSpatialVariable::RADIAL_ACCELERATION == misc_graph.variable);
    vector<scalar> &vec = misc_graph.data;
    uint i;
    for_each_node_cpu(i) {
        vec[i] = solver_curvlin.compute_radial_acceleration(i, pipe, hole, buf);
        misc_graph.data_min = min(misc_graph.data_min, (float)vec[i]);
        misc_graph.data_max = max(misc_graph.data_max, (float)vec[i]);
    }
}

void GUI_Manager::update_tangential_acceleration(uint top_node, uint N, MiscSpatialGraph &misc_graph,
                                                 const curvlin::PipeSolver &solver_curvlin, const Pipe &pipe,
                                                 const Hole &hole, const Config &config, const byte *buf) {
    assert(MiscPlotSpatialVariable::TANGENTIAL_ACCELERATION == misc_graph.variable);
    vector<scalar> &vec = misc_graph.data;
    uint i;
    for_each_node_cpu(i) {
        vec[i] = solver_curvlin.compute_tangential_acceleration(i, pipe, hole, buf);
        misc_graph.data_min = min(misc_graph.data_min, (float)vec[i]);
        misc_graph.data_max = max(misc_graph.data_max, (float)vec[i]);
    }
}

void GUI_Manager::update_axial_strain(uint top_node, uint N, MiscSpatialGraph &misc_graph, const Pipe &pipe,
                                      const byte *buf) {
    assert(MiscPlotSpatialVariable::AXIAL_STRAIN == misc_graph.variable);
    vector<scalar> &du_ax = misc_graph.data;
    const vector<scalar> &u =
        spatial_graphs[(uint)PlotSpatialVariable::DISPLACEMENT_TRANS * 3 + (uint)Direction::X].data;

    for (uint ie = top_node; ie < N - 1; ie++) {
        // Calculate cumulated axial tension along the string
        scalar dS = pipe.dS_e(ie, buf); /*Distance between centroids of two elements*/
        du_ax[ie + 1] = (u[ie + 1] - u[ie]) / dS;
        misc_graph.data_min = min(misc_graph.data_min, (float)du_ax[ie + 1]);
        misc_graph.data_max = max(misc_graph.data_max, (float)du_ax[ie + 1]);
    }

    // Copy end values
    du_ax[top_node] = du_ax[top_node + 1];
    du_ax[N] = du_ax[N - 1];
}

void GUI_Manager::update_von_mises_stress(uint top_node, uint N, const Config &config,
                                          const curvlin::PipeSolver &solver_curvlin, MiscSpatialGraph &misc_graph,
                                          const Pipe &pipe, const Hole &hole, const byte *buf) {
    //============================================================
    // This function computes the von Mises stress at the middle of
    // each element
    //============================================================
    assert(MiscPlotSpatialVariable::VON_MISES_STRESS == misc_graph.variable &&
           config.pipe_solver_type == PipeSolverType::CURVILINEAR);
    vector<scalar> &sigma_vm = misc_graph.data;
    const scalar xi = 0.5;

    for (uint ie = top_node; ie < N - 1; ++ie) {
        scalar sigma_vm_e = solver_curvlin.compute_vm_stress_curvlin(ie, xi, pipe, hole, config, buf);
        sigma_vm[ie + 1] = sigma_vm_e / 1.0e6; // Convert Pa to MPa
        misc_graph.data_min = min(misc_graph.data_min, (float)sigma_vm[ie + 1]);
        misc_graph.data_max = max(misc_graph.data_max, (float)sigma_vm[ie + 1]);
    }

    // Copy end values
    sigma_vm[top_node] = sigma_vm[top_node + 1];
    sigma_vm[N] = sigma_vm[N - 1];
}

void GUI_Manager::update_linear_contact_force(uint top_node, uint N, MiscSpatialGraph &misc_graph, const Pipe &pipe,
                                              const byte *buf) {
    assert(MiscPlotSpatialVariable::LINEAR_CONTACT_FORCE == misc_graph.variable);

    vector<scalar> &w_c = misc_graph.data;
    const vector<scalar> &Fy = spatial_graphs[(uint)PlotSpatialVariable::FORCE_EXT_DYN * 3 + (uint)Direction::Y].data;
    const vector<scalar> &Fz = spatial_graphs[(uint)PlotSpatialVariable::FORCE_EXT_DYN * 3 + (uint)Direction::Z].data;

    for (uint i = top_node; i < N - 1; i++) {
        scalar F_i = sqrt(Fy[i] * Fy[i] + Fz[i] * Fz[i]); // Convert to kN
        scalar dS = pipe.dS_node_avg(i, buf);             /*Distance between centroids of two elements*/
        w_c[i] = F_i / dS;                                // Initialize contact force
        misc_graph.data_min = min(misc_graph.data_min, (float)w_c[i]);
        misc_graph.data_max = max(misc_graph.data_max, (float)w_c[i]);
    }
}

void GUI_Manager::update_frictional_contact_force(uint top_node, uint N, MiscSpatialGraph &misc_graph, const Pipe &pipe,
                                                  const byte *buf) {
    assert(MiscPlotSpatialVariable::TANGENTIAL_CONTACT_FORCE == misc_graph.variable);

    vector<scalar> &w_t = misc_graph.data;
    const vector<scalar> &Fx = spatial_graphs[(uint)PlotSpatialVariable::FORCE_EXT_DYN * 3 + (uint)Direction::X].data;
    const vector<scalar> &Fy_dyn =
        spatial_graphs[(uint)PlotSpatialVariable::FORCE_EXT_DYN * 3 + (uint)Direction::Y].data;
    const vector<scalar> &Fz_dyn =
        spatial_graphs[(uint)PlotSpatialVariable::FORCE_EXT_DYN * 3 + (uint)Direction::Z].data;
    const vector<scalar> &Mx = spatial_graphs[(uint)PlotSpatialVariable::MOMENT_EXT_DYN * 3 + (uint)Direction::X].data;

    for (uint i = top_node; i < N - 1; i++) {

        // safe-guard area -> radius (arm)
        uint ic = pipe.get_field<PipeField::num_ic_to_ie>(buf)[i];
        const scalar arm = pipe.get_field<PipeField::rc>(buf)[ic];

        // torque produces a tangential shear ~ |M_x| / arm
        scalar f_t = (arm > 0.0f) ? (std::abs(Mx[i]) / arm) : 0.0f;

        scalar F_i = sqrt(Fx[i] * Fx[i] + f_t * f_t); // Convert to kN
        scalar dS = pipe.dS_node_avg(i, buf);         /*Distance between centroids of two elements*/
        w_t[i] = F_i / dS;                            // Initialize contact force
        misc_graph.data_min = min(misc_graph.data_min, (float)w_t[i]);
        misc_graph.data_max = max(misc_graph.data_max, (float)w_t[i]);
    }
}

void GUI_Manager::update_standoff_ratio(uint top_node, uint N, const curvlin::PipeSolver &solver_curvlin,
                                        MiscSpatialGraph &misc_graph, const Pipe &pipe, const Hole &hole,
                                        const byte *buf) {
    assert(MiscPlotSpatialVariable::STANDOFF_RATIO == misc_graph.variable);
    vector<scalar> &so_ratio = misc_graph.data;
    for (uint i = top_node; i < N - 1; i++) {
        so_ratio[i] = solver_curvlin.compute_standoff(i, pipe, hole, buf);
    }
}