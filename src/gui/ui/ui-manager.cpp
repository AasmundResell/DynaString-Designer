#include "ui-manager.hpp"
#include "gui/frame-data.hpp"
#include "pipe-solver/pipe-solver-curvlin/numerics/gauss-legendre-utils.hpp"
#include "sim-tools/gui/includes-GL.hpp"
#include "sim-tools/utils.hpp"
#include <stdexcept>
#include <string>

UI_Manager::UI_Manager(GLFWwindow *window, float screen_width, float screen_height)
    : plot_groups{
          PlotGroup{"Axial Forces",
                    "Force [kN]",
                    1000.0,
                    200.0f,
                    {DrillingVariable::HOOK_LOAD, DrillingVariable::WOB},
                    {"Hook Load", "WOB"}},
          PlotGroup{"Torques",
                    "Torque [kNm]",
                    1000.0,
                    1.0f,
                    {DrillingVariable::TOP_DRIVE_TORQUE, DrillingVariable::TOB},
                    {"Top Drive Torque", "TOB"}},
          PlotGroup{"Axial velocities",
                    "Velocity [m/h]",
                    1.0 / 3600.0,
                    100.0f,
                    {DrillingVariable::FEED_RATE, DrillingVariable::ROP},
                    {"Feed Rate", "ROP"}},
          PlotGroup{"Angular velocities",
                    "Angular velocity [RPM]",
                    M_PI / 30.0f, // Conversion from rad/s to RPM
                    200.0f,
                    {DrillingVariable::OMEGA_TOP_DRIVE, DrillingVariable::OMEGA_BIT},
                    {"Omega Top Drive", "Omega Bit"}},
          PlotGroup{"Hoist Works", "Position [m]", 1.0, 0.1f, {DrillingVariable::BLOCK_POSITION}, {"Block Position"}},
          PlotGroup{"Mechanical Specific Energy",
                    "Mechanical Specific Energy [J/m]",
                    1.0,
                    0.1f,
                    {DrillingVariable::MSE},
                    {"MSE"}}} {

    imgui_manager = make_unique<ImGuiManager>(window);
    ui_utils.set_scale(screen_width, screen_height);
    file_chooser = make_unique<ImGuiFileChooser>();

    // Enable docking
    ImGuiIO &io = ImGui::GetIO();

    ImFont *customFont = io.Fonts->AddFontFromFileTTF(
        "../src/external/sim-tools/vendor/imgui/fonts/Noto_Sans/static/NotoSans-Regular.ttf", 16.0f);
    if (customFont == nullptr) {
        cerr << "Failed to load font!" << endl;
        // Handle the error
    } else {
        io.FontDefault = customFont;
    }

    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    // Initialize default plot tiles
    initialize_default_plot_tiles();
}

void UI_Manager::new_frame() {
    imgui_manager->new_frame();
    frame_data.imgui_frame_active = true;

    // Update ImGui mouse status
    update_imgui_mouse_status();

    ImGuiWindowFlags window_flags = imgui_set_wiew_port();
    ImGui::Begin("DockSpace Demo", nullptr, window_flags);

    ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
    ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_None);

    // Sets the inital dockspace configuration based on the previous settings
    ImGui::SetNextWindowDockID(dockspace_id, ImGuiCond_FirstUseEver);
}

void UI_Manager::call_file_chooser(string *input_file, std::atomic<bool> &start_simulator, bool running) {
    assert(file_chooser != nullptr);

    file_chooser->call_imgui_file_chooser(input_file, running);

    // Check if a file has been selected

    if (!input_file->empty()) {
        start_simulator = true;
        string full_path = *input_file;
        string search_folder = "input-files/";
        size_t pos = full_path.find(search_folder);

        if (pos != string::npos) {
            *input_file = full_path.substr(pos);
        } else {
            *input_file = full_path;
        }

        cout << "Selected file: " << *input_file << endl;
    }
}

UI_Manager::~UI_Manager() {}

void UI_Manager::update_imgui_mouse_status() {
    ImGuiIO &io = ImGui::GetIO();
    frame_data.mouse_is_over_viewport = ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow) && io.MouseDown[0];
    frame_data.is_mouse_dragging_imgui = frame_data.mouse_is_over_viewport;
}

void UI_Manager::call_imgui_user_control_panel(ConfigGraphics &config_graphics) {
    // Create ImGui control window
    if (ImGui::CollapsingHeader("User Controls", ImGuiTreeNodeFlags_DefaultOpen)) {

        imgui_manager->toggle_button("FPS Mode", &frame_data.fps_mode, "First-person (shooter) mode");

        imgui_manager->toggle_button("Show Hole Triads", &config_graphics.show_hole_triads,
                                     "Draws the hole frame described by the Bishop equations");

        if (config_graphics.show_hole_triads) {
            ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
            ImGui::SliderFloat("Arrow Length [m]##hole", &config_graphics.hole_triad_scale, 0.0f, 1.0f); // Max 1 m
        }

        imgui_manager->toggle_button("Show Pipe Triads", &config_graphics.show_pipe_triads,
                                     "Shows the orientation of the FEM nodes");

        if (config_graphics.show_pipe_triads) {
            ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
            ImGui::SliderFloat("Arrow Length [m]##pipe", &config_graphics.pipe_triad_scale, 0.5f, 10.0f); // Max 1 m
        }

        ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
        ImGui::SliderFloat("Scale", &config_graphics.radial_scale, 1.0f, 100.0f);

        imgui_manager->toggle_button("Wireframe Mode", &config_graphics.wireframe_mode,
                                     "Draws the wireframes of the graphics");
    }
}

void UI_Manager::call_imgui_solver_control_panel(Config &config, std::atomic<bool> &restart_solver) {

    float CFL = static_cast<float>(config.conf_stat.CFL);

    float prev_CFL = CFL;
    float prev_dt = static_cast<float>(config.conf_stat.dt);

    if (ImGui::SliderFloat("CFL Number", &CFL, 0.1f, 0.9f, "%.2f ")) {
        // dt_new = dt_old * (CFL_new / CFL_old)
        config.conf_stat.CFL = static_cast<scalar>(CFL);
        config.conf_stat.dt = static_cast<scalar>(prev_dt * (CFL / prev_CFL));
    }

    if (ImGui::Button("Restart Solver")) {
        restart_solver = !restart_solver;
    }

    ImGui::Separator();

    ImGui::Text("CSV Output Controls");

    // Save interval in timesteps
    int save_interval = static_cast<int>(config.n_write);
    if (ImGui::InputInt("Save Interval (timesteps)", &save_interval)) {
        // Ensure positive value
        save_interval = std::max(1, save_interval);
        config.n_write = static_cast<uint>(save_interval);
    }

    // Start/Stop button that toggles CSV saving
    if (ImGui::Button(config.save_csv ? "Stop CSV Recording" : "Start CSV Recording")) {
        config.save_csv = !config.save_csv;
    }

    // Show current status
    ImGui::SameLine();
    ImGui::TextColored(config.save_csv ? ImVec4(0.0f, 1.0f, 0.0f, 1.0f) : ImVec4(1.0f, 0.0f, 0.0f, 1.0f),
                       config.save_csv ? "Recording" : "Stopped");

    if (config.save_csv) {
        ImGui::Checkbox("Write Spatial Quantities", &config.write_misc_quantities);
    }
    ImGui::Separator();
    ImGui::Text("Number of iterations for the uneven contact algorithm");

    int nc_iter = static_cast<int>(config.conf_stat.Nc_iter);
    if (ImGui::InputInt("Number of iterations", &nc_iter)) {
        // Ensure the number of iterations is positive
        nc_iter = std::max(1, nc_iter);
        config.conf_stat.Nc_iter = static_cast<uint>(nc_iter);
    }
    ImGui::Separator();

    if (config.pipe_solver_type == PipeSolverType::COROTATIONAL) {
        ImGui::Text("Corotational solver settings");

        // Corotational solver controls
        std::array<const char *, (uint)CorotationalFormulation::COUNT> corot_formulation_types;
        std::array<CorotationalFormulation, (uint)CorotationalFormulation::COUNT> formulation_type_values;

        for (uint i = 0; i < (uint)CorotationalFormulation::COUNT; i++) {
            auto it = std::next(corotational_formulation_from_string.begin(), i);
            corot_formulation_types[i] = it->first.c_str();
            formulation_type_values[i] = it->second;
        }

        int current_corot_formulation = std::distance(
            formulation_type_values.begin(), std::find(formulation_type_values.begin(), formulation_type_values.end(),
                                                       config.conf_stat.corotational_formulation));

        if (ImGui::Combo("Corotational Solver Type", &current_corot_formulation, corot_formulation_types.data(),
                         corot_formulation_types.size())) {
            config.conf_stat.corotational_formulation = formulation_type_values[current_corot_formulation];
        }

    } else if (config.pipe_solver_type == PipeSolverType::CURVILINEAR) {
        ImGui::Text("Curvilinear solver settings");

        // Curvilinear solver controls
        std::array<const char *, (uint)CurvilinearIntegrationType::COUNT> curvilinear_integration_types;
        std::array<CurvilinearIntegrationType, (uint)CurvilinearIntegrationType::COUNT> integration_type_values;

        for (uint i = 0; i < (uint)CurvilinearIntegrationType::COUNT; i++) {
            auto it = std::next(curvilinear_integration_type_from_string.begin(), i);
            curvilinear_integration_types[i] = it->first.c_str();
            integration_type_values[i] = it->second;
        }

        int current_curvilinear_integration = std::distance(
            integration_type_values.begin(), std::find(integration_type_values.begin(), integration_type_values.end(),
                                                       config.conf_stat.curvilinear_integration_type));

        if (ImGui::Combo("Curvilinear Integration Type", &current_curvilinear_integration,
                         curvilinear_integration_types.data(), curvilinear_integration_types.size())) {
            config.conf_stat.curvilinear_integration_type = integration_type_values[current_curvilinear_integration];
        }

        // Render checkboxes based on the selected integration type
        switch (config.conf_stat.curvilinear_integration_type) {
        case CurvilinearIntegrationType::GAUSS_LEGENDRE: {

            int n_points = static_cast<int>(config.conf_stat.N_points_gauss_legendre);
            if (ImGui::InputInt("Number of Gauss points", &n_points)) {
                // Ensure the number is positive
                n_points = std::max(1, n_points);
                config.conf_stat.N_points_gauss_legendre = static_cast<uint>(n_points);
            }
            break;
        }
        case CurvilinearIntegrationType::PRE_DERIVED:
            ImGui::Checkbox("Curvature strains", &config.conf_stat.curvature_enabled);
            ImGui::Checkbox("2nd order beam stiffness", &config.conf_stat.beam_stiffness_2nd_order_enabled);
            break;
        default:
            break;
        }
    }
}

void UI_Manager::call_imgui_physics_control_panel(Config &config) {

    imgui_manager->toggle_button("Gravity Enabled", &config.conf_stat.gravity_enabled,
                                 "Warning: Disabling gravity may affect simulation stability");

    if (config.conf_stat.string_type == StringType::DRILL_STRING) {
        float ec_amplifier = static_cast<float>(config.conf_stat.ec_amplifier);
        // Show as percentage [0, 100], but store as fraction [0, 1]
        float ecc_percent_display = ec_amplifier * 100.0f;

        ImGui::Text("Mass eccentricity amplifier [%] of radial mass imbalance");
        ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
        if (ImGui::SliderFloat("##ecc_percent_display", &ecc_percent_display, 0.0f, 1000.0f, "%.2f")) {
            config.conf_stat.ec_amplifier = static_cast<scalar>(ecc_percent_display / 100.0f);
            config.reassemble_mass_vectors = true;
        }
    }

    if (ImGui::CollapsingHeader("Contact Controls")) {
        imgui_manager->toggle_button("Contact Enabled", &config.conf_stat.contact_enabled,
                                     "Warning: Disabling contact may affect simulation stability");

        if (config.conf_stat.contact_enabled) {
            // Add input fields for alpha and beta parameters
            float K_normal = static_cast<float>(config.conf_stat.K_normal);
            float C_normal = static_cast<float>(config.conf_stat.C_normal);
            float K_tangent = static_cast<float>(config.conf_stat.K_tangent);
            float C_tangent = static_cast<float>(config.conf_stat.C_tangent);
            float mu_kinetic = static_cast<float>(config.conf_stat.mu_kinetic);
            float mu_static = static_cast<float>(config.conf_stat.mu_static);
            float crit_stribeck_vel = static_cast<float>(config.conf_stat.critical_stribeck_velocity);

            ImGui::Text("K - contact stiffness");
            ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
            if (ImGui::SliderFloat("##K_normal", &K_normal, 10000.0f, 1000000000.0f, "%.0f",
                                   ImGuiSliderFlags_Logarithmic)) {
                config.conf_stat.K_normal = static_cast<scalar>(K_normal);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Wall stiffness parameter (logarithmic scale)");
            }

            ImGui::Text("C - normal contact damping");
            ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
            if (ImGui::SliderFloat("##C_normal", &C_normal, 0.0f, 100000.0f, "%.0f", ImGuiSliderFlags_Logarithmic)) {
                config.conf_stat.C_normal = static_cast<scalar>(C_normal);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Normal contact damping parameter (logarithmic scale)");
            }

            ImGui::Text("K - tangential contact stiffness");
            ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
            if (ImGui::SliderFloat("##K_tangent", &K_tangent, 0.0f, 1000000000.0f, "%.0f",
                                   ImGuiSliderFlags_Logarithmic)) {
                config.conf_stat.K_tangent = static_cast<scalar>(K_tangent);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Wall stiffness parameter (logarithmic scale)");
            }

            ImGui::Text("C - tangential contact damping");
            ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
            if (ImGui::SliderFloat("##C_tangent", &C_tangent, 0.0f, 1000000.0f, "%.0f", ImGuiSliderFlags_Logarithmic)) {
                config.conf_stat.C_tangent = static_cast<scalar>(C_tangent);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Tangential contact damping parameter (logarithmic scale)");
            }

            ImGui::Text("Mu kinetic friction");
            ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
            if (ImGui::SliderFloat("##mu_kinetic", &mu_kinetic, 0.0f, 1.0f, "%.2f")) {
                config.conf_stat.mu_kinetic = static_cast<scalar>(mu_kinetic);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Kinetic friction coefficient");
            }

            ImGui::Text("Mu static friction");
            ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
            if (ImGui::SliderFloat("##mu_static", &mu_static, 0.0f, 1.0f, "%.2f")) {
                config.conf_stat.mu_static = static_cast<scalar>(mu_static);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Static friction coefficient");
            }

            ImGui::Text("Critical Stribeck velocity");
            ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
            if (ImGui::SliderFloat("##Critical_Stribeck_Velocity", &crit_stribeck_vel, SMALL_SCALAR, 5.0f, "%.1f")) {
                config.conf_stat.critical_stribeck_velocity = static_cast<scalar>(crit_stribeck_vel);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Critical Stribeck velocity, smooths between kinetic and static Coulomb friction");
            }

            if (config.conf_stat.string_type == StringType::CASING_STRING) {

                float K_bs = static_cast<float>(config.conf_stat.K_bs);
                float C_bs = static_cast<float>(config.conf_stat.C_bs);
                uint N_bs = config.conf_stat.N_bs;
                float p_bs = static_cast<float>(config.conf_stat.p_bs);

                ImGui::Text("K_bs - bow-spring stiffness");
                ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
                if (ImGui::SliderFloat("##K_bow_spring", &K_bs, 0.0f, 100000000.0f, "%.0f",
                                       ImGuiSliderFlags_Logarithmic)) {
                    config.conf_stat.K_bs = static_cast<scalar>(K_bs);
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Bow-spring stiffness parameter (logarithmic scale)");
                }

                ImGui::Text("C_bs - bow-spring damping");
                ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
                if (ImGui::SliderFloat("##C_bow_spring", &C_bs, 0.0f, 1000000.0f, "%.0f",
                                       ImGuiSliderFlags_Logarithmic)) {
                    config.conf_stat.C_bs = static_cast<scalar>(C_bs);
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Bow-spring damping parameter (logarithmic scale)");
                }

                ImGui::Text("N_bs - bow-spring number of blades");
                ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
                if (ImGui::SliderInt("##N_bow_spring", reinterpret_cast<int *>(&N_bs), 1, 6)) {
                    config.conf_stat.N_bs = static_cast<uint>(N_bs);
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Number of bow-spring blades");
                }

                ImGui::Text("p_bs - bow-spring stiffness exponent");
                ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
                if (ImGui::SliderFloat("##p_bow_spring", &p_bs, 1.0f, 3.0f)) {
                    config.conf_stat.p_bs = static_cast<scalar>(p_bs);
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Bow-spring stiffness power-law exponent");
                }
            }
        }
    }

    if (ImGui::CollapsingHeader("Damping Controls")) {
        imgui_manager->toggle_button("Rayleigh Damping Enabled", &config.conf_stat.rayleigh_damping_enabled,
                                     "Enable/Disable Rayleigh damping");

        if (config.conf_stat.rayleigh_damping_enabled) {

            // Add input fields for alpha and beta parameters
            float alpha = static_cast<float>(config.conf_stat.alpha);
            float beta = static_cast<float>(config.conf_stat.beta);

            ImGui::Text("Mass Proportional Damping");
            ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
            if (ImGui::InputFloat("Alpha", &alpha, 0.01f, 0.1f, "%.4f")) {
                config.conf_stat.alpha = static_cast<scalar>(alpha);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Mass proportional damping");
            }

            ImGui::Text("Stiffness Proportional Damping");
            ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
            if (ImGui::InputFloat("Beta", &beta, 0.0000001f, 0.000001f, "%.8f")) {
                config.conf_stat.beta = static_cast<scalar>(beta);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Stiffness proportional damping");
            }
        }
    }

    if (config.pipe_solver_type == PipeSolverType::CURVILINEAR) {
        if (ImGui::CollapsingHeader("Fluid Controls")) {

            float rho_fi = static_cast<float>(config.conf_stat.rho_fi);
            ImGui::Text("Inner fluid Density (kg/m^3)");
            ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
            ImGui::InputFloat("##FluidDensity", &rho_fi, 0.01f, 0.1f, "%.4f");
            config.conf_stat.rho_fi = static_cast<scalar>(rho_fi);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Inner fluid density");
            }

            float rho_fo = static_cast<float>(config.conf_stat.rho_fo);
            ImGui::Text("Outer fluid Density (kg/m^3)");
            ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
            ImGui::InputFloat("##OuterFluidDensity", &rho_fo, 0.01f, 0.1f, "%.4f");
            config.conf_stat.rho_fo = static_cast<scalar>(rho_fo);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Outer fluid density");
            }

            imgui_manager->toggle_button(
                "Fluid dynamics enabled", &config.conf_stat.fluid_dynamics_enabled,
                "Enables dynamic fluid effects such as dynamic pressure forces and viscous forces ");

            if (config.conf_stat.fluid_dynamics_enabled) {
                ImGui::Indent(ui_utils.INDENT_SIZE); // Indent the sub-items

                // Display and input fluid flow rate in cubic meters per minute (m^3/min)
                float Q_f_m3min = static_cast<float>(config.conf_stat.Q_f) * 60.0f;

                if (imgui_manager->InputFloatClamped("Fluid Flow Rate (m^3/min)", &Q_f_m3min, 0.0f, 10.0f, 0.1f, 0.01f,
                                                     "%.2f")) {
                    config.conf_stat.Q_f = static_cast<scalar>(Q_f_m3min / 60.0f);
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Fluid flow rate (cubic meters per minute)");
                }

                float mu_f = static_cast<float>(config.conf_stat.mu_f);

                if (ImGui::InputFloat("Fluid Viscosity (Pa.s)", &mu_f, 0.0001f, 0.001f, "%.6f")) {
                    config.conf_stat.mu_f = static_cast<scalar>(mu_f);
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Fluid viscosity");
                }
                ImGui::Unindent(ui_utils.INDENT_SIZE);
            }
            if (!config.conf_stat.contact_enabled) {
                config.conf_stat.curvature_enabled = false;
                config.conf_stat.beam_stiffness_2nd_order_enabled = false;
            }
        }
    }
}

void UI_Manager::call_imgui_top_control_control_panel(Config &config) {

    if (ImGui::CollapsingHeader("Top BC Controls")) {
        // The rest of your content goes here, and will be shown/hidden when the header is expanded/collapsed.

        ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);

        // Convert m/s to m/h for display
        float v_top_mh = static_cast<float>(config.conf_stat.v_top_input * 3600.0);
        ImGui::Text("Top Velocity (m/h)");
        ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH); // Use your sidebar width or a fixed value
        if (ImGui::InputFloat("##TopVelocity", &v_top_mh, 1.0f, 10.0f, "%.1f")) {
            config.conf_stat.v_top_input = static_cast<scalar>(v_top_mh / 3600.0);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Target rate of penetration");
        }
        float omega_top_rpm = static_cast<float>(config.conf_stat.omega_top_input * 30.0 / M_PI);

        ImGui::Text("Top Angular Velocity (rpm)");
        ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
        if (ImGui::InputFloat("##TargetAngularVelocity", &omega_top_rpm, 1.0f, 10.0f, "%.1f")) {
            config.conf_stat.omega_top_input = static_cast<scalar>(omega_top_rpm * M_PI / 30.0);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Rotation speed at the top drive");
        }

        if (config.conf_stat.bc_top_kinematics_type == BC_TopKinematicsType::NEUMANN) {

            ImGui::Separator();

            ImGui::Text("Top-drive PID Controller Gains");

            ImGui::Indent(ui_utils.INDENT_SIZE);

            // PID Gains inputs
            float PID_Kp = static_cast<float>(config.conf_stat.PID_Kp);
            float PID_Ki = static_cast<float>(config.conf_stat.PID_Ki);
            float PID_Kd = static_cast<float>(config.conf_stat.PID_Kd);

            ImGui::Text("PID: Proportional (Kp)");
            ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
            if (ImGui::InputFloat("##PID_Kp", &PID_Kp, 100.0f, 1000.0f, "%.1f")) {
                config.conf_stat.PID_Kp = static_cast<scalar>(std::max(0.0f, PID_Kp));
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Increase the effective damping of the top-drive");
            }

            ImGui::Text("PID: Integral (Ki)");
            ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
            if (ImGui::InputFloat("##PID_Ki", &PID_Ki, 10.0f, 100.0f, "%.1f")) {
                config.conf_stat.PID_Ki = static_cast<scalar>(std::max(0.0f, PID_Ki));
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Increase the effective stiffness of the top-drive");
            }

            ImGui::Text("PID: Derivative (Kd)");
            ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
            if (ImGui::InputFloat("##PID_Kd", &PID_Kd, 1.0f, 10.0f, "%.1f")) {
                config.conf_stat.PID_Kd = static_cast<scalar>(std::max(0.0f, PID_Kd));
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Increase the effective inertia, helps with stability");
            }

            ImGui::Unindent(ui_utils.INDENT_SIZE);
        }
    }
}

void UI_Manager::call_imgui_bottom_bc_control_panel(Config &config) {

    if (ImGui::CollapsingHeader("Bottom BC Controls")) {
        if (config.conf_stat.bc_bit_rock_type != BC_BitRockType::NO_BIT) {

            ImGui::Text("Bit-rock settings");

            // Bit geometry
            const uint n_blades = static_cast<uint>(config.conf_stat.n_blades);
            ImGui::Text("Number of Blades: %u", n_blades);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Not modifiable during simulation");
            }

            const float r_bit = static_cast<float>(config.conf_stat.r_br);
            ImGui::Text("Bit Radius (m): %.3f", r_bit);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Not runtime modifiable");
            }

            ImGui::Text("Bit-rock cutting settings");
            ImGui::Checkbox("Use Analytical Cutting", &config.conf_stat.bc_rock_cutting_analytical);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Enables analytical cutting model for bit-rock interaction");
            }

            float m_bit = static_cast<float>(config.conf_stat.m_br);
            if (ImGui::InputFloat("Drill bit mass (kg)", &m_bit, 0.0f, 10.0f, "%.3f")) {
                config.conf_stat.m_br = static_cast<scalar>(m_bit);
                config.reassemble_mass_vectors = true;
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Mass of the drill bit");
            }

            float J_bit = static_cast<float>(config.conf_stat.J_br);
            if (ImGui::InputFloat("Drill bit inertia (kg*m^2)", &J_bit, 0.0f, 10.0f, "%.3f")) {
                config.conf_stat.J_br = static_cast<scalar>(J_bit);
                config.reassemble_mass_vectors = true;
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Inertia of the drill bit");
            }

            if (config.conf_stat.bc_bit_rock_type == BC_BitRockType::DETOURNAY ||
                config.conf_stat.bc_bit_rock_type == BC_BitRockType::DETOURNAY_REGULARIZED) {

                // Cutter parameters
                float xi = static_cast<float>(config.conf_stat.xi_br);
                if (ImGui::InputFloat("Cutter Inclination (-)", &xi, 0.01f, 0.1f, "%.3f")) {
                    config.conf_stat.xi_br = static_cast<scalar>(xi);
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Inclination angle of the cutter face relative to the cutting direction");
                }
                // Convert Pa to MPa for display
                float epsilon_MPa = static_cast<float>(config.conf_stat.epsilon_br / 1.0e6);
                if (ImGui::InputFloat("Rock Strength (MPa)", &epsilon_MPa, 0.1f, 1.0f, "%.1f")) {
                    config.conf_stat.epsilon_br = static_cast<scalar>(epsilon_MPa * 1.0e6);
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Intrinsic specific energy of the rock (energy required to cut unit volume)");
                }

                float gamma = static_cast<float>(config.conf_stat.gamma_br);
                if (ImGui::InputFloat("Bit Geometry Parameter (-)", &gamma, 0.01f, 0.1f, "%.3f")) {
                    config.conf_stat.gamma_br = static_cast<scalar>(gamma);
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Parameter relating cutting area to depth of cut");
                }
                // Convert Pa to MPa for display
                float sigma_MPa = static_cast<float>(config.conf_stat.sigma_br / 1.0e6);
                if (ImGui::InputFloat("Contact Stress (MPa)", &sigma_MPa, 0.1f, 1.0f, "%.1f")) {
                    config.conf_stat.sigma_br = static_cast<scalar>(sigma_MPa * 1.0e6);
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Normal contact stress between wear flat and rock surface");
                }
                float l = static_cast<float>(config.conf_stat.l_br);
                if (ImGui::InputFloat("Wear Flat Length (m)", &l, 0.0001f, 0.001f, "%.4f")) {
                    config.conf_stat.l_br = static_cast<scalar>(l);
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Length of the wear flat surface on the cutter");
                }
                float mu = static_cast<float>(config.conf_stat.mu_br);
                if (ImGui::InputFloat("Friction Coefficient (-)", &mu, 0.01f, 0.1f, "%.3f")) {
                    config.conf_stat.mu_br = static_cast<scalar>(mu);
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Friction coefficient between wear flat and rock surface");
                }

                if (config.conf_stat.bc_bit_rock_type == BC_BitRockType::DETOURNAY_REGULARIZED) {

                    float e_reg = static_cast<float>(config.conf_stat.e_reg);
                    ImGui::Text("Regularization parameter");
                    ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
                    if (ImGui::SliderFloat("##RegularizationSlider", &e_reg, 1.0f, 100.0f, "%.1f")) {
                        config.conf_stat.e_reg = static_cast<scalar>(e_reg);
                    }
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip(
                            "Regularization parameter for the  Detournay & Defourny regularized cutting model");
                    }
                }

            }

            else if (config.conf_stat.bc_bit_rock_type == BC_BitRockType::TUCKER_WANG ||
                     config.conf_stat.bc_bit_rock_type == BC_BitRockType::TUCKER_WANG_TORSIONAL) {
                float a0 = static_cast<float>(config.conf_stat.a0_br);
                if (ImGui::InputFloat("a0 Constant (-)", &a0)) {
                    config.conf_stat.a0_br = static_cast<scalar>(a0);
                }
                float a1 = static_cast<float>(config.conf_stat.a1_br);
                if (ImGui::InputFloat("a1 Constant (-)", &a1)) {
                    config.conf_stat.a1_br = static_cast<scalar>(a1);
                }
                float a2 = static_cast<float>(config.conf_stat.a2_br);
                if (ImGui::InputFloat("a2 Constant (-)", &a2)) {
                    config.conf_stat.a2_br = static_cast<scalar>(a2);
                }

                float a3 = static_cast<float>(config.conf_stat.a3_br);
                if (ImGui::InputFloat("a3 Constant (-)", &a3)) {
                    config.conf_stat.a3_br = static_cast<scalar>(a3);
                }
                if (config.conf_stat.bc_bit_rock_type == BC_BitRockType::TUCKER_WANG) {
                    float a4 = static_cast<float>(config.conf_stat.a4_br);
                    if (ImGui::InputFloat("a4 Constant (-)", &a0)) {
                        config.conf_stat.a4_br = static_cast<scalar>(a4);
                    }
                }
            }

            ImGui::Text("Bit-rock regularization parameters");

            float e_heaviside_reg = static_cast<float>(config.conf_stat.e_heaviside_br);
            ImGui::Text("Heaviside Regularization");
            ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
            if (ImGui::SliderFloat("##HeavisideRegSlider", &e_heaviside_reg, 1e-6f, 0.99f, "%.6f",
                                   ImGuiSliderFlags_Logarithmic)) {
                config.conf_stat.e_heaviside_br = static_cast<scalar>(e_heaviside_reg);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Regularization parameter for the Heaviside function (logarithmic scale, 0 < e < 1)");
            }

            float e_ramp_reg = static_cast<float>(config.conf_stat.e_ramp_br);
            ImGui::Text("Ramp Regularization - Depth of Cut");
            ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
            if (ImGui::SliderFloat("##RampRegSlider", &e_ramp_reg, 1e-6f, 0.99f, "%.6f",
                                   ImGuiSliderFlags_Logarithmic)) {
                config.conf_stat.e_ramp_br = static_cast<scalar>(e_ramp_reg);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(
                    "Regularization parameter for the Ramp function of depth of cut (logarithmic scale, 0 < e < 1)");
            }

            float e_sign_reg = static_cast<float>(config.conf_stat.e_sign_br);
            ImGui::Text("Sign Regularization - Angular Velocity");
            ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
            if (ImGui::SliderFloat("##SignRegSlider", &e_sign_reg, 1e-6f, 0.99f, "%.6f",
                                   ImGuiSliderFlags_Logarithmic)) {
                config.conf_stat.e_sign_br = static_cast<scalar>(e_sign_reg);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(
                    "Regularization parameter for the Sign function of omega (logarithmic scale, 0 < e < 1)");
            }

            float tol_heaviside_reg = static_cast<float>(config.conf_stat.tol_heaviside_br);
            ImGui::Text("Heaviside Tolerance");
            ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
            if (ImGui::SliderFloat("##HeavisideTolSlider", &tol_heaviside_reg, 1e-6f, 0.99f, "%.6f",
                                   ImGuiSliderFlags_Logarithmic)) {
                config.conf_stat.tol_heaviside_br = static_cast<scalar>(tol_heaviside_reg);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Tolerance parameter for the Heaviside function (logarithmic scale, 0 < e < 1)");
            }

            float tol_ramp_reg = static_cast<float>(config.conf_stat.tol_ramp_br);
            ImGui::Text("Ramp Tolerance");
            ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
            if (ImGui::SliderFloat("##RampTolSlider", &tol_ramp_reg, 1e-6f, 0.99f, "%.6f",
                                   ImGuiSliderFlags_Logarithmic)) {
                config.conf_stat.tol_ramp_br = static_cast<scalar>(tol_ramp_reg);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Tolerance parameter for the Ramp function (logarithmic scale, 0 < e < 1)");
            }
        }

        ImGui::Text("Static bottom forces");
        float kWOB = static_cast<float>(config.conf_stat.WOB_constant) / 1000.0;
        float kTOB = static_cast<float>(config.conf_stat.TOB_constant) / 1000.0;
        // Input field with direct numeric input
        if (ImGui::InputFloat("WOB (kN)", &kWOB, 0.0f, 1000.0f, "%.0f")) {
            config.conf_stat.WOB_constant = static_cast<scalar>(kWOB * 1000);
        }
        // Optional: Add tooltips for explanation
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Constant weight on bit");
        }
        if (ImGui::InputFloat("TOB (kNm)", &kTOB, 0.0f, 100.0f, "%.0f")) {
            config.conf_stat.TOB_constant = static_cast<scalar>(kTOB * 1000);
        }
        // Optional: Add tooltips for explanation
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Constant torque on bit");
        }
    }
}

void UI_Manager::call_imgui_orbital_plot_control_panel(Config &config, const Pipe &pipe, const uint top_node,
                                                       const byte *buf) {
    if (ImGui::CollapsingHeader("Orbital plots")) {

        float &S_input = ui_settings.slider_S_orbital;

        // Add new orbital plot control
        float L_max = (float)pipe.L_tot;

        ImGui::Text("Add new orbital plot:");
        // Use an input box with clamped range [0, L_max]
        if (imgui_manager->InputFloatClamped("##New S Parameter", &S_input, 0.0f, L_max, 0.1f, 0.01f, "%.2f")) {
            S_input = std::min(std::max(S_input, 0.0f), L_max);
            ImGui::Text("Will add at S = %.2f [m]", S_input);
        }

        if (ImGui::Button("Add Orbital Plot")) {
            // Find closest node
            const ArrayView<scalar> S_nodes = pipe.get_field<PipeField::S>(buf);
            uint closest_node = 0;
            // Clamp S_input to [S_nodes[0], S_nodes[pipe.N-1]]
            if (S_input <= S_nodes[0]) {
                closest_node = 0;
            } else if (S_input >= S_nodes[pipe.N - 1]) {
                closest_node = pipe.N - 1;
            } else {
                scalar min_dist = std::abs(S_nodes[0] - S_input);
                for (uint i = 1; i < pipe.N; ++i) {
                    scalar dist = std::abs(S_nodes[i] - S_input);
                    if (dist < min_dist) {
                        min_dist = dist;
                        closest_node = i;
                    }
                }
            }
            scalar S_node = S_nodes[closest_node];
            orbital_plots.push_back({S_node, closest_node, false, {}});

            // Add a corresponding tile to the main plot window
            uint orbital_plot_index = orbital_plots.size() - 1;
            string title = "Orbital Plot Node " + to_string(closest_node) + ", S = " + to_string((int)S_node) + "m";
            plot_tiles.push_back({PlotTileType::ORBITAL_PLOT, title, 1, 1, true, 0, 0, orbital_plot_index, 0, 0});
        }
        ImGui::Separator();

        // List and control existing plots
        ImGui::Text("Active orbital plots:");
        for (uint i = 0; i < orbital_plots.size(); i++) {
            OrbitalPlot &plot = orbital_plots[i];

            ImGui::PushID(i);
            ImGui::Text("Plot %d (S = %.2f m)", i + 1, plot.S_plot);

            if (!plot.is_recording) {
                if (ImGui::Button("Start Trail")) {
                    plot.is_recording = true;
                    plot.trail_points.clear();
                }
            } else {
                if (ImGui::Button("Stop Trail")) {
                    plot.is_recording = false;
                }
            }

            ImGui::SameLine();
            if (!plot.trail_points.empty()) {
                if (ImGui::Button("Reset Trail")) {
                    plot.trail_points.clear();
                }
                ImGui::SameLine();
            }

            if (ImGui::Button("Remove")) {
                // Remove the corresponding tile first
                // Find and remove the tile that corresponds to this orbital plot
                for (auto it = plot_tiles.begin(); it != plot_tiles.end(); ++it) {
                    if (it->type == PlotTileType::ORBITAL_PLOT && it->orbital_plot_index == i) {
                        plot_tiles.erase(it);
                        break;
                    }
                }

                // Update tile indices for plots that come after this one
                for (auto &tile : plot_tiles) {
                    if (tile.type == PlotTileType::ORBITAL_PLOT && tile.orbital_plot_index > i) {
                        tile.orbital_plot_index--;
                        // Update title as well
                        uint node = orbital_plots[tile.orbital_plot_index].node;
                        float S_plot = static_cast<float>(orbital_plots[tile.orbital_plot_index].S_plot);
                        char s_buf[16];
                        snprintf(s_buf, sizeof(s_buf), "%.1f", S_plot);
                        std::string title = "Orbital Plot Node " + std::to_string(node) + ", S = " + s_buf + " m";
                    }
                }

                // Remove the orbital plot
                orbital_plots.erase(orbital_plots.begin() + i);
                i--; // Adjust index after removal
            }

            ImGui::PopID();
            ImGui::Separator();
        }
    }
}

void UI_Manager::call_imgui_orbital_subplot(OrbitalPlot &plot, Config &config, const curvlin::PipeSolver &pipesolver,
                                            const Pipe &pipe, const Hole &hole, const byte *buf) {
    using namespace curvlin;
    const BeamData &beam = pipesolver.beam;
    const ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);
    const ArrayView<scalar> S = pipe.get_field<PipeField::S>(buf);
    ArrayView<uint> i_pipe_to_ie_hole = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);
    ArrayView<scalar> a_arr = hole.get_field<HoleField::a>(buf);
    ArrayView<scalar> b_arr = hole.get_field<HoleField::b>(buf);
    ArrayView<scalar> alpha_arr = hole.get_field<HoleField::alpha>(buf);
    ArrayView<Vec2> co_arr = hole.get_field<HoleField::co>(buf);
    ArrayView<Quaternion> quat_arr = hole.get_field<HoleField::q>(buf);
    const ArrayView<Vec3> theta = beam.get_field<BeamField::theta>(buf);
    const ArrayView<scalar> S_nodes = pipe.get_field<PipeField::S>(buf);

    uint node = plot.node;
    scalar S_node = S_nodes[node];
    uint i_hole = i_pipe_to_ie_hole[node];

    // Get nodal displacements
    scalar ux = theta[node].x();
    scalar uy = u[node].y();
    scalar uz = u[node].z();

    // Get hole center at this node (if needed)
    const ArrayView<scalar> s_arr = hole.get_field<HoleField::s>(buf);

    scalar s_i = pipe.calc_s(node, ux, buf);

    Vec2 co_hole = hole.lerp_hole_property_from_pipe_node_position<Vec2>(co_arr, s_i, i_hole, buf);
    scalar a = hole.lerp_hole_property_from_pipe_node_position<scalar>(a_arr, s_i, i_hole, buf);
    scalar b = hole.lerp_hole_property_from_pipe_node_position<scalar>(b_arr, s_i, i_hole, buf);
    scalar alpha = hole.lerp_hole_property_from_pipe_node_position<scalar>(alpha_arr, s_i, i_hole, buf);

    const scalar s_hole_i = s_arr[i_hole];
    const scalar s_hole_ip = s_arr[i_hole + 1];

    const scalar t = max(0.0, min(1.0, (s_i - s_hole_i) / (s_hole_ip - s_hole_i)));

    const Quaternion &q_hole_ih = quat_arr[i_hole];
    const Quaternion &q_hole_ihp = quat_arr[i_hole + 1];
    const Quaternion quat_hole = Quaternion::lerp(q_hole_ih, q_hole_ihp, t);

    // Pipe radius
    ArrayView<scalar> rc = pipe.get_field<PipeField::rc>(buf);
    ArrayView<uint> num_ic_to_ie = pipe.get_field<PipeField::num_ic_to_ie>(buf);
    uint ic = min(num_ic_to_ie[node], pipe.Nc - 1);
    scalar r_pipe = rc[ic];

    // const Mat3 R = quat_hole.to_matrix();
    // const Vec3 tangent = R.col(0);

    // Define up and right for the plot frame
    // Vec3 up = Vec3(0, 0, 1); // global Z
    // Vec3 right;

    // If tangent is nearly vertical, use global Y as right
    // if (std::abs(tangent.dot(up)) > 0.99) {
    //    right = Vec3(0, 1, 0); // global Y
    //} else {
    //    // Project global Y onto the plane normal to tangent
    //    right = Vec3(0, 1, 0) - tangent * tangent.dot(Vec3(0, 1, 0));
    //    right.normalize();
    //}

    // Record trail point if active
    if (plot.is_recording) {
        plot.trail_points.push_back({uy, uz});
    }

    // Set axis limits based on hole dimensions with some padding, always centered at (0,0)
    scalar max_dim = std::max(a, b);
    // scalar offset_norm = std::sqrt(co_hole[0] * co_hole[0] + co_hole[1] * co_hole[1]);
    scalar offset_norm = 0.0;
    scalar padding = 0.5 * max_dim;
    scalar plot_limit = std::max(max_dim + padding, offset_norm + r_pipe + padding);

    ImPlot::SetupAxis(ImAxis_X1, "Y [m]");
    ImPlot::SetupAxis(ImAxis_Y1, "Z [m]");
    ImPlot::SetupAxisLimits(ImAxis_X1, -plot_limit, plot_limit, ImGuiCond_Always);
    ImPlot::SetupAxisLimits(ImAxis_Y1, -plot_limit, plot_limit, ImGuiCond_Always);

    // Plot hole cross-section (ellipse)
    constexpr uint n_points = 100;
    std::array<float, n_points> hole_y, hole_z, pipe_y, pipe_z;

    for (uint j = 0; j < n_points; j++) {
        float psi = 2.0f * M_PI * j / (n_points - 1);
        // Hole ellipse
        hole_y[j] = a * cos(psi) * cos(alpha) - b * sin(psi) * sin(alpha);
        hole_z[j] = a * cos(psi) * sin(alpha) + b * sin(psi) * cos(alpha);

        // Pipe circle
        pipe_y[j] = r_pipe * cos(psi) + uy;
        pipe_z[j] = r_pipe * sin(psi) + uz;
    }

    ImPlot::PlotLine("Hole", hole_y.data(), hole_z.data(), n_points);
    ImPlot::PlotLine("Pipe", pipe_y.data(), pipe_z.data(), n_points);
    ImPlot::PlotScatter("Pipe Center", &uy, &uz, 1);

    // Plot hole centerline at (0,0)
    float zero_y = 0.0f, zero_z = 0.0f;
    ImPlot::PushStyleVar(ImPlotStyleVar_Marker, ImPlotMarker_Cross);
    ImPlot::PushStyleVar(ImPlotStyleVar_MarkerSize, 10.0f);
    ImPlot::PushStyleColor(ImPlotCol_MarkerOutline, IM_COL32(0, 0, 255, 255));
    ImPlot::PlotScatter("Hole Centerline", &zero_y, &zero_z, 1);
    ImPlot::PopStyleColor();
    ImPlot::PopStyleVar(2);

    // If offset is nonzero, plot the offset marker
    if (offset_norm > 1e-8f) {
        float offset_y = co_hole[0];
        float offset_z = co_hole[1];
        ImPlot::PushStyleVar(ImPlotStyleVar_Marker, ImPlotMarker_Square);
        ImPlot::PushStyleVar(ImPlotStyleVar_MarkerSize, 8.0f);
        ImPlot::PushStyleColor(ImPlotCol_MarkerOutline, IM_COL32(255, 0, 0, 255));
        ImPlot::PlotScatter("Hole Offset", &offset_y, &offset_z, 1);
        ImPlot::PopStyleColor();
        ImPlot::PopStyleVar(2);
    }

    // Plot trail if any points exist
    if (!plot.trail_points.empty()) {
        std::vector<float> trail_y, trail_z;
        for (const auto &point : plot.trail_points) {
            trail_y.push_back(point.first);
            trail_z.push_back(point.second);
        }
        ImPlot::PlotLine("Trail", trail_y.data(), trail_z.data(), trail_y.size());
    }
}

void UI_Manager::call_imgui_spatial_graph_plot_control_panel(Config &config, array<SpatialGraph, N_SG> &spatial_graphs,
                                                             array<MiscSpatialGraph, N_MG> &misc_spatial_graphs,
                                                             array<FluidSpatialGraph, N_FG> &fluid_spatial_graphs) {

    // Create ImGui control window
    if (ImGui::CollapsingHeader("Spatial Graph Plots")) {

        static int selected_index = 0;
        constexpr int num_items = (uint)PlotSpatialVariable::COUNT + 2; // +2 for misc and fluid graphs

        // Build combo labels from current graph instances:
        std::array<std::string, (uint)PlotSpatialVariable::COUNT + 2> combo_labels;
        combo_labels[0] = "Miscellaneous";
        combo_labels[1] = "Fluid Properties";
        for (int i = 2; i < num_items; ++i) {
            uint var_idx = (uint)(i - 2); // which PlotSpatialVariable
            uint sg_index = var_idx * 3;  // use the first of the three direction graphs
            if (sg_index < spatial_graphs.size()) {
                combo_labels[i] = spatial_graphs[sg_index].label;
            } else {
                combo_labels[i] = std::string("Unknown ") + std::to_string(var_idx);
            }
        }

        ImGui::Text("Select variables to add to graph plot:");
        ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
        if (ImGui::BeginCombo("##GraphComponent ", combo_labels[selected_index].c_str())) {
            for (int i = 0; i < num_items; ++i) {
                bool is_selected = (selected_index == i);
                if (ImGui::Selectable(combo_labels[i].c_str(), is_selected)) {
                    selected_index = i;
                }
                if (is_selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        bool misc_selected = selected_index == 0;
        bool fluid_selected = selected_index == 1;

        ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);

        string label = "Plot component ";

        if (ImGui::BeginCombo("##", label.c_str())) {
            if (misc_selected) {
                for (MiscSpatialGraph &misc_graph : misc_spatial_graphs) {
                    if (ImGui::Checkbox(misc_graph.label.c_str(), &misc_graph.show_graph_plot)) {
                        sync_tiles_with_checkbox_states(config, spatial_graphs, misc_spatial_graphs,
                                                        fluid_spatial_graphs);
                    }
                }
            } else if (fluid_selected) {
                for (FluidSpatialGraph &fluid_graph : fluid_spatial_graphs) {
                    if (ImGui::Checkbox(fluid_graph.label.c_str(), &fluid_graph.show_graph_plot)) {
                        sync_tiles_with_checkbox_states(config, spatial_graphs, misc_spatial_graphs,
                                                        fluid_spatial_graphs);
                    }
                }
            } else {
                float current_ui_xpos = ImGui::GetCursorPosX();
                constexpr float button_width = 50.0f;
                for (uint i = 0; i < 3; i++) {
                    ImGui::SetCursorPosX(current_ui_xpos + i * button_width);
                    SpatialGraph &graph = spatial_graphs[(selected_index - 2) * 3 + i];
                    if (ImGui::Checkbox(directions_strings[i], &graph.show_graph_plot)) {
                        sync_tiles_with_checkbox_states(config, spatial_graphs, misc_spatial_graphs,
                                                        fluid_spatial_graphs);
                    }
                    ImGui::SameLine();
                }
            }
            ImGui::EndCombo();
        }

        ImGui::Separator();
        ImGui::Text("Plot limits control:");
        for (MiscSpatialGraph &graph : misc_spatial_graphs) {
            if (graph.show_graph_plot) {
                bool reset_scale = false;
                call_imgui_spatial_plot_control_limit_selection(graph.label, graph.y_min, graph.y_max, graph.s_min,
                                                                graph.s_max, graph.auto_size, reset_scale);
                if (reset_scale) {
                    graph.reset_scale();
                }
            }
        }
        for (FluidSpatialGraph &graph : fluid_spatial_graphs) {
            if (graph.show_graph_plot) {
                bool reset_scale = false;
                call_imgui_spatial_plot_control_limit_selection(graph.label, graph.y_min, graph.y_max, graph.s_min,
                                                                graph.s_max, graph.auto_size, reset_scale);
                if (reset_scale) {
                    graph.reset_scale();
                }
            }
        }
        for (SpatialGraph &graph : spatial_graphs) {
            if (graph.show_graph_plot) {
                bool reset_scale = false;
                call_imgui_spatial_plot_control_limit_selection(graph.get_variable_string(), graph.y_min, graph.y_max,
                                                                graph.s_min, graph.s_max, graph.auto_size, reset_scale);
                if (reset_scale) {
                    graph.reset_scale();
                }
            }
        }
    }
}

void UI_Manager::call_imgui_spatial_plot_control_limit_selection(string name, float &y_min, float &y_max, float &s_min,
                                                                 float &s_max, bool &auto_size, bool &reset_scale) {
    constexpr float step = 0.1f;
    constexpr float step_fast = 1.0f;

    ImGui::BeginGroup();
    ImGui::Indent(ui_utils.INDENT_SIZE);

    float settings_width = ui_utils.SETTINGS_WIDTH * 3.0f / 4.0f;

    ImGui::Text("%s Limits Control: ", name.c_str());

    bool prev_auto_size = auto_size;
    ImGui::Checkbox(("Auto scale##" + name).c_str(), &auto_size);
    if (!prev_auto_size && auto_size) {
        reset_scale = true;
    }

    ImGui::Text("Value limits : Min/Max");

    ImGui::SetNextItemWidth(settings_width);
    if (auto_size)
        ImGui::BeginDisabled();
    ImGui::InputFloat(("## Min " + name).c_str(), &y_min, step, step_fast, "%.2f");
    if (auto_size)
        ImGui::EndDisabled();

    ImGui::SameLine();

    ImGui::SetNextItemWidth(settings_width);
    if (auto_size)
        ImGui::BeginDisabled();
    ImGui::InputFloat(("## Max " + name).c_str(), &y_max, step, step_fast, "%.2f");
    if (auto_size)
        ImGui::EndDisabled();

    ImGui::SameLine();
    ImGui::Dummy(ImVec2(5.0f * ui_utils.scale, 0.0f));

    ImGui::Text("MD limits: Min/Max");

    ImGui::SetNextItemWidth(settings_width);
    if (auto_size)
        ImGui::BeginDisabled();
    ImGui::InputFloat(("## s min " + name).c_str(), &s_min, step_fast, step_fast + 2.0f, "%1.0f");
    if (auto_size)
        ImGui::EndDisabled();

    ImGui::SameLine();

    ImGui::SetNextItemWidth(settings_width);
    if (auto_size)
        ImGui::BeginDisabled();
    ImGui::InputFloat(("## s max " + name).c_str(), &s_max, step_fast, step_fast + 2.0f, "%1.0f");
    if (auto_size)
        ImGui::EndDisabled();

    ImGui::Unindent(ui_utils.INDENT_SIZE);
    ImGui::Spacing();

    ImGui::EndGroup();

    ImGui::GetWindowDrawList()->AddRect(ImGui::GetItemRectMin(), ImGui::GetItemRectMax(), IM_COL32(255, 255, 255, 255),
                                        5.0f);
}
void UI_Manager::call_imgui_contour_plot_control_panel(Config &config, ConfigGraphics &config_graphics,
                                                       array<SpatialGraph, N_SG> &spatial_graphs,
                                                       array<MiscSpatialGraph, N_MG> &misc_spatial_graphs,
                                                       array<FluidSpatialGraph, N_FG> &fluid_spatial_graphs) {

    // Create ImGui control window
    if (ImGui::CollapsingHeader("Contour Plots")) {
        ImGui::Text("Select variable for contour plot:");

        // 'None' option
        if (ImGui::RadioButton("None", config_graphics.contour_variable == ContourPlotVariable::NONE)) {
            config_graphics.contour_variable = ContourPlotVariable::NONE;
        }

        ImGui::Separator();

        // Build radio buttons from misc spatial graphs (use each graph's stored contour_variable)
        for (const auto &misc_graph : misc_spatial_graphs) {
            // Each misc_graph must have a unique label and an associated contour_variable value
            if (ImGui::RadioButton(misc_graph.get_graph_label().c_str(),
                                   config_graphics.contour_variable == misc_graph.contour_variable)) {
                config_graphics.contour_variable = misc_graph.contour_variable;
                config_graphics.colorbar_title = misc_graph.get_colorbar_label();
            }
        }

        // Fluid graphs
        for (const auto &fluid_graph : fluid_spatial_graphs) {
            if (ImGui::RadioButton(fluid_graph.get_graph_label().c_str(),
                                   config_graphics.contour_variable == fluid_graph.contour_variable)) {
                config_graphics.contour_variable = fluid_graph.contour_variable;
                config_graphics.colorbar_title = fluid_graph.get_colorbar_label();
            }
        }

        ImGui::Separator();

        ImGui::Checkbox("Grey outside limits", &config_graphics.grey_outside_limits);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("When enabled, values outside the display limits are shown as grey (visual cue)");
        }

        ImGui::Checkbox("Draw horizontal colorbar", &config_graphics.draw_horizontal_colorbar);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("When enabled, a horizontal colorbar is drawn instead of a vertical one");
        }

        ImGui::Checkbox("Auto scale limits", &config_graphics.contour_auto_limits);

        constexpr float step = 0.1f;
        constexpr float step_fast = 1.0f;
        if (config_graphics.contour_auto_limits)
            ImGui::BeginDisabled();
        ImGui::InputFloat("Max", &config_graphics.contour_lim_max, step, step_fast, "%.2f");
        ImGui::InputFloat("Min", &config_graphics.contour_lim_min, step, step_fast, "%.2f");
        if (config_graphics.contour_auto_limits) {
            ImGui::EndDisabled();
        }
    }
}

ImGuiWindowFlags UI_Manager::imgui_set_wiew_port() {
    // Create the dockspace
    ImGuiViewport *viewport = ImGui::GetMainViewport();

    const float DEFAULT_SIDEBAR_WIDTH = this->ui_utils.MENU_BAR_WIDTH; // use configurable width

    ImVec2 work_pos = viewport->WorkPos;
    ImVec2 work_size = viewport->WorkSize;

    float reserved = std::min(DEFAULT_SIDEBAR_WIDTH, work_size.x * 0.35f);
    ImVec2 pos = ImVec2(work_pos.x + reserved, work_pos.y);
    ImVec2 size = ImVec2(std::max(0.0f, work_size.x - reserved), work_size.y);

    ImGui::SetNextWindowPos(pos);
    ImGui::SetNextWindowSize(size);
    ImGui::SetNextWindowViewport(viewport->ID);

    ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;
    window_flags |=
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
    window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

    // Push style variables
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PopStyleVar(2); // Pop the style variables we pushed in imgui_set_wiew_port

    return window_flags;
}

void UI_Manager::render_imgui() {
    // Draw the sidebar after the dockspace has been created (and before final ImGui::Render)
    // so the sidebar stays visible (on top) even in fullscreen.

    imgui_manager->render();
    frame_data.imgui_frame_active = false;
}

void UI_Manager::init_simulation(Simulator *simulator, const array<SpatialGraph, N_SG> &spatial_graphs,
                                 const array<MiscSpatialGraph, N_MG> &misc_spatial_graphs,
                                 const array<FluidSpatialGraph, N_FG> &fluid_spatial_graphs) {

    const Pipe &pipe = simulator->pipe;
    const Hole &hole = simulator->hole;
    Config &config = simulator->config;
    const byte *buf = simulator->double_buffer_gui->get_arena_h().buf;

    const uint N = pipe.N;
    s_nodewise.resize(N);
    s_elementwise.resize(N + 1);

    hole_plot_data.allocate(hole, buf);
    bit_rock_plot.init_surface_and_limits(config.conf_stat.n_blades, config.L_hole_depth_initial,
                                          simulator->bit_rock_data.get_bit_rock_field(buf).N_cells);

    // --- Make all PlotGroups and their graphs visible on simulation startup ---
    for (auto &group : plot_groups) {
        group.show = true;
        for (auto &graph : group.graphs) {
            graph.show = true;
            graph.initialized = true;
        }
    }

    // After initializing plot_groups and setting show=true for all
    sync_tiles_with_checkbox_states(config, spatial_graphs, misc_spatial_graphs, fluid_spatial_graphs);
}

void UI_Manager::call_imgui_display_pipe_data(const std::vector<PipeComponent> &pipe) {
    if (ImGui::CollapsingHeader("Pipe Components", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::BeginTable("PipeTable", 12,
                              ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable)) {
            ImGui::TableSetupColumn("Name");
            ImGui::TableSetupColumn("Type");
            ImGui::TableSetupColumn("Length [m]");
            ImGui::TableSetupColumn("Do [mm]");
            ImGui::TableSetupColumn("Di [mm]");
            ImGui::TableSetupColumn("Tool Joint L [m]");
            ImGui::TableSetupColumn("Tool Joint D [mm]");
            ImGui::TableSetupColumn("Stabilizer D [mm]");
            ImGui::TableSetupColumn("Stabilizer L [m]");
            ImGui::TableSetupColumn("Sensor L [m]");
            ImGui::TableSetupColumn("Sensor S [m]");
            ImGui::TableSetupColumn("Special Flags");
            ImGui::TableHeadersRow();

            for (const auto &comp : pipe) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("%s", comp.name.c_str());
                ImGui::TableNextColumn();
                ImGui::Text("%s", base_string_from_component_type[static_cast<uint>(comp.type)]);
                ImGui::TableNextColumn();
                ImGui::Text("%.3f", comp.L);
                ImGui::TableNextColumn();
                ImGui::Text("%.1f", comp.Do * 1000.0f);
                ImGui::TableNextColumn();
                ImGui::Text("%.1f", comp.Di * 1000.0f);
                ImGui::TableNextColumn();
                ImGui::Text("%.3f", comp.L_tool);
                ImGui::TableNextColumn();
                ImGui::Text("%.1f", comp.D_tool * 1000.0f);
                ImGui::TableNextColumn();
                ImGui::Text("%.1f", comp.D_stabilizer * 1000.0f);
                ImGui::TableNextColumn();
                ImGui::Text("%.3f", comp.L_stabilizer);
                ImGui::TableNextColumn();
                ImGui::Text("%.3f", comp.L_sensor);
                ImGui::TableNextColumn();
                ImGui::Text("%.3f", comp.S_sensor);

                // Special flags
                ImGui::TableNextColumn();
                std::string flags;
                if (comp.has_tool_joints)
                    flags += "ToolJoint ";
                if (comp.has_sensors)
                    flags += "Sensor ";
                if (comp.has_damping)
                    flags += "Damping ";
                if (comp.has_steering)
                    flags += "Steering ";
                if (comp.has_stabilizer)
                    flags += "Stabilizer ";
                if (comp.has_mass_imbalance)
                    flags += "MassImbalance ";
                ImGui::Text("%s", flags.c_str());
            }
            ImGui::EndTable();
        }
    }
}

void UI_Manager::plot_single_graph(const scalar *y_data, const scalar *s_data, uint N_start, uint N_end, scalar y_min,
                                   scalar y_max, scalar s_min, scalar s_max, const string &y_label,
                                   const string &legend, const string &plot_title) {

    constexpr char *x_label = "Measured depth [m]";
    const uint N_active = N_end - N_start;

    if (ImPlot::BeginPlot(("##" + plot_title).c_str(), ImVec2(-1, -1))) {
        ImPlot::SetupAxis(ImAxis_X1, x_label);
        ImPlot::SetupAxis(ImAxis_Y1, y_label.c_str());
        ImPlot::SetupAxisLimits(ImAxis_X1, s_min, s_max, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, y_min, y_max, ImGuiCond_Always);

        ImPlot::PlotLine(legend.c_str(),
                         s_data + N_start, // s data starting at N_start
                         y_data + N_start, // y data starting at N_start + N_offset
                         N_active,         // number of points to plot
                         ImPlotLineFlags_None);
        ImPlot::EndPlot();
    }
}

ImGuiFileChooser::ImGuiFileChooser() : selected_file("") {
    using namespace std;
    try {
        // Try initializing the current path
        current_path = filesystem::current_path() / "input-files";
        if (!filesystem::exists(current_path) || !filesystem::is_directory(current_path)) {
            cerr << "Warning: Default input-files directory does not exist. Using current path instead.\n";
            current_path = filesystem::current_path();
        }
    } catch (const std::exception &e) {
        cerr << "Error initializing file chooser: " << e.what() << std::endl;
        current_path = filesystem::current_path();
    }
}

void ImGuiFileChooser::call_imgui_file_chooser(string *input_file, bool running) {
    using namespace std;

    // Verify ImGui context
    if (!ImGui::GetCurrentContext()) {
        cerr << "Error: ImGui context is not initialized." << std::endl;
        return;
    }

    // Display current directory
    ImGui::Text("Current Directory: %s", current_path.string().c_str());
    ImGui::Separator();

    // Navigate up
    if (ImGui::Button("..")) {
        current_path = current_path.parent_path();
    }
    ImGui::SameLine();
    ImGui::Text("Navigate Up");

    ImGui::Separator();

    // Display files and directories
    try {
        if (ImGui::BeginTable("FileTable", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_Borders)) {
            for (const auto &entry : filesystem::directory_iterator(current_path)) {
                ImGui::TableNextRow();

                ImGui::TableNextColumn();

                if (ImGui::Selectable(entry.path().filename().string().c_str(), false)) {
                    if (entry.is_directory()) {
                        current_path = entry.path();
                    } else {
                        selected_file = entry.path().string();
                    }
                }

                ImGui::TableNextColumn();

                if (entry.is_directory()) {
                    ImGui::Text("[DIR]");
                } else {
                    ImGui::Text("[FILE]");
                }
            }
            ImGui::EndTable();
        }
    } catch (const std::exception &e) {
        ImGui::Text("Error reading directory: %s", e.what());
    }

    ImGui::Separator();

    // Selected file
    if (!selected_file.empty()) {
        ImGui::Text("Selected file: ");
        if (running) {
            ImGui::Text("%s", selected_file.c_str());
            if (ImGui::Button("Rebuild simulation")) {
                *input_file = selected_file;
                selected_file.clear();
            }
        } else {
            ImGui::Text("%s", selected_file.c_str());
            if (ImGui::Button("Build simulation")) {
                *input_file = selected_file;
                selected_file.clear();
            }
        }

        ImGui::SameLine();
        if (ImGui::Button("Cancel")) {
            selected_file.clear();
        }

        if (running) {
            ImGui::Text("Warning: this will override current simulation and its settings");
        }
    }
}

void UI_Manager::call_imgui_key_quantities_plot_control_panel(Config &config, array<SpatialGraph, N_SG> &spatial_graphs,
                                                              array<MiscSpatialGraph, N_MG> &misc_spatial_graphs,
                                                              array<FluidSpatialGraph, N_FG> &fluid_spatial_graphs) {

    if (ImGui::CollapsingHeader("Drilling Parameters Plot Settings")) {

        static int selected_group_index = 0;
        constexpr int num_groups = sizeof(plot_groups) / sizeof(plot_groups[0]);

        ImGui::Text("Select plot group:");
        ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
        if (ImGui::BeginCombo("##PlotGroupCombo", plot_groups[selected_group_index].title.c_str())) {
            for (int i = 0; i < num_groups; ++i) {
                bool is_selected = (selected_group_index == i);
                if (ImGui::Selectable(plot_groups[i].title.c_str(), is_selected)) {
                    selected_group_index = i;
                }
                if (is_selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        // Show checkbox to control visibility of the selected plot group
        PlotGroup &selected_group = plot_groups[selected_group_index];

        ImGui::SameLine();
        ImGui::Checkbox("Show", &selected_group.show);

        // Only show sub-variable checkboxes if the group is visible
        if (selected_group.show) {
            ImGui::Text("Select variables to plot:");
            ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);

            // Show checkboxes for each variable in the selected group
            for (uint i = 0; i < selected_group.graphs.size(); ++i) {
                DrillingQuantityGraph &graph = selected_group.graphs[i];
                // Checked by default if not set
                if (!graph.initialized) {
                    graph.show = true;
                    graph.initialized = true;
                }
                if (ImGui::Checkbox(graph.label.c_str(), &graph.show)) {
                    sync_tiles_with_checkbox_states(config, spatial_graphs, misc_spatial_graphs, fluid_spatial_graphs);
                }
            }
        }

        ImGui::Separator();
        ImGui::Text("Plot limits control:");
        // Iterate through all variables in all groups
        for (auto &group : plot_groups) {
            if (!group.show)
                continue;
            for (auto &graph : group.graphs) {
                if (graph.show) {
                    const char *name = drilling_time_variable_name_array[(uint)graph.variable];
                    call_imgui_key_quantity_plot_control_limit_selection(graph, name);
                }
            }
        }
    }
}

void UI_Manager::call_imgui_key_quantity_plot_control_limit_selection(DrillingQuantityGraph &plot, const char *name) {
    constexpr float step = 0.1f;
    constexpr float step_fast = 1.0f;

    float settings_width = ui_utils.SETTINGS_WIDTH * 3.0f / 4.0f;

    ImGui::BeginGroup();
    ImGui::Indent(ui_utils.INDENT_SIZE);

    ImGui::Text("%s Control", name);

    ImGui::Checkbox((std::string(name) + " Auto scale").c_str(), &plot.auto_scale);

    ImGui::Text("%s Limits: Min/Max", name);

    ImGui::SetNextItemWidth(settings_width);
    if (plot.auto_scale) {
        ImGui::BeginDisabled();
    }
    ImGui::InputFloat(("## Min " + std::string(name)).c_str(), &plot.value_min, step, step_fast, "%.2f");
    if (plot.auto_scale) {
        ImGui::EndDisabled();
    }

    ImGui::SameLine();

    ImGui::SetNextItemWidth(settings_width);
    if (plot.auto_scale) {
        ImGui::BeginDisabled();
    }
    ImGui::InputFloat(("## Max " + std::string(name)).c_str(), &plot.value_max, step, step_fast, "%.2f");
    if (plot.auto_scale) {
        ImGui::EndDisabled();
    }

    ImGui::Unindent(ui_utils.INDENT_SIZE);
    ImGui::Spacing();
    ImGui::EndGroup();

    ImGui::GetWindowDrawList()->AddRect(ImGui::GetItemRectMin(), ImGui::GetItemRectMax(), IM_COL32(255, 255, 255, 255),
                                        5.0f);
}

void UI_Manager::call_imgui_hole_quantities_control_and_plot_window() {

    static int selected_quantity = 0;
    const char *hole_variable_names[] = {"Curvatures", "Hole Diameter", "Offsets", "Cross-section Rotation"};
    constexpr int num_items = sizeof(hole_variable_names) / sizeof(hole_variable_names[0]);

    ImGui::Text("Select quantity to plot:");
    ImGui::SetNextItemWidth(ui_utils.SETTINGS_WIDTH);
    if (ImGui::BeginCombo("##QuantitySelector", hole_variable_names[selected_quantity])) {
        for (int i = 0; i < num_items; ++i) {
            bool is_selected = (selected_quantity == i);
            if (ImGui::Selectable(hole_variable_names[i], is_selected)) {
                selected_quantity = i;
            }
            if (is_selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
    const uint N_hole = hole_plot_data.N_hole;

    const scalar *s = hole_plot_data.s.data();
    if (selected_quantity == 0) { // Curvature
        if (ImPlot::BeginPlot("Curvature Plot")) {
            ImPlot::SetupAxis(ImAxis_X1, "s [m]");
            ImPlot::SetupAxis(ImAxis_Y1, "Curvature [1/m]");
            ImPlot::SetupAxisLimits(ImAxis_Y1, hole_plot_data.y_min_curvature, hole_plot_data.y_max_curvature,
                                    ImGuiCond_Always);
            ImPlot::PlotLine("kappa_y", s, hole_plot_data.kappa_y.data(), N_hole);
            ImPlot::PlotLine("kappa_z", s, hole_plot_data.kappa_z.data(), N_hole);
            ImPlot::EndPlot();
        }
    } else if (selected_quantity == 1) { // Hole Radius
        if (ImPlot::BeginPlot("Hole Radius Plot")) {
            ImPlot::SetupAxis(ImAxis_X1, "s [m]");
            ImPlot::SetupAxis(ImAxis_Y1, "Radius [m]");
            ImPlot::SetupAxisLimits(ImAxis_Y1, hole_plot_data.y_min_radius - 0.05, hole_plot_data.y_max_radius + 0.05,
                                    ImGuiCond_Always);
            ImPlot::PlotLine("Radius a", s, hole_plot_data.a.data(), N_hole);
            ImPlot::PlotLine("Radius b (Elliptical)", s, hole_plot_data.b.data(), N_hole);

            ImPlot::EndPlot();
        }
    } else if (selected_quantity == 2) { // Offset
        if (ImPlot::BeginPlot("Hole Offset Plot")) {
            ImPlot::SetupAxis(ImAxis_X1, "s [m]");
            ImPlot::SetupAxis(ImAxis_Y1, "Offset [m]");
            ImPlot::SetupAxisLimits(ImAxis_Y1, hole_plot_data.y_min_offsets, hole_plot_data.y_max_offsets,
                                    ImGuiCond_Always);
            ImPlot::PlotLine("Offset_y", s, hole_plot_data.offset_y.data(), N_hole);
            ImPlot::PlotLine("Offset_z", s, hole_plot_data.offset_z.data(), N_hole);
            ImPlot::EndPlot();
        }
    } else if (selected_quantity == 3) { // Cross-section rotation
        if (ImPlot::BeginPlot("Cross-section Rotation Angle Plot")) {
            ImPlot::SetupAxis(ImAxis_X1, "s [m]");
            ImPlot::SetupAxis(ImAxis_Y1, "Rotation [deg]");
            ImPlot::SetupAxisLimits(ImAxis_Y1, hole_plot_data.y_min_rotation, hole_plot_data.y_max_rotation,
                                    ImGuiCond_Always);
            ImPlot::PlotLine("Rotation", s, hole_plot_data.alpha.data(), N_hole);
            ImPlot::EndPlot();
        }
    }
}

void UI_Manager::call_imgui_popup_message_with_cooldown(const char *message, bool *show_message,
                                                        bool *trigger_condition, float cooldown_time) {

    /*
    This function is used to show a warning message with a cooldown time, ensuring that the warning
    is not shown too often as certain ImGui assets can trigger multiple events.
    */
    static std::unordered_map<const char *, float> last_warning_times;

    if (*trigger_condition && !*show_message) {
        float current_time = ImGui::GetTime();
        if (current_time - last_warning_times[message] >= cooldown_time) {
            *show_message = true;
            *trigger_condition = false;
            last_warning_times[message] = current_time;
            ImGui::OpenPopup("Message");
        } else {
            // Silently handle the condition without showing warning
            *trigger_condition = false;
        }
    }

    if (*show_message) {
        call_imgui_popup_message(message, show_message);
    }
}

void UI_Manager::call_imgui_popup_message(const char *message, bool *show_message) {
    // Center the modal
    ImVec2 center = ImGui::GetMainViewport()->GetCenter();

    // Calculate text size and add padding
    ImVec2 text_size = ImGui::CalcTextSize(message);
    float button_width = 120.0f * ui_utils.scale;
    float padding = ImGui::GetStyle().WindowPadding.x * 2.0f * ui_utils.scale;
    float min_width = std::max(text_size.x + padding, button_width + padding);

    // Set fixed window size
    ImGui::SetNextWindowSize(ImVec2(min_width, text_size.y + ImGui::GetFrameHeightWithSpacing() * 3.0f));
    ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

    if (ImGui::BeginPopupModal("Message", show_message, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings)) {

        ImGui::TextWrapped("%s", message);
        ImGui::Separator();

        // Center the button
        float button_x = (ImGui::GetWindowSize().x - button_width) * 0.5f * ui_utils.scale;
        ImGui::SetCursorPosX(button_x);

        if (ImGui::Button("OK", ImVec2(button_width, 0))) {
            ImGui::CloseCurrentPopup();
            *show_message = false;
        }
        ImGui::EndPopup();
    }
}

DrillingQuantityGraph &UI_Manager::get_graph(DrillingVariable var) {
    for (auto &group : plot_groups) {
        for (auto &graph : group.graphs) {
            if (graph.variable == var) {
                return graph;
            }
        }
    }
    throw std::runtime_error("Graph not found for variable");
}

void UI_Manager::initialize_default_plot_tiles() {
    // Initialize empty tile system - tiles will be added through settings
    plot_tiles.clear();
}

void UI_Manager::call_graph_plot_window(Config &config, const ConfigDynamic &conf_dyn,
                                        const curvlin::PipeSolver &pipesolver,
                                        const array<SpatialGraph, N_SG> &spatial_graphs,
                                        const array<MiscSpatialGraph, N_MG> &misc_spatial_graphs,
                                        const array<FluidSpatialGraph, N_FG> &fluid_spatial_graphs, const Pipe &pipe,
                                        const Hole &hole, const byte *buf, const BitRockData &bit_rock_data) {

    if (plot_tiles.empty()) {
        ImGui::Text("No plots added yet. Use the control panels to add plots.");
        return;
    }

    ImVec2 content_region = ImGui::GetContentRegionAvail();

    // Calculate base tile size
    float base_tile_height = content_region.y / UI_utils::tiles_per_column;
    float base_tile_width = base_tile_height; // Make base tiles square

    // Calculate how many columns we need based on tile positions
    uint max_col = 0;
    for (const auto &tile : plot_tiles) {
        if (tile.visible) {
            max_col = max(max_col, tile.grid_col + tile.width_tiles);
        }
    }

    // Create horizontal scrollable region
    float total_width = max_col * base_tile_width;
    ImGui::BeginChild("PlotTiles", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);

    // Ensure we have enough width for all tiles
    ImGui::SetCursorPosX(0);
    ImGui::Dummy(ImVec2(total_width, 1)); // Dummy element to set scrollable width

    // Render each tile at its calculated position
    for (const auto &tile : plot_tiles) {
        if (tile.visible) {
            // Calculate position and size for this tile
            float x_pos = tile.grid_col * base_tile_width;
            float y_pos = tile.grid_row * base_tile_height;
            float tile_width = tile.width_tiles * base_tile_width;
            float tile_height = tile.height_tiles * base_tile_height;

            ImGui::SetCursorPos(ImVec2(x_pos, y_pos));
            render_plot_tile(tile, config, conf_dyn, pipesolver, spatial_graphs, misc_spatial_graphs,
                             fluid_spatial_graphs, pipe, hole, buf, bit_rock_data, ImVec2(tile_width, tile_height));
        }
    }
    ImGui::EndChild();
}

void UI_Manager::render_plot_tile(const PlotTile &tile, Config &config, const ConfigDynamic &conf_dyn,
                                  const curvlin::PipeSolver &pipesolver,
                                  const array<SpatialGraph, N_SG> &spatial_graphs,
                                  const array<MiscSpatialGraph, N_MG> &misc_spatial_graphs,
                                  const array<FluidSpatialGraph, N_FG> &fluid_spatial_graphs, const Pipe &pipe,
                                  const Hole &hole, const byte *buf, const BitRockData &bit_rock_data,
                                  const ImVec2 &tile_size) {

    // Remove border and make child fill available space
    ImGui::BeginChild(tile.title.c_str(), tile_size, false, ImGuiWindowFlags_NoScrollbar);

    switch (tile.type) {
    case PlotTileType::ORBITAL_PLOT: {
        if (orbital_plots.size() > tile.orbital_plot_index) {
            auto &plot = orbital_plots[tile.orbital_plot_index];

            // Format S_plot with one decimal
            std::string s_plot_str = std::to_string(static_cast<int>(plot.S_plot * 10) / 10.0f);
            if (ImPlot::BeginPlot(
                    ("Plot " + std::to_string(tile.orbital_plot_index + 1) + ", S = " + s_plot_str + " m").c_str(),
                    ImVec2(-1, -1),
                    ImPlotFlags_Equal |           // Keep aspect ratio equal
                        ImPlotFlags_NoMenus |     // No right-click menu
                        ImPlotFlags_NoBoxSelect | // No box selection
                        ImPlotFlags_NoLegend)) {  // No legend

                // Now call the subplot function within the plot context
                call_imgui_orbital_subplot(plot, config, pipesolver, pipe, hole, buf);
                ImPlot::EndPlot();
            }
        } else {
            ImGui::Text("Orbital plot not available");
        }
        break;
    }

    case PlotTileType::BIT_ROCK_PLOT: {

        BitRock br = bit_rock_data.get_bit_rock_field(buf);

        // Update plot limits using ArrayView<scalar>
        bit_rock_plot.update_plot_and_limits(br.s_blade, bit_rock_data.get_cut_depths(buf));

        // Draw plot with original logic and fill available space
        if (ImPlot::BeginPlot("Bit-Rock Interaction", ImVec2(-1, -1))) {
            // Set up axes with Y-axis inverted (positive downwards)
            ImPlot::SetupAxis(ImAxis_X1, "Theta [rad]");
            ImPlot::SetupAxis(ImAxis_Y1, "S [m]", ImPlotAxisFlags_Invert);
            ImPlot::SetupAxisLimits(ImAxis_X1, bit_rock_plot.x_min, bit_rock_plot.x_max);
            ImPlot::SetupAxisLimits(ImAxis_Y1, bit_rock_plot.y_min, bit_rock_plot.y_max, ImPlotCond_Always);

            // Plot filled area below rock surface
            ImPlot::PushStyleColor(ImPlotCol_Fill, ImVec4(1.0f, 1.0f, 0.8f, 0.5f)); // Gray fill color
            ImPlot::PlotShaded("Rock Surface", bit_rock_plot.surface_x.data(), bit_rock_plot.surface_y.data(),
                               bit_rock_plot.N_surface_points,
                               INFINITY); // yref value for the bottom of shading

            ImPlot::PopStyleColor();

            // Plot current bit position
            ImPlot::PushStyleVar(ImPlotStyleVar_Marker, ImPlotMarker_Circle);                // Set marker type first
            ImPlot::PushStyleVar(ImPlotStyleVar_MarkerSize, 6.0f);                           // Control marker size
            ImPlot::PushStyleVar(ImPlotStyleVar_MarkerWeight, 2.0f);                         // Marker outline weight
            ImPlot::PushStyleColor(ImPlotCol_MarkerFill, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));    // Red fill
            ImPlot::PushStyleColor(ImPlotCol_MarkerOutline, ImVec4(1.0f, 0.0f, 0.0f, 1.0f)); // Red outline
            ImPlot::PlotScatter("Bit Position", &br.theta, &br.s_blade, 1);
            ImPlot::PopStyleColor(2);
            ImPlot::PopStyleVar(3);

            ImPlot::EndPlot();
        }
        break;
    }

    case PlotTileType::SPATIAL_GRAPH_PLOT: {
        // This is a regular spatial graph
        if (tile.spatial_graph_index < spatial_graphs.size()) {
            const auto &graph = spatial_graphs[tile.spatial_graph_index];
            if (graph.show_graph_plot && graph.data.size() > 0) {
                const uint top_node = config.top_node;
                const uint N = pipe.N;
                plot_single_graph(graph.data.data(), s_nodewise.data(), top_node, N, graph.y_min, graph.y_max,
                                  graph.s_min, graph.s_max, graph.get_y_label(), graph.get_variable_string(),
                                  tile.title);
            } else {
                ImGui::Text("Spatial graph not available");
            }
        } else {
            ImGui::Text("Spatial graph index out of range");
        }
        break;
    }

    case PlotTileType::FLUID_SPATIAL_GRAPH_PLOT: {
        // This is a regular spatial graph
        if (tile.spatial_graph_index < fluid_spatial_graphs.size()) {
            const auto &graph = fluid_spatial_graphs[tile.spatial_graph_index];
            if (graph.show_graph_plot && graph.data.size() > 0) {
                const uint top_node = config.top_node;
                const uint Nf = pipe.N + 1;
                plot_single_graph(graph.data.data(), s_elementwise.data(), top_node, Nf, graph.y_min, graph.y_max,
                                  graph.s_min, graph.s_max, graph.get_graph_label(), graph.label, tile.title);
            } else {
                ImGui::Text("Spatial graph not available");
            }
        } else {
            ImGui::Text("Spatial graph index out of range");
        }
        break;
    }

    case PlotTileType::MISC_SPATIAL_GRAPH_PLOT: {
        // This is a misc spatial graph
        if (tile.spatial_graph_index < misc_spatial_graphs.size()) {
            const auto &misc_graph = misc_spatial_graphs[tile.spatial_graph_index];
            if (misc_graph.show_graph_plot && misc_graph.data.size() > 0) {
                const uint top_node = config.top_node;
                const uint N = pipe.N;
                if (misc_graph.elementwise) {
                    plot_single_graph(misc_graph.data.data(), s_elementwise.data(), top_node, N + 1, misc_graph.y_min,
                                      misc_graph.y_max, misc_graph.s_min, misc_graph.s_max,
                                      misc_graph.get_graph_label(), misc_graph.label, tile.title);

                } else {
                    plot_single_graph(misc_graph.data.data(), s_nodewise.data(), top_node, N, misc_graph.y_min,
                                      misc_graph.y_max, misc_graph.s_min, misc_graph.s_max,
                                      misc_graph.get_graph_label(), misc_graph.label, tile.title);
                }
            } else {
                ImGui::Text("Misc spatial graph not available");
            }
        } else {
            ImGui::Text("Misc spatial graph index out of range");
        }
        break;
    }

    case PlotTileType::KEY_QUANTITIES_PLOT: {
        if (plot_groups.size() > tile.plot_group_index) {
            auto &group = plot_groups[tile.plot_group_index];

            if (!group.show)
                break;

            float time = (float)config.t;

            // Update data for each graph in this group (similar to old function)
            for (size_t i = 0; i < group.graphs.size(); i++) {
                auto &graph = group.graphs[i];
                if (!graph.show)
                    continue;

                // Update data - same logic as old function
                scalar current_value = 0.0;
                switch (graph.variable) {
                case DrillingVariable::HOOK_LOAD:
                    current_value = conf_dyn.hook_load / group.scale_factor;
                    break;
                case DrillingVariable::WOB:
                    current_value = conf_dyn.WOB / group.scale_factor;
                    break;
                case DrillingVariable::TOP_DRIVE_TORQUE:
                    current_value = conf_dyn.top_drive_torque / group.scale_factor;
                    break;
                case DrillingVariable::TOB:
                    current_value = conf_dyn.TOB / group.scale_factor;
                    break;
                case DrillingVariable::BLOCK_POSITION:
                    current_value = conf_dyn.block_position / group.scale_factor;
                    break;
                case DrillingVariable::FEED_RATE:
                    current_value = conf_dyn.v_top / group.scale_factor;
                    break;
                case DrillingVariable::ROP:
                    current_value = conf_dyn.ROP / group.scale_factor;
                    break;
                case DrillingVariable::OMEGA_TOP_DRIVE:
                    current_value = conf_dyn.omega_top / group.scale_factor;
                    break;
                case DrillingVariable::OMEGA_BIT:
                    current_value = conf_dyn.omega_bit / group.scale_factor;
                    break;
                case DrillingVariable::MSE:
                    current_value = conf_dyn.MSE / group.scale_factor;
                    break;
                }

                // Update circular buffer
                graph.times[graph.current_index] = time;
                graph.values[graph.current_index] = current_value;
                graph.current_index = (graph.current_index + 1) % graph.BUFFER_SIZE;
                graph.data_size = std::min(graph.data_size + 1, graph.BUFFER_SIZE);

                // Update plot indices for time window
                graph.update_plot_limits(time);
                graph.update_incremental_limits(current_value, time);
            }

            // Update group-level limits
            group.update_group_limits();

            if (ImPlot::BeginPlot(group.title.c_str(), ImVec2(-1, -1))) {
                // Setup axes - use original logic
                ImPlot::SetupAxis(ImAxis_X1, group.label.c_str());
                ImPlot::SetupAxis(ImAxis_Y1, "Time [s]", ImPlotAxisFlags_Invert);
                ImPlot::SetupAxisLimits(ImAxis_X1, group.group_min, group.group_max, ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, time + TIME_PADDING_PLOT, std::max(0.0f, time - TIME_WINDOW_PLOT),
                                        ImGuiCond_Always);

                // Plot each active variable using original logic
                for (size_t i = 0; i < group.graphs.size(); i++) {
                    auto &graph = group.graphs[i];
                    if (!graph.show) // Check show instead of active
                        continue;

                    // Calculate segment sizes
                    // int first_segment_size = min(graph.plot_size, graph.BUFFER_SIZE - graph.plot_start_idx);
                    // int second_segment_size = graph.plot_size - first_segment_size - 1;

                    int first_segment_size =
                        std::min((int)graph.plot_size, (int)(graph.BUFFER_SIZE - graph.plot_start_idx));
                    int second_segment_size = graph.plot_size - first_segment_size;

                    const char *plot_label = graph.label.c_str();

                    // Plot the first segment (from plot_start_idx to the end of the buffer)
                    if (first_segment_size > 0) {
                        ImPlot::PlotLine(plot_label, &graph.values[graph.plot_start_idx],
                                         &graph.times[graph.plot_start_idx], first_segment_size);
                    }

                    // Plot the second segment (from the beginning of the buffer to plot_start_idx - 1)
                    if (second_segment_size > 0) {
                        ImPlot::PlotLine(plot_label, graph.values.data(), graph.times.data(), second_segment_size);
                    }
                }

                ImPlot::EndPlot();
            }
        } else {
            ImGui::Text("Key quantities plot not available");
        }
        break;
    }
    }

    ImGui::EndChild();
}

void UI_Manager::sync_tiles_with_checkbox_states(Config &config, const array<SpatialGraph, N_SG> &spatial_graphs,
                                                 const array<MiscSpatialGraph, N_MG> &misc_spatial_graphs,
                                                 const array<FluidSpatialGraph, N_FG> &fluid_spatial_graphs) {
    // Sync tiles with their checkbox states

    // Add missing spatial graph tiles for checked misc graphs
    for (uint i = 0; i < misc_spatial_graphs.size(); i++) {
        if (misc_spatial_graphs[i].show_graph_plot) {
            // Check if tile already exists
            bool tile_exists = false;
            for (const auto &tile : plot_tiles) {
                if (tile.type == PlotTileType::MISC_SPATIAL_GRAPH_PLOT && tile.spatial_graph_index == i) {
                    tile_exists = true;
                    break;
                }
            }

            if (!tile_exists) {
                string title = misc_spatial_graphs[i].label;
                plot_tiles.push_back({
                    PlotTileType::MISC_SPATIAL_GRAPH_PLOT, title, 3, 1, true, 0,
                    0,      // width_tiles=2, height_tiles=1, visible, grid_row, grid_col
                    0, i, 0 // orbital_plot_index, spatial_graph_index (misc graph index), plot_group_index
                });
            }
        }
    }

    for (uint i = 0; i < fluid_spatial_graphs.size(); i++) {
        if (fluid_spatial_graphs[i].show_graph_plot) {
            // Check if tile already exists
            bool tile_exists = false;
            for (const auto &tile : plot_tiles) {
                if (tile.type == PlotTileType::FLUID_SPATIAL_GRAPH_PLOT && tile.spatial_graph_index == i) {
                    tile_exists = true;
                    break;
                }
            }

            if (!tile_exists) {
                string title = fluid_spatial_graphs[i].label;
                plot_tiles.push_back({
                    PlotTileType::FLUID_SPATIAL_GRAPH_PLOT, title, 3, 1, true, 0,
                    0,      // width_tiles=2, height_tiles=1, visible, grid_row, grid_col
                    0, i, 0 // orbital_plot_index, spatial_graph_index (misc graph index), plot_group_index
                });
            }
        }
    }

    // Add missing spatial graph tiles for checked regular graphs
    for (uint i = 0; i < spatial_graphs.size(); i++) {
        if (spatial_graphs[i].show_graph_plot) {
            // Check if tile already exists
            bool tile_exists = false;
            for (const auto &tile : plot_tiles) {
                if (tile.type == PlotTileType::SPATIAL_GRAPH_PLOT && tile.spatial_graph_index == i) {
                    tile_exists = true;
                    break;
                }
            }

            if (!tile_exists) {
                string title = spatial_graphs[i].get_variable_string();
                plot_tiles.push_back({
                    PlotTileType::SPATIAL_GRAPH_PLOT, title, 3, 1, true, // width_tiles=2, height_tiles=1, visible
                    0, 0,                                                // grid_row, grid_col
                    0, i, 0 // orbital_plot_index, spatial_graph_index, plot_group_index
                });
            }
        }
    }

    for (int i = plot_tiles.size() - 1; i >= 0; --i) {
        auto &tile = plot_tiles[i];
        if (tile.type == PlotTileType::MISC_SPATIAL_GRAPH_PLOT) {
            if (tile.spatial_graph_index < misc_spatial_graphs.size() &&
                !misc_spatial_graphs[tile.spatial_graph_index].show_graph_plot) {
                plot_tiles.erase(plot_tiles.begin() + i);
            }
        } else if (tile.type == PlotTileType::FLUID_SPATIAL_GRAPH_PLOT) {
            if (tile.spatial_graph_index < fluid_spatial_graphs.size() &&
                !fluid_spatial_graphs[tile.spatial_graph_index].show_graph_plot) {
                plot_tiles.erase(plot_tiles.begin() + i);
            }
        } else if (tile.type == PlotTileType::SPATIAL_GRAPH_PLOT) {
            if (tile.spatial_graph_index < spatial_graphs.size() &&
                !spatial_graphs[tile.spatial_graph_index].show_graph_plot) {
                plot_tiles.erase(plot_tiles.begin() + i);
            }
        }
        // Add similar logic for other tile types if needed
    }

    // Sync bit-rock plot tile
    bool bit_rock_shown = ui_settings.show_bit_rock_plot && config.conf_stat.bc_bit_rock_type != BC_BitRockType::NO_BIT;

    bool bit_rock_tile_exists = false;
    for (const auto &tile : plot_tiles) {
        if (tile.type == PlotTileType::BIT_ROCK_PLOT) {
            bit_rock_tile_exists = true;
            break;
        }
    }

    if (bit_rock_shown && !bit_rock_tile_exists) {
        plot_tiles.push_back({
            PlotTileType::BIT_ROCK_PLOT, "Bit-Rock Plot", 2, 1, true, // width_tiles, height_tiles, visible
            0, 0,                                                     // grid_row, grid_col
            0, 0, 0 // orbital_plot_index, spatial_graph_index, plot_group_index
        });
    } else if (!bit_rock_shown && bit_rock_tile_exists) {
        plot_tiles.erase(std::remove_if(plot_tiles.begin(), plot_tiles.end(),
                                        [](const PlotTile &tile) { return tile.type == PlotTileType::BIT_ROCK_PLOT; }),
                         plot_tiles.end());
    }

    // Sync key quantities plot tiles
    for (uint group_idx = 0; group_idx < plot_groups.size(); group_idx++) {
        const auto &group = plot_groups[group_idx];

        // Check if any graph in this group is active (only check show, not active array)
        bool group_has_active_graphs = false;
        for (size_t i = 0; i < group.graphs.size(); i++) {
            if (group.graphs[i].show) {
                group_has_active_graphs = true;
                break;
            }
        }

        // Check if tile exists for this group
        bool group_tile_exists = false;
        for (const auto &tile : plot_tiles) {
            if (tile.type == PlotTileType::KEY_QUANTITIES_PLOT && tile.plot_group_index == group_idx) {
                group_tile_exists = true;
                break;
            }
        }

        if (group_has_active_graphs && !group_tile_exists) {
            plot_tiles.push_back({
                PlotTileType::KEY_QUANTITIES_PLOT, group.title, 1, 2,
                true,           // width_tiles=1, height_tiles=2, visible
                0, 0,           // grid_row, grid_col
                0, 0, group_idx // orbital_plot_index, spatial_graph_index, plot_group_index
            });
        } else if (!group_has_active_graphs && group_tile_exists) {
            plot_tiles.erase(std::remove_if(plot_tiles.begin(), plot_tiles.end(),
                                            [group_idx](const PlotTile &tile) {
                                                return tile.type == PlotTileType::KEY_QUANTITIES_PLOT &&
                                                       tile.plot_group_index == group_idx;
                                            }),
                             plot_tiles.end());
        }
    }

    // Recalculate positions after any changes
    calculate_tile_positions();
}

void UI_Manager::calculate_tile_positions() {
    // Create a grid to track occupied cells
    const uint max_rows = UI_utils::tiles_per_column;
    const uint max_cols = 20; // Reasonable max columns for now
    vector<vector<bool>> occupied(max_rows, vector<bool>(max_cols, false));

    for (auto &tile : plot_tiles) {
        if (!tile.visible)
            continue;

        // Find the leftmost available position for this tile
        bool placed = false;
        for (uint col = 0; col <= max_cols - tile.width_tiles && !placed; col++) {
            for (uint row = 0; row <= max_rows - tile.height_tiles && !placed; row++) {

                // Check if this position is available
                bool can_place = true;
                for (uint r = row; r < row + tile.height_tiles && can_place; r++) {
                    for (uint c = col; c < col + tile.width_tiles && can_place; c++) {
                        if (occupied[r][c]) {
                            can_place = false;
                        }
                    }
                }

                if (can_place) {
                    // Place the tile here
                    tile.grid_row = row;
                    tile.grid_col = col;

                    // Mark the cells as occupied
                    for (uint r = row; r < row + tile.height_tiles; r++) {
                        for (uint c = col; c < col + tile.width_tiles; c++) {
                            occupied[r][c] = true;
                        }
                    }
                    placed = true;
                }
            }
        }

        // If we couldn't place the tile, put it at the end
        if (!placed) {
            // Find the rightmost occupied column and place after it
            uint last_col = 0;
            for (uint r = 0; r < max_rows; r++) {
                for (uint c = 0; c < max_cols; c++) {
                    if (occupied[r][c]) {
                        last_col = max(last_col, c + 1);
                    }
                }
            }

            tile.grid_row = 0;
            tile.grid_col = last_col;

            // Mark the cells as occupied (if within bounds)
            for (uint r = tile.grid_row; r < min(tile.grid_row + tile.height_tiles, max_rows); r++) {
                for (uint c = tile.grid_col; c < min(tile.grid_col + tile.width_tiles, max_cols); c++) {
                    occupied[r][c] = true;
                }
            }
        }
    }
}

void UI_Manager::render_side_menu_bar(Simulator *simulator, ConfigGraphics &config_graphics, string *input_file,
                                      std::atomic<bool> &start_simulator, std::atomic<bool> &restart_solver,
                                      array<SpatialGraph, N_SG> &spatial_graphs,
                                      array<MiscSpatialGraph, N_MG> &misc_spatial_graphs,
                                      array<FluidSpatialGraph, N_FG> &fluid_spatial_graphs) {
    using namespace std;

    ImGuiViewport *viewport = ImGui::GetMainViewport();
    if (!viewport)
        return;

    // All category labels are of equal width
    float button_width = this->ui_utils.MENU_BAR_WIDTH;

    float expanded_width = ui_utils.MENU_BAR_WIDTH_EXPANDED;
    float total_width =
        button_width + (ui_settings.current_settings_mode != SettingsViewMode::NONE ? expanded_width : 0);

    // Tweak this value to increase/decrease vertical gap between buttons
    float button_vertical_spacing = this->ui_utils.MENU_BAR_VERT_SPACING;

    // Position and size for the left-side bar
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(ImVec2(total_width, viewport->WorkSize.y));
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings |
                             ImGuiWindowFlags_NoBringToFrontOnFocus;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, button_vertical_spacing));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0.0f, 0.0f));
    // Ensure squared corners for frames
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 0.0f);

    ImGui::Begin("SideBar", nullptr, flags);

    ImGui::BeginChild("SidebarButtons", ImVec2(button_width, viewport->WorkSize.y), false);

    static std::array<float, 5> hover_start_times = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    constexpr float TOOLTIP_DELAY = 0.2f; // seconds

    for (int i = 0; i < (int)settings_categories.size(); ++i) {
        ImGui::PushID(i);

        // Determine if this category is currently selected
        bool selected = (ui_settings.current_settings_mode == (SettingsViewMode)i);

        if (selected) {
            // Use a slightly brighter gray for selected button
            const ImVec4 bright_gray(0.55f, 0.55f, 0.55f, 1.0f);        // normal
            const ImVec4 bright_gray_hover(0.65f, 0.65f, 0.65f, 1.0f);  // hovered
            const ImVec4 bright_gray_active(0.45f, 0.45f, 0.45f, 1.0f); // active

            ImGui::PushStyleColor(ImGuiCol_Button, bright_gray);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, bright_gray_hover);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, bright_gray_active);
        } else {
            // Darker gray/black button colors (tweak these if you want them lighter/darker)
            const ImVec4 btn_col(0.26f, 0.26f, 0.26f, 1.0f);        // normal
            const ImVec4 btn_col_hover(0.35f, 0.35f, 0.35f, 1.0f);  // hovered
            const ImVec4 btn_col_active(0.18f, 0.18f, 0.18f, 1.0f); // active

            ImGui::PushStyleColor(ImGuiCol_Button, btn_col);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, btn_col_hover);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, btn_col_active);
        }

        // Align left and draw square button
        ImGui::SetCursorPosX(0.0f);

        bool has_texture = (i < (int)ui_utils.sidebar_textures.size() && ui_utils.sidebar_textures[i] != 0);

        if (has_texture) {
            ImTextureID tex_id = (ImTextureID)(intptr_t)ui_utils.sidebar_textures[i];
            const float img_scale = ui_utils.ICON_SCALE; // adjust to change icon size inside the button
            std::string btn_id = std::string("##sidebar_btn_") + std::to_string(i);

            // Clicking sets the view mode according to the category index
            if (ImGui::Button(btn_id.c_str(), ImVec2(button_width, button_width))) {
                if (ui_settings.current_settings_mode == (SettingsViewMode)i) {
                    ui_settings.current_settings_mode = SettingsViewMode::NONE;
                } else {
                    ui_settings.current_settings_mode = (SettingsViewMode)i;
                }
            }
            ImVec2 img_size(button_width * img_scale, button_width * img_scale);
            if (i < (int)ui_utils.sidebar_texture_sizes.size() && ui_utils.sidebar_texture_sizes[i].x > 0 &&
                ui_utils.sidebar_texture_sizes[i].y > 0) {
                float tw = ui_utils.sidebar_texture_sizes[i].x;
                float th = ui_utils.sidebar_texture_sizes[i].y;
                float aspect = tw / th;
                float base = button_width * img_scale;
                if (aspect >= 1.0f) {
                    img_size.x = base;
                    img_size.y = base / aspect;
                } else {
                    img_size.y = base;
                    img_size.x = base * aspect;
                }
            } else {
                img_size = ImVec2(button_width * img_scale, button_width * img_scale);
            }
            ImVec2 rmin = ImGui::GetItemRectMin();
            ImVec2 rmax = ImGui::GetItemRectMax();
            ImVec2 center = ImVec2((rmin.x + rmax.x) * 0.5f, (rmin.y + rmax.y) * 0.5f);

            ImVec2 img_tl(center.x - img_size.x * 0.5f, center.y - img_size.y * 0.5f);
            ImVec2 img_br(center.x + img_size.x * 0.5f, center.y + img_size.y * 0.5f);

            ImGui::GetWindowDrawList()->AddImage(tex_id, img_tl, img_br, ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f),
                                                 IM_COL32(255, 255, 255, 255));

            // If selected, draw a subtle outline/glow on top to emphasize choice
            if (selected) {
                ImU32 outline_col = IM_COL32(100, 170, 255, 200);
                float rounding = 2.0f;
                float thickness = 2.0f;
                ImGui::GetWindowDrawList()->AddRect(rmin, rmax, outline_col, rounding, 0, thickness);
            }

        } else {
            // Fallback: render button with category text only
            std::string btn_id = std::string("##sidebar_btn_") + std::to_string(i);
            if (ImGui::Button(btn_id.c_str(), ImVec2(button_width, button_width))) {
                if (ui_settings.current_settings_mode == (SettingsViewMode)i) {
                    ui_settings.current_settings_mode = SettingsViewMode::NONE;
                } else {
                    ui_settings.current_settings_mode = (SettingsViewMode)i;
                }
            }
            // Center the text label inside the button
            ImVec2 rmin = ImGui::GetItemRectMin();
            ImVec2 rmax = ImGui::GetItemRectMax();
            ImVec2 center = ImVec2((rmin.x + rmax.x) * 0.5f, (rmin.y + rmax.y) * 0.5f);
            ImVec2 text_size = ImGui::CalcTextSize(settings_categories[i]);
            ImVec2 text_pos(center.x - text_size.x * 0.5f, center.y - text_size.y * 0.5f);
            ImGui::GetWindowDrawList()->AddText(text_pos, IM_COL32(255, 255, 255, 255), settings_categories[i]);
            // If selected, draw a subtle outline/glow on top to emphasize choice
            if (selected) {
                ImU32 outline_col = IM_COL32(100, 170, 255, 200);
                float rounding = 2.0f;
                float thickness = 2.0f;
                ImGui::GetWindowDrawList()->AddRect(rmin, rmax, outline_col, rounding, 0, thickness);
            }
        }

        // Delayed tooltip logic (unchanged)
        if (ImGui::IsItemHovered()) {
            if (hover_start_times[i] == 0.0f) {
                hover_start_times[i] = ImGui::GetTime();
            } else {
                if (ImGui::GetTime() - hover_start_times[i] >= TOOLTIP_DELAY) {
                    ImGui::BeginTooltip();
                    ImGui::Text("Switch to %s", settings_categories[i]);
                    ImGui::EndTooltip();
                }
            }
        } else {
            hover_start_times[i] = 0.0f;
        }

        ImGui::PopStyleColor(3);
        ImGui::PopID();
    }

    ImGui::PopStyleVar(4);

    ImGui::EndChild();

    if (ui_settings.current_settings_mode != SettingsViewMode::NONE) {

        ImGui::SetCursorPosX(button_width);
        ImGui::SetCursorPosY(0.0);
        ImGui::BeginChild("SettingsPanel", ImVec2(expanded_width, viewport->WorkSize.y), true);
        ImGui::SetCursorPosY(1.0);

        // Switch based on selected category
        switch (ui_settings.current_settings_mode) {
        case SettingsViewMode::HOME:
            render_main_settings(simulator, input_file, start_simulator, restart_solver);
            break;
        case SettingsViewMode::PHYSICS_SETTINGS:
            render_physics_settings(simulator);
            break;
        case SettingsViewMode::GRAPHICS_SETTINGS:
            render_graphics_settings(simulator, config_graphics, spatial_graphs, misc_spatial_graphs,
                                     fluid_spatial_graphs);
            break;
        case SettingsViewMode::GRAPH_PLOTS_SETTINGS:
            render_graph_plots_settings(simulator, spatial_graphs, misc_spatial_graphs, fluid_spatial_graphs);
            break;
        default:
            // Optionally handle unknown category
            break;
        }

        ImGui::EndChild();
    }

    ImVec2 win_pos = ImGui::GetWindowPos();
    ImVec2 panel_min = ImVec2(win_pos.x + button_width, win_pos.y);
    ImVec2 panel_max = ImVec2(panel_min.x + expanded_width, win_pos.y + ImGui::GetWindowSize().y);

    ImVec2 mouse_pos = ImGui::GetMousePos();

    ImVec2 button_min = win_pos;
    ImVec2 button_max = ImVec2(win_pos.x + button_width, win_pos.y + ImGui::GetWindowSize().y);

    bool mouse_in_buttons = mouse_pos.x >= button_min.x && mouse_pos.x <= button_max.x && mouse_pos.y >= button_min.y &&
                            mouse_pos.y <= button_max.y;

    bool mouse_in_panel = mouse_pos.x >= panel_min.x && mouse_pos.x <= panel_max.x && mouse_pos.y >= panel_min.y &&
                          mouse_pos.y <= panel_max.y;

    bool mouse_in_sidebar = mouse_in_buttons || mouse_in_panel;

    static float mouse_outside_start_time = 0.0f;
    constexpr float CLOSE_DELAY = 0.3f;

    if (ui_settings.current_settings_mode != SettingsViewMode::NONE) {
        if (!mouse_in_sidebar) {
            if (mouse_outside_start_time == 0.0f) {
                mouse_outside_start_time = ImGui::GetTime();
            } else if (ImGui::GetTime() - mouse_outside_start_time > CLOSE_DELAY) {
                ui_settings.current_settings_mode = SettingsViewMode::NONE;
                mouse_outside_start_time = 0.0f;
            }
        } else {
            mouse_outside_start_time = 0.0f;
        }
        if (ImGui::IsMouseClicked(0) && !mouse_in_sidebar) {
            ui_settings.current_settings_mode = SettingsViewMode::NONE;
            mouse_outside_start_time = 0.0f;
        }
    }
    ImGui::End();
}

void UI_Manager::render_main_settings(Simulator *simulator, string *input_file, std::atomic<bool> &start_simulator,
                                      std::atomic<bool> &restart_solver) {
    call_file_chooser(input_file, start_simulator, simulator != nullptr);

    if (simulator == nullptr) {
        return;
    }
    call_imgui_solver_control_panel(simulator->config, restart_solver);
}

void UI_Manager::render_physics_settings(Simulator *simulator) {
    if (simulator == nullptr) {
        return;
    }
    Config &config = simulator->config;
    call_imgui_physics_control_panel(config);
    call_imgui_top_control_control_panel(config);
    call_imgui_bottom_bc_control_panel(config);
}

void UI_Manager::render_graphics_settings(Simulator *simulator, ConfigGraphics &config_graphics,
                                          array<SpatialGraph, N_SG> &spatial_graphs,
                                          array<MiscSpatialGraph, N_MG> &misc_spatial_graphs,
                                          array<FluidSpatialGraph, N_FG> &fluid_spatial_graphs) {

    if (simulator == nullptr) {
        return;
    }
    Config &config = simulator->config;
    call_imgui_user_control_panel(config_graphics);

    call_imgui_contour_plot_control_panel(config, config_graphics, spatial_graphs, misc_spatial_graphs,
                                          fluid_spatial_graphs);
}

void UI_Manager::render_graph_plots_settings(Simulator *simulator, array<SpatialGraph, N_SG> &spatial_graphs,
                                             array<MiscSpatialGraph, N_MG> &misc_spatial_graphs,
                                             array<FluidSpatialGraph, N_FG> &fluid_spatial_graphs) {

    if (simulator == nullptr) {
        return;
    }
    Config &config = simulator->config;

    call_imgui_spatial_graph_plot_control_panel(config, spatial_graphs, misc_spatial_graphs, fluid_spatial_graphs);
    call_imgui_orbital_plot_control_panel(config, simulator->pipe, config.top_node,
                                          simulator->double_buffer_gui->get_arena_h().buf);
    call_imgui_key_quantities_plot_control_panel(config, spatial_graphs, misc_spatial_graphs, fluid_spatial_graphs);
    sync_tiles_with_checkbox_states(config, spatial_graphs, misc_spatial_graphs, fluid_spatial_graphs);

    imgui_manager->toggle_button("Show bit-rock plot", &ui_settings.show_bit_rock_plot,
                                 "Displays the rocksurface plot and the bit-blade position. Note: Only the surface for "
                                 "a single blade is shown due to symmetry");
}

size_t UI_Manager::call_load_sidebar_textures(const std::vector<std::string> &paths) {

    // Example log
    // printf("Recommended icon export size: %.0f px (per dimension for square icons)\n", target_pixels);
    free_sidebar_textures(); // clear old ones
    std::vector<GLuint> &button_textures = ui_utils.sidebar_textures;

    button_textures.clear();
    button_textures.resize(paths.size(), 0); // keep indices aligned with paths

    ui_utils.sidebar_texture_sizes.clear();
    ui_utils.sidebar_texture_sizes.resize(paths.size(), ImVec2(0, 0));

    // Keep previous flip or change depending on your source images.
    stbi_set_flip_vertically_on_load(1);

    for (size_t i = 0; i < paths.size(); ++i) {
        const auto &p = paths[i];
        int x = 0, y = 0, n = 0;
        unsigned char *data = stbi_load(p.c_str(), &x, &y, &n, 4);
        if (!data) {
            button_textures[i] = 0;
            continue;
        }

        // Premultiply alpha to avoid dark fringes on semi-transparent edges.
        const int pxcount = x * y;
        for (int k = 0; k < pxcount; ++k) {
            unsigned char *pixel = data + k * 4;
            unsigned int a = pixel[3];
            if (a == 255)
                continue; // already opaque
            // integer-safe premultiply
            pixel[0] = (unsigned char)((pixel[0] * a + 127) / 255);
            pixel[1] = (unsigned char)((pixel[1] * a + 127) / 255);
            pixel[2] = (unsigned char)((pixel[2] * a + 127) / 255);
        }

        GLuint tex = 0;
        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);

        // Use sRGB internal format to preserve color when framebuffer is sRGB (recommended for UI fonts/art)
        // If you prefer linear upload, change to GL_RGBA8.
#ifdef GL_SRGB8_ALPHA8
        GLenum internal_fmt = GL_SRGB8_ALPHA8;
#else
        GLenum internal_fmt = GL_RGBA8;
#endif

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, internal_fmt, x, y, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);

        // Trilinear filtering + mipmaps
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        // Clamp to edge to avoid bleeding from transparent borders
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        glGenerateMipmap(GL_TEXTURE_2D);

        // Optional: anisotropic filtering if supported
        if (glewIsSupported("GL_EXT_texture_filter_anisotropic")) {
            GLfloat maxAniso = 0.0f;
            glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &maxAniso);
            // Use a sane mid value (2-8). Limit to maxAniso.
            GLfloat aniso = std::min<GLfloat>(4.0f, maxAniso);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, aniso);
        }

        glBindTexture(GL_TEXTURE_2D, 0);

        // store size for later aspect-preserving draw
        ui_utils.sidebar_texture_sizes[i] = ImVec2((float)x, (float)y);

        stbi_image_free(data);
        button_textures[i] = tex;
    }

    return std::count_if(button_textures.begin(), button_textures.end(), [](GLuint t) { return t != 0; });
}

void UI_Manager::free_sidebar_textures() {
    std::vector<GLuint> &button_textures = ui_utils.sidebar_textures;

    if (button_textures.empty())
        return;
    std::vector<GLuint> nonzero;
    for (auto t : button_textures)
        if (t != 0)
            nonzero.push_back(t);
    if (!nonzero.empty()) {
        glDeleteTextures((GLsizei)nonzero.size(), nonzero.data());
    }
    button_textures.clear();
}