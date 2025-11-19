#include "sim-tools/gui/ui/imgui-utils.hpp"

// ImGuiManager::ImGuiManager(GLFWwindow *window) {
//     IMGUI_CHECKVERSION();
//     ImGui::CreateContext();
//     ImPlot::CreateContext();
//     ImGuiIO &io = ImGui::GetIO();
//     (void)io;
//     io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
//     io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
//     io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

//     ImGui::StyleColorsLight();

//     ImGui_ImplGlfw_InitForOpenGL(window, true);
//     ImGui_ImplOpenGL3_Init("#version 330");
// }

// ImGuiManager::~ImGuiManager() {
//     ImGui_ImplOpenGL3_Shutdown();
//     ImGui_ImplGlfw_Shutdown();
//     ImPlot::DestroyContext();
//     ImGui::DestroyContext();
// }

// void ImGuiManager::new_frame() {
//     ImGui_ImplOpenGL3_NewFrame();
//     ImGui_ImplGlfw_NewFrame();
//     ImGui::NewFrame();
// }

// void ImGuiManager::render() {
//     ImGui::Render();
//     ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

//     ImGuiIO &io = ImGui::GetIO();
//     if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
//         GLFWwindow *backup_current_context = glfwGetCurrentContext();
//         ImGui::UpdatePlatformWindows();
//         ImGui::RenderPlatformWindowsDefault();
//         glfwMakeContextCurrent(backup_current_context);
//     }
// }

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

    ImGui::Begin("File Select");

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

    ImGui::End();
}
