#include "imgui-manager.hpp"
#include "sim-tools/utils.hpp"
ImGuiManager::ImGuiManager(GLFWwindow *window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
    io.FontGlobalScale = 1.2f; // Make all ImGui text 20% bigger
    ImGuiStyle *style = &ImGui::GetStyle();
    style->WindowRounding = 6.0f;
    style->FrameRounding = 6.0f;
    style->GrabRounding = 6.0f;
    style->TabRounding = 6.0f;
    style->TabBorderSize = 1.0f;
    style->WindowPadding = ImVec2(12, 12);

    ImGui::StyleColorModern(style);

    // Test these color options for the tab highlight line
    // ImGui::StyleColorsLight(style);
    //   ImGui::StyleColorsClassic(style);

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
}

ImGuiManager::~ImGuiManager() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
}

void ImGuiManager::new_frame() {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void ImGuiManager::render() {
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    ImGuiIO &io = ImGui::GetIO();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        GLFWwindow *backup_current_context = glfwGetCurrentContext();
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault();
        glfwMakeContextCurrent(backup_current_context);
    }
}

bool ImGuiManager::toggle_button(const char *str_id, bool *v, const char *tooltip) {
    /*
    Custom toggle button for ImGui as suggested by the developer of ImGui:
    https://github.com/ocornut/imgui/issues/1537
    */
    ImVec4 *colors = ImGui::GetStyle().Colors;
    ImVec2 p = ImGui::GetCursorScreenPos();
    ImDrawList *draw_list = ImGui::GetWindowDrawList();

    float height = ImGui::GetFrameHeight();
    float width = height * 1.55f;
    float radius = height * 0.50f;

    ImGui::InvisibleButton(str_id, ImVec2(width, height));
    if (ImGui::IsItemClicked())
        return *v = !*v;
    ImGuiContext &gg = *GImGui;
    float ANIM_SPEED = 0.085f;
    if (gg.LastActiveId == gg.CurrentWindow->GetID(str_id)) // && g.LastActiveIdTimer < ANIM_SPEED)
        float t_anim = ImSaturate(gg.LastActiveIdTimer / ANIM_SPEED);
    if (ImGui::IsItemHovered())
        draw_list->AddRectFilled(
            p, ImVec2(p.x + width, p.y + height),
            ImGui::GetColorU32(*v ? colors[ImGuiCol_ButtonActive] : ImVec4(0.78f, 0.78f, 0.78f, 1.0f)), height * 0.5f);
    else
        draw_list->AddRectFilled(p, ImVec2(p.x + width, p.y + height),
                                 ImGui::GetColorU32(*v ? colors[ImGuiCol_Button] : ImVec4(0.85f, 0.85f, 0.85f, 1.0f)),
                                 height * 0.50f);
    draw_list->AddCircleFilled(ImVec2(p.x + radius + (*v ? 1 : 0) * (width - radius * 2.0f), p.y + radius),
                               radius - 1.5f, IM_COL32(255, 255, 255, 255));

    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(tooltip);
    }
    ImGui::SameLine();
    ImGui::Text(str_id);

    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(tooltip);
    }

    return *v;
}

void ImGuiManager::apply_custom_style() {
    ImGuiStyle *style = &ImGui::GetStyle();
    style->WindowRounding = 6.0f;
    style->FrameRounding = 6.0f;
    style->GrabRounding = 6.0f;
    style->TabRounding = 6.0f;
    style->TabBorderSize = 1.0f;
    style->WindowPadding = ImVec2(12, 12);

    ImGui::StyleColorModern(style);
}

void ImGuiManager::apply_imgui_screen_scaling(int screen_width, int screen_height) {
    ImGui::StyleColorsDark();
    apply_custom_style();

    float scale_x = screen_width / float(sito::VIRTUAL_WIDTH);
    float scale_y = screen_height / float(sito::VIRTUAL_HEIGHT);
    float imgui_scale = std::min(scale_x, scale_y);

    ImGui::GetStyle().ScaleAllSizes(imgui_scale);
    ImGui::GetIO().FontGlobalScale = imgui_scale * 1.2f;
    ;
}

bool ImGuiManager::InputFloatClamped(const char *label, float *v, float v_min, float v_max, float step, float step_fast,
                                     const char *format, ImGuiInputTextFlags flags) {
    // Let ImGui handle the editing; then clamp if changed.
    bool changed = ImGui::InputFloat(label, v, step, step_fast, format, flags);
    if (changed) {
        const float clamped = ImClamp(*v, v_min, v_max);
        if (clamped != *v)
            *v = clamped;
    }
    return changed;
}