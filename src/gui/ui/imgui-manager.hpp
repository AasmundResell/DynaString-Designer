#pragma once
#include "external/sim-tools/vendor/imgui/imgui_src/imgui.h"
#include "external/sim-tools/vendor/imgui/imgui_src/imgui_impl_glfw.h"
#include "external/sim-tools/vendor/imgui/imgui_src/imgui_impl_opengl3.h"
#include "external/sim-tools/vendor/imgui/implot_src/implot.h"
#include "external/sim-tools/vendor/imgui/implot_src/implot_internal.h"
#include "external/sim-tools/vendor/stb/stb_image.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

class ImGuiManager {
  public:
    ImGuiManager(GLFWwindow *window);
    ~ImGuiManager();

    void apply_imgui_screen_scaling(int screen_width, int screen_height);

    bool InputFloatClamped(const char *label, float *v, float v_min, float v_max, float step = 0.0f,
                           float step_fast = 0.0f, const char *format = NULL, ImGuiInputTextFlags flags = 0);
    void apply_custom_style();
    void new_frame();
    void render();
    bool toggle_button(const char *str_id, bool *v, const char *tooltip);
};
