#pragma once
#include "imgui/imgui_src/imgui.h"
#include "imgui/imgui_src/imgui_impl_glfw.h"
#include "imgui/imgui_src/imgui_impl_opengl3.h"
#include "imgui/implot_src/implot.h"
#include "sim-tools/gui/includes-GL.hpp"

// class ImGuiManager {
//   public:
//     ImGuiManager(GLFWwindow *window);
//     ~ImGuiManager();

//     void new_frame();
//     void render();
// };

class ImGuiFileChooser {
  public:
    string selected_file;
    ImGuiFileChooser();

    void call_imgui_file_chooser(string *input_file, bool running);

  private:
    path current_path;
};
