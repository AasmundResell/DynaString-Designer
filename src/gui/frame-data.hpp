#pragma once
#include "includes-imgui.hpp"
#include "sim-tools/gui/includes-GL.hpp"
#include <bitset>
using std::bitset;

// struct FrameData {
//     bitset<(uint)Button::COUNT> is_pressed;
//     double cursor_x_old;
//     double cursor_y_old;
//     double delta_cursor_x;
//     double delta_cursor_y;
//     double scroll_y_offset;
//     double delta_time;
//     double last_frame;                  // Time since last frame
//     bool mouse_is_over_graphics_window; // Whether the mouse is over the graphics window
//     bool mouse_is_over_viewport;        // Whether the mouse is over the viewport
//     bool imgui_frame_active;            // should be true when between ImGui::NewFrame() and ImGui::End()
//     bool is_mouse_dragging_imgui;
//     bool camera_replace;
//     bool fps_mode;
//     double left_x;
//     double left_y;
//     uint width;  // Window width
//     uint height; // Window height

//     ImVec2 last_viewport_size;

//     float near_plane = 0.1f;
//     float far_plane = 20000.0f;
// };

struct FrameData {
    bitset<(uint)Button::COUNT> is_pressed;
    float cursor_x_old;
    float cursor_y_old;
    float delta_cursor_x_frame; // delta cursor accumulated over the timestep
    float delta_cursor_y_frame; // delta cursor accumulated over the timestep

    float scroll_y_offset;
    float delta_time = 0;
    double last_frame;
    double elapsed_time = 0;
    bool first_mouse = true;
    bool mouse_is_over_graphics_window; // Whether the mouse is over the graphics window
    bool mouse_is_over_viewport;        // Whether the mouse is over the viewport
    bool imgui_frame_active;            // should be true when between ImGui::NewFrame() and ImGui::End()
    bool is_mouse_dragging_imgui;
    bool camera_replace;
    bool fps_mode;
    double left_x;
    double left_y;
    uint width;  // Window width
    uint height; // Window height

    static constexpr float near_plane = 0.1f;
    static constexpr float far_plane = 20000.0f;

    ImVec2 last_viewport_size;
};

// This needs to be global so that the callbacks can access it
inline FrameData frame_data = {};