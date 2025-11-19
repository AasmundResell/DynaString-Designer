#pragma once

#include "gui/graphics/includes-graphics.hpp"
#include "gui/includes-imgui.hpp"
#include "hole-renderer.hpp"
#include "hole/hole.hpp"
#include "misc/config.hpp"
#include "pipe-renderer.hpp"
#include "pipe-solver/pipe-solver-corot/pipe-solver-corot.hpp"
#include "pipe-solver/pipe-solver-curvlin/pipe-solver-curvlin.hpp"
#include "pipe/pipe.hpp"

class GraphicsManager {
  public:
    Camera camera;

    GLuint virtual_fbo_ = 0;
    GLuint virtual_color_tex_ = 0;
    GLuint virtual_depth_rbo_ = 0;
    Shader shader_default;
    Shader shader_pipe;
    Shader shader_hole;

    unique_ptr<HoleRenderer> hole_renderer;
    unique_ptr<PipeRenderer> pipe_renderer;

    TriadOrigin triad_origin;
    TriadScreen triad_screen;
    TextRenderer text_renderer;
    GraphicsModel drill_bit_model;
    Colorbar colorbar;
    UniformsCommon uniforms_common;

    uint framebuffer = 0;
    uint intermediate_FBO = 0;
    uint texture_colorbuffer = 0;
    uint texture_colorbuffer_multi_sampled = 0;
    uint rbo = 0;

    GraphicsManager(const Config &config, const ConfigGraphics &config_graphics, const Hole &hole, const Pipe &pipe,
                    const PipeRenderComponents &pipe_render_components, const vector<PipeComponent> &pipe_assembly,
                    const byte *buf);

    ~GraphicsManager(); // consider freeing opengl buffers etc here, but this isnt first priority

    void render(GLFWwindow *window, Config &config, ConfigGraphics &config_graphics, const ConfigDynamic &conf_dyn,
                const curvlin::PipeSolver *solver_curvlin, const Hole &hole, const Pipe &pipe,
                const vector<PipeComponent> &pipe_assembly, const vector<Vec3> &x_pipe,
                const vector<Quaternion> &q_pipe, const array<MiscSpatialGraph, N_MG> misc_spatial_graphs,
                const array<FluidSpatialGraph, N_FG> fluid_spatial_graphs, const byte *buf);

  private:
    void call_imgui_camera_control_slider(ConfigGraphics &config_graphics, const scalar S_feed,
                                          const ImVec2 &viewport_panel_size, const Pipe &pipe,
                                          const vector<Vec3> &x_pipe, const byte *buf);
    void create_multisampled_frame_buffer_on_window_resize();

    void check_if_mouse_is_over_graphics_window(ImVec2 viewport_panel_size);
    ImVec2 last_viewport_size = ImVec2(0, 0);

    glm::mat4 calc_model_matrix_drill_bit(const vector<Vec3> &x_pipe, const vector<Quaternion> &q_pipe,
                                          float radial_scale);

    void setup_camera();
    scalar d_drill_bit; // Used to scale the drill bit model
};