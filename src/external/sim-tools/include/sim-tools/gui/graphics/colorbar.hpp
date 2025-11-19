#pragma once
#include "sim-tools/utils.hpp"
#include "text-renderer.hpp"
#include "vertex-buffer.hpp"

namespace sito {

class Colorbar {
  public:
    using Vertex = VertexPos2Color3;

    uint VAO_colors, VAO_lines;
    glm::vec2 anchor_point; // Bottom-left corner of the colorbar in screen coordinates
    VertexBuffer<Vertex> vb_colors;
    VertexBuffer<glm::vec2> vb_lines;
    Shader shader_colors, shader_lines;
    float lim_max_old = 0, lim_min_old = 0;
    bool prev_frame_outside_limits = true;
    ColorMapType color_map_type;
    bool grey_outside_limits = true;
    bool last_horizontal = false;

    uint screen_width, screen_height;

    // vector<Vertex> vertices_colors;   // The colors itself
    // vector<glm::vec2> vertices_lines; // The lines surrounding it

    void create(ColorMapType color_map_type_, uint screen_width, uint screen_height);
    void draw(const bool draw_horizontal, float val_min, float val_max, float lim_min, float lim_max,
              TextRenderer &text_renderer, const std::string &title, uint screen_width_, uint screen_height_);

  private:
    void set_vertices_horizontal(uint screen_width_, uint screen_height_);
    void set_vertices(uint screen_width_, uint screen_height_);
    void update_colors(float val_min, float val_max, float lim_min, float lim_max);
    void update_colors_horizontal(float val_min, float val_max, float lim_min, float lim_max);
};

} // namespace sito