#pragma once
#include "gui/graphics/includes-graphics.hpp"
#include "hole/hole.hpp"

class HoleRenderer {
    static constexpr uint NUM_POINTS_CIRCUMF = 60;
    struct Vertex {
        glm::vec3 point;
        glm::vec3 normal;
    };

  public:
    HoleRenderer(const Hole &hole, const byte *buf, const float L_char_triad_world);

    void draw_walls(const Hole &hole, bool wireframe_mode, float new_scale, const byte *buf);
    void draw_fill(const Hole &hole, const byte *buf, float new_scale, scalar s_depth);
    void draw_triads(Shader &shader, const glm::mat4 &view_matrix, const glm::mat4 &proj_matrix,
                     float L_char_triad_world);
    ~HoleRenderer();

  private:
    uint get_index_count() const { return (N_hole - 1) * NUM_POINTS_CIRCUMF * 6; }
    uint get_vertex_count() const { return N_hole * NUM_POINTS_CIRCUMF; }
    void update_wall_vertex_buffer(const Hole &hole, const byte *buf);
    void update_fill_vertex_buffer(const Hole &hole, const byte *buf, scalar fill_depth, uint N_current);

    void create_indices(const scalar *arr_theta);
    void setup_buffers();

    uint N_hole, N_hole_fill;

    vector<Vertex> vertex_buffer;
    vector<uint> index_buffer;

    uint VAO, VBO, EBO;
    scalar radial_scale;

    vector<Triad> hole_triads;
    vector<glm::vec3> triads_positions;
    vector<glm::mat4> triads_orientations;
};