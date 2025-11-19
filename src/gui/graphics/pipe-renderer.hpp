#pragma once
#include "gui/graphics/includes-graphics.hpp"
#include "pipe-solver/pipe-solver-curvlin/pipe-solver-curvlin.hpp"

#include "pipe/pipe.hpp"
struct BowSpring {
    float S_bs;      // Curvilinear coordinate of the b-s on the pipe
    glm::vec3 pos;   // Position of the bow-spring
    float r_bs;      // Bow-spring radius
    glm::vec3 color; // Color for visualization
};

constexpr float BOW_SPRING_LENGTH = 2.0f;

class PipeRenderer {

  public:
    Shader shader;

    struct Vertex {
        glm::vec3 pos;
        glm::vec3 normal;
        glm::vec3 color;

        static void set_vertex_attrib_pointer(uint VBO);
    };

    struct MeshVertex {
        Vec3 pos;
        Vec3 normal;
        glm::vec3 color;
    };
    // TODO:
    // 1: Read in the undeformed mesh from the polygons. Rings first. Finf a way to create normals. Also set
    // necessary values for S depending on elements, etc.
    // 2: Popolulate index and vertex buffer
    // 3: Redo the function for calculating current pos. This will also have to rotate the reference normal
    // 4: Bonus: Add coloring based on beam formula stresses. might need to rethink what goes into the solution.
    //           Alternatively just color based on nodal internal forces.

    PipeRenderer(const Config &config, const Pipe &pipe, const PipeRenderComponents &pipe_render_components,
                 const vector<PipeComponent> &pipe_assembly, const byte *buf);

    uint get_Npoly() const { return ptr_poly_to_ie.size(); }

    void draw(const Config &config, ConfigGraphics &config_graphics, const curvlin::PipeSolver *solver_curvlin,
              const Pipe &pipe, const Hole &hole, const vector<PipeComponent> &pipe_assembly,
              const vector<Vec3> &x_pipe, const vector<Quaternion> &q_pipe,
              const array<MiscSpatialGraph, N_MG> &misc_plots, const array<FluidSpatialGraph, N_FG> &fluid_plots,
              const byte *buf);

    void draw_triads(uint top_node, uint N, const vector<Vec3> &x_pipe, const vector<Quaternion> &q_pipe,
                     Shader &shader_triads, const glm::mat4 &view_matrix, const glm::mat4 &proj_matrix,
                     float L_char_triad_world);

    ~PipeRenderer();

  private:
    uint VAO, VBO, EBO;
    vector<MeshVertex> mesh; /*Undeformed mesh. Contains primarily positions and normals*/
    // vector<Vertex>
    //     mesh_undeformed; /*Each computational element has its own local mesh, with local x from [0, dS_element]*/
    vector<uint> ptr_vertex, ptr_index; /*Size (Ncomp+1) Indices of the starting points of each polygon in the
    global vertex- and index-buffer, respectively*/

    vector<scalar> arr_S_poly;   /*Size (Npoly+1) Stores the S coordinate where each component begins*/
    vector<uint> ptr_poly_to_ie; /*Size (Npoly) Index of the fem element that the polygon belongs to.*/
    std::vector<BowSpring> bow_springs;

    vector<Triad> pipe_triads;

    vector<Vertex> vertex_buffer;
    vector<uint> index_buffer;
    // In PipeRenderer:
    std::vector<std::vector<Vertex>> bow_spring_vertex_buffers; // Transformed vertices for each bow-spring
    std::vector<std::vector<Vertex>> bow_spring_meshes;         // Undeformed mesh for each bow-spring (local coords)
    int ipoly_top = -1;

    struct ComponentInfo {
        uint start_poly; // First polygon index for this component
        uint end_poly;   // Last polygon index for this component
        scalar L;        // Length of the component
        string name;     // Name of the component
    };

    vector<ComponentInfo> component_info; // Information about each whole component

    void generate_mesh(const Config &config, const vector<PipeComponent> &pipe_assembly,
                       const PipeRenderComponents &pipe_render_components);

    void update_vertex_buffer(const Config &config, ConfigGraphics &config_graphics,
                              const curvlin::PipeSolver *solver_curvlin, const Pipe &pipe, const Hole &hole,
                              const vector<Vec3> &x_pipe, const vector<Quaternion> &q_pipe,
                              const array<MiscSpatialGraph, N_MG> &misc_plots,
                              const array<FluidSpatialGraph, N_FG> &fluid_plots, const byte *buf);

    void update_bow_spring_vertex_buffer(const Pipe &pipe, const std::vector<Vec3> &x_pipe,
                                         const std::vector<Quaternion> &q_pipe, float radial_scale, const byte *buf);
};
