#pragma once
#include "gui-manager-base.hpp"
#include "gui/graphics/graphics-manager.hpp"
#include "gui/plot-utils.hpp"
#include "sim-tools/gui/includes-GL.hpp"
#include "ui/ui-manager.hpp"

/*User input is either handled by callbacks or polling. Callback means that the input is handled by a callback function,
which is called by GLFW when an event occurs. Polling means explicitly checking if a button is pressed. This function
updates the state of frame_data that is not handled by callbacks.*/
void update_frame_data_polling(GLFWwindow *window);

class GUI_Manager : public GUI_ManagerBase {

    GLFWwindow *window = nullptr;
    vector<Vec3> x_pipe;
    vector<Quaternion> q_pipe;
    unique_ptr<GraphicsManager> graphics_manager;
    ConfigGraphics config_graphics; // State variables used by graphics and updated by ui
    array<SpatialGraph, N_SG> spatial_graphs;
    array<MiscSpatialGraph, N_MG> misc_spatial_graphs;
    array<FluidSpatialGraph, N_FG> fluid_spatial_graphs;
    uint last_processed_version = 0;

  public:
    GUI_Manager();
    unique_ptr<UI_Manager> ui_manager;

    void init_simulation(Simulator *simulator, PipeRenderComponents pipe_render_components) final;

    void render(Simulator *simulator, string *input_file, std::atomic<bool> &stop_program,
                std::atomic<bool> &start_simulator, std::atomic<bool> &restart_solver);

    void process_simulator(Simulator *simulator);

    void render_well_design(Simulator *simulator);

    void render_graphics(Simulator *simulator);

    void render_graphs(Simulator *simulator);

    void update_spatial_graphs(Simulator *simulator);

    void update_effective_tension(uint top_node, uint N, MiscSpatialGraph &misc_graph, const Pipe &pipe,
                                  const curvlin::FluidData &fluid, const byte *buf);

    ~GUI_Manager();

  private:
    void transform_solution_to_global_frame_curvlin(const Hole &hole, const Pipe &pipe,
                                                    const curvlin::PipeSolver &solver_curvlin, uint top_node,
                                                    float r_scale, const ArenaBump &arena_h);
    void transform_solution_to_global_frame_corot(const Hole &hole, const Pipe &pipe,
                                                  const corot::PipeSolver &solver_corot, uint top_node, float r_scale,
                                                  const ArenaBump &arena_h);
    void update_spatial_graphs_curvlin(const Pipe &pipe, const curvlin::PipeSolver &solver_curvlin, const uint top_node,
                                       const byte *buf);
    void update_spatial_graphs_corot(const Pipe &pipe, const Hole &hole, const corot::PipeSolver &solver_corot,
                                     const uint top_node, const byte *buf);
    void update_axial_tension(uint top_node, uint N, MiscSpatialGraph &misc_graph);
    void update_effective_torque(uint top_node, uint N, MiscSpatialGraph &misc_graph);
    void update_radial_acceleration(uint top_node, uint N, MiscSpatialGraph &misc_graph,
                                    const curvlin::PipeSolver &solver_curvlin, const Pipe &pipe, const Hole &hole,
                                    const Config &config, const byte *buf);
    void update_tangential_acceleration(uint top_node, uint N, MiscSpatialGraph &misc_graph,
                                        const curvlin::PipeSolver &solver_curvlin, const Pipe &pipe, const Hole &hole,
                                        const Config &config, const byte *buf);
    void update_axial_strain(uint top_node, uint N, MiscSpatialGraph &misc_graph, const Pipe &pipe, const byte *buf);
    void update_von_mises_stress(uint top_node, uint N, const Config &config, const curvlin::PipeSolver &solver_curvlin,
                                 MiscSpatialGraph &misc_graph, const Pipe &pipe, const Hole &hole, const byte *buf);
    void update_linear_contact_force(uint top_node, uint N, MiscSpatialGraph &misc_graph, const Pipe &pipe,
                                     const byte *buf);
    void update_frictional_contact_force(uint top_node, uint N, MiscSpatialGraph &misc_graph, const Pipe &pipe,
                                         const byte *buf);
    void update_standoff_ratio(uint top_node, uint N, const curvlin::PipeSolver &solver_curvlin,
                               MiscSpatialGraph &misc_graph, const Pipe &pipe, const Hole &hole, const byte *buf);
};
