#pragma once
#include "hole/hole.hpp"
#include "misc/config.hpp"
#include "misc/double-buffer-gui.hpp"
#include "misc/includes.hpp"
#include "pipe-solver/bcs.hpp"
#include "pipe-solver/pipe-solver-corot/pipe-solver-corot.hpp"
#include "pipe-solver/pipe-solver-curvlin/pipe-solver-curvlin.hpp"
#include "pipe/pipe.hpp"
#include "sim-tools/arena/arena.hpp"

class Simulator {
  public:
    Config config;
    BCsData bc_data;
    BitRockData bit_rock_data;
    Hole hole;
    Pipe pipe;
    vector<PipeComponent> pipe_assembly;

    unique_ptr<curvlin::PipeSolver> pipe_solver_curvlin = nullptr;
    unique_ptr<corot::PipeSolver> pipe_solver_corot = nullptr;

    unique_ptr<DoubleBufferGUI> double_buffer_gui;

    Simulator(string &input_file, ConfigGUI &config_gui, PipeRenderComponents *pipe_render_components);
    ~Simulator();
    void progress_solver(ConfigGUI &config_gui, std::atomic<bool> &stop_program);
    void step_solver();
    void restart_solver();
    void stop_timer();
    void update_constant_data_gpu();
    void damping_initialization(ConfigGUI &config_gui);
    void sleep_solver(bool enforce_real_time);
    const ConfigDynamic &get_config_dynamic_gui() const;

  private:
    // ConfigDynamic and solver arenas private to ensure gui is not acessing them directly
    ConfigDynamic config_dynamic;
    ArenaBump arena_h, arena_d;
    void print_setup_info();
    void create_output_dir();
};