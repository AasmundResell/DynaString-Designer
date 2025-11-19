#pragma once
#include "misc/includes.hpp"
#include "pipe/pipe.hpp"
#include "simulator.hpp"

class GUI_ManagerBase {
  public:
    static unique_ptr<GUI_ManagerBase> create();
    virtual void init_simulation(Simulator *simulator, PipeRenderComponents pipe_render_components) = 0;
    virtual void render(Simulator *simulator, string *input_file, std::atomic<bool> &stop_program,
                        std::atomic<bool> &start_simulator, std::atomic<bool> &restart_solver) = 0;
};