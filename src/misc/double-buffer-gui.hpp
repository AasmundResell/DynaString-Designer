#pragma once
#include "misc/config.hpp"
#include "misc/includes.hpp"
#include "misc/utils.hpp"
#include "pipe-solver/pipe-solver-corot/pipe-solver-corot.hpp"
#include "pipe-solver/pipe-solver-curvlin/beam-curvlin.hpp"
#include "pipe-solver/pipe-solver-curvlin/fluid-curvlin.hpp"
#include "pipe-solver/pipe-solver-curvlin/pipe-solver-curvlin.hpp"

#include <mutex>
/*================================================================================
 Used for enabling multithreading of the GUI and solver at the same time. GUI thread
 will read data from this double buffer. The double buffers mirrors the Arenas
 used in the solver. The solver thread locks one buffer while writing to it and then
 swaps the buffers when done. The GUI thread reads from the other buffer.
==================================================================================*/

class DoubleBufferGUI {

    struct BufferData {
        ArenaBump arena_h;
        ConfigDynamic config_dynamic;
    };

    Array<BufferData, 2> buffers;
    std::atomic<uint> active_buffer{0};
    std::mutex swap_mutex;

    std::atomic<uint> buffer_version{0};

  public:
    DoubleBufferGUI(const ArenaBump &arena_h, const Config &config, const ConfigDynamic &conf_dyn) {
        for (uint i = 0; i < 2; i++) {
            ArenaBump &arena_h_gui = buffers[i].arena_h;
            arena_h_gui.create(MemType::HOST, arena_h.capacity_);
            ArenaBump::copy_all(arena_h_gui, arena_h);
            buffers[i].config_dynamic = conf_dyn;
        }
    }

    ~DoubleBufferGUI() {
        for (uint i = 0; i < 2; i++) {
            buffers[i].arena_h.destroy();
        }
    }

    uint get_buffer_version() const { return buffer_version.load(); }
    void increment_buffer_version() { buffer_version++; }

    inline uint get_active_buffer() const { return active_buffer.load(std::memory_order_acquire); }

    const ArenaBump &get_arena_h() const {
        assert(active_buffer == 0 || active_buffer == 1);
        return buffers[get_active_buffer()].arena_h;
    }

    const ConfigDynamic &get_config_dynamic() const {
        assert(active_buffer == 0 || active_buffer == 1);
        return buffers[get_active_buffer()].config_dynamic;
    }

    void update_from_solver(curvlin::PipeSolver *solver_curvlin, const corot::PipeSolver *solver_corot,
                            const Pipe &pipe, const Hole &hole, const BitRockData &bit_rock_data,
                            const ArenaBump &arena_src, ConfigDynamic &conf_dyn, Config &config);
};