#include "double-buffer-gui.hpp"

void DoubleBufferGUI::update_from_solver(curvlin::PipeSolver *solver_curvlin, const corot::PipeSolver *solver_corot,
                                         const Pipe &pipe, const Hole &hole, const BitRockData &bit_rock_data,
                                         const ArenaBump &arena_src, ConfigDynamic &conf_dyn, Config &config) {

    const uint write_buffer = 1 - active_buffer;

    ArenaBump &arena_h_write = buffers[write_buffer].arena_h;
    /*================================================================================
     Copy relavant fields from solver to the write buffer
    ==================================================================================*/
    if (config.pipe_solver_type == PipeSolverType::CURVILINEAR) {
        assert(solver_curvlin != nullptr);
        assert(arena_src.buf != nullptr);

        solver_curvlin->copy_beam_field_to_dst<curvlin::BeamField::u>(arena_h_write, arena_src);
        solver_curvlin->copy_beam_field_to_dst<curvlin::BeamField::v>(arena_h_write, arena_src);
        solver_curvlin->copy_beam_field_to_dst<curvlin::BeamField::a>(arena_h_write, arena_src);
        solver_curvlin->copy_beam_field_to_dst<curvlin::BeamField::theta>(arena_h_write, arena_src);
        solver_curvlin->copy_beam_field_to_dst<curvlin::BeamField::omega>(arena_h_write, arena_src);
        solver_curvlin->copy_beam_field_to_dst<curvlin::BeamField::alpha>(arena_h_write, arena_src);
        solver_curvlin->copy_beam_field_to_dst<curvlin::BeamField::f_int>(arena_h_write, arena_src);
        solver_curvlin->copy_beam_field_to_dst<curvlin::BeamField::f_dyn>(arena_h_write, arena_src);
        solver_curvlin->copy_beam_field_to_dst<curvlin::BeamField::f_hyd>(arena_h_write, arena_src);
        solver_curvlin->copy_beam_field_to_dst<curvlin::BeamField::m_int>(arena_h_write, arena_src);
        solver_curvlin->copy_beam_field_to_dst<curvlin::BeamField::m_dyn>(arena_h_write, arena_src);
        solver_curvlin->copy_beam_field_to_dst<curvlin::BeamField::m_hyd>(arena_h_write, arena_src);
        solver_curvlin->copy_beam_field_to_dst<curvlin::BeamField::i_pipe_to_ie_hole>(arena_h_write, arena_src);
        if (config.conf_stat.bc_bit_rock_type != BC_BitRockType::NO_BIT) {
            solver_curvlin->copy_bit_rock_to_dst(arena_h_write, arena_src, bit_rock_data);
        }

        solver_curvlin->update_drilling_quantities_curvlin(config, conf_dyn, pipe, arena_h_write, bit_rock_data);

    } else {
        assert(config.pipe_solver_type == PipeSolverType::COROTATIONAL);
        assert(solver_corot != nullptr);
    }

    /*================================================================================
     Update dynamic config data
    ==================================================================================*/
    buffers[write_buffer].config_dynamic = conf_dyn;

    {
        std::lock_guard<std::mutex> lock(swap_mutex);
        active_buffer = write_buffer;
    }
}