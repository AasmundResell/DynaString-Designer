#pragma once
#include "beam-corot.hpp"
#include "pipe-solver/bcs.hpp"
#include "pipe-solver/beam-common.hpp"
#include "sim-tools/arena/arena.hpp"
#include "utils-corot.hpp"

namespace corot {

class PipeSolver {
  public:
    BeamData beam;
    PipeSolver(Config &config, ConfigDynamic &conf_dyn, const Pipe &pipe, const vector<PipeComponent> &pipe_assembly,
               const Hole &hole, BCsData &bcs, ArenaBump &arena_h);
    void step(Config &config, ConfigDynamic &conf_dyn, const Pipe &pipe, const vector<PipeComponent> &pipe_assembly,
              const Hole &hole, BCsData &bcs, ArenaBump &arena_h);

  private:
    void step_cpu(Config &config, ConfigDynamic &conf_dyn, const Pipe &pipe, const vector<PipeComponent> &pipe_assembly,
                  const Hole &hole, BCsData &bcs, ArenaBump &arena_h);
    void calc_static_loads(const uint top_node, const ConfigStatic &conf_stat, const BCsData &bcs, const Pipe &pipe,
                           vector<Vec3> &f_stat_trans, vector<Vec3> &f_stat_rot, ArenaBump &arena_h);
    void update_pipe_feed(Config &config, const Pipe &pipe, const Hole &hole, const BCsData &bcs, ArenaBump &arena_h);
    void step_gpu(Config &config, const Pipe &pipe, const vector<PipeComponent> &pipe_assembly, const Hole &hole,
                  BCsData &bcs, ArenaBump &arena_d);
    void save_csv(Config &config, const ConfigDynamic &conf_dyn, const Pipe &pipe, const Hole &hole, const BCsData &bcs,
                  ArenaBump &arena_h);
    void save_csv_global(Config &config, const ConfigDynamic &conf_dyn, const Pipe &pipe, const BCsData &bcs,
                         ArenaBump &arena_h);
    void save_csv_curvlin(Config &config, const ConfigDynamic &conf_dyn, const Pipe &pipe, const Hole &hole,
                          const BCsData &bcs, ArenaBump &arena_h);
    void initialize_beam_segment_solution(const Config &config, uint start_node, uint end_node, const Hole &hole,
                                          const Pipe &pipe, const BCsData &bcs, byte *buf);
};
} // namespace corot