#pragma once
#include "pipe-solver/bcs.hpp"
#include "pipe-solver/pipe-solver-corot/beam-corot.hpp"
#include "pipe-solver/pipe-solver-corot/utils-corot.hpp"

namespace corot {

void check_energy_balance(const Config &config, const ConfigDynamic &conf_dyn);

void set_initial_configuration(const Config &config, const Hole &hole, const Pipe &pipe, vector<Vec3> &X,
                               vector<Vec3> &d_trans, vector<Quaternion> &d_rot, const vector<uint> &indices,
                               const BCsData &bcs, const byte *buf);

void calc_initial_accelerations(const ConfigStatic &conf_stat, const uint top_node, const Pipe &pipe, const Hole &hole,
                                BeamData &beam, ArenaBump &arena_h);

} // namespace corot