#pragma once
#include "pipe-solver/bcs.hpp"
#include "pipe-solver/beam-common.hpp"
#include "pipe-solver/pipe-solver-corot/beam-corot.hpp"
#include "pipe-solver/pipe-solver-corot/utils-corot.hpp"

namespace corot {

void calc_hole_contact_forces_nodewise(const ConfigStatic &config_s, const uint top_node, const Pipe &pipe,
                                       const Hole &hole, BeamData &beam, byte *buf);

template <bool initial_search>
void update_hole_contact_indices(const uint N, const Hole &hole, vector<uint> &i_pipe_to_ie_hole, const vector<Vec3> &X,
                                 const vector<Vec3> &d_trans, byte *buf);

void initialize_hole_contact(const ConfigStatic &conf_stat, const Pipe &pipe, const Hole &hole, BeamData &beam,
                             byte *buf);

} // namespace corot
