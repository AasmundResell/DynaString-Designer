#pragma once
#include "pipe-solver/pipe-solver-corot/beam-corot.hpp"
#include "pipe-solver/pipe-solver-corot/numerics/battini-beam.hpp"
#include "pipe-solver/pipe-solver-corot/numerics/crisfield-beam.hpp"
#include "pipe-solver/pipe-solver-corot/utils-corot.hpp"
//#include "bcs-corot.hpp"
//#include "contact-corot.hpp"

namespace corot {

void calc_forces(const ConfigStatic &conf_stat, const uint top_node, const Pipe &pipe, const Hole &hole, BeamData &beam,
                 ArenaBump &arena_h);

void work_update_partial(uint N, const uint top_node, const vector<Vec3> &delta_d_trans,
                         const vector<Vec3> &delta_d_rot, const vector<Vec3> &f_int_trans,
                         const vector<Vec3> &f_int_rot, const vector<Vec3> &f_ext_trans, const vector<Vec3> &f_ext_rot,
                         const vector<Vec3> &f_stat_trans, const vector<Vec3> &f_stat_rot, scalar &W_ext,
                         scalar &W_int);
void kinetic_energy_update(uint N, const vector<scalar> &M, const vector<Vec3> &J_u, const vector<Vec3> &v_trans,
                           const vector<Vec3> &v_rot, scalar &KE);

} // namespace corot