#pragma once
#include "misc/includes.hpp"
#include "misc/utils.hpp"
#include "pipe-solver/bit-rock.hpp"
#include "pipe-solver/pipe-solver-corot/beam-corot.hpp"
#include "pipe-solver/pipe-solver-curvlin/beam-curvlin.hpp"

struct BCs {
    scalar theta_top_n{0.0}, theta_top_np{0.0}; /*Absolute angle top*/
    scalar omega_top_n{0.0}, omega_top_np{0.0}; /*Angular velocity top*/
    scalar alpha_top_n{0.0}, alpha_top_np{0.0}; /*Angular acceleration top*/
    scalar u_top_n{0.0}, u_top_np{0.0};         /*Axial displacement top*/
    scalar v_top_n{0.0}, v_top_np{0.0};         /*Axial velocity top*/
    scalar a_top_n{0.0}, a_top_np{0.0};         /*Axial acceleration top*/
};

struct BCsData {
    Offset<BCs> bcs_field;

    DEVICE_FUNC BCs &get_bcs_field(byte *buf) const {
        ArrayView<BCs> bcs_field_view = {buf, bcs_field};
        assert(bcs_field_view.count == 1);
        return bcs_field_view[0];
    }
    DEVICE_FUNC const BCs &get_bcs_field(const byte *buf) const {
        ArrayView<BCs> bcs_field_view = {buf, bcs_field};
        assert(bcs_field_view.count == 1);
        return bcs_field_view[0];
    }
};

BCsData create_bcs(ArenaBump &arena_h);
void set_pipe_feed_length_initial(Config &config, ConfigDynamic &conf_dyn, BCsData &bcs, const Pipe &pipe,
                                  const vector<PipeComponent> &pipe_assembly, byte *buf);
void update_top_node(Config &config, ConfigDynamic &conf_dyn, const Pipe &pipe,
                     const vector<PipeComponent> &pipe_assembly);

void update_top_kinematics(scalar dt, BCs &bcs_field, scalar v_top_target, scalar omega_top_target);

namespace corot {

void set_bc_force_dynamic(const uint N, const ConfigStatic &conf_stat, BeamData &beam, const Hole &hole, byte *buf);

void set_bc_top(bool first_call, const uint top_node, ConfigStatic &conf_stat, BCs &bcs_field, const Pipe &pipe,
                const Hole &hole, BeamData &beam, byte *buf);

} // namespace corot

namespace curvlin {

void set_bc_kinematic(const Config &config, const ConfigDynamic &conf_dyn, const Pipe &pipe, BeamData &beam,
                      BCsData &bcs, byte *buf);

void set_bc_kinematic_lateral(const Config &config, const Pipe &pipe, BeamData &beam, byte *buf);

void set_bc_force_dynamic(const Config &config, const Pipe &pipe, BeamData &beam, BitRockData &bit_rock_data,
                          const BCsData &bcs, byte *buf);

void set_bc_kinematic_ax_tor(scalar dt, const Config &config, const ConfigDynamic &conf_dyn, const Pipe &pipe,
                             BeamData &beam, BCsData &bcs, byte *buf);

} // namespace curvlin