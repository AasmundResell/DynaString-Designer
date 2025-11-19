#pragma once
#include "pipe-solver/bcs.hpp"
#include "pipe-solver/pipe-solver-curvlin/beam-curvlin.hpp"
#include "pipe-solver/pipe-solver-curvlin/fluid-curvlin.hpp"
#include "pipe-solver/pipe-solver-curvlin/numerics/contact-curvlin.hpp"
#include "pipe-solver/pipe-solver-curvlin/numerics/pre-derived-integration.hpp"

namespace curvlin {

void integrate_time_ax_tor(const uint grid_dim, const uint block_dim, scalar dt, const Config &config,
                           const uint top_node, BeamData &beam, const Pipe &pipe, const Hole &hole, byte *buf);

void integrate_time_lateral(const uint grid_dim, const uint block_dim, const Config &config, const uint top_node,
                            BeamData &beam, const Pipe &pipe, const Hole &hole, byte *buf);

void integrate_time(const uint grid_dim, const uint block_dim, const Config &config, const uint top_node,
                     BeamData &beam, const Pipe &pipe, const Hole &hole,
                    byte *buf);
void assemble_even(const uint grid_dim, const uint block_dim, const Config &config, const uint top_node,
                   const BCsData &bc, BeamData &beam, const FluidData &fluid, const Pipe &pipe, const Hole &hole,
                   byte *buf);

void initialize_forces_and_assemble_odd(const uint grid_dim, const uint block_dim, const Config &config,
                                        const uint top_node, const BCsData &bc, BeamData &beam, const FluidData &fluid,
                                        const Pipe &pipe, const Hole &hole, byte *buf);

void assemble_even_ax_tor(const uint grid_dim, const uint block_dim, const Config &config, const uint top_node,
                   const BCsData &bc, BeamData &beam, const FluidData &fluid, const Pipe &pipe, const Hole &hole,
                   byte *buf);

void initialize_forces_and_assemble_odd_ax_tor(const uint grid_dim, const uint block_dim, const Config &config,
                                        const uint top_node, const BCsData &bc, BeamData &beam, const FluidData &fluid,
                                        const Pipe &pipe, const Hole &hole, byte *buf);


DEVICE_FUNC void calc_element_forces_gauss_legendre(uint ie, const ConfigStatic &conf_stat, const BCsData &bcs,
                                                    BeamData &beam, const FluidData &fluid, const Pipe &pipe,
                                                    const Hole &hole, byte *buf);

void calc_nodal_friction_forces(const uint grid_dim, const uint block_dim, const Config &config,
                               const uint top_node, const Pipe &pipe, const Hole &hole, BeamData &beam, byte *buf);

void update_fluid_mass(ConfigStatic &conf_stat, const uint top_node, const Pipe &pipe, const Hole &hole, BeamData &beam,
                       ArenaBump &arena_h);
void calc_static_loads(ConfigStatic &conf_stat, BeamData &beam, const FluidData &fluid, const Hole &hole,
                       const Pipe &pipe, const uint top_node, ArenaBump &arena_h);

void calc_softstring_axial_tension(const Pipe &pipe, BeamData &beam, const ConfigStatic &conf_stat, byte *buf);

void calc_fluid_newtonian(const ConfigStatic &conf_stat, FluidData &fluid, const Hole &hole, const Pipe &pipe,
                          const BeamData &beam, const uint top_node, ArenaBump &arena_h);
void calc_fluid_field_bernoulli(ConfigStatic &conf_stat, const uint top_node, FluidData &fluid, const BeamData &beam,
                                const Pipe &pipe, const Hole &hole, ArenaBump &arena_h);

/*--------------------------------------------------------------------
Setup functions
--------------------------------------------------------------------*/
void constant_data_setup(byte *buf_d, const uint top_node, const ConfigStatic &conf_stat, const BeamData &beam,
                         const FluidData &fluid, const Pipe &pipe, const Hole &hole, const BCsData &bc_data,
                         const BitRockData &bit_rock_data);

void initialize_beam_segment_solution(uint start_node, uint end_node, const Hole &hole, const Pipe &pipe,
                                      BeamData &beam, const BCsData &bcs, byte *buf);

} // namespace curvlin
