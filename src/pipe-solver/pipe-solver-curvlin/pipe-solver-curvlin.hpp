#pragma once
#include "beam-curvlin.hpp"
#include "fluid-curvlin.hpp"
#include "pipe-solver/bcs.hpp"

namespace curvlin {

class PipeSolver {
  public:
    BeamData beam;
    FluidData fluid;
    uint grid_dim_nodal;

    PipeSolver(Config &config, ConfigDynamic &conf_dyn, const Pipe &pipe, const vector<PipeComponent> &pipe_assembly,
               const Hole &hole, BCsData &bcs, BitRockData &bit_rock_data, ArenaBump &arena_h, ArenaBump &arena_d);
    void update_constant_data_gpu(const Config &config, const Pipe &pipe, const Hole &hole, const BCsData &bc_data,
                                  const BitRockData &bit_rock_data, ArenaBump &arena_d);

    void step(Config &config, ConfigDynamic &conf_dyn, const Pipe &pipe, const vector<PipeComponent> &pipe_assembly,
              const Hole &hole, BCsData &bcs, BitRockData &bit_rock_data, ArenaBump &arena_h, ArenaBump &arena_d);

    /*Copy fields device to host stuff*/
    template <BeamField field> void copy_beam_field_to_dst(ArenaBump &arena_h, const ArenaBump &arena_d);
    template <FluidField field> void copy_fluid_field_to_dst(ArenaBump &arena_dst, const ArenaBump &arena_src);
    void copy_bcs_to_dst(ArenaBump &arena_h, const ArenaBump &arena_d, const BCsData &bcs);
    void copy_bit_rock_to_dst(ArenaBump &arena_h, const ArenaBump &arena_d, const BitRockData &bit_rock_data);

    void update_drilling_quantities_curvlin(Config &config, ConfigDynamic &conf_dyn, const Pipe &pipe,
                                            const ArenaBump &arena_h, const BitRockData &bit_rock_data) const;

    void compute_generalized_strains(const Vec12 &U, const Vec10 &N, const Vec10 &dN, scalar kappa_y, scalar kappa_z,
                                     Vec3 &gamma, Vec3 &kappa) const;

    scalar compute_vm_stress_curvlin(const uint ie, scalar xi, const Pipe &pipe, const Hole &hole, const Config &config,
                                     const byte *buf) const;

    scalar compute_radial_acceleration(const uint i, const Pipe &pipe, const Hole &hole, const byte *buf) const;

    scalar compute_tangential_acceleration(const uint i, const Pipe &pipe, const Hole &hole, const byte *buf) const;

    scalar compute_standoff(const uint i, const Pipe &pipe, const Hole &hole, const byte *buf) const;

  private:
    vector<scalar> m_vec; // Temporary storage for mass vector during assembly
    vector<Vec3> J_vec;   // Temporary storage for mass vector during assembly

    void update_pipe_feed(Config &config, const Pipe &pipe, const Hole &hole, const BCsData &bc_data,
                          const BitRockData &bit_rock_data, ArenaBump &arena_h, ArenaBump &arena_d);
    void assemble_mass_vectors(Config &config, const Pipe &pipe, const Hole &hole, byte *buf);
    void save_csv(Config &config, ConfigDynamic &conf_dyn, const Pipe &pipe_h, const Hole &hole_h, const BCsData &bcs,
                  const BitRockData &bit_rock_data, ArenaBump &arena_h, ArenaBump &arena_d);
    void step_solver(Config &config, ConfigDynamic &conf_dyn, const Pipe &pipe,
                     const vector<PipeComponent> &pipe_assembly, const Hole &hole, BCsData &bc_data,
                     BitRockData &bit_rock_data, ArenaBump &arena_h, ArenaBump &arena_d);
    void step_solver_soft_string(Config &config, ConfigDynamic &conf_dyn, const Pipe &pipe,
                                 const vector<PipeComponent> &pipe_assembly, const Hole &hole, BCsData &bc_data,
                                 BitRockData &bit_rock_data, ArenaBump &arena_h, ArenaBump &arena_d);
    void update_control_input(scalar dt, Config &config, ConfigDynamic &conf_dyn, const Pipe &pipe,
                              const vector<PipeComponent> &pipe_assembly, const BCsData &bc_data, ArenaBump &arena_h,
                              ArenaBump &arena_d);
    void save_csv_curvlin(Config &config, const ConfigDynamic &conf_dyn, const Pipe &pipe, const Hole &hole,
                          const BCsData &bcs, ArenaBump &arena_h);
    void save_csv_global(Config &config, const ConfigDynamic &conf_dyn, const Pipe &pipe, const Hole &hole,
                         const BCsData &bcs, ArenaBump &arena_h);
    void save_csv_fluid(Config &config, const Pipe &pipe, const uint top_node, ArenaBump &arena_h);
    void copy_current_pipe_feed_to_host(ArenaBump &arena_h, ArenaBump &arena_d);
    void copy_static_force_and_fluid_fields_to_device(ArenaBump &arena_h, ArenaBump &arena_d);
    void copy_beam_kinematics_to_device(ArenaBump &arena_h, ArenaBump &arena_d);
};

template <BeamField field> void PipeSolver::copy_beam_field_to_dst(ArenaBump &arena_dst, const ArenaBump &arena_src) {
    CUDA_CHECK_ERROR(cudaDeviceSynchronize()); // probably unnecessary
    static_assert(field < BeamField::COUNT);

    const OffsetUntyped offset = beam.offsets[(uint)field];
    using T = typename BeamFieldTraits<field>::variable_type;
    ArenaBump::copy_slice(arena_dst, arena_src, offset.offset, offset.offset + offset.count * sizeof(T));
}

template <FluidField field> void PipeSolver::copy_fluid_field_to_dst(ArenaBump &arena_dst, const ArenaBump &arena_src) {
    CUDA_CHECK_ERROR(cudaDeviceSynchronize()); // probably unnecessary
    static_assert(field < FluidField::COUNT);

    const OffsetUntyped offset = fluid.offsets[(uint)field];
    using T = typename FluidFieldTraits<field>::variable_type;
    ArenaBump::copy_slice(arena_dst, arena_src, offset.offset, offset.offset + offset.count * sizeof(T));
}
} // namespace curvlin