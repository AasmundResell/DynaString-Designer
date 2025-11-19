#ifdef __CUDACC__
#include "pipe-solver/pipe-solver-curvlin/numerics/constant-data.hpp"

namespace curvlin {
    __constant__ ConstantData cd_d;

void constant_data_setup(byte *buf_d, const uint top_node, const ConfigStatic &conf_stat, 
                        const BeamData &beam, const FluidData &fluid,
                        const Pipe &pipe, const Hole &hole,
                        const BCsData &bc_data, const BitRockData &bit_rock_data) {

    ConstantData cd_h{};
    cd_h.buf = buf_d;
    cd_h.top_node = top_node;
    cd_h.conf_stat = conf_stat;
    cd_h.beam_field = beam;
    cd_h.fluid_field = fluid;
    cd_h.pipe = pipe;
    cd_h.hole = hole;
    cd_h.bc_data = bc_data;
    cd_h.bit_rock_data = bit_rock_data;
    CUDA_CHECK_ERROR(cudaMemcpyToSymbol(cd_d, &cd_h, sizeof(ConstantData)));
}

} // namespace curvlin
#endif

