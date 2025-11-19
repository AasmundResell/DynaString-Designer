#pragma once
#ifdef __CUDACC__

#include "hole/hole.hpp"
#include "misc/config.hpp"
#include "pipe-solver/bcs.hpp"
#include "pipe-solver/pipe-solver-curvlin/beam-curvlin.hpp"
#include "pipe-solver/pipe-solver-curvlin/fluid-curvlin.hpp"
#include "pipe/pipe.hpp"

namespace curvlin {

struct ConstantData {
    byte *buf;
    uint top_node;
    ConfigStatic conf_stat;
    BeamData beam_field;
    FluidData fluid_field;
    Pipe pipe;
    Hole hole;
    BCsData bc_data;
    BitRockData bit_rock_data;
};

extern __constant__ ConstantData cd_d;

} // namespace curvlin
#endif
