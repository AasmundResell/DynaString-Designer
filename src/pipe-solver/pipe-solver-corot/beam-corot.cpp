#include "beam-corot.hpp"
namespace corot {

BeamData::BeamData(Config &config, const Pipe &pipe, byte *buf) {
    const uint N = pipe.N;
    const uint Ne = N - 1;
    X.resize(N, Vec3::Zero());
    d_trans.resize(N, Vec3::Zero());
    d_rot.resize(N);
    v_trans.resize(N, Vec3::Zero());
    v_rot.resize(N, Vec3::Zero());
    a_rot.resize(N, Vec3::Zero());
    a_trans.resize(N, Vec3::Zero());
    L_rot.resize(N, Vec3::Zero());
    m_rot.resize(N, Vec3::Zero());
    f_int_trans.resize(N, Vec3::Zero());
    f_int_rot.resize(N, Vec3::Zero());
    f_ext_trans.resize(N, Vec3::Zero());
    f_ext_rot.resize(N, Vec3::Zero());
    f_stat_trans.resize(N, Vec3::Zero());
    f_stat_rot.resize(N, Vec3::Zero());
    M.resize(N, 0.0);
    J_u.resize(N, Vec3::Zero());
    i_pipe_to_ie_hole.resize(N, 0);
    
    if (config.conf_stat.check_energy_balance) {
        delta_d_trans.resize(N);
        delta_d_rot.resize(N);
    }
}
} // namespace corot
