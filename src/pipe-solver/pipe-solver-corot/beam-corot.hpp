#pragma once
#include "hole/hole.hpp"
#include "pipe/pipe.hpp"
#include "utils-corot.hpp"

namespace corot {

extern uint n_glob;
extern scalar t_glob;

template <typename T> void print_beam(const vector<T> &vec) {
    for (const T &element : vec) {
        cout << element << " ";
    }
    cout << endl;
}

struct BeamData {
    vector<Vec3> X;
    vector<Vec3> d_trans;
    vector<Quaternion> d_rot;
    vector<Vec3> v_trans;
    vector<Vec3> a_trans;
    vector<Vec3> v_rot;                                  // angular velocity in body frame
    vector<Vec3> a_rot;                                  // angular acceleration in body frame
    vector<Vec3> L_rot;                                  // Angular impulse
    vector<Vec3> m_rot;                                  // Moment at t_{n+1/2}
    vector<Vec3> f_int_trans, f_ext_trans, f_stat_trans; /*Nodal translational forces*/
    vector<Vec3> f_int_rot, f_ext_rot, f_stat_rot;       /*Nodal rotational forces (moments)*/
    vector<scalar> M;                                    /*Lumped mass */
    vector<Vec3> J_u;                                    /*Moment of inertia in body frame*/
    vector<Vec3> delta_d_trans;                          /*Used if checking energy balance*/
    vector<Vec3> delta_d_rot;
    vector<uint> i_pipe_to_ie_hole; /*Used to refer to the index of the nearest hole if contact is enabled*/

    BeamData(Config &config, const Pipe &pipe, byte *buf);
};

} // namespace corot