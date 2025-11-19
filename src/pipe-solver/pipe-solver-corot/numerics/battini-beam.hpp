#pragma once
#include "pipe-solver/pipe-solver-corot/utils-corot.hpp"

namespace corot {

namespace BattiniBeam {
Vec12 r_vector(const Vec3 &x1, const Vec3 &x2);

Mat12_3 G_matrix(const Vec3 &x1, const Vec3 &x2, const Mat3 &U1, const Mat3 &U2);

Mat6_12 P_matrix(const Vec3 &x1, const Vec3 &x2, const Mat3 &U1, const Mat3 &U2);

Mat3 co_rotating_rotation_matrix_current(const Vec3 &x1, const Vec3 &x2, const Mat3 &U1, const Mat3 &U2);

Mat12 E_matrix(const Vec3 &x1, const Vec3 &x2, const Mat3 &U1, const Mat3 &U2);

Vec7 local_deformations(const Vec3 &x1, const Vec3 &x2, const Vec3 &X1, const Vec3 &X2, const Mat3 &U1, const Mat3 &U2);

Mat3 rotation_vector_inverse_T_s(const uint node_id, const Vec3 &x1, const Vec3 &x2, const Vec3 &X1, const Vec3 &X2,
                                 const Mat3 &U1, const Mat3 &U2);

Mat7_12 b_matrix_g(const Vec3 &x1, const Vec3 &x2, const Mat3 &U1, const Mat3 &U2);
Mat7 b_matrix_a(const Vec3 &x1, const Vec3 &x2, const Vec3 &X1, const Vec3 &X2, const Mat3 &U1, const Mat3 &U2);

Mat7 local_stiffness_matrix(const Vec3 &X1, const Vec3 &X2, const scalar E, const scalar A, const scalar G,
                            const scalar It, const scalar Iy, const scalar Iz);

Vec7 local_internal_forces(const Vec3 &x1, const Vec3 &x2, const Vec3 &X1, const Vec3 &X2, const Mat3 &U1,
                           const Mat3 &U2, const scalar E, const scalar A, const scalar G, const scalar It,
                           const scalar Iy, const scalar Iz);

Vec7 local_internal_forces_a(const Vec3 &x1, const Vec3 &x2, const Vec3 &X1, const Vec3 &X2, const Mat3 &U1,
                             const Mat3 &U2, const scalar E, const scalar A, const scalar G, const scalar It,
                             const scalar Iy, const scalar Iz);

Vec12 global_internal_forces(const uint ie, const vector<Vec3> &X, const vector<Vec3> &d_trans,
                             const vector<Quaternion> &d_rot, const scalar A, const scalar Iy, const scalar Iz,
                             const scalar It, const scalar youngs, const scalar G);

void test();
} // namespace BattiniBeam
} // namespace corot
