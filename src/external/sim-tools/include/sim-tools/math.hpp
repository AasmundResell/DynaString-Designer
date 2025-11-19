#include "includes.hpp"
#include "quaternion.hpp"
using sito::Quaternion;

inline Vec3 interp_spline_cubic_hermite(const Vec3 &x_A, const Vec3 &x_B, const Vec3 &e1_A, const Vec3 &e1_B,
                                        scalar xi) {
    assert(xi >= 0 && xi <= 1);
    /*The first basis vector is tangential to the spline*/

    const scalar xi_2 = powi(xi, 2);
    const scalar xi_3 = powi(xi, 3);

    const Vec3 x = (2 * xi_3 - 3 * xi_2 + 1) * x_A + (xi_3 - 2 * xi_2 + xi) * e1_A + (-2 * xi_3 + 3 * xi_2) * x_B +
                   (xi_3 - xi_2) * e1_B;
    return x;
}

inline Vec3 interp_spline_cubic_hermite(const Vec3 &x_A, const Vec3 &x_B, const Quaternion &q_A, const Quaternion &q_B,
                                        scalar xi) {
    scalar path_length = (x_B - x_A).norm();
    const Vec3 e1_B = q_B.get_e1() * path_length;
    const Vec3 e1_A = q_A.get_e1() * path_length;

    return interp_spline_cubic_hermite(x_A, x_B, e1_A, e1_B, xi);
}
