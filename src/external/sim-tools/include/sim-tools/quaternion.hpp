#pragma once
#include "includes.hpp"
namespace sito {

struct Quaternion {
    scalar q0;
    Vec3 q;

    DEVICE_FUNC Mat3 to_matrix() const;

    /*Obtain indiviual basis vectors*/
    DEVICE_FUNC Vec3 get_e1() const;
    DEVICE_FUNC Vec3 get_e2() const;
    DEVICE_FUNC Vec3 get_e3() const;

    DEVICE_FUNC void from_matrix(const Mat3 &R, scalar input_matrix_tol = SMALL_SCALAR);
    DEVICE_FUNC scalar norm() const;
    DEVICE_FUNC scalar norm_sqr() const;
    DEVICE_FUNC void normalize();
    // Quaternion() : q0{1}, q{Vec3::Zero()} {}
    DEVICE_FUNC Quaternion() {}

    /*Create from pseudo vector*/
    DEVICE_FUNC Quaternion(const Vec3 &Theta);

    DEVICE_FUNC Quaternion product(const Quaternion &a) const;

    /*Updates the rotation as U_{n+1} = exp(S(theta))*U_n,
    where theta is an incremental rotation in the inertial frame*/
    DEVICE_FUNC void exponential_map_inertial_frame(const Vec3 &theta);

    /*Updates the rotation as U_{n+1} = U_n*exp(S(theta_u)),
    where theta_u is an incremental rotation in the body frame*/
    DEVICE_FUNC void exponential_map_body_frame(const Vec3 &theta_u);

    /*"Linear interpolation" for rotations*/
    DEVICE_FUNC static Quaternion lerp(const Quaternion &q1, const Quaternion &q2, scalar t);

    /*Performs the operation vn = R(q)*v0*/
    DEVICE_FUNC Vec3 rotate_vector(const Vec3 &v0) const;

    /*Performs the operation vn = R(q)^T*v0*/
    DEVICE_FUNC Vec3 rotate_vector_reversed(const Vec3 &v0) const;

    friend ostream &operator<<(ostream &os, const Quaternion &rhs) {
        return os << "[q0, q1, q2, q3] = [" << rhs.q0 << ", " << rhs.q.x() << ", " << rhs.q.y() << ", " << rhs.q.z()
                  << "]\n";
    }

    DEVICE_FUNC Quaternion operator+(Quaternion rhs) const;
    DEVICE_FUNC Quaternion operator*(scalar rhs) const;
};

inline Mat3 Quaternion::to_matrix() const {
    assert(is_close(this->norm(), 1.0));
    scalar q1 = q.x();
    scalar q2 = q.y();
    scalar q3 = q.z();
    Mat3 triad = 2 * Mat3{{q0 * q0 + q1 * q1 - (scalar)0.5, q1 * q2 - q3 * q0, q1 * q3 + q2 * q0},
                          {q2 * q1 + q3 * q0, q0 * q0 + q2 * q2 - (scalar)0.5, q2 * q3 - q1 * q0},
                          {q3 * q1 - q2 * q0, q3 * q2 + q1 * q0, q0 * q0 + q3 * q3 - (scalar)0.5}};

    assert(is_orthonormal(triad));
    return triad;
}

inline DEVICE_FUNC Vec3 Quaternion::get_e1() const {
    scalar q1 = q.x();
    scalar q2 = q.y();
    scalar q3 = q.z();
    Vec3 e1 = 2 * Vec3{q0 * q0 + q1 * q1 - (scalar)0.5, q2 * q1 + q3 * q0, q3 * q1 - q2 * q0};
    assert(is_close(e1.norm(), 1.0));
    return e1;
}
inline DEVICE_FUNC Vec3 Quaternion::get_e2() const {
    scalar q1 = q.x();
    scalar q2 = q.y();
    scalar q3 = q.z();
    Vec3 e2 = 2 * Vec3{q1 * q2 - q3 * q0, q0 * q0 + q2 * q2 - (scalar)0.5, q2 * q3 - q1 * q0};
    assert(is_close(e2.norm(), 1.0));
    return e2;
}
inline DEVICE_FUNC Vec3 Quaternion::get_e3() const {
    scalar q1 = q.x();
    scalar q2 = q.y();
    scalar q3 = q.z();
    Vec3 e3 = 2 * Vec3{q1 * q3 + q2 * q0, q2 * q3 - q1 * q0, q0 * q0 + q3 * q3 - (scalar)0.5};
    assert(is_close(e3.norm(), 1.0));
    return e3;
}
inline void Quaternion::from_matrix(const Mat3 &R, scalar input_matrix_tol) {

    assert(is_orthonormal(R, input_matrix_tol));
    //   See chapter 16.10 crisfield
    scalar R11 = R(0, 0);
    scalar R22 = R(1, 1);
    scalar R33 = R(2, 2);
    scalar R12 = R(0, 1);
    scalar R21 = R(1, 0);
    scalar R23 = R(1, 2);
    scalar R32 = R(2, 1);
    scalar R13 = R(0, 2);
    scalar R31 = R(2, 0);
    scalar trR = R11 + R22 + R33;
    scalar &q1 = this->q.x();
    scalar &q2 = this->q.y();
    scalar &q3 = this->q.z();

    scalar a = max(max(trR, R11), max(R22, R33));
    if (a == trR) {
        q0 = 0.5 * sqrt(1 + a);
        q1 = (R32 - R23) / (4 * q0);
        q2 = (R13 - R31) / (4 * q0);
        q3 = (R21 - R12) / (4 * q0);
    } else if (a == R11) {
        q1 = sqrt(0.5 * a + 0.25 * (1 - trR));
        q0 = 0.25 * (R32 - R23) / q1;
        q2 = 0.25 * (R21 + R12) / q1;
        q3 = 0.25 * (R31 + R13) / q1;
    } else if (a == R22) {
        q2 = sqrt(0.5 * a + 0.25 * (1 - trR));
        q0 = 0.25 * (R13 - R31) / q2;
        q1 = 0.25 * (R12 + R21) / q2;
        q3 = 0.25 * (R32 + R23) / q2;
    } else {
        assert(a == R33);
        q3 = sqrt(0.5 * a + 0.25 * (1 - trR));
        q0 = 0.25 * (R21 - R12) / q3;
        q1 = 0.25 * (R13 + R31) / q3;
        q2 = 0.25 * (R23 + R32) / q3;
    }
    assert(is_close(norm(), 1.0));
    this->normalize(); // needed ?
}

inline scalar Quaternion::norm() const {
    return sqrt(norm_sqr());
}
inline scalar Quaternion::norm_sqr() const {
    return q0 * q0 + q.x() * q.x() + q.y() * q.y() + q.z() * q.z();
}

inline void Quaternion::normalize() {
    scalar inv_norm = 1.0 / this->norm();
    q0 *= inv_norm; // probably not faster than just divding by norm after compiler optimizations
    q.x() *= inv_norm;
    q.y() *= inv_norm;
    q.z() *= inv_norm;
}

inline Quaternion::Quaternion(const Vec3 &Theta) {
    scalar theta = Theta.norm();
    q0 = cos(theta / 2);
    q = sin(theta / 2) * Theta.normalized();
    assert(is_close(this->norm_sqr(), 1));
}

inline Quaternion Quaternion::product(const Quaternion &a) const {
    assert(is_close(a.norm_sqr(), 1));
    /*16.72 crisfield*/
    const Quaternion &b{*this};
    Quaternion q_ab;
    q_ab.q0 = a.q0 * b.q0 - a.q.dot(b.q);
    q_ab.q = a.q0 * b.q + b.q0 * a.q - a.q.cross(b.q);
    assert(is_close(q_ab.norm_sqr(), 1));
    return q_ab;
}

inline void Quaternion::exponential_map_inertial_frame(const Vec3 &theta) {
    Quaternion delta_q{theta};
    *this = delta_q.product(*this);
}

inline void Quaternion::exponential_map_body_frame(const Vec3 &theta_u) {
    Quaternion delta_q{theta_u};
    *this = this->product(delta_q);
}
// inline Quaternion Quaternion::lerp(const Quaternion &q1, const Quaternion &q2, scalar t) {
//     assert(t >= -SMALL_SCALAR && t <= 1 + SMALL_SCALAR);
//     assert(is_close(q1.norm_sqr(), 1) && is_close(q2.norm_sqr(), 1));
//     Quaternion q_lerp;
//     q_lerp = q1 * (1 - t) + q2 * t;
//     q_lerp.normalize();
//     assert(is_close(q_lerp.norm_sqr(), 1));
//     return q_lerp;
// }

inline Quaternion Quaternion::lerp(const Quaternion &q1, const Quaternion &q2, scalar t) {
    assert(t >= -SMALL_SCALAR && t <= 1 + SMALL_SCALAR);

    // Ensure shortest path (flip q2 if in opposite hemisphere)
    scalar dot = q1.q0 * q2.q0 + q1.q.dot(q2.q);
    Quaternion q2c = q2;
    if (dot < 0.0) {
        q2c.q0 = -q2c.q0;
        q2c.q = -q2c.q;
        dot = -dot;
    }

    Quaternion q_lerp = q1 * (1 - t) + q2c * t;

    // Guard against near-zero norm (can happen if quaternions nearly cancel)
    scalar n = q_lerp.norm();
    if (n < SMALL_SCALAR) {
        // fallback: return the closer endpoint to avoid NaNs
        return (t < 0.5) ? q1 : q2c;
    }
    q_lerp.q0 /= n;
    q_lerp.q /= n;
    return q_lerp;
}

inline Vec3 Quaternion::rotate_vector(
    const Vec3 &v0) const { /*Simply multiplying Rodrigues formula for quaternions with the vector*/
    const scalar q1 = q.x();
    const scalar q2 = q.y();
    const scalar q3 = q.z();
    Vec3 vn;
    vn.x() = 2 * ((q0 * q0 + q1 * q1 - 0.5) * v0.x() + (q1 * q2 - q3 * q0) * v0.y() + (q1 * q3 + q2 * q0) * v0.z());
    vn.y() = 2 * ((q2 * q1 + q3 * q0) * v0.x() + (q0 * q0 + q2 * q2 - 0.5) * v0.y() + (q2 * q3 - q1 * q0) * v0.z());
    vn.z() = 2 * ((q3 * q1 - q2 * q0) * v0.x() + (q3 * q2 + q1 * q0) * v0.y() + (q0 * q0 + q3 * q3 - 0.5) * v0.z());
    assert(is_close(vn.norm(), v0.norm()));
    return vn;
}

inline Vec3 Quaternion::rotate_vector_reversed(
    const Vec3 &v0) const { /*Simply multiplying the transposed of Rodrigues formula for quaternions with the vector*/
    const scalar q1 = q.x();
    const scalar q2 = q.y();
    const scalar q3 = q.z();
    Vec3 vn;
    vn.x() = 2 * ((q0 * q0 + q1 * q1 - 0.5) * v0.x() + (q2 * q1 + q3 * q0) * v0.y() + (q3 * q1 - q2 * q0) * v0.z());
    vn.y() = 2 * ((q1 * q2 - q3 * q0) * v0.x() + (q0 * q0 + q2 * q2 - 0.5) * v0.y() + (q3 * q2 + q1 * q0) * v0.z());
    vn.z() = 2 * ((q1 * q3 + q2 * q0) * v0.x() + (q2 * q3 - q1 * q0) * v0.y() + (q0 * q0 + q3 * q3 - 0.5) * v0.z());
    assert(is_close(this->norm_sqr(), 1));
    assert(is_close(vn.norm(), v0.norm()));
    return vn;
}

inline Quaternion Quaternion::operator+(Quaternion rhs) const {
    Quaternion res{*this};
    res.q0 += rhs.q0;
    res.q += rhs.q;
    return res;
}
inline Quaternion Quaternion::operator*(scalar rhs) const {
    Quaternion res{*this};
    res.q0 *= rhs;
    res.q *= rhs;
    return res;
}
} // namespace sito