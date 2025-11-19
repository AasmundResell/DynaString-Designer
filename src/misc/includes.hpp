#pragma once
#include "sim-tools/arena/arena.hpp"
#include "sim-tools/array.hpp"
#include "sim-tools/includes.hpp"
#include "sim-tools/quaternion.hpp"

using sito::ArenaBump;
using sito::Array;
using sito::MemType;
using sito::Quaternion;

#ifdef SRC_DIR
constexpr char src_dir[] = SRC_DIR;
constexpr char gui_dir[] = SRC_DIR "/gui";
#else
constexpr char src_dir[] = "";
constexpr char gui_dir[] = "";
static_assert(false, "SRC_DIR must be defined when compiling");
#endif

#ifdef _OPENMP
#include <omp.h>
#else
#define omp_get_thread_num() 0
#define omp_get_num_threads() 1
#endif

/*Modulo for floating point that always returns positive output*/
inline DEVICE_FUNC scalar fmod_pos(const scalar a, const scalar b) {
    const scalar ans = fmod(fmod(a, b) + b, b);
    assert(ans >= 0.0 && ans < b);
    return ans;
}

inline DEVICE_FUNC uint heaviside(scalar x) {
    return x >= 0 ? 1 : 0;
}

inline DEVICE_FUNC uint h(scalar x, scalar lambda) {
    return (1 - 1 / (1 + (x / lambda) * (x / lambda))) * heaviside(x);
}

inline DEVICE_FUNC scalar ramp(scalar x) {
    return x * heaviside(x);
}

inline DEVICE_FUNC scalar ramp_reg(scalar x, scalar e, scalar tol) {
    // Regularized ramp function with shift (using softplus and shift so ramp_reg_shifted(0) = tol)
    // Softplus: e * log(1 + exp(x / e))
    // Compute the shift so that ramp_reg_shifted(0) = tol
    assert(tol >= 0.0 && tol < 1.0);
    assert(e >= 0.0);
    if (e < 1e-3)
        return x >= 0 ? x : 0;
    scalar x_shift = -e * log(exp(tol / e) - 1.0);
    scalar softplus = e * log(1.0 + exp((x - x_shift) / e));
    return softplus >= 0.0 ? softplus : 0.0;
}

inline DEVICE_FUNC scalar heaviside_reg(scalar x, scalar e, scalar tol) {
    if (e <= SMALL_SCALAR)
        return x >= 0 ? 1.0 : 0.0;

    // Compute the shift so that heaviside_reg_shifted(0) = tol
    scalar x_shift = e * log(1.0 / (tol + 1e-12) - 1.0);
    // Avoid overflow in exp by clamping the input
    scalar exp_input = -(x - x_shift) / e; // Division OK due to earlier check

    // Clamp exp_input to [-700, 700] to avoid overflow
    if (exp_input < -700.0)
        exp_input = -700.0;
    if (exp_input > 700.0)
        exp_input = 700.0;
    return 1.0 / (1.0 + exp(exp_input));
}

// Regularized sign function: transitions smoothly for -10 <= x <= 10
inline DEVICE_FUNC scalar sign_reg(scalar x, scalar e) {
    assert(e >= 0.0);
    if (e <= SMALL_SCALAR)
        return (x > 0) - (x < 0);
    if (x > 10.0)
        return 1.0;
    if (x < -10.0)
        return -1.0;
    return x / sqrt(x * x + e * e);
}

constexpr scalar L_MAX_LENGTH = 1e6; // [m], arbitrary large value
