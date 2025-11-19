#pragma once
#include "../vendor/eigen/Dense"
#include "../vendor/eigen/Sparse"
#include <bitset>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

using std::array;
using std::bitset;
using std::cerr;
using std::cout;
using std::endl;
using std::exception;
using std::ifstream;
using std::make_unique;
using std::map;
using std::max;
using std::min;
using std::move;
using std::ofstream;
using std::ostream;
using std::pair;
using std::runtime_error;
using std::set;
using std::stringstream;
using std::unordered_map;
using std::filesystem::path;

using std::isfinite;
using std::isnan;
using std::string;
using std::to_string;
using std::unique_ptr;
using std::vector;

#ifdef SINGLE_PRECISION
using scalar = float;
constexpr scalar SMALL_SCALAR = 1e-4;
constexpr scalar LARGEST_SCALAR = 3.402823e+38; // since std::numeric_limits<scalar>::max(); can't be used with cuda
#else
using scalar = double;
constexpr scalar SMALL_SCALAR = 1e-8;
constexpr scalar LARGEST_SCALAR =
    1.7976931348623157e+308; // since std::numeric_limits<scalar>::max(); can't be used with cuda.
#endif
using uint = uint32_t;
using u8int = uint8_t;
using byte = unsigned char;

using Vec2 = Eigen::Vector<scalar, 2>;
using Vec3 = Eigen::Vector<scalar, 3>;
using Vec4 = Eigen::Vector<scalar, 4>;
using Vec5 = Eigen::Vector<scalar, 5>;
using Vec10 = Eigen::Vector<scalar, 10>;
using Vec12 = Eigen::Vector<scalar, 12>;

using Mat2 = Eigen::Matrix<scalar, 2, 2>;
using Mat3 = Eigen::Matrix<scalar, 3, 3>;
using Mat4 = Eigen::Matrix<scalar, 4, 4>;

using Vec5Map = Eigen::Map<Vec5>;
using MatSparse = Eigen::SparseMatrix<scalar>;
using VecX = Eigen::VectorX<scalar>;

#ifdef SIM_TOOLS_HOME_DIR
constexpr char sim_tools_home_dir[] = SIM_TOOLS_HOME_DIR;
#else
constexpr char sim_tools_home_dir[] = "";
#error "SIM_TOOLS_HOME_DIR undefined"
#endif

#ifndef _OPENMP
#define omp_get_thread_num() 0
#define omp_get_num_threads() 1
#define omp_set_num_threads(num_threads)
#endif

#ifdef CUDA_BUILD
constexpr bool CUDA_ENABLED = true;
#else
constexpr bool CUDA_ENABLED = false;
#endif

#ifdef __CUDACC__
#include <cuda_runtime.h>
#define DEVICE_FUNC __host__ __device__
#define FORCE_INLINE __forceinline__

#define CUDA_CHECK_LAST_ERROR()                                                                                        \
    do {                                                                                                               \
        cudaError_t err = cudaGetLastError();                                                                          \
        if (err != cudaSuccess) {                                                                                      \
            fprintf(stderr, "Previous CUDA error detected: %s at %s:%d\n", cudaGetErrorString(err), __FILE__,          \
                    __LINE__);                                                                                         \
            fprintf(stderr, "*** FAILED - ABORTING\n");                                                                \
            assert(false);                                                                                             \
            exit(1);                                                                                                   \
        }                                                                                                              \
    } while (0)

#define CUDA_CHECK_ERROR(err)                                                                                          \
    do {                                                                                                               \
        if (err != cudaSuccess) {                                                                                      \
            fprintf(stderr, "Fatal error: %s at %s:%d\n", cudaGetErrorString(err), __FILE__, __LINE__);                \
            fprintf(stderr, "*** FAILED - ABORTING\n");                                                                \
            assert(false);                                                                                             \
            exit(1);                                                                                                   \
        }                                                                                                              \
    } while (0)

#else
#define __global__
#define __constant__
#define __host__
#define __device__
#define DEVICE_FUNC
#define FORCE_INLINE inline __attribute__((always_inline))
#define CUDA_CHECK_ERROR(err)
#define CUDA_CHECK_LAST_ERROR()
using cudaMemcpyKind = int;
#define cudaMemcpyHostToDevice -1
#define cudaMemcpyDeviceToHost -1
#endif

inline void print_dbg_only(const string &msg) {
#ifndef NDEBUG
    printf("%s\n", msg.c_str());
#endif
}

#define THROW_RUNTIME_ERROR(msg)                                                                                       \
    do {                                                                                                               \
        print_dbg_only("Error: " + string(msg));                                                                       \
        assert(false); /* Just so that it breaks in debug mode */                                                      \
        throw runtime_error((msg));                                                                                    \
    } while (0)

template <typename T> DEVICE_FUNC int sign(T val) {
    return (val > 0) - (val < 0);
}

inline DEVICE_FUNC float st_abs(float val) {
    return fabsf(val);
}
inline DEVICE_FUNC double st_abs(double val) {
    return fabs(val);
}

inline DEVICE_FUNC bool is_close(scalar a, scalar b, scalar tolerance = SMALL_SCALAR) {
    return st_abs(a - b) < tolerance;
}

inline DEVICE_FUNC bool is_close_componentwise(const Vec3 &a, const Vec3 &b, scalar tol = SMALL_SCALAR) {
    return (a.array() - b.array()).abs().maxCoeff() < tol;
}
inline DEVICE_FUNC bool is_close_componentwise(const Mat3 &a, const Mat3 &b, scalar tol = SMALL_SCALAR) {
    return (a.array() - b.array()).abs().maxCoeff() < tol;
}

inline DEVICE_FUNC bool is_orthonormal(const Mat3 &R, const scalar tol = SMALL_SCALAR) {

    return is_close(R.determinant(), 1.0, tol) && is_close((R.transpose() - R.inverse()).norm(), 0.0, tol);
}

inline DEVICE_FUNC bool is_orthogonal(const Vec3 &a, const Vec3 &b, double tol = SMALL_SCALAR) {
    return abs(a.dot(b)) < tol;
}

inline DEVICE_FUNC Mat3 skew(const Vec3 &a) {
    return Mat3{{0, -a.z(), a.y()}, {a.z(), 0, -a.x()}, {-a.y(), a.x(), 0}};
}

/*Tested that when this is called with exp known at compile time, the for loop gets totally optimized away, much
 * faster than pow for integer exponentiation*/
constexpr DEVICE_FUNC scalar powi(scalar base, uint exp) {
    assert(exp < 10); // likely an error otherwise, but increase limit if needed
    scalar res = (scalar)1;
    for (uint i = 0; i < exp; i++) {
        res *= base;
    }
    return res;
}

/*AllFinite() function is eigen is __host__ only*/
template <typename EigenMatrix> inline DEVICE_FUNC bool eigen_all_finite(const EigenMatrix &m) {
    for (uint i = 0; i < EigenMatrix::RowsAtCompileTime; i++) {
        for (uint j = 0; j < EigenMatrix::ColsAtCompileTime; j++) {
            if (!isfinite(m(i, j)))
                return false;
        }
    }
    return true;
}

template <typename EigenMatrix> inline DEVICE_FUNC bool is_symmetric(const EigenMatrix &m) {
    for (uint i = 0; i < EigenMatrix::RowsAtCompileTime; i++) {
        for (uint j = 0; j < EigenMatrix::ColsAtCompileTime; j++) {
            if (!is_close(m(i, j), m(j, i)))
                return false;
        }
    }
    return true;
}

template <typename T> inline void print_raw_array(const T *v, uint N, const string &label = "") {
    cout << "\n";
    if (!label.empty())
        cout << label << ":\n";
    if (N > 0)
        for (uint i = 0; i < N; i++) {
            cout << "i=" << i << ", [" << v[i] << "]\n";
        }
    else {
        cout << "Empty array\n";
    }
}
inline void print_raw_array(const Vec3 *v, uint N, const string &label = "") {
    cout << "\n";
    if (!label.empty())
        cout << label << ":\n";
    if (N > 0)
        for (uint i = 0; i < N; i++) {
            cout << "i=" << i << ", [" << v[i].transpose() << "]\n";
        }
    else {
        cout << "Empty array\n";
    }
}
template <typename T> inline void print_std_vector(const vector<T> &v, const string &label = "") {
    print_raw_array(v.data(), v.size(), label);
}
