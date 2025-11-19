#pragma once
#include "includes.hpp"

namespace sito {
/*================================================================================
 A simple debug range checked static array that works with cuda
==================================================================================*/
template <typename T, uint count> struct Array {
    T data[count];
    constexpr DEVICE_FUNC T &operator[](uint i) {
        assert(i < count);
        return data[i];
    }
    constexpr DEVICE_FUNC const T &operator[](uint i) const {
        assert(i < count);
        return data[i];
    }
};

} // namespace sito