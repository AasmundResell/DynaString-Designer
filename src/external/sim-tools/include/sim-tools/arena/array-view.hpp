#pragma once
#include "sim-tools/includes.hpp"

template <typename T> struct Offset { uint offset, count; };

using OffsetUntyped = Offset<void>;

/*Simple array view that checks ranges in debug mode. No destructor. Can be used inside cuda*/
template <typename T> struct ArrayView {
    T *data;
    uint count;

    DEVICE_FUNC ArrayView(byte *buf, Offset<T> offset) : data{(T *)(buf + offset.offset)}, count{offset.count} {
        assert(buf != nullptr);
        assert(offset.offset > 0);
        // assert(offset.count > 0);
    }
    DEVICE_FUNC ArrayView(const byte *buf, Offset<T> offset) : data{(T *)(buf + offset.offset)}, count{offset.count} {
        assert(buf != nullptr);
        assert(offset.offset > 0);
        // assert(offset.count > 0);
    }

    DEVICE_FUNC ArrayView(T *data_, uint count_) : data{data_}, count{count_} {
        assert(data != nullptr);
        // assert(count > 0);
    }
    // DEVICE_FUNC ArrayView(const T *data_, uint count_) : data{data_}, count{count_} {
    //     assert(data != nullptr);
    //     // assert(count > 0);
    // }
    constexpr DEVICE_FUNC T &operator[](uint i) {
        assert(data != nullptr);
        assert(i < count);
        return data[i];
    }
    constexpr DEVICE_FUNC const T &operator[](uint i) const {
        assert(data != nullptr && i < count);
        return data[i];
    }
};
template <typename T> inline void print_array_view(const ArrayView<T> &arr, const string &label = "") {
    print_raw_array(arr.data, arr.count, label);
}
