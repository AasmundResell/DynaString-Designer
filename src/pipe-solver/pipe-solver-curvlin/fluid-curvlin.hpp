#pragma once
#include "beam-curvlin.hpp"

namespace curvlin {

enum class FluidField {
    p_i,  /* (Ne + 2) Inner fluid pressure, elementwise + two boundary nodes*/
    p_o,  /* (Ne + 2) Outer fluid pressure, elementwise + two boundary nodes*/
    v_i,  /* (Ne + 2) Inner fluid velocity, elementwise + two boundary nodes*/
    v_o,  /* (Ne + 2) Outer fluid velocity, elementwise + two boundary nodes*/
    dp_i, /* (Ne + 2) Inner pressure gradient, elementwise + two boundary nodes*/
    dp_o, /* (Ne + 2) Outer pressure gradient, elementwise + two boundary nodes*/
    COUNT
};

// clang-format off
template <FluidField array> struct FluidFieldTraits;
/*The variable type of each field*/
template <> struct FluidFieldTraits<FluidField::p_i> { using variable_type = scalar; };
template <> struct FluidFieldTraits<FluidField::p_o> { using variable_type = scalar; };
template <> struct FluidFieldTraits<FluidField::v_i> { using variable_type = scalar; };
template <> struct FluidFieldTraits<FluidField::v_o> { using variable_type = scalar; };
template <> struct FluidFieldTraits<FluidField::dp_i> { using variable_type = scalar; };
template <> struct FluidFieldTraits<FluidField::dp_o> { using variable_type = scalar; };
// clang-format on

struct FluidData {
    uint Nf;
    OffsetUntyped offsets[(uint)FluidField::COUNT];
    void get_field_begin_end(uint *begin, uint *end) const;

    template <FluidField field>
    DEVICE_FUNC ArrayView<typename FluidFieldTraits<field>::variable_type> get_field(byte *buf) const {
        static_assert((uint)field < (uint)FluidField::COUNT);
        using T = typename FluidFieldTraits<field>::variable_type;
        OffsetUntyped uoffset = offsets[(uint)field];
        Offset<T> offset = {.offset = uoffset.offset, .count = uoffset.count};
        assert(offset.offset > 0 && offset.count > 0);
        return {buf, offset};
    }

    template <FluidField field>
    DEVICE_FUNC const ArrayView<typename FluidFieldTraits<field>::variable_type> get_field(const byte *buf) const {
        static_assert((uint)field < (uint)FluidField::COUNT);
        using T = typename FluidFieldTraits<field>::variable_type;
        OffsetUntyped uoffset = offsets[(uint)field];
        Offset<T> offset = {.offset = uoffset.offset, .count = uoffset.count};
        assert(offset.offset > 0 && offset.count > 0);
        return {buf, offset};
    }
};

template <typename T>
DEVICE_FUNC inline void fluid_fill(const uint N, const uint top_node, ArrayView<T> v, const T fill_val) {
    for (uint i = top_node; i < N; i++) {
        v[i] = fill_val;
    }
}

FluidData create_and_allocate_fluid(const uint Nf, ArenaBump &arena_h);

} // namespace curvlin