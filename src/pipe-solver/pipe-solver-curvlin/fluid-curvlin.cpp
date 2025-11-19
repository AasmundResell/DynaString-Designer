#include "fluid-curvlin.hpp"

namespace curvlin {

template <FluidField field> inline void allocate_single_fluid_field(ArenaBump &arena_h, FluidData &fluid, uint count) {
    static_assert(field < FluidField::COUNT);
    using T = typename FluidFieldTraits<field>::variable_type;
    Offset<T> offset{};
    arena_h.allocate(&offset, count);
    fluid.offsets[(uint)field] = {.offset = offset.offset, .count = offset.count};
}

FluidData create_and_allocate_fluid(const uint Nf, ArenaBump &arena_h) {
    FluidData fluid;
    fluid.Nf = Nf;

    for (uint i = 0; i < (uint)FluidField::COUNT; i++) {
        if (FluidField::p_i == (FluidField)i) {
            allocate_single_fluid_field<FluidField::p_i>(arena_h, fluid, Nf);
            fluid_fill(Nf, 0, fluid.get_field<FluidField::p_i>(arena_h.buf), 0.0);
        } else if (FluidField::p_o == (FluidField)i) {
            allocate_single_fluid_field<FluidField::p_o>(arena_h, fluid, Nf);
            fluid_fill(Nf, 0, fluid.get_field<FluidField::p_o>(arena_h.buf), 0.0);
        } else if (FluidField::v_i == (FluidField)i) {
            allocate_single_fluid_field<FluidField::v_i>(arena_h, fluid, Nf);
            fluid_fill(Nf, 0, fluid.get_field<FluidField::v_i>(arena_h.buf), 0.0);
        } else if (FluidField::v_o == (FluidField)i) {
            allocate_single_fluid_field<FluidField::v_o>(arena_h, fluid, Nf);
            fluid_fill(Nf, 0, fluid.get_field<FluidField::v_o>(arena_h.buf), 0.0);
        } else if (FluidField::dp_i == (FluidField)i) {
            allocate_single_fluid_field<FluidField::dp_i>(arena_h, fluid, Nf);
            fluid_fill(Nf, 0, fluid.get_field<FluidField::dp_i>(arena_h.buf), 0.0);
        } else if (FluidField::dp_o == (FluidField)i) {
            allocate_single_fluid_field<FluidField::dp_o>(arena_h, fluid, Nf);
            fluid_fill(Nf, 0, fluid.get_field<FluidField::dp_o>(arena_h.buf), 0.0);
        } else {
            assert(false);
        }
    }

    return fluid;
}

void FluidData::get_field_begin_end(uint *begin, uint *end) const {
    assert(begin != nullptr && end != nullptr);
    *begin = offsets[0].offset;
    using T_last = typename FluidFieldTraits<(FluidField)((uint)FluidField::COUNT - 1)>::variable_type;
    *end = offsets[(uint)FluidField::COUNT - 1].offset + offsets[(uint)FluidField::COUNT - 1].count * sizeof(T_last);
}

} // namespace curvlin