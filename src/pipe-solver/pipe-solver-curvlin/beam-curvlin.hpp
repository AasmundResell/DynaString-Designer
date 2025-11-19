#pragma once
#include "hole/hole.hpp"
#include "pipe/pipe.hpp"
#include "sim-tools/arena/arena.hpp"

namespace curvlin {

enum class BeamField {
    u,                 /* (N * Vec3)Displacements*/
    v,                 /* (N * Vec3)Velocities*/
    a,                 /* (N * Vec3)Accelerations*/
    theta,             /* (N * Vec3)Rotations*/
    omega,             /* (N * Vec3)Angular velocities*/
    alpha,             /* (N * Vec3)Angular accelerations*/
    f_int,             /* (N * Vec3)Internal force vector (inner strains)*/
    f_dyn,             /* (N * Vec3)External dynamic force vector (contact and bit-rock)*/
    f_hyd,             /* (N * Vec3)Hydrodynamic, buoyancy, and gravity force vector*/
    m_int,             /* (N * Vec3)Internal moment vector (inner strains)*/
    m_dyn,             /* (N * Vec3)External moment force vector (contact and bit-rock)*/
    m_hyd,             /* (N * Vec3)Hydrodynamic, buoyancy, and gravity moment vector*/
    p_ref,             /* (N * Vec3)Reference position for stick-slip contact model*/
    f_axial,           /* (N) Axial tension*/
    m,                 /* (N) Pipe mass vector*/
    mfy,               /* (N)Added fluid mass y-direction*/
    mfz,               /* (N)Added fluid mass z-direction*/
    Jx,                /* (N)Axial moment of inertia */
    Jr,                /* (N)Radial moment of inertia*/
    i_pipe_to_ie_hole, /* (N)Pipe to hole index*/
    // w_plus,            // (N) axial force at left end of section
    // w_minus,           // (N) axial force at right end of section
    // tau_plus,          // (N) torque at left end of section
    // tau_minus,         // (N) torque at right end of section
    // alpha_a,           /* (Nr)Axial Riemann invariant (positive direction)*/
    // alpha_t,           /* (Nr)Torsional Riemann invariant (positive direction)*/
    // beta_a,            /*  (Nr)Axial Riemann invariant (negative direction)*/
    // beta_t,            /*  (Nr)Torsional Riemann invariant (negative direction)*/
    COUNT
};

// clang-format off
template <BeamField array> struct BeamFieldTraits;
/*The variable type of each field*/
template <> struct BeamFieldTraits<BeamField::a> { using variable_type = Vec3; };
template <> struct BeamFieldTraits<BeamField::v> { using variable_type = Vec3; };
template <> struct BeamFieldTraits<BeamField::u> { using variable_type = Vec3; };
template <> struct BeamFieldTraits<BeamField::alpha> { using variable_type = Vec3; };
template <> struct BeamFieldTraits<BeamField::omega> { using variable_type = Vec3; };
template <> struct BeamFieldTraits<BeamField::theta> { using variable_type = Vec3; };
template <> struct BeamFieldTraits<BeamField::m> { using variable_type = scalar; };
template <> struct BeamFieldTraits<BeamField::mfy> { using variable_type = scalar; };
template <> struct BeamFieldTraits<BeamField::mfz> { using variable_type = scalar; };
template <> struct BeamFieldTraits<BeamField::Jx> { using variable_type = scalar; };
template <> struct BeamFieldTraits<BeamField::Jr> { using variable_type = scalar; };
template <> struct BeamFieldTraits<BeamField::f_int> { using variable_type = Vec3; };
template <> struct BeamFieldTraits<BeamField::f_dyn> { using variable_type = Vec3; };
template <> struct BeamFieldTraits<BeamField::f_hyd> { using variable_type = Vec3; };
template <> struct BeamFieldTraits<BeamField::m_int> { using variable_type = Vec3; };
template <> struct BeamFieldTraits<BeamField::m_dyn> { using variable_type = Vec3; };
template <> struct BeamFieldTraits<BeamField::m_hyd> { using variable_type = Vec3; };
template <> struct BeamFieldTraits<BeamField::i_pipe_to_ie_hole> { using variable_type = uint; };
template <> struct BeamFieldTraits<BeamField::f_axial> { using variable_type = scalar; };
template <> struct BeamFieldTraits<BeamField::p_ref> { using variable_type = Vec3; };

//template <> struct BeamFieldTraits<BeamField::w_plus>   { using variable_type = scalar; };
//template <> struct BeamFieldTraits<BeamField::w_minus>  { using variable_type = scalar; };
//template <> struct BeamFieldTraits<BeamField::tau_plus> { using variable_type = scalar; };
//template <> struct BeamFieldTraits<BeamField::tau_minus>{ using variable_type = scalar; };
//template <> struct BeamFieldTraits<BeamField::alpha_a> { using variable_type = scalar; };
//template <> struct BeamFieldTraits<BeamField::alpha_t> { using variable_type = scalar; };
//template <> struct BeamFieldTraits<BeamField::beta_a> { using variable_type = scalar; };
//template <> struct BeamFieldTraits<BeamField::beta_t> { using variable_type = scalar; };
//  clang-format on

struct BeamData {
    OffsetUntyped offsets[(uint)BeamField::COUNT];
    template <BeamField field>
    DEVICE_FUNC ArrayView<typename BeamFieldTraits<field>::variable_type> get_field(byte *buf) const {
        static_assert((uint)field < (uint)BeamField::COUNT);
        using T = typename BeamFieldTraits<field>::variable_type;
        OffsetUntyped uoffset = offsets[(uint)field];
        Offset<T> offset = {.offset = uoffset.offset, .count = uoffset.count};
        assert(offset.offset > 0 && offset.count > 0);
        return {buf, offset};
    }

    template <BeamField field>
    DEVICE_FUNC const ArrayView<typename BeamFieldTraits<field>::variable_type> get_field(const byte *buf) const {
        static_assert((uint)field < (uint)BeamField::COUNT);
        using T = typename BeamFieldTraits<field>::variable_type;
        OffsetUntyped uoffset = offsets[(uint)field];
        Offset<T> offset = {.offset = uoffset.offset, .count = uoffset.count};
        assert(offset.offset > 0 && offset.count > 0);
        return {buf, offset};
    }
};

BeamData create_and_allocate_beam(uint N, ArenaBump &arena_h);

enum class BeamFieldArrayDOF {
    x,
    y,
    z,
};

// template <BeamField field, typename T>
// void print_beam(const uint N, const uint top_node, const ArrayView<T> arr, BeamFieldArrayDOF field_DOF, byte *buf);

/*
template <BeamField field, typename T>
void print_beam_mass(const uint N, const uint top_node, const ArrayView<T> arr, byte *buf);
*/

DEVICE_FUNC inline void beam_fill(const uint N, const uint start_node, const uint end_node, ArrayView<Vec3> arr,
                                  BeamFieldArrayDOF field_DOF, const scalar fill_val) {
    for (uint i = start_node; i < end_node; i++) {
        switch (field_DOF) {
        case BeamFieldArrayDOF::x:
            arr[i].x() = fill_val;
            break;
        case BeamFieldArrayDOF::y:
            arr[i].y() = fill_val;
            break;
        case BeamFieldArrayDOF::z:
            arr[i].z() = fill_val;
            break;
        default:
            assert(false);
        }
    }
}

} // namespace curvlin