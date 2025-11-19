#include "beam-curvlin.hpp"

namespace curvlin {

template <BeamField field> inline void allocate_single_beam_field(ArenaBump &arena_h, BeamData &beam, uint count) {
    static_assert(field < BeamField::COUNT);
    using T = typename BeamFieldTraits<field>::variable_type;
    Offset<T> offset{};
    arena_h.allocate(&offset, count);
    beam.offsets[(uint)field] = {.offset = offset.offset, .count = offset.count};
}

BeamData create_and_allocate_beam(uint N, ArenaBump &arena_h) {
    BeamData beam;

    // Allocate vector fields (Vec3)
    allocate_single_beam_field<BeamField::u>(arena_h, beam, N);
    allocate_single_beam_field<BeamField::v>(arena_h, beam, N);
    allocate_single_beam_field<BeamField::a>(arena_h, beam, N);
    allocate_single_beam_field<BeamField::f_int>(arena_h, beam, N);
    allocate_single_beam_field<BeamField::f_dyn>(arena_h, beam, N);
    allocate_single_beam_field<BeamField::f_hyd>(arena_h, beam, N);
    allocate_single_beam_field<BeamField::alpha>(arena_h, beam, N);
    allocate_single_beam_field<BeamField::theta>(arena_h, beam, N);
    allocate_single_beam_field<BeamField::omega>(arena_h, beam, N);
    allocate_single_beam_field<BeamField::m_int>(arena_h, beam, N);
    allocate_single_beam_field<BeamField::m_dyn>(arena_h, beam, N);
    allocate_single_beam_field<BeamField::m_hyd>(arena_h, beam, N);
    allocate_single_beam_field<BeamField::f_axial>(arena_h, beam, N);
    allocate_single_beam_field<BeamField::p_ref>(arena_h, beam, N);

    // Allocate scalar fields (mass, inertia, etc.)
    allocate_single_beam_field<BeamField::m>(arena_h, beam, N);
    allocate_single_beam_field<BeamField::mfy>(arena_h, beam, N);
    allocate_single_beam_field<BeamField::mfz>(arena_h, beam, N);
    allocate_single_beam_field<BeamField::Jx>(arena_h, beam, N);
    allocate_single_beam_field<BeamField::Jr>(arena_h, beam, N);

    // Allocate index mapping
    allocate_single_beam_field<BeamField::i_pipe_to_ie_hole>(arena_h, beam, N);

    // Initialize fields to zero
    ArrayView<Vec3> a = beam.get_field<BeamField::a>(arena_h.buf);
    ArrayView<Vec3> v = beam.get_field<BeamField::v>(arena_h.buf);
    ArrayView<Vec3> u = beam.get_field<BeamField::u>(arena_h.buf);
    ArrayView<Vec3> f_int = beam.get_field<BeamField::f_int>(arena_h.buf);
    ArrayView<Vec3> f_dyn = beam.get_field<BeamField::f_dyn>(arena_h.buf);
    ArrayView<Vec3> f_hyd = beam.get_field<BeamField::f_hyd>(arena_h.buf);
    ArrayView<Vec3> alpha = beam.get_field<BeamField::alpha>(arena_h.buf);
    ArrayView<Vec3> theta = beam.get_field<BeamField::theta>(arena_h.buf);
    ArrayView<Vec3> omega = beam.get_field<BeamField::omega>(arena_h.buf);
    ArrayView<Vec3> m_int = beam.get_field<BeamField::m_int>(arena_h.buf);
    ArrayView<Vec3> m_dyn = beam.get_field<BeamField::m_dyn>(arena_h.buf);
    ArrayView<Vec3> m_hyd = beam.get_field<BeamField::m_hyd>(arena_h.buf);
    ArrayView<Vec3> p_ref = beam.get_field<BeamField::p_ref>(arena_h.buf);
    ArrayView<scalar> f_axial = beam.get_field<BeamField::f_axial>(arena_h.buf);

    ArrayView<scalar> m = beam.get_field<BeamField::m>(arena_h.buf);
    ArrayView<scalar> mfy = beam.get_field<BeamField::mfy>(arena_h.buf);
    ArrayView<scalar> mfz = beam.get_field<BeamField::mfz>(arena_h.buf);
    ArrayView<scalar> Jx = beam.get_field<BeamField::Jx>(arena_h.buf);
    ArrayView<scalar> Jr = beam.get_field<BeamField::Jr>(arena_h.buf);
    ArrayView<uint> i_pipe_to_ie_hole = beam.get_field<BeamField::i_pipe_to_ie_hole>(arena_h.buf);

    for (uint i = 0; i < N; i++) {
        u[i].setZero();
        v[i].setZero();
        a[i].setZero();
        f_int[i].setZero();
        f_dyn[i].setZero();
        f_hyd[i].setZero();
        theta[i].setZero();
        omega[i].setZero();
        alpha[i].setZero();
        m_int[i].setZero();
        m_dyn[i].setZero();
        m_hyd[i].setZero();
        p_ref[i].setZero();
        // p_ref[i] = Vec3(NAN, NAN, NAN);

        f_axial[i] = 0.0;
        m[i] = 0.0;
        mfy[i] = 0.0;
        mfz[i] = 0.0;
        Jx[i] = 0.0;
        Jr[i] = 0.0;
        i_pipe_to_ie_hole[i] = 0;
    }

    return beam;
}
/*
template <BeamField field, typename T>
void print_beam_mass(const uint N, const uint top_node, const ArrayView<T> arr, byte *buf) {
    assert(field == BeamField::m || field == BeamField::mfy || field == BeamField::mfz || field == BeamField::Jx ||
    field == BeamField::Jr);

    string arr_str, DOF_str;
    scalar val;

    switch (field) {
        case BeamField::m:
        arr_str = "m";
        break;
        case BeamField::mfy:
        arr_str = "mfy";
        break;
        case BeamField::mfz:
        arr_str = "mfz";
        break;
        case BeamField::Jx:
        arr_str = "Jx";
        break;
        case BeamField::Jr:
        arr_str = "Jr";
        break;
        default:
        assert(false);
    }

    cout << arr_str << " = [";

    for (uint i = top_node; i < N; i++) {

    switch (field) {
        case BeamField::m:
        val = arr[i];
        break;
        case BeamField::mfy:
        val = arr[i];
        break;
        case BeamField::mfz:
        val = arr[i];
        break;
        case BeamField::Jx:
        val = arr[i];
        break;
        case BeamField::Jr:
        val = arr[i];
        break;
        default:
        assert(false);
    }
    cout << val;

        if (i < N - 1) {
            cout << ", ";
        }
    }

    cout << "]" << endl;
}


template <BeamField field, typename T>
void print_beam(const uint N, const uint top_node, const ArrayView<T> arr, BeamFieldArrayDOF field_DOF, byte *buf) {

string arr_str, DOF_str;
scalar val;
switch (field) {
    case BeamField::v:
    arr_str = "v";
        break;
        case BeamField::u:
        arr_str = "u";
        break;
        case BeamField::m:
        assert(field_DOF == BeamFieldArrayDOF::trans_x || field_DOF == BeamFieldArrayDOF::trans_y ||
        field_DOF == BeamFieldArrayDOF::trans_z);
        arr_str = "m";
        break;
        case BeamField::mfy:
        assert(field_DOF == BeamFieldArrayDOF::trans_y);
        arr_str = "mfy";
        break;
        case BeamField::mfz:
        assert(field_DOF == BeamFieldArrayDOF::trans_z);
        arr_str = "mfz";
        break;
        case BeamField::Jx:
        assert(field_DOF == BeamFieldArrayDOF::rot_x);
        arr_str = "Jx";
        break;
        case BeamField::Jr:
        assert(field_DOF == BeamFieldArrayDOF::rot_y || field_DOF == BeamFieldArrayDOF::rot_z);
        arr_str = "Jr";
        break;
        case BeamField::f_int:
        arr_str = "f_internal";
        break;
        case BeamField::f_dyn:
        arr_str = "f_external";
        break;
        case BeamField::f_hyd:
        arr_str = "f_static";
        break;
        default:
        assert(false);
    }

    switch (field_DOF) {
        case BeamFieldArrayDOF::trans_x:
        DOF_str = "ux";
        break;
        case BeamFieldArrayDOF::trans_y:
        DOF_str = "uy";
        break;
        case BeamFieldArrayDOF::trans_z:
        DOF_str = "uz";
        break;
        case BeamFieldArrayDOF::rot_x:
        DOF_str = "rx";
        break;
        case BeamFieldArrayDOF::rot_y:
        DOF_str = "ry";
        break;
        case BeamFieldArrayDOF::rot_z:
        DOF_str = "rz";
        break;
        default:
        assert(false);
    }

    cout << arr_str << "_" << DOF_str << " = [";

    if (field == BeamField::m || field == BeamField::mfy || field == BeamField::mfz || field == BeamField::Jx ||
    field == BeamField::Jr) {
        for (uint i = top_node; i < N; i++) {

        switch (field_DOF) {
            case BeamFieldArrayDOF::trans_x:
            val = arr[i];
            break;
            case BeamFieldArrayDOF::trans_y:
            val = arr[i];
            break;
            case BeamFieldArrayDOF::trans_z:
            val = arr[i];
            break;
            case BeamFieldArrayDOF::rot_x:
            val = arr[i];
            break;
            case BeamFieldArrayDOF::rot_y:
            val = arr[i];
            break;
            case BeamFieldArrayDOF::rot_z:
            val = arr[i];
            break;
            default:
            assert(false);
        }
        cout << val;

        if (i < N - 1) {
            cout << ", ";
        }
    }

    cout << "]" << endl;

} else {
    scalar offset = 0.0;

    for (uint i = top_node; i < N; i++) {

    switch (field_DOF) {
        case BeamFieldArrayDOF::trans_x:
        val = arr[i].x();
        break;
        case BeamFieldArrayDOF::trans_y:
        val = arr[i].y();
        break;
        case BeamFieldArrayDOF::trans_z:
        val = arr[i].z();
        break;
        case BeamFieldArrayDOF::rot_x:
        val = arr[IX_rx(i)];
        break;
        case BeamFieldArrayDOF::rot_y:
        val = arr[IX_ry(i)];
        break;
        case BeamFieldArrayDOF::rot_z:
        val = arr[IX_rz(i)];
        break;
        default:
        assert(false);
    }
    cout << val - offset;

    if (i < N - 1) {
        cout << ", ";
    }
}

cout << "]" << endl;
}
}
*/

} // namespace curvlin