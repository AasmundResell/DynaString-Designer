#pragma once
#include "pipe.hpp"

DEVICE_FUNC inline scalar Pipe::calc_s(uint i, scalar ux_i, const byte *buf) const {
    assert(i < N);
    const ArrayView<scalar> S = get_field<PipeField::S>(buf);
    return -L_tot + S[i] + ux_i;
}

DEVICE_FUNC inline scalar Pipe::calc_sc(uint ic, scalar ux_i, const byte *buf) const {
    assert(ic < Nc);
    const ArrayView<scalar> Sc = get_field<PipeField::Sc>(buf);
    return -L_tot + Sc[ic] + ux_i;
}

DEVICE_FUNC inline scalar Pipe::calc_sbs(uint ibs, scalar ux_i, const byte *buf) const {
    assert(ibs < Nbs);
    const ArrayView<scalar> Sbs = get_field<PipeField::Sbs>(buf);
    return -L_tot + Sbs[ibs] + ux_i;
}

DEVICE_FUNC inline scalar Pipe::dS_e(uint ie, const byte *buf) const {
    assert(ie < get_Ne());
    const ArrayView<scalar> S = get_field<PipeField::S>(buf);
    return (S[ie + 1] - S[ie]);
}

DEVICE_FUNC inline scalar Pipe::ro_n_max(uint i, const byte *buf) const {
    const ArrayView<scalar> ro = get_field<PipeField::ro>(buf);
    if (i == 0) {
        return ro[0];
    } else if (i == N - 1) {
        return ro[N - 2];
    }
    return max(ro[i - 1], ro[i]);
}

DEVICE_FUNC inline scalar Pipe::ro_n_min(uint i, const byte *buf) const {
    const ArrayView<scalar> ro = get_field<PipeField::ro>(buf);
    if (i == 0) {
        return ro[0];
    } else if (i == N - 1) {
        return ro[N - 2];
    }
    return min(ro[i - 1], ro[i]);
}

DEVICE_FUNC inline scalar Pipe::Ap_e(uint ie, const byte *buf) const {
    assert(ie < get_Ne());
    const ArrayView<scalar> ro = get_field<PipeField::ro>(buf);
    const ArrayView<scalar> ri = get_field<PipeField::ri>(buf);
    return M_PI * (powi(ro[ie], 2) - powi(ri[ie], 2));
}

DEVICE_FUNC inline scalar Pipe::Af_e(uint ie, const byte *buf) const {
    assert(ie < get_Ne());
    const ArrayView<scalar> ri = get_field<PipeField::ri>(buf);
    return M_PI * powi(ri[ie], 2);
}

DEVICE_FUNC inline scalar Pipe::Ip_e(uint ie, const byte *buf) const {
    assert(ie < get_Ne());
    const ArrayView<scalar> ro = get_field<PipeField::ro>(buf);
    const ArrayView<scalar> ri = get_field<PipeField::ri>(buf);
    return M_PI / 4 * (powi(ro[ie], 4) - powi(ri[ie], 4));
}

DEVICE_FUNC inline scalar Pipe::If_e(uint ie, const byte *buf) const {
    assert(ie < get_Ne());
    const ArrayView<scalar> ri = get_field<PipeField::ri>(buf);
    return M_PI / 4 * powi(ri[ie], 4);
}

DEVICE_FUNC inline scalar Pipe::k_s(uint ie, const byte *buf) const {
    assert(ie < get_Ne());
    const ArrayView<scalar> ri = get_field<PipeField::ri>(buf);
    const ArrayView<scalar> ro = get_field<PipeField::ro>(buf);
    const scalar zeta = ri[ie] / ro[ie];
    return 6 * (1 + nu) * (1 + zeta) * (1 + zeta) /
           ((7 + 6 * nu) * (1 + zeta) * (1 + zeta) + (20 + 12 * nu) * zeta * zeta);
}

DEVICE_FUNC inline scalar Pipe::dS_node_avg(uint i, const byte *buf) const {
    assert(i < N);
    if (i == 0) {
        return dS_e(0, buf) / 2;
    } else if (i == N - 1) {
        return dS_e(N - 2, buf) / 2;
    } else {
        return (dS_e(i - 1, buf) + dS_e(i, buf)) / 2;
    }
}

inline uint Pipe::find_node_index_from_curve_length(const scalar S_i, const byte *buf, const uint i_prev) const {
    assert(S_i >= 0.0 && S_i <= L_tot); // S must be within the valid range [0, L_tot]
    /*Edge cases*/
    if (S_i == 0.0) {
        return 0;
    } else if (S_i == L_tot) {
        return N - 1;
    }
    int i;
    const ArrayView<scalar> S = get_field<PipeField::S>(buf);
    /*We should have maximally have moved 1 node away since the previous search, and take advantage of this fact*/
    assert(i_prev < N);
    /*Edge cases*/
    if (i_prev == 0) {
        assert(S_i < S[1]);
        return 0;
    } else if (i_prev == (int)N - 1) {
        assert(S_i > S[N - 2]);
        return N - 2;
    }
    /*Normal case*/
    if (S_i >= S[i_prev]) {
        assert(S_i < S[i_prev + 1]);
        i = i_prev;
    } else {
        assert(S_i > S[i_prev - 1]);
        i = i_prev - 1;
    }

    assert(i >= 0 && i < (int)N - 1);
    assert(S_i >= S[i]);
    assert(S_i - S[i] <= S[i + 1] - S[i]);
    return i;
}

inline uint Pipe::find_intial_top_node_from_curve_length(const scalar S_i, const byte *buf) const {
    assert(S_i >= 0.0 && S_i <= L_tot); // S must be within the valid range [0, L_tot]
    /*Edge cases*/
    if (S_i == 0.0) {
        return 0;
    } else if (abs(L_tot - S_i) < SMALL_SCALAR) {
        return N - 1;
    }
    int i;
    const ArrayView<scalar> S = get_field<PipeField::S>(buf);
    for (i = 0; i < (int)N; i++) {
        if (S[i] > S_i) {
            i--;
            break;
        }
    }
    assert(i >= 0 && i < (int)N - 1);
    assert(S_i >= S[i]);
    assert(S_i - S[i] <= S[i + 1] - S[i]);
    return (uint)i;
}
