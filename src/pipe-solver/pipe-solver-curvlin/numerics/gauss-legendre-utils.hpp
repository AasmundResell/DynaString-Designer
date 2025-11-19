#pragma once

namespace curvlin {

struct Terms {
    scalar u, v, w, tx, ty, tz, u_s, v_s, w_s, tx_s, ty_s, tz_s, u_t, v_t, w_t, tx_t, ty_t, tz_t, u_tt, v_tt, w_tt,
        tx_tt, ty_tt, tz_tt, u_ts, v_ts, w_ts, tx_ts, ty_ts, tz_ts;
};

// New compact Variations: Vec2 for axial/torsion (node 1, node 2), Vec4 for lateral/bending (hermite DOFs)
struct Variations {
    Vec2 du;
    Vec2 dtx;
    Vec4 dv;
    Vec4 dw;
    Vec4 dtz;
    Vec4 dty;
    Vec2 du_s;
    Vec2 dtx_s;
    Vec4 dv_s;
    Vec4 dw_s;
    Vec4 dtz_s;
    Vec4 dty_s;
};

// Shape functions for Timoshenko beam element
DEVICE_FUNC inline Vec10 shape_functions(scalar xi, scalar alpha, scalar L) {

    const scalar beta = 1.0 / (1.0 + alpha);

    // Linear shape functions
    const scalar n_1 = 1 - xi;
    const scalar n_2 = xi;

    // Hermitian shape functions, lateral displacements
    const scalar h1_1 = beta * (1 - 3 * powi(xi, 2) + 2 * powi(xi, 3) + (1 / beta - 1) * (1 - xi));
    const scalar h1_2 = L * beta * (xi - 2 * powi(xi, 2) + powi(xi, 3) + (1 / beta - 1) / 2 * (xi - powi(xi, 2)));
    const scalar h1_3 = beta * (3 * powi(xi, 2) - 2 * powi(xi, 3) + (1 / beta - 1) * xi);
    const scalar h1_4 = L * beta * (-powi(xi, 2) + powi(xi, 3) + (1 / beta - 1) / 2 * (-xi + powi(xi, 2)));

    // Hermitian shape functions, lateral rotations
    const scalar h2_1 = 6 * beta / L * (-xi + powi(xi, 2));
    const scalar h2_2 = beta * (1 - 4 * xi + 3 * powi(xi, 2) + (1 / beta - 1) * (1 - xi));
    const scalar h2_3 = -6 * beta / L * (-xi + powi(xi, 2));
    const scalar h2_4 = beta * (-2 * xi + 3 * powi(xi, 2) + (1 / beta - 1) * xi);

    return Vec10(n_1, n_2, h1_1, h1_2, h1_3, h1_4, h2_1, h2_2, h2_3, h2_4);
}

// Shape function derivatives
DEVICE_FUNC inline Vec10 shape_function_derivatives(scalar xi, scalar alpha, scalar dS) {
    const scalar beta = 1.0 / (1.0 + alpha);

    // Linear shape functions derivatives
    const scalar dn_1 = -1 / dS;
    const scalar dn_2 = 1 / dS;

    // Shape functions, lateral displacements derivatives
    const scalar dh1_1 = beta * (6 * powi(xi, 2) - 6 * xi + 1 - 1 / beta) / dS;
    const scalar dh1_2 = beta * (3 * powi(xi, 2) - 4 * xi + (-1 / 2.0 + 1 / (2 * beta)) * (1 - 2 * xi) + 1);
    const scalar dh1_3 = beta * (-6 * powi(xi, 2) + 6 * xi - 1 + 1 / beta) / dS;
    const scalar dh1_4 = beta * (3 * powi(xi, 2) - 2 * xi + (-1 / 2.0 + 1 / (2 * beta)) * (2 * xi - 1));

    // Shape functions, lateral rotations derivatives
    const scalar dh2_1 = 6 * beta * (2 * xi - 1) / (dS * dS);
    const scalar dh2_2 = beta * (6 * xi - 3 - 1 / beta) / dS;
    const scalar dh2_3 = -6 * beta * (2 * xi - 1) / (dS * dS);
    const scalar dh2_4 = beta * (6 * xi - 3 + 1 / beta) / dS;

    return Vec10(dn_1, dn_2, dh1_1, dh1_2, dh1_3, dh1_4, dh2_1, dh2_2, dh2_3, dh2_4);
}

DEVICE_FUNC inline void update_terms_and_variations(scalar xi, scalar ds, scalar alpha, const Vec12 &U, const Vec12 &V,
                                                    const Vec12 &A, Terms &terms, Variations &variations) {
    Vec10 N = shape_functions(xi, alpha, ds);
    Vec10 dN = shape_function_derivatives(xi, alpha, ds);

    const scalar u1 = U[0];
    const scalar u2 = U[1];
    const scalar u3 = U[2];
    const scalar u4 = U[3];
    const scalar u5 = U[4];
    const scalar u6 = U[5];
    const scalar u7 = U[6];
    const scalar u8 = U[7];
    const scalar u9 = U[8];
    const scalar u10 = U[9];
    const scalar u11 = U[10];
    const scalar u12 = U[11];

    const scalar v1 = V[0];
    const scalar v2 = V[1];
    const scalar v3 = V[2];
    const scalar v4 = V[3];
    const scalar v5 = V[4];
    const scalar v6 = V[5];
    const scalar v7 = V[6];
    const scalar v8 = V[7];
    const scalar v9 = V[8];
    const scalar v10 = V[9];
    const scalar v11 = V[10];
    const scalar v12 = V[11];

    const scalar a1 = A[0];
    const scalar a2 = A[1];
    const scalar a3 = A[2];
    const scalar a4 = A[3];
    const scalar a5 = A[4];
    const scalar a6 = A[5];
    const scalar a7 = A[6];
    const scalar a8 = A[7];
    const scalar a9 = A[8];
    const scalar a10 = A[9];
    const scalar a11 = A[10];
    const scalar a12 = A[11];

    const scalar n_1 = N[0];
    const scalar n_2 = N[1];
    const scalar h1_1 = N[2];
    const scalar h1_2 = N[3];
    const scalar h1_3 = N[4];
    const scalar h1_4 = N[5];
    const scalar h2_1 = N[6];
    const scalar h2_2 = N[7];
    const scalar h2_3 = N[8];
    const scalar h2_4 = N[9];

    const scalar dn_1 = dN[0];
    const scalar dn_2 = dN[1];
    const scalar dh1_1 = dN[2];
    const scalar dh1_2 = dN[3];
    const scalar dh1_3 = dN[4];
    const scalar dh1_4 = dN[5];
    const scalar dh2_1 = dN[6];
    const scalar dh2_2 = dN[7];
    const scalar dh2_3 = dN[8];
    const scalar dh2_4 = dN[9];

    // terms.u = u1 * n_1 + u7 * n_2; //Uncomment if needed!
    terms.tx = u4 * n_1 + u10 * n_2;
    terms.v = u2 * h1_1 + u6 * h1_2 + u8 * h1_3 + u12 * h1_4;
    terms.w = u3 * h1_1 - u5 * h1_2 + u9 * h1_3 - u11 * h1_4;
    terms.tz = u2 * h2_1 + u6 * h2_2 + u8 * h2_3 + u12 * h2_4;
    terms.ty = -u3 * h2_1 + u5 * h2_2 - u9 * h2_3 + u11 * h2_4;

    terms.u_t = v1 * n_1 + v7 * n_2;
    terms.tx_t = v4 * n_1 + v10 * n_2;
    terms.v_t = v2 * h1_1 + v6 * h1_2 + v8 * h1_3 + v12 * h1_4;
    terms.w_t = v3 * h1_1 - v5 * h1_2 + v9 * h1_3 - v11 * h1_4;
    terms.tz_t = v2 * h2_1 + v6 * h2_2 + v8 * h2_3 + v12 * h2_4;
    terms.ty_t = -v3 * h2_1 + v5 * h2_2 - v9 * h2_3 + v11 * h2_4;

    terms.u_tt = a1 * n_1 + a7 * n_2;
    terms.tx_tt = a4 * n_1 + a10 * n_2;
    terms.tz_tt = a2 * h2_1 + a6 * h2_2 + a8 * h2_3 + a12 * h2_4;
    terms.ty_tt = -a3 * h2_1 + a5 * h2_2 - a9 * h2_3 + a11 * h2_4;
    terms.v_tt = a2 * h1_1 + a6 * h1_2 + a8 * h1_3 + a12 * h1_4;
    terms.w_tt = a3 * h1_1 - a5 * h1_2 + a9 * h1_3 - a11 * h1_4;

    terms.u_s = u1 * dn_1 + u7 * dn_2;
    terms.tx_s = u4 * dn_1 + u10 * dn_2;
    terms.v_s = u2 * dh1_1 + u6 * dh1_2 + u8 * dh1_3 + u12 * dh1_4;
    terms.w_s = u3 * dh1_1 - u5 * dh1_2 + u9 * dh1_3 - u11 * dh1_4;
    terms.tz_s = u2 * dh2_1 + u6 * dh2_2 + u8 * dh2_3 + u12 * dh2_4;
    terms.ty_s = -u3 * dh2_1 + u5 * dh2_2 - u9 * dh2_3 + u11 * dh2_4;

    // Might need some of these for coriolis effects from fluid flow eventually
    terms.u_ts = v1 * dn_1 + v7 * dn_2;
    terms.tx_ts = v4 * dn_1 + v10 * dn_2;
    terms.v_ts = v2 * dh1_1 + v6 * dh1_2 + v8 * dh1_3 + v12 * dh1_4;
    terms.w_ts = v3 * dh1_1 - v5 * dh1_2 + v9 * dh1_3 - v11 * dh1_4;
    terms.tz_ts = v2 * dh2_1 + v6 * dh2_2 + v8 * dh2_3 + v12 * dh2_4;
    terms.ty_ts = -v3 * dh2_1 + v5 * dh2_2 - v9 * dh2_3 + v11 * dh2_4;

    // Variations
    variations.du = Vec2(n_1, n_2);
    variations.dtx = Vec2(n_1, n_2);
    variations.dv = Vec4(h1_1, h1_2, h1_3, h1_4);
    variations.dw = Vec4(h1_1, -h1_2, h1_3, -h1_4);
    variations.dtz = Vec4(h2_1, h2_2, h2_3, h2_4);
    variations.dty = Vec4(-h2_1, h2_2, -h2_3, h2_4);

    // Variations derivatives
    variations.du_s = Vec2(dn_1, dn_2);
    variations.dtx_s = Vec2(dn_1, dn_2);
    variations.dv_s = Vec4(dh1_1, dh1_2, dh1_3, dh1_4);
    variations.dw_s = Vec4(dh1_1, -dh1_2, dh1_3, -dh1_4);
    variations.dtz_s = Vec4(dh2_1, dh2_2, dh2_3, dh2_4);
    variations.dty_s = Vec4(-dh2_1, dh2_2, -dh2_3, dh2_4);
}

DEVICE_FUNC inline void add_contrib_du(Vec12 &out, const Vec2 &contrib) {
    out[0] += contrib[0];
    out[6] += contrib[1];
}
DEVICE_FUNC inline void add_contrib_dtx(Vec12 &out, const Vec2 &contrib) {
    out[3] += contrib[0];
    out[9] += contrib[1];
}
DEVICE_FUNC inline void add_contrib_dv(Vec12 &out, const Vec4 &contrib) {
    out[1] += contrib[0];
    out[5] += contrib[1];
    out[7] += contrib[2];
    out[11] += contrib[3];
}
DEVICE_FUNC inline void add_contrib_dtz(Vec12 &out, const Vec4 &contrib) {
    // dtz maps to same indices as dv
    add_contrib_dv(out, contrib);
}
DEVICE_FUNC inline void add_contrib_dw(Vec12 &out, const Vec4 &contrib) {
    out[2] += contrib[0];
    out[4] += contrib[1];
    out[8] += contrib[2];
    out[10] += contrib[3];
}
DEVICE_FUNC inline void add_contrib_dty(Vec12 &out, const Vec4 &contrib) {
    // dty maps to same indices as dw (sign pattern already stored in variations)
    add_contrib_dw(out, contrib);
}

} // namespace curvlin