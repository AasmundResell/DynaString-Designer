#include "hole.hpp"
#include <random>
using VecX = Eigen::VectorX<scalar>;
using MatX = Eigen::MatrixX<scalar>;
using AngleD = Eigen::AngleAxisd;

void build_hole_profile_from_minimum_curvature_method(uint N_hole, vector<scalar> &MD, vector<scalar> &inclinations,
                                                      vector<scalar> &azimuths, vector<Vec3> &nodes,
                                                      vector<scalar> &s_coordinates, vector<Vec2> &curvatures,
                                                      vector<Mat3> &R);
void build_hole_profile_from_continuous_curvature_method(uint N_hole, const vector<scalar> &MD,
                                                         const vector<scalar> &inclinations,
                                                         const vector<scalar> &azimuths, vector<Vec3> &nodes,
                                                         vector<scalar> &s_coordinates, vector<Vec2> &curvatures,
                                                         vector<Mat3> &R);

void create_hole_nodes_from_B_spline(uint N_hole, vector<Vec3> &nodes, vector<Vec3> &midnodes, vector<scalar> &kappas,
                                     const vector<Vec3> &trajectory);

void create_local_basis_Bishop_frame(vector<Mat3> &R, const vector<Vec3> midnodes);
void create_local_basis_Frenet_Serret(vector<Mat3> &R, const vector<Vec3> nodes, const vector<Vec3> midnodes);
void calculate_curvature_from_trajectory(vector<Vec2> &curvatures, const vector<scalar> &s, const vector<Mat3> &R,
                                         const vector<Vec3> &nodes, const vector<Vec3> &midnodes);

void minimum_curvature_method(vector<Vec3> &trajectory, vector<scalar> &MD, vector<scalar> &inclinations,
                              vector<scalar> &azimuths);
void calculate_gravity(vector<Vec3> &g_local, const ArrayView<Quaternion> q, const uint N_hole);
uint calculate_N_coeff(uint p, uint n, uint m, scalar u_val, const vector<scalar> &u, VecX &N);

uint calculate_N_prime_coeff(uint p, uint n, uint m, scalar u_val, const vector<scalar> &u, VecX &N_prime);
uint calculate_N_scalar_prime_coeff(uint p, uint n, uint m, scalar u_val, const vector<scalar> &u,
                                    VecX &N_scalar_prime);

void smooth_curvatures(std::vector<Vec2> &curvatures, uint window_size);

template <HoleField field> inline void allocate_single_hole_field(ArenaBump &arena_h, Hole &hole, uint count) {
    static_assert(field < HoleField::COUNT);
    using T = typename HoleFieldTraits<field>::variable_type;
    Offset<T> offset{};
    arena_h.allocate(&offset, count);
    hole.offsets[(uint)field] = {.offset = offset.offset, .count = offset.count};
}

Hole create_and_allocate_hole(uint N_hole, ArenaBump &arena_h) {
    Hole hole = {};
    hole.N_hole = N_hole;

    allocate_single_hole_field<HoleField::s>(arena_h, hole, N_hole + 2);
    allocate_single_hole_field<HoleField::x>(arena_h, hole, N_hole + 2);
    allocate_single_hole_field<HoleField::q>(arena_h, hole, N_hole + 2);
    allocate_single_hole_field<HoleField::g>(arena_h, hole, N_hole + 2);
    allocate_single_hole_field<HoleField::kappa>(arena_h, hole, N_hole + 2);
    allocate_single_hole_field<HoleField::a>(arena_h, hole, N_hole + 2);
    allocate_single_hole_field<HoleField::b>(arena_h, hole, N_hole + 2);
    allocate_single_hole_field<HoleField::alpha>(arena_h, hole, N_hole + 2);
    allocate_single_hole_field<HoleField::co>(arena_h, hole, N_hole + 2);

    return hole;
}

Hole hole_create_from_vector(HoleTrajectoryType trajectory_type, uint N_hole, vector<scalar> &MD,
                             vector<scalar> &inclinations, vector<scalar> &azimuths, ArenaBump &arena_h) {
    /*--------------------------------------------------------------------
    Hole geometry, main constructor

    Curve interpolation:
    https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/INT-APP/CURVE-INT-global.html

    More information - Spline lecture notes:
    https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/notes.html
    --------------------------------------------------------------------*/
    vector<scalar> s_coordinates;
    vector<Vec3> nodes;
    vector<Mat3> R;
    vector<Vec2> curvatures;

    s_coordinates.resize(N_hole);
    R.resize(N_hole);
    nodes.resize(N_hole);
    curvatures.resize(N_hole);

    if (MD[0] > 0.0) {
        // Insert a zero point at the beginning of MD vector
        MD.insert(MD.begin(), 0.0);
        inclinations.insert(inclinations.begin(), inclinations.front());
        azimuths.insert(azimuths.begin(), azimuths.front());
    }

    if (trajectory_type == HoleTrajectoryType::MINIMUM_CURVATURE) {
        build_hole_profile_from_minimum_curvature_method(N_hole, MD, inclinations, azimuths, nodes, s_coordinates,
                                                         curvatures, R);
    } else if (trajectory_type == HoleTrajectoryType::CONTINUOUS_CURVATURE) {
        build_hole_profile_from_continuous_curvature_method(N_hole, MD, inclinations, azimuths, nodes, s_coordinates,
                                                            curvatures, R);
    }

    // Hole object generation
    Hole hole = create_and_allocate_hole(N_hole, arena_h);

    // Lastly, assigning all vectors of the hole object
    byte *buf = arena_h.buf;
    ArrayView<Vec3> x = hole.get_field<HoleField::x>(buf);
    ArrayView<Quaternion> q = hole.get_field<HoleField::q>(buf);
    ArrayView<Vec2> kappa = hole.get_field<HoleField::kappa>(buf);
    ArrayView<scalar> s = hole.get_field<HoleField::s>(buf);
    ArrayView<Vec3> g = hole.get_field<HoleField::g>(buf);

    // "Out of bounds" values for the drill floor
    const Vec3 t_top = R[0].col(0);
    x[0] = t_top * -L_MAX_LENGTH;
    x[1] = t_top * -0.1; // 10 cm outside the hole
    kappa[0] = Vec2{0, 0};
    kappa[1] = Vec2{0, 0};

    smooth_curvatures(curvatures, 40);

    q[0].from_matrix(R[0], 1E-5);
    q[1].from_matrix(R[0], 1E-5);
    s[0] = -L_MAX_LENGTH;
    s[1] = -0.1; // 10 cm outside the hole

    for (uint i = 0; i < N_hole; i++) {
        x[i + 2] = nodes[i];
        kappa[i + 2] = curvatures[i];
        q[i + 2].from_matrix(R[i], 1E-3);
        s[i + 2] = s_coordinates[i];
    }

    // Vector holding gravity
    vector<Vec3> g_local;
    g_local.resize(N_hole);

    /*Calculating the gravity vectors in the local frames*/
    calculate_gravity(g_local, q, N_hole);

    // Ghost values for the drill floor
    g[0] = g_local[0];
    g[1] = g_local[0];

    for (uint i = 0; i < N_hole; i++) {
        g[i + 2] = g_local[i];
    }

    return hole;
}

Hole hole_create_from_linear(HoleTrajectoryType trajectory_type, uint N_hole, vector<scalar> &MD,
                             scalar inclination_top, scalar inclination_build, scalar azimuth_top, scalar azimuth_build,
                             scalar L_total, scalar L_front, scalar L_end, ArenaBump &arena_h) {
    /*--------------------------------------------------------------------
    Hole geometry generated from "LINEAR" method
    --------------------------------------------------------------------*/

    // Number of interpolation points for spline
    uint n_interpolate = MD.size();

    vector<scalar> inclinations;
    vector<scalar> azimuths;

    scalar d_l = L_total / (n_interpolate - 1); // Length increments for curve generation

    inclinations.resize(n_interpolate);
    azimuths.resize(n_interpolate);

    inclinations[0] = inclination_top;
    azimuths[0] = azimuth_top;

    // Initialize vectors
    for (uint i = 1; i < n_interpolate; i++) {
        MD[i] = d_l + MD[i - 1];
        inclinations[i] = inclination_top;
        azimuths[i] = azimuth_top;
    }

    uint n_front = (uint)L_front / d_l; // Number of interpolation points for straigt front section
    uint n_back = (uint)L_end / d_l;    // Number of interpolation points for straight end section

    if (L_front <= 0.0)
        assert(n_front == 0);
    if (L_end <= 0.0)
        assert(n_back == 0);

    uint n_build = n_interpolate - n_front - n_back;

    scalar d_inc = (scalar)(inclination_build - inclination_top) / (n_build - 1);
    scalar d_az = (scalar)(azimuth_build - azimuth_top) / (n_build - 1);

    inclinations[n_front] = inclination_top;
    for (uint i = n_front + 1; i < n_interpolate - n_back; i++) {
        inclinations[i] = d_inc + inclinations[i - 1];
        azimuths[i] = d_az + azimuths[i - 1];
    }

    for (uint i = n_interpolate - n_back; i < n_interpolate; i++) {
        inclinations[i] = inclination_build;
        azimuths[i] = azimuth_build;
    }

    return hole_create_from_vector(trajectory_type, N_hole, MD, inclinations, azimuths, arena_h);
}

void evaluate_tangent_and_curvature_from_angles(Vec3 &t, scalar &kappa, Vec3 &n, const MatX &C0, const MatX &C1,
                                                const MatX &C2, const MatX &C3, scalar s, uint segment_idx) {
    // Evaluate cubic polynomials
    scalar inc = C0(segment_idx, 0) * std::pow(s, 3) + C1(segment_idx, 0) * std::pow(s, 2) + C2(segment_idx, 0) * s +
                 C3(segment_idx, 0);
    scalar az = C0(segment_idx, 1) * std::pow(s, 3) + C1(segment_idx, 1) * std::pow(s, 2) + C2(segment_idx, 1) * s +
                C3(segment_idx, 1);

    // Calculate derivatives
    scalar dinc = 3 * C0(segment_idx, 0) * powi(s, 2) + 2 * C1(segment_idx, 0) * s + C2(segment_idx, 0);
    scalar daz = 3 * C0(segment_idx, 1) * powi(s, 2) + 2 * C1(segment_idx, 1) * s + C2(segment_idx, 1);

    // Calculate curvature
    kappa = sqrt(powi(dinc, 2) + powi(sin(inc), 2) * powi(daz, 2));

    // Calculate tangent vector (negative z-component to ensure inc = 0 points downwards)
    t << sin(inc) * cos(az), sin(inc) * sin(az), -cos(inc);

    // Calculate normal vector from dtds = n * kappa
    Vec3 dtds;
    dtds << cos(inc) * cos(az) * dinc - sin(az) * sin(inc) * daz, cos(inc) * sin(az) * dinc + cos(az) * sin(inc) * daz,
        -sin(inc) * dinc;

    if (kappa > SMALL_SCALAR) {
        n = dtds / kappa;
    } else {
        n = Vec3{0, 0, 1}; // Arbitrary value
    }
    assert(abs(n.squaredNorm() - 1.0) < SMALL_SCALAR);
}

void continuous_curvature_method(const vector<scalar> &MD, const vector<scalar> &inclinations,
                                 const vector<scalar> &azimuths, const Vec3 &start_pos, vector<scalar> &S_interp,
                                 vector<Vec3> &nodes, vector<scalar> &kappas, vector<Vec3> &midnodes,
                                 vector<Vec3> &normals) {
    assert(MD.size() == inclinations.size());
    assert(MD[0] == 0.0);

    const uint N_points = MD.size();
    const uint N_hole = nodes.size();
    const uint N_seg = N_points - 1;

    // For cubic interpolation, we need 4 coefficients per segment
    MatX C0(N_seg, 2); // Cubic coefficient
    MatX C1(N_seg, 2); // Quadratic coefficient
    MatX C2(N_seg, 2); // Linear coefficient
    MatX C3(N_seg, 2); // Constant coefficient

    // Convert angles to radians
    vector<scalar> inc_rad(N_points), az_rad(N_points);
    for (uint i = 0; i < N_points; i++) {
        inc_rad[i] = inclinations[i] * M_PI / 180.0;
        az_rad[i] = azimuths[i] * M_PI / 180.0;
    }

    const int n_eqs = 4 * N_seg;
    MatSparse A(n_eqs, n_eqs);
    VecX b_inc(n_eqs);
    VecX b_az(n_eqs);

    A.setZero();
    b_inc.setZero();
    b_az.setZero();

    // Fill the sparse matrix
    vector<Eigen::Triplet<scalar>> triplets;
    triplets.reserve(n_eqs * 4); // Approximate size

    for (uint i = 0; i < N_seg; i++) {
        scalar s_N = MD[i];
        scalar s_Np = MD[i + 1];

        // Add acceleration constraint
        if (i == 0) {
            triplets.push_back(Eigen::Triplet<scalar>(4 * i, 4 * i, 6 * s_N));
            triplets.push_back(Eigen::Triplet<scalar>(4 * i, 4 * i + 1, 2));

        } else {
            triplets.push_back(Eigen::Triplet<scalar>(4 * i, 4 * i - 4, 6 * s_N));
            triplets.push_back(Eigen::Triplet<scalar>(4 * i, 4 * i - 3, 2));
            triplets.push_back(Eigen::Triplet<scalar>(4 * i, 4 * i, -6 * s_N));
            triplets.push_back(Eigen::Triplet<scalar>(4 * i, 4 * i + 1, -2));
        }

        // Add position constraints
        triplets.push_back(Eigen::Triplet<scalar>(4 * i + 1, 4 * i, powi(s_N, 3)));
        triplets.push_back(Eigen::Triplet<scalar>(4 * i + 1, 4 * i + 1, powi(s_N, 2)));
        triplets.push_back(Eigen::Triplet<scalar>(4 * i + 1, 4 * i + 2, s_N));
        triplets.push_back(Eigen::Triplet<scalar>(4 * i + 1, 4 * i + 3, 1));

        triplets.push_back(Eigen::Triplet<scalar>(4 * i + 2, 4 * i, powi(s_Np, 3)));
        triplets.push_back(Eigen::Triplet<scalar>(4 * i + 2, 4 * i + 1, powi(s_Np, 2)));
        triplets.push_back(Eigen::Triplet<scalar>(4 * i + 2, 4 * i + 2, s_Np));
        triplets.push_back(Eigen::Triplet<scalar>(4 * i + 2, 4 * i + 3, 1));

        // Add velocity constraints
        if (i < N_seg - 1) {
            triplets.push_back(Eigen::Triplet<scalar>(4 * i + 3, 4 * i, 3 * powi(s_Np, 2)));
            triplets.push_back(Eigen::Triplet<scalar>(4 * i + 3, 4 * i + 1, 2 * s_Np));
            triplets.push_back(Eigen::Triplet<scalar>(4 * i + 3, 4 * i + 2, 1));
            triplets.push_back(Eigen::Triplet<scalar>(4 * i + 3, 4 * i + 4, -3 * powi(s_Np, 2)));
            triplets.push_back(Eigen::Triplet<scalar>(4 * i + 3, 4 * i + 5, -2 * s_Np));
            triplets.push_back(Eigen::Triplet<scalar>(4 * i + 3, 4 * i + 6, -1));
        } else {
            triplets.push_back(Eigen::Triplet<scalar>(4 * i + 3, 4 * i, 6 * s_Np));
            triplets.push_back(Eigen::Triplet<scalar>(4 * i + 3, 4 * i + 1, 2));
        }

        // Fill b vectors
        b_inc[4 * i + 1] = inc_rad[i];
        b_inc[4 * i + 2] = inc_rad[i + 1];
        b_az[4 * i + 1] = az_rad[i];
        b_az[4 * i + 2] = az_rad[i + 1];
    }

    A.setFromTriplets(triplets.begin(), triplets.end());
    A.makeCompressed();

    // Solve the system for both inclination and azimuth
    Eigen::SparseLU<MatSparse> solver;
    solver.compute(A);
    if (solver.info() != Eigen::Success) {
        THROW_RUNTIME_ERROR("Failed to compute the LU decomposition of the matrix");
    }

    VecX x_inc = solver.solve(b_inc);
    VecX x_az = solver.solve(b_az);

    // Reshape solutions into coefficient matrices
    for (uint i = 0; i < N_seg; i++) {
        C0(i, 0) = x_inc[4 * i];     // Cubic coeff for inclination
        C1(i, 0) = x_inc[4 * i + 1]; // Quadratic coeff for inclination
        C2(i, 0) = x_inc[4 * i + 2]; // Linear coeff for inclination
        C3(i, 0) = x_inc[4 * i + 3]; // Constant coeff for inclination

        C0(i, 1) = x_az[4 * i];     // Cubic coeff for azimuth
        C1(i, 1) = x_az[4 * i + 1]; // Quadratic coeff for azimuth
        C2(i, 1) = x_az[4 * i + 2]; // Linear coeff for azimuth
        C3(i, 1) = x_az[4 * i + 3]; // Constant coeff for azimuth
    }

    // Generate interpolation points
    scalar s_start = 0.0;
    assert(MD.front() == 0.0);
    scalar s_end = MD.back();
    scalar ds = (s_end - s_start) / (N_hole - 1);

    nodes[0] = start_pos;
    midnodes[0] = start_pos;

    for (uint i = 1; i < N_hole; i++) {
        scalar s = i * ds;
        scalar s_mid = (i - 0.5) * ds;

        S_interp[i] = s;

        // Find segment containing s
        uint segment_idx = 0;
        while (segment_idx < N_seg - 1 && MD[segment_idx + 1] < s) {
            segment_idx++;
        }
        segment_idx = min(segment_idx, N_points - 2);

        // Find segment containing s_mid
        uint segment_idx_mid = 0;
        while (segment_idx_mid < N_seg - 1 && MD[segment_idx_mid + 1] < s_mid) {
            segment_idx_mid++;
        }
        segment_idx_mid = min(segment_idx_mid, N_points - 2);

        Vec3 t;
        evaluate_tangent_and_curvature_from_angles(t, kappas[i], normals[i], C0, C1, C2, C3, s, segment_idx);

        Vec3 t_mid;
        Vec3 n_dummy;
        scalar kappa_dummy;
        evaluate_tangent_and_curvature_from_angles(t_mid, kappa_dummy, n_dummy, C0, C1, C2, C3, s_mid, segment_idx_mid);

        nodes[i] = nodes[i - 1] + t * ds;
        midnodes[i] = nodes[i - 1] + t_mid * ds * 0.5;
    }

    midnodes[N_hole] = nodes[N_hole - 1];
}

void build_hole_profile_from_minimum_curvature_method(uint N_hole, vector<scalar> &MD, vector<scalar> &inclinations,
                                                      vector<scalar> &azimuths, vector<Vec3> &nodes,
                                                      vector<scalar> &s_coordinates, vector<Vec2> &curvatures,
                                                      vector<Mat3> &R) {

    vector<Vec3> trajectory; /*Point trajectory of borehole used for borehole interpolation*/

    /*Generate borehole trajectory from survey vectors*/
    minimum_curvature_method(trajectory, MD, inclinations, azimuths);

    vector<Vec3> midnodes;
    vector<scalar> kappas;
    kappas.resize(N_hole);
    midnodes.resize(N_hole + 1);

    /*Increase resolution of the trajectory by cubic b-spline interpolation*/
    create_hole_nodes_from_B_spline(N_hole, nodes, midnodes, kappas, trajectory);

    // Total length of the borehole used in the computations
    scalar L = 0.0;
    for (uint i = 1; i < N_hole; i++) {
        L += (nodes[i] - nodes[i - 1]).norm();
    }
    // Constant spacing for now
    scalar ds = L / (N_hole - 1) - SMALL_SCALAR;
    for (uint i = 0; i < N_hole; i++) {
        s_coordinates[i] = i * ds;
    }

    // Generates orientation basis at each hole node
    create_local_basis_Bishop_frame(R, midnodes);

    // Calculates curvatures in each node
    calculate_curvature_from_trajectory(curvatures, s_coordinates, R, nodes, midnodes);
}

void build_hole_profile_from_continuous_curvature_method(uint N_hole, const vector<scalar> &MD,
                                                         const vector<scalar> &inclinations,
                                                         const vector<scalar> &azimuths, vector<Vec3> &nodes,
                                                         vector<scalar> &s_coordinates, vector<Vec2> &curvatures,
                                                         vector<Mat3> &R) {

    vector<Vec3> midnodes(N_hole + 1);
    vector<scalar> kappas(N_hole);
    vector<Vec3> normals(N_hole);

    /*Generate borehole trajectory from survey vectors*/
    continuous_curvature_method(MD, inclinations, azimuths, Vec3{0, 0, 0}, s_coordinates, nodes, kappas, midnodes,
                                normals);

    // Generates orientation basis at each hole node
    create_local_basis_Bishop_frame(R, midnodes);

    for (uint i = 0; i < N_hole; i++) {
        // Retrieve the two normal vectors in the Bishop frame
        Vec3 e2 = R[i].col(1);
        Vec3 e3 = R[i].col(2);

        scalar kappa = kappas[i];
        Vec3 n = normals[i];

        // Calculate the projections of the curvature
        // (WHY NEGATIVE SIGN, differente sign convention for mechanics vs. math acording to AI)
        scalar kappa_y = -kappa * n.dot(e3); // Curvature about e2 (y-axis)
        scalar kappa_z = -kappa * n.dot(e2); // Curvature about e3 (z-axis)

        curvatures[i] = Vec2{kappa_y, kappa_z};
    }
}

void create_hole_nodes_from_B_spline(uint N_hole, vector<Vec3> &nodes, vector<Vec3> &midnodes, vector<scalar> &kappas,
                                     const vector<Vec3> &trajectory) {

    const uint n = trajectory.size() - 1; // - 1 For consistency with lecture notes

    // Total interpolation length
    scalar L_ip = 0.0;
    for (uint i = 1; i < n + 1; i++)
        L_ip += (trajectory[i] - trajectory[i - 1]).norm();

    // Chord lengths
    vector<scalar> t(n + 1, 0.0);

    t[n] = 1.0;

    // Normalize t on the domain [0,1]
    for (uint i = 1; i < n; i++)
        t[i] = (trajectory[i] - trajectory[i - 1]).norm() / L_ip + t[i - 1];

    const uint p = 3; // Cubic interpolation

    const uint m = n + p + 1; // Multiplicity

    vector<scalar> u(m + 1, 0.0); // Knot vector

    /*Start is clamped implicitly by u being initialized as zero vector
    Clamped end*/
    for (uint i = m - p; i < m + 1; i++)
        u[i] = 1.0;

    // Average method -> Internal knot is average of p surrounding parameters
    for (uint j = 1; j < n - p + 1; j++) {
        for (uint i = j; i < j + p; i++)
            u[p + j] = u[p + j] + (scalar)1 / p * t[i];
    }

    MatX D(n + 1, 3);      // Interpolation points matrix
    MatX Np(n + 1, n + 1); // Coefficient matrix

    Np.setZero();
    VecX N_row(n + 1);
    VecX N_coeff(p + 1);

    // Populate D and Np
    for (uint i = 0; i < n + 1; i++) {
        D.row(i) = trajectory[i].cast<scalar>();
        uint k = calculate_N_coeff(p, n, m, t[i], u, N_coeff);
        Np.row(i).segment(k - p, p + 1) = N_coeff.transpose();
    }

    // Switch the sign of the z coordinates so negative points downwards
    D.col(2) *= -1.0;

    /* FullPivLu ensures accurate start/end points,
    consider testing other alternatives */
    MatX P = Np.fullPivLu().solve(D); // Solve D = N * P

    for (uint i = 0; i < N_hole; i++) {
        scalar u_val = (scalar)i / (N_hole - 1); // Even spacing for node generation
        uint k = calculate_N_coeff(p, n, m, u_val, u, N_coeff);
        nodes[i] = N_coeff.transpose() * P.block(k - p, 0, p + 1, p);
    }

    midnodes[0] = nodes[0];

    for (uint i = 1; i < N_hole; i++) {
        scalar u_val = (scalar)(i - 0.5) / (N_hole - 1);
        uint k = calculate_N_coeff(p, n, m, u_val, u, N_coeff);
        midnodes[i] = N_coeff.transpose() * P.block(k - p, 0, p + 1, p);
    }

    // Boundary nodes for triads at beginning and end
    midnodes[N_hole] = nodes[N_hole - 1];
}

void create_local_basis_Bishop_frame(vector<Mat3> &R, const vector<Vec3> midnodes) {

    /*=====================================================================
    Hole geometry generated using the Bishop frame

    de1/ds =  kappa_y * e3 + kappa_z * e2
    de2/ds = -kappa_z * e1
    de3/ds = -kappa_y * e1
    =====================================================================*/

    assert(midnodes[0].isApprox(Vec3{0, 0, 0}));

    uint N = midnodes.size() - 1;

    Vec3 e1_old = Vec3{0, 0, -1.0};     // points negative global Z (Downwards);
    Vec3 e2_old = Vec3{0, 1.0, 0};      // points in global Y (North);
    Vec3 e3_old = e1_old.cross(e2_old); // points in global X (East);

    Vec3 e1;
    Vec3 e2;
    Vec3 e3;

    Mat3 R_i;

    for (uint i = 0; i < N; i++) {
        // e1: tangent = dr/ds/||dr/ds||
        e1 = (midnodes[i + 1] - midnodes[i]).normalized();

        Vec3 m = e1_old.cross(e1);

        // If m is zero, a straight section is encountered
        scalar norm = m.norm();

        if (norm < SMALL_SCALAR) {
            e2 = e2_old;
            e3 = e3_old;
        } else {
            m = m.normalized();
            // Clamp dot product to [-1, 1] to avoid NaN from acos
            scalar dot = e1_old.dot(e1);
            dot = max(-1.0, min(1.0, dot));
            scalar theta = acos(dot);
            e2 = e2_old * cos(theta) + (m.cross(e2_old)) * sin(theta) + m * (m.dot(e2_old)) * (1 - cos(theta));
            e3 = e3_old * cos(theta) + (m.cross(e3_old)) * sin(theta) + m * (m.dot(e3_old)) * (1 - cos(theta));
        }

        R_i.col(0) = e1;
        R_i.col(1) = e2;
        R_i.col(2) = e3;

        // assert(is_orthonormal(R_i, 1e-5));

        R[i] = R_i;

        e1_old = e1;
        e2_old = e2;
        e3_old = e3;
    }
}

void minimum_curvature_method(vector<Vec3> &trajectory, vector<scalar> &MD, vector<scalar> &inclinations,
                              vector<scalar> &azimuths) {

    scalar TVD_tot = 0;
    scalar North_tot = 0;
    scalar East_tot = 0;

    scalar d2r = M_PI / 180;

    scalar d_TVD;
    scalar d_North;
    scalar d_East;

    assert(MD[0] == 0.0);
    // Insert additional points if d_MD > max_segment_length
    scalar max_segment_length = 10.0; // Set this to your desired maximum segment length

    vector<scalar> new_MD;
    vector<scalar> new_inclinations;
    vector<scalar> new_azimuths;

    new_MD.push_back(MD[0]);
    new_inclinations.push_back(inclinations[0]);
    new_azimuths.push_back(azimuths[0]);

    for (uint i = 0; i < MD.size() - 1; i++) {
        // Always keep the original point
        if (i > 0) {
            new_MD.push_back(MD[i]);
            new_inclinations.push_back(inclinations[i]);
            new_azimuths.push_back(azimuths[i]);
        }

        scalar d_MD = MD[i + 1] - MD[i];
        scalar inc0 = inclinations[i];
        scalar inc1 = inclinations[i + 1];
        scalar az0 = azimuths[i];
        scalar az1 = azimuths[i + 1];

        uint n_segments = static_cast<uint>(ceil(d_MD / max_segment_length));
        n_segments = std::max(n_segments, 1u);

        // Only add extra points between i and i+1 (not including endpoints)
        for (uint seg = 1; seg < n_segments; ++seg) {
            scalar frac = static_cast<scalar>(seg) / n_segments;
            scalar md_val = MD[i] + frac * d_MD;
            scalar inc_val = inc0 + frac * (inc1 - inc0);
            scalar az_val = az0 + frac * (az1 - az0);

            new_MD.push_back(md_val);
            new_inclinations.push_back(inc_val);
            new_azimuths.push_back(az_val);
        }
    }
    // Always keep the last original point
    new_MD.push_back(MD.back());
    new_inclinations.push_back(inclinations.back());
    new_azimuths.push_back(azimuths.back());

    // Copy back to MD, inclinations, azimuths if needed elsewhere
    // (If you want to update the original vectors, uncomment below)
    MD = move(new_MD);
    inclinations = move(new_inclinations);
    azimuths = move(new_azimuths);
    trajectory.resize(MD.size());
    trajectory[0] = Vec3({North_tot, East_tot, TVD_tot});

    for (uint i = 0; i < MD.size() - 1; i++) {

        scalar d_MD = MD[i + 1] - MD[i];

        if (d_MD < SMALL_SCALAR) {
            THROW_RUNTIME_ERROR("MD increment is not positive");
        }
        scalar d_inc = inclinations[i + 1] - inclinations[i];
        scalar d_az = azimuths[i + 1] - azimuths[i];

        scalar beta = acos(cos(d_inc * d2r) -
                           sin(inclinations[i] * d2r) * sin(inclinations[i + 1] * d2r) * (1 - cos(d_az * d2r)));

        if (abs(beta) < SMALL_SCALAR) // Straight section
        {
            scalar d_H = sin(inclinations[i] * d2r) * d_MD; // Horizontal projection

            d_TVD = d_MD * cos(inclinations[i] * d2r);
            d_North = d_H * cos(azimuths[i] * d2r);
            d_East = d_H * sin(azimuths[i] * d2r);
        }

        else {
            scalar RF = 2 / beta * tan(beta / 2);

            d_TVD = d_MD / 2 * (cos(inclinations[i] * d2r) + cos(inclinations[i + 1] * d2r)) * RF;
            d_North = d_MD / 2 *
                      (sin(inclinations[i] * d2r) * cos(azimuths[i] * d2r) +
                       sin(inclinations[i + 1] * d2r) * cos(azimuths[i + 1] * d2r)) *
                      RF;
            d_East = d_MD / 2 *
                     (sin(inclinations[i] * d2r) * sin(azimuths[i] * d2r) +
                      sin(inclinations[i + 1] * d2r) * sin(azimuths[i + 1] * d2r)) *
                     RF;
        }

        TVD_tot = TVD_tot + d_TVD;
        North_tot += d_North;
        East_tot += d_East;

        trajectory[i + 1] = Vec3({North_tot, East_tot, TVD_tot});
    }
}

uint calculate_N_coeff(uint p, uint n, uint m, scalar u_val, const vector<scalar> &u, VecX &N) {

    /*--------------------------------------------------------------------
    Algorithm overview:
    https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/INT-APP/CURVE-INT-global.html
    --------------------------------------------------------------------*/
    assert(u_val >= u[0] and u_val <= u[m]);
    N.setZero();
    uint k = p;
    // Handle special case
    if (u_val == u[0]) {
        N[0] = 1.0;
        return k;
    } else if (u_val == u[u.size() - 1]) {
        N[N.size() - 1] = 1.0;
        k = n;
        return k;
    }

    for (uint i = 0; i < n - p + 1; i++) {
        if (u_val >= u[p + i] && u_val <= u[p + i + 1])
            break;
        k += 1;
    }

    N[p] = 1.0; // order p=0 coeff

    // For each polynomial degree
    for (uint d = 1; d < p + 1; d++) {
        N[p - d] = (u[k + 1] - u_val) / (u[k + 1] - u[k - d + 1]) * N[p - d + 1];
        for (uint i = k - d + 1; i < k - 1 + 1; i++) {
            N[i - k + p] = (u_val - u[i]) / (u[i + d] - u[i]) * N[i - k + p] +
                           (u[i + d + 1] - u_val) / (u[i + d + 1] - u[i + 1]) * N[i + 1 - k + p];
        }
        N[p] = (u_val - u[k]) / (u[k + d] - u[k]) * N[p];
    }

    return k;
}

uint calculate_N_prime_coeff(uint p, uint n, uint m, scalar u_val, const vector<scalar> &u, VecX &N_prime) {
    N_prime.setZero();
    uint k = p;

    // Handle boundary cases
    if (u_val == u[0]) {
        return k;
    } else if (u_val == u[u.size() - 1]) {
        k = n;
        return k;
    }

    // Find the knot span
    for (uint i = 0; i < n - p + 1; i++) {
        if (u_val >= u[p + i] && u_val <= u[p + i + 1]) {
            break;
        }
        k += 1;
    }

    // Temporary storage for lower order basis functions
    VecX N_p(p + 1);
    VecX N_p_minus_1(p);

    // Calculate the p-th degree basis functions
    calculate_N_coeff(p, n, m, u_val, u, N_p);
    // Calculate the (p-1)-th degree basis functions
    calculate_N_coeff(p - 1, n, m, u_val, u, N_p_minus_1);

    // Apply the derivative formula
    for (uint i = 0; i < p; i++) {
        scalar denom1 = u[k + p - i] - u[k - i];
        scalar denom2 = u[k + p + 1 - i] - u[k + 1 - i];

        if (denom1 > SMALL_SCALAR) {
            N_prime[i] += p * N_p_minus_1[i] / denom1;
        }
        if (denom2 > SMALL_SCALAR) {
            N_prime[i] -= p * N_p_minus_1[i] / denom2;
        }
    }

    return k;
}

uint calculate_N_scalar_prime_coeff(uint p, uint n, uint m, scalar u_val, const vector<scalar> &u,
                                    VecX &N_scalar_prime) {
    N_scalar_prime.setZero();
    uint k = p;

    // Handle boundary cases
    if (u_val == u[0]) {
        return k;
    } else if (u_val == u[u.size() - 1]) {
        k = n;
        return k;
    }

    // Find the knot span
    for (uint i = 0; i < n - p + 1; i++) {
        if (u_val >= u[p + i] && u_val <= u[p + i + 1]) {
            break;
        }
        k += 1;
    }

    // Temporary storage for lower order basis functions
    VecX N_p(p + 1);
    VecX N_p_minus_1(p);
    VecX N_p_minus_2(p - 1);

    // Calculate the needed lower degree basis functions
    calculate_N_coeff(p, n, m, u_val, u, N_p);
    calculate_N_coeff(p - 1, n, m, u_val, u, N_p_minus_1);
    calculate_N_coeff(p - 2, n, m, u_val, u, N_p_minus_2);

    // Apply the second derivative formula
    for (uint i = 0; i < p - 1; i++) {
        scalar denom1 = u[k + p - i] - u[k - i];
        scalar denom2 = u[k + p - 1 - i] - u[k - i];
        scalar denom3 = u[k + p + 1 - i] - u[k + 1 - i];
        scalar denom4 = u[k + p - i] - u[k + 1 - i];

        if (denom1 > SMALL_SCALAR && denom2 > SMALL_SCALAR) {
            N_scalar_prime[i] += p * (p - 1) * N_p_minus_2[i] / (denom1 * denom2);
        }
        if (denom2 > SMALL_SCALAR && denom3 > SMALL_SCALAR) {
            N_scalar_prime[i] -= 2 * p * (p - 1) * N_p_minus_2[i] / (denom2 * denom3);
        }
        if (denom3 > SMALL_SCALAR && denom4 > SMALL_SCALAR) {
            N_scalar_prime[i] += p * (p - 1) * N_p_minus_2[i] / (denom3 * denom4);
        }
    }

    return k;
}

// Helper: Exponential covariance matrix
MatX exp_cov(uint N, double std, double cor, const std::vector<double> &s_vals) {
    MatX cov(N, N);
    for (uint i = 0; i < N; ++i) {
        for (uint j = 0; j < N; ++j) {
            cov(i, j) = std * std * std::exp(-std::abs(s_vals[i] - s_vals[j]) / cor);
        }
        cov(i, i) += 1e-8; // Numerical stability
    }
    return cov;
}

// Stationary Gaussian field with the first value fixed to mean
vector<scalar> stationary_gaussian_field_mean_first(uint N, scalar mean, scalar std, scalar cor,
                                                    const vector<scalar> &s_vals, std::mt19937 &rng) {
    MatX cov = exp_cov(N, std, cor, s_vals);

    // Partition covariance
    MatX cov_11 = cov.block(0, 0, 1, 1);         // (1,1)
    MatX cov_12 = cov.block(0, 1, 1, N - 1);     // (1,N-1)
    MatX cov_21 = cov.block(1, 0, N - 1, 1);     // (N-1,1)
    MatX cov_22 = cov.block(1, 1, N - 1, N - 1); // (N-1,N-1)

    // Conditional covariance
    MatX cond_cov = cov_22 - (cov_21 * cov_12) / cov_11(0, 0);

    // Standard cholesky decomposition (LL^T)
    Eigen::LLT<MatX> llt(cond_cov);

    MatX L = llt.matrixL();
    std::normal_distribution<> d(0.0, 1.0);
    VecX z(N - 1);
    for (uint i = 0; i < N - 1; ++i) {
        z(i) = d(rng);
    }

    vector<scalar> vals(N);
    vals[0] = mean;
    VecX vals1 = L * z;
    vals1.array() += mean;
    for (uint i = 1; i < N; ++i) {
        vals[i] = vals1(i - 1);
    }
    return vals;
}

void create_hole_surface(Config &config, Hole &hole, const vector<HoleSection> &hole_sections, ArenaBump &arena_h) {
    const uint N_hole = hole.N_hole;
    byte *buf = arena_h.buf;
    const ArrayView<scalar> s = hole.get_field<HoleField::s>(buf);
    ArrayView<scalar> a = hole.get_field<HoleField::a>(buf);
    ArrayView<scalar> b = hole.get_field<HoleField::b>(buf);
    ArrayView<scalar> alpha = hole.get_field<HoleField::alpha>(buf);
    ArrayView<Vec2> co = hole.get_field<HoleField::co>(buf);

    // "Out of bounds" values for the drill floor
    a[0] = L_MAX_LENGTH; // No contact possible
    a[1] = L_MAX_LENGTH; // No contact possible
    b[0] = L_MAX_LENGTH; // No contact possible
    b[1] = L_MAX_LENGTH; // No contact possible
    alpha[0] = 0;
    alpha[1] = 0;
    co[0] = Vec2{0, 0};
    co[1] = Vec2{0, 0};

    Vec2 last_offset = {0, 0}; // Tracks the offset at the end of the previous section

    for (uint sec_idx = 0; sec_idx < hole_sections.size(); ++sec_idx) {
        const HoleSection &section = hole_sections[sec_idx];
        scalar start_depth = section.depth;
        assert(sec_idx != 0 || start_depth == 0.0);
        bool is_last_section = (sec_idx + 1 == hole_sections.size());

        scalar end_depth = (sec_idx + 1 < hole_sections.size()) ? hole_sections[sec_idx + 1].depth : hole.get_L(buf);
        assert(end_depth > start_depth && start_depth >= 0.0);

        uint N_sec = 0;
        vector<uint> idxs;
        vector<scalar> s_vals_sec;
        for (uint i = 2; i < N_hole + 2; ++i) {
            if (s[i] >= start_depth && (is_last_section ? s[i] <= end_depth : s[i] < end_depth)) {
                idxs.push_back(i);
                s_vals_sec.push_back(s[i]);
                N_sec++;
            }
        }
        if (N_sec == 0)
            THROW_RUNTIME_ERROR("No valid nodes found in section");

        /*================================================================================
        Random field generation
        ==================================================================================*/
        std::vector<scalar> a_rnd(N_sec);
        std::vector<scalar> b_rnd(N_sec);
        std::vector<scalar> dalpha_rnd(N_sec);
        std::vector<scalar> offset_rnd_noise_y(N_sec);
        std::vector<scalar> offset_rnd_noise_z(N_sec);
        std::vector<scalar> diameter_spiral_rnd(N_sec);
        std::vector<scalar> pitch_spiral_rnd(N_sec);

        std::mt19937 gen(config.random_seed); // Use section.random_seed for reproducibility
        if (section.cross_section_build_type == HoleCrossSectionBuildType::RANDOM_FIELD) {
            a_rnd = stationary_gaussian_field_mean_first(N_sec, section.a_mean, section.std_pr_radial * section.a_mean,
                                                         section.cor_radial, s_vals_sec, gen);
            if (section.surface_type == HoleSurfaceType::ELLIPTICAL) {
                b_rnd = stationary_gaussian_field_mean_first(
                    N_sec, section.b_mean, section.std_pr_radial * section.b_mean, section.cor_radial, s_vals_sec, gen);
                dalpha_rnd = stationary_gaussian_field_mean_first(N_sec, 0.0, section.std_pr_dalpha * section.dalpha,
                                                                  section.cor_dalpha, s_vals_sec, gen);
            }

            if (section.surface_type != HoleSurfaceType::CIRCULAR &&
                section.surface_type != HoleSurfaceType::ELLIPTICAL) {
                THROW_RUNTIME_ERROR("Invalid hole surface type");
            }
        }

        if (section.offset_build_type == HoleOffsetBuildType::RANDOM_FIELD) {
            // Generate random field for offset
            diameter_spiral_rnd = stationary_gaussian_field_mean_first(
                N_sec, section.diameter_spiral, section.std_pr_diameter_spiral * section.diameter_spiral,
                section.cor_diameter_spiral, s_vals_sec, gen);
            pitch_spiral_rnd = stationary_gaussian_field_mean_first(
                N_sec, section.pitch, section.std_pr_pitch * section.pitch, section.cor_pitch, s_vals_sec, gen);
            offset_rnd_noise_y = stationary_gaussian_field_mean_first(
                N_sec, 0.0, section.std_pr_off_noise * section.diameter_spiral, section.cor_off_noise, s_vals_sec, gen);
            offset_rnd_noise_z = stationary_gaussian_field_mean_first(
                N_sec, 0.0, section.std_pr_off_noise * section.diameter_spiral, section.cor_off_noise, s_vals_sec, gen);
        }

        /*================================================================================
        For spiral continuity: compute the phase offset at the start of this section
        ==================================================================================*/
        scalar spiral_x_start = 0.0;
        scalar spiral_y_start = 0.0;

        // Determine diameter and pitch to use at the section start (use per-node randomized values
        // if RANDOM_FIELD so phase continuity uses the same parameters as the first produced node).
        scalar D_spiral_start = std::max(section.diameter_spiral, SMALL_SCALAR);
        scalar P_spiral_start = std::max(section.pitch, SMALL_SCALAR);
        if (section.offset_build_type == HoleOffsetBuildType::RANDOM_FIELD && !diameter_spiral_rnd.empty() &&
            !pitch_spiral_rnd.empty()) {
            D_spiral_start = std::min(diameter_spiral_rnd[0], section.diameter_drift);
            P_spiral_start = std::max(pitch_spiral_rnd[0], SMALL_SCALAR);
        }

        // Compute starting phase at section start and the corresponding start-coordinates,
        // then shift by last_offset to enforce continuity.
        scalar s_start = start_depth;
        scalar phi_start = 2 * M_PI * s_start / P_spiral_start;
        spiral_x_start = (D_spiral_start / 2.0) * sin(phi_start);
        spiral_y_start = (D_spiral_start / 2.0) * cos(phi_start);
        Vec2 spiral_phase_offset = {last_offset.x() - spiral_x_start, last_offset.y() - spiral_y_start};

        /*================================================================================
         Loop through nodes in this section
        ==================================================================================*/
        scalar phi = phi_start; // start phase consistent with previous section
        Vec2 co_i;
        for (uint j = 0; j < N_sec; ++j) {
            uint i = idxs[j];

            /*================================================================================
             Set cross-section parameters
            ==================================================================================*/
            if (section.cross_section_build_type == HoleCrossSectionBuildType::CONSTANT) {
                if (section.surface_type == HoleSurfaceType::CIRCULAR) {
                    a[i] = section.D / 2.0;
                    b[i] = a[i];
                    alpha[i] = 0;
                } else if (section.surface_type == HoleSurfaceType::ELLIPTICAL) {
                    a[i] = section.a_mean;
                    b[i] = section.b_mean;
                    alpha[i] = section.dalpha * s[i];
                } else {
                    THROW_RUNTIME_ERROR("Invalid hole surface type");
                }
            } else if (section.cross_section_build_type == HoleCrossSectionBuildType::RANDOM_FIELD) {

                if (section.surface_type == HoleSurfaceType::CIRCULAR) {
                    a[i] = std::max(a_rnd[j], section.D / 2.0);
                    b[i] = a[i];
                    alpha[i] = 0; // Circular holes have no angle
                } else if (section.surface_type == HoleSurfaceType::ELLIPTICAL) {
                    a[i] = std::max(a_rnd[j], section.D / 2.0);
                    b[i] = std::max(b_rnd[j], section.D / 2.0);
                    static scalar alpha_cum = 0.0;
                    if (j == 0)
                        alpha_cum = 0.0; // Reset at start of section if needed
                    alpha_cum += dalpha_rnd[j] * (j == 0 ? 0.0 : (s[idxs[j]] - s[idxs[j - 1]]));
                    alpha[i] = alpha_cum;
                } else {
                    THROW_RUNTIME_ERROR("Invalid hole surface type");
                }
            } else {
                THROW_RUNTIME_ERROR("Invalid hole cross-section build type");
            }

            /*================================================================================
             Set offset-vector
            ==================================================================================*/
            // Set spiral parameters based on offset build type ---
            scalar D_spiral, P_spiral, noise_y, noise_z;
            if (section.offset_build_type == HoleOffsetBuildType::NONE) {
                D_spiral = 0.0;
                P_spiral = 1.0;
                noise_y = 0.0;
                noise_z = 0.0;
            } else if (section.offset_build_type == HoleOffsetBuildType::ANALYTICAL_SPIRAL) {
                D_spiral = std::max(section.diameter_spiral, SMALL_SCALAR);
                P_spiral = std::max(section.pitch, SMALL_SCALAR);
                noise_y = 0.0;
                noise_z = 0.0;
            } else if (section.offset_build_type == HoleOffsetBuildType::RANDOM_FIELD) {
                // Limit max spiral diameter based on the drift
                D_spiral = std::max(std::min(diameter_spiral_rnd[j], section.diameter_drift), SMALL_SCALAR);
                P_spiral = std::max(pitch_spiral_rnd[j], SMALL_SCALAR);
                noise_y = offset_rnd_noise_y[j];
                noise_z = offset_rnd_noise_z[j];

                // ---- Curvature-based pitch clipping ----
                const scalar max_curvature = section.max_curvature;

                scalar r = D_spiral / 2.0;
                scalar val = r / max_curvature - r * r;
                if (val > 0.0) {
                    scalar min_pitch_for_curv = 2.0 * M_PI * std::sqrt(val);
                    P_spiral = std::max(P_spiral, min_pitch_for_curv);
                } else {
                    // If val <= 0, set pitch to a small positive value to avoid divide by zero
                    P_spiral = std::max(P_spiral, SMALL_SCALAR);
                }
            } else {
                THROW_RUNTIME_ERROR("Invalid hole offset build type");
            }

            scalar dphi = 2 * M_PI * (j == 0 ? 0 : (s[idxs[j]] - s[idxs[j - 1]]) / P_spiral);
            phi += dphi;
            co_i.x() = (D_spiral / 2) * std::sin(phi) + noise_y;
            co_i.y() = (D_spiral / 2) * std::cos(phi) + noise_z;

            // For continuity: add constant phase offset
            co_i += spiral_phase_offset;
            co[i] = co_i;
        }
        last_offset = co_i; // Update last offset for next section
    }

    // Determine hole type based on sections
    bool has_elliptical = false;
    for (const auto &section : hole_sections) {
        if (section.surface_type == HoleSurfaceType::ELLIPTICAL) {
            has_elliptical = true;
            break;
        }
    }
    if (has_elliptical) {
        hole.type = HoleSurfaceType::ELLIPTICAL;
    } else {
        hole.type = HoleSurfaceType::CIRCULAR;
    }
}

void calculate_curvature_from_trajectory(vector<Vec2> &curvatures, const vector<scalar> &vec_s, const vector<Mat3> &R,
                                         const vector<Vec3> &nodes, const vector<Vec3> &midnodes) {
    assert(nodes[0].isApprox(Vec3{0, 0, 0}));

    uint N_hole = nodes.size();

    Vec3 t;
    Vec3 n;
    Vec3 b;
    scalar kappa_y, kappa_z;
    scalar kappa = 0.0;

    // u & v spans the plane of the nodes 0, 1/2, 1
    assert(nodes[0].isZero());
    Vec3 u = (midnodes[1] - nodes[0]).normalized();
    Vec3 v = (nodes[1] - midnodes[1]).normalized();
    scalar ds = vec_s[1] - vec_s[0];

    t = R[0].col(0).normalized();
    // Curvature = ||dt/ds|| (forward difference):
    kappa = ((v - t) / (ds / 2.0)).norm();
    b = v.cross(t).normalized();

    // n : normal unit vector, pointing to the "center" of the curvature
    n = t.cross(b).normalized();

    // Retrieve the two normal vectors in the Bishop frame
    Vec3 e2 = R[0].col(1);
    Vec3 e3 = R[0].col(2);

    kappa_y = -kappa * n.dot(e3); // Curvature about e2 (y-axis)
    kappa_z = -kappa * n.dot(e2); // Curvature about e3 (z-axis)

    // First node
    curvatures[0] = Vec2{kappa_y, kappa_z};

    for (uint i = 1; i < N_hole - 1; i++) {
        ds = vec_s[i + 1] - vec_s[i];

        // u & v spans the plane of the nodes i-1/2, i, i+1/2
        u = (nodes[i] - midnodes[i]).normalized();
        v = (midnodes[i + 1] - nodes[i]).normalized();

        // tangent = dr/ds/||dr/ds||
        t = (midnodes[i + 1] - midnodes[i]).normalized();

        // Curvature = ||dt/ds|| (central difference):
        kappa = ((v - u) / (ds / 2.0)).norm();

        // b(s) : The binormal: normal of plane spanned by u and v
        b = v.cross(u).normalized();

        // n : normal unit vector, pointing to the "center" of the curvature
        n = t.cross(b).normalized();

        // Retrieve the two normal vectors in the Bishop frame
        e2 = R[i].col(1);
        e3 = R[i].col(2);

        // Calculate the projections of the curvature
        // (WHY NEGATIVE SIGN, differente sign convention for mechanics vs. math acording to AI)
        kappa_y = -kappa * n.dot(e3); // Curvature about e2 (y-axis)
        kappa_z = -kappa * n.dot(e2); // Curvature about e3 (z-axis)

        curvatures[i] = Vec2{kappa_y, kappa_z};
        // assert(is_close(kappa, sqrt(kappa_y * kappa_y + kappa_z * kappa_z), 1.0E-5));
    }

    t = R[N_hole - 1].col(0).normalized();
    v = (nodes[N_hole - 2] - midnodes[N_hole - 2]).normalized();
    ds = vec_s[N_hole - 1] - vec_s[N_hole - 2];

    // Curvature = ||dt/ds|| (backwards difference):
    kappa = ((t - v) / ds).norm();

    b = t.cross(v).normalized();

    // n : normal unit vector, pointing in the direction of curvature
    n = t.cross(b).normalized();

    // Retrieve the two normal vectors in the Bishop frame
    e2 = R[N_hole - 1].col(1);
    e3 = R[N_hole - 1].col(2);

    kappa_y = -kappa * n.dot(e3); // Curvature about e2 (y-axis)
    kappa_z = -kappa * n.dot(e2); // Curvature about e3 (z-axis)

    // Assume same as last for now
    curvatures[N_hole - 1] = Vec2{kappa_y, kappa_z};
}

void calculate_gravity(vector<Vec3> &g_local, const ArrayView<Quaternion> q, const uint N_hole) {
    const Vec3 g_inertial_frame{0, 0, -STANDARD_GRAVITY};
    for (uint i = 0; i < N_hole; i++) {
        assert(is_close(q[i].norm_sqr(), 1.0));
        const Vec3 g_loc = q[i].rotate_vector_reversed(g_inertial_frame);
        assert(is_close(g_loc.norm(), STANDARD_GRAVITY));
        g_local[i] = g_loc;
    }
}

void save_hole_csv(const Config &config, const Hole &hole, ArenaBump &arena_h) {
    using namespace std;

    // const string filename = config.get_current_output_subdir() + "hole.csv";
    const path file_path = config.output_dir / "hole.csv";
    ofstream ost{file_path.c_str()};
    if (!ost) {
        throw runtime_error("Failed to open hole geometry file: " + file_path.string() + "\n");
    }

    string surface_type_str;
    for (const auto &pair : hole_surface_type_from_string) {
        if (pair.second == hole.type) {
            surface_type_str = pair.first;
            break;
        }
    }

    /*--------------------------------------------------------------------
    Create header
    --------------------------------------------------------------------*/
    ost << "N_bh, L_bh, surface_type\n"
        << hole.N_hole << ", " << hole.get_L(arena_h.buf) << ", " << surface_type_str << "\n";
    ost << "s, x, y, z , R_00, R_10, R_20, R_01, R_11, R_21, R_02, R_12, R_22, kappa_y, kappa_z, g_x, g_y, g_z";

    switch (hole.type) {
    case HoleSurfaceType::CIRCULAR:
        ost << ", r, co_y, co_z\n";
        break;
    case HoleSurfaceType::ELLIPTICAL:
        ost << ", a, b, theta, co_y, co_z\n";
        break;
    default:
        assert(false);
    }

    ArrayView<scalar> s = hole.get_field<HoleField::s>(arena_h.buf);
    ArrayView<Vec3> x = hole.get_field<HoleField::x>(arena_h.buf);
    ArrayView<Quaternion> q = hole.get_field<HoleField::q>(arena_h.buf);
    ArrayView<Vec3> g = hole.get_field<HoleField::g>(arena_h.buf);
    ArrayView<Vec2> kappa = hole.get_field<HoleField::kappa>(arena_h.buf);
    ArrayView<scalar> a = hole.get_field<HoleField::a>(arena_h.buf);
    ArrayView<scalar> b = hole.get_field<HoleField::b>(arena_h.buf);
    ArrayView<scalar> alpha = hole.get_field<HoleField::alpha>(arena_h.buf);
    ArrayView<Vec2> co = hole.get_field<HoleField::co>(arena_h.buf);

    for (uint i = 0; i < hole.N_hole + 2; i++) {
        /*--------------------------------------------------------------------
        Write the hole profile
        --------------------------------------------------------------------*/
        const Mat3 &R_i = q[i].to_matrix();
        const Vec3 &g_i = g[i];
        const Vec2 &curvatures = kappa[i];
        const scalar &kappa_y = curvatures.x();
        const scalar &kappa_z = curvatures.y();

        ost << s[i] << ", " << x[i][0] << ", " << x[i][1] << ", " << x[i][2] << ", ";
        ost << R_i(0, 0) << ", " << R_i(1, 0) << ", " << R_i(2, 0) << ", " << R_i(0, 1) << ", " << R_i(1, 1) << ", "
            << R_i(2, 1) << ", " << R_i(0, 2) << ", " << R_i(1, 2) << ", " << R_i(2, 2) << ", " << kappa_y << ", "
            << kappa_z << ", " << g_i.x() << ", " << g_i.y() << ", " << g_i.z() << ", ";

        /*--------------------------------------------------------------------
        Write surface details
        --------------------------------------------------------------------*/

        switch (hole.type) {
        case HoleSurfaceType::CIRCULAR:
            ost << a[i] << ", " << co[i].x() << ", " << co[i].y() << "\n";
            break;
        case HoleSurfaceType::ELLIPTICAL:
            ost << a[i] << ", " << b[i] << ", " << alpha[i] << ", " << co[i].x() << ", " << co[i].y() << "\n";
            break;
        default:
            assert(false);
        }
    }
}

void smooth_curvatures(std::vector<Vec2> &curvatures, uint window_size) {
    std::vector<Vec2> smoothed(curvatures.size());
    uint half_window = window_size / 2;
    for (size_t i = 0; i < curvatures.size(); ++i) {
        Vec2 sum = Vec2::Zero();
        int count = 0;
        for (int j = std::max<int>(0, i - half_window); j <= std::min<int>(curvatures.size() - 1, i + half_window);
             ++j) {
            sum += curvatures[j];
            ++count;
        }
        smoothed[i] = sum / count;
    }
    curvatures = std::move(smoothed);
}