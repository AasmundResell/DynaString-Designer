#pragma once
#include "bcs.hpp"
#include "misc/config.hpp"
#include "misc/includes.hpp"
#include "pipe-solver-curvlin/numerics/gauss-legendre-utils.hpp"

#define for_each_node_cpu(i) for (i = top_node; i < N; i++)
#define for_each_node_gpu(i) if (i >= top_node && i < N)
#define for_each_element_cpu_even(ie)                                                                                  \
    for (ie = top_node; ie < N - 1; ie++)                                                                              \
        if (ie % 2 == 0)
#define for_each_element_cpu_odd(ie)                                                                                   \
    for (ie = top_node; ie < N - 1; ie++)                                                                              \
        if (ie % 2 == 1)

DEVICE_FUNC inline void calc_contact_force_3D(Vec3 &fc, Vec3 &mc, scalar mu_s, scalar mu_k, scalar v_cs, scalar K_c,
                                              scalar C_c, scalar r_pipe, scalar d_wall, const Vec3 &n, const Vec3 &v,
                                              const Vec3 &omega) {

    // Calculate the contact force and moment in full 3D -> Does not work as of now

    /*--------------------------------------------------------------------
    Penetration distance
    --------------------------------------------------------------------*/
    const scalar delta = r_pipe - d_wall;

    /*--------------------------------------------------------------------
    Heaviside function, which sets the contact force to zero in case
    of no penetration.
    --------------------------------------------------------------------*/
    const scalar H = heaviside(delta);

    /*--------------------------------------------------------------------
    Contact velocity: v = v + omega X (r_pipe * n)
    --------------------------------------------------------------------*/
    const Vec3 vc = (v + omega.cross(r_pipe * n));

    /*--------------------------------------------------------------------
    Normal component of contact velocity
    --------------------------------------------------------------------*/
    const scalar vcn = vc.dot(n);

    /*--------------------------------------------------------------------
    3D slip (tangential) velocity vector: v_slip = v_contact - v_normal
    ------------------------------------------------------------------*/
    const Vec3 v_slip = vc - n * vcn;

    /*--------------------------------------------------------------------
    Coloumb friction model
    ------------------------------------------------------------------*/
    const scalar mu = mu_k + (mu_s - mu_k) * exp(-v_slip.norm() / (v_cs + SMALL_SCALAR));

    ///*--------------------------------------------------------------------
    // Normal and tangential force per unit length
    //--------------------------------------------------------------------*/
    const scalar fn = (K_c * delta + C_c * vcn) * H;
    const scalar ft = mu * fn;
    ///*--------------------------------------------------------------------
    // The 3D tangential vector t = v_slip/||v_slip||.
    // Total force per unit length:  F = Fn * n + Ft * t
    //--------------------------------------------------------------------*/
    const Vec3 t_slip = v_slip.normalized();
    fc = -n * fn - t_slip * ft;

    ///*--------------------------------------------------------------------
    // Total moment per unit length: M = r X F
    //--------------------------------------------------------------------*/
    mc = r_pipe * n.cross(fc);
}

DEVICE_FUNC inline void apply_contact_friction_softstring(Vec2 &f_t,   // out: tangential friction force
                                                          Vec3 &p_ref, // inout: stick anchor (surface point)
                                                          /* IN  */ scalar F_normal, // scalar normal force
                                                          /* IN  */ Vec2 x_t,        // Tangential position
                                                          /* IN  */ Vec2 v_t,        // Tangential velocity
                                                          /* IN  */ scalar mu_s,     // static friction coeff
                                                          /* IN  */ scalar mu_k,     // kinetic friction coeff
                                                          /* IN  */ scalar K_t,      // tangential stiffness (2D)
                                                          /* IN  */ scalar C_t,      // tangential damping   (2D)
                                                          /* IN  */ scalar v_cs      // Stribeck characteristic velocity
) {
    scalar v_mag = v_t.norm();
    Vec2 d_slip = x_t - Vec2{p_ref.x(), p_ref.y()};

    // 2D Kelvin–Voigt trial stick force:
    Vec2 F_trial = -K_t * d_slip - C_t * v_t;
    scalar F_trial_mag = F_trial.norm();

    scalar F_static_lim = mu_s * F_normal;

    constexpr scalar v_crit = 0.05; //Could potentially be an input, same as Ambrus 2024 article

    // --- STICK region ----------------------------------------------
    if (v_mag <= v_crit) {

        scalar F_crit = min(F_static_lim, F_trial_mag);
        f_t = F_trial * (F_crit / (F_trial_mag + SMALL_SCALAR));

    } else {

        // --- SLIP region -----------------------------------------------
        // Reset anchor for next stick episode:
        p_ref.x() = x_t.x(); // Axial
        p_ref.y() = x_t.y(); // Torsional

        const scalar mu = mu_k + (mu_s - mu_k) * exp(-v_mag / v_cs);

        // Pure Coulomb kinetic friction (direction = sliding direction)
        f_t = -(mu * F_normal) * v_t / v_mag;
    }
}

DEVICE_FUNC inline void calc_bow_spring_centralizer_forces_vector(Vec3 &f_bs, Vec3 &m_bs, const Vec2 &u_c, scalar r_bs,
                                                                  scalar r_h, uint N_b, scalar K_bs, scalar C_bs,
                                                                  scalar p_bs,
                                                                  // Friction parameters
                                                                  scalar mu_s, scalar mu_k, scalar d_c,
                                                                  // Kinematics
                                                                  const Vec3 &v, const Vec3 &omega) {
    f_bs = Vec3{0.0, 0.0, 0.0};
    m_bs = Vec3{0.0, 0.0, 0.0};

    // Length from hole center to centralizer center
    scalar r_c = u_c.norm();

    for (uint i = 0; i < N_b; ++i) {
        scalar alpha = (2.0 * M_PI * i) / N_b;
        Vec2 blade_dir_2d{cos(alpha), sin(alpha)};
        Vec3 blade_dir_3d{0.0, blade_dir_2d[0], blade_dir_2d[1]};

        // Vector from centralizer center to blade tip
        Vec2 u_bi = r_bs * blade_dir_2d;

        // Quadratic formula for intersection of blade line and hole circle
        scalar a = r_bs * r_bs;
        scalar b = u_bi.dot(u_c);
        scalar c = r_c * r_c - r_h * r_h;
        scalar d = sqrt(b * b - a * c);

        scalar l;
        scalar l1 = (-b + d) / a;
        scalar l2 = (-b - d) / a;

        if (l1 >= 0 && l1 <= 1) {
            l = l1;
        } else if (l2 >= 0 && l2 <= 1) {
            l = l2;
        } else {
            continue; // No contact for this blade
        }

        assert(l >= 0 && l <= 1); // Intersection parameter must be in [0,1]

        // Vector from hole center to blade contact point
        Vec2 u_bi_h = u_c + l * u_bi;

        // Vector from centralizer center to blade contact point
        Vec2 u_bi_c = u_bi_h - u_c;

        // Penetration distance
        scalar r_bi_c = u_bi_c.norm();
        scalar delta = r_bs - r_bi_c;

        // Relative contact velocity at contact point
        Vec3 v_c = v + omega.cross(r_bi_c * blade_dir_3d);

        scalar v_radial = v_c.dot(blade_dir_3d);
        scalar f_mag = K_bs * pow(delta, p_bs) + C_bs * v_radial;

        // Axial unit vector and contact velocity
        Vec3 e_x{1.0, 0.0, 0.0};
        scalar v_x = v_c.x();

        // Tangential unit vector and contact velocity
        Vec3 e_theta{0.0, -blade_dir_2d[1], blade_dir_2d[0]};
        scalar v_theta = v_c.dot(e_theta);

        // Calculate friction coefficients for each direction
        scalar mu_z = mu_k + (mu_s - mu_k) * exp(-d_c * fabs(v_x));
        scalar mu_theta = mu_k + (mu_s - mu_k) * exp(-d_c * fabs(v_theta));

        // Friction forces
        Vec3 f_fric_axial = mu_z * f_mag * e_x * sign(v_x);
        Vec3 f_fric_tangential = mu_theta * f_mag * e_theta * sign(v_theta);

        // Total force for this blade
        Vec3 f_i = f_mag * blade_dir_3d + f_fric_axial + f_fric_tangential;

        // Moment arm (from pipe center to blade tip)
        Vec3 r_arm_i = r_bs * blade_dir_3d;

        // Accumulate (negative gives force from wall to spring)
        f_bs -= f_i;
        m_bs -= r_arm_i.cross(f_i);
    }
}

DEVICE_FUNC inline scalar find_ellipse_intersection_parameter_newton(scalar e0, scalar e1, scalar y0, scalar y1) {

    /*
    Implementation and notation following:
    https://www.geometrictools.com/Documentation/DistancePointEllipseEllipsoid.pdf
    */

    assert(e0 >= e1 && e1 > 0);
    assert(y0 > 0 && y1 > 0);

    scalar t = 0.0;
    scalar t0 = -e1 * e1 + e1 * y1;

    constexpr uint maxIter = 4;

    // Newton-Raphson iteration
    for (uint i = 0; i < maxIter; i++) {
        scalar ratioY0 = e0 * y0 / (t0 + e0 * e0);
        scalar ratioY1 = e1 * y1 / (t0 + e1 * e1);

        scalar f = ratioY0 * ratioY0 + ratioY1 * ratioY1 - 1;
        scalar df = -2 * powi(ratioY0, 3) / (e0 * y0) - 2 * powi(ratioY1, 3) / (e1 * y1);
        t = t0 - f / df;

        t0 = t;
    }

    return t;
}

DEVICE_FUNC inline scalar find_ellipse_intersection_parameter_bisection(scalar r0, scalar z0, scalar z1,
                                                                        const uint N_iter_contact) {

    /*
    Implementation and notation following, sec 2.9: Robust bisection method
    https://www.geometrictools.com/Documentation/DistancePointEllipseEllipsoid.pdf
    */

    scalar n0 = r0 * z0;
    scalar s0 = z1 - 1;

    scalar s1 = sqrt((r0 * z0) * (r0 * z0) + z1 * z1) - 1;
    scalar s = 0;

    for (uint i = 0; i < N_iter_contact; i++) {
        s = (s0 + s1) / 2;

        scalar ratio0 = n0 / (s + r0);
        scalar ratio1 = z1 / (s + 1);
        scalar g = ratio0 * ratio0 + ratio1 * ratio1 - 1;
        if (g > 0) {
            s0 = s;
        } else if (g <= 0) {
            s1 = s;
        }
    }
    return s;
}

DEVICE_FUNC inline Vec2 get_ellipse_quadrant_vector(scalar e0, scalar e1, scalar y0, scalar y1,
                                                    const uint N_iter_contact) {
    /*
    Returns the vector from the pipe center (y0, y1) to the ellipse quadrant
    where the point (x0, x1) is located.

    Implementation and notation following:
    https://www.geometrictools.com/Documentation/DistancePointEllipseEllipsoid.pdf
    */

    bool swapped = false;
    if (e0 < e1) {
        scalar temp = e0;
        e0 = e1;
        e1 = temp;
        scalar temp2 = y0;
        y0 = y1;
        y1 = temp2;
        swapped = true;
    }
    assert(e0 >= e1 && e1 > 0);
    assert(y0 >= 0 && y1 >= 0);

    Vec2 wall_vec;
    constexpr scalar BC_SMALL_SCALAR = 1e-3;
    if (y1 > BC_SMALL_SCALAR) {
        if (y0 > BC_SMALL_SCALAR) {
            // Compute F(t) on [e1 * e1 , + infinity);
            scalar r0 = e0 * e0 / (e1 * e1);
            scalar z0 = y0 / e0;
            scalar z1 = y1 / e1;

            scalar s_bar = find_ellipse_intersection_parameter_bisection(r0, z0, z1, N_iter_contact);
            scalar x0 = r0 * y0 / (s_bar + r0);
            scalar x1 = y1 / (s_bar + 1);

            wall_vec = Vec2{x0, x1} - Vec2{y0, y1};
        } else {
            // y0 == 0
            wall_vec = Vec2{0.0, e1} - Vec2{0.0, y1};
        }
    } else {
        // y1 == 0
        if (y0 < (e0 * e0 - e1 * e1) / e0) {
            scalar x0 = e0 * e0 * y0 / (e0 * e0 - e1 * e1);
            scalar x1 = e1 * sqrt(1.0 - (x0 / e0) * (x0 / e0));
            wall_vec = Vec2{x0, x1} - Vec2{y0, 0.0};
        } else {
            wall_vec = Vec2{e0, 0.0} - Vec2{y0, 0.0};
        }
    }

    if (swapped) {
        wall_vec = Vec2{wall_vec[1], wall_vec[0]};
    }

    return wall_vec;
}

// ...existing code...
DEVICE_FUNC inline Vec2 get_ellipse_quadrant_vector_new(scalar e0, scalar e1, scalar y0, scalar y1,
                                                        const uint N_iter_contact) {
    /*
    Returns the vector from the pipe center (y0, y1) to the ellipse quadrant
    where the point (x0, x1) is located.

    Implementation and notation following:
    https://www.geometrictools.com/Documentation/DistancePointEllipseEllipsoid.pdf
    */

    bool swapped = false;
    if (e0 < e1) {
        scalar temp = e0;
        e0 = e1;
        e1 = temp;
        scalar temp2 = y0;
        y0 = y1;
        y1 = temp2;
        swapped = true;
    }
    assert(e0 >= e1 && e1 > 0);
    assert(y0 >= 0 && y1 >= 0);

    Vec2 wall_vec;
    constexpr scalar BC_SMALL_SCALAR = 1e-3;
    if (y1 > BC_SMALL_SCALAR) {
        if (y0 > BC_SMALL_SCALAR) {
            // Compute F(t) on [e1 * e1 , + infinity);
            scalar r0 = e0 * e0 / (e1 * e1);
            scalar z0 = y0 / e0;
            scalar z1 = y1 / e1;

            scalar s_bar = find_ellipse_intersection_parameter_bisection(r0, z0, z1, N_iter_contact);
            scalar x0 = r0 * y0 / (s_bar + r0);
            scalar x1 = y1 / (s_bar + 1);

            wall_vec = Vec2{x0, x1} - Vec2{y0, y1};
        } else {
            // y0 == 0
            wall_vec = Vec2{0.0, e1} - Vec2{0.0, y1};
        }
    } else {
        // y1 == 0
        if (y0 < (e0 * e0 - e1 * e1) / e0) {
            scalar x0 = e0 * e0 * y0 / (e0 * e0 - e1 * e1);
            scalar x1 = e1 * sqrt(1.0 - (x0 / e0) * (x0 / e0));
            wall_vec = Vec2{x0, x1} - Vec2{y0, 0.0};
        } else {
            wall_vec = Vec2{e0, 0.0} - Vec2{y0, 0.0};
        }
    }

    if (swapped) {
        wall_vec = Vec2{wall_vec[1], wall_vec[0]};
    }

    // Robustness: Ensure wall_vec always points from (y0, y1) toward the ellipse boundary
    // If wall_vec points more toward the center than away, flip it
    Vec2 center_to_wall = wall_vec;
    Vec2 center_vec = Vec2{y0, y1};
    if (center_to_wall.dot(center_vec) < 0) {
        wall_vec = -wall_vec;
    }

    return wall_vec;
}

inline scalar von_mises_stress_from_strains(const Vec3 &gamma, const Vec3 &kappa, scalar E, scalar G, scalar R) {
    // Axial + bending normal stress at outer fiber
    scalar sigma_x = E * gamma.x() + E * R * sqrt(kappa.y() * kappa.y() + kappa.z() * kappa.z());

    // Shear stresses (with correction factor)
    scalar tau_xy = G * gamma.y();
    scalar tau_xz = G * gamma.z();

    // Torsional shear stress at outer fiber
    scalar tau_torsion = G * kappa.x() * R;

    // Von Mises stress
    scalar sigma_vm = sqrt(sigma_x * sigma_x + 3.0 * (tau_xy * tau_xy + tau_xz * tau_xz + tau_torsion * tau_torsion));
    return sigma_vm;
}

void calc_dt_and_inertia_scaling(Config &config, const Pipe &pipe, const byte *buf);

void calc_contact_stiffness_and_damping(Config &config, const uint N, const ArrayView<scalar> &mass, const byte *buf);

void create_mass_vectors(vector<scalar> &m, vector<Vec3> &J_u, Config &config, const Pipe &pipe, byte *buf);

void save_csv_header_file(const uint N, const Config &config, const ConfigDynamic &conf_dyn, const string output_subdir,
                          const BCsData &bcs, byte *buf);

void save_csv_curvlin_header(const Config &config, ofstream &ost);

void save_csv_global_header(const Config &config, ofstream &ost);

void save_csv_global_line(ofstream &ost, const Vec3 &X, const Vec3 &d, const Mat3 &U, const Vec3 &v,
                          const Vec3 &omega_u);

void save_csv_fluid_header(const Config &config, ofstream &ost);
