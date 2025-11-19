#pragma once
#include "pipe-solver/beam-common.hpp"
#include "pipe-solver/pipe-solver-curvlin/numerics/gauss-legendre-utils.hpp"

namespace curvlin {



DEVICE_FUNC inline void calc_contact_friction_nodewise_ax_tor(
    const uint i,
    const ConfigStatic &conf_stat,
    const Pipe  &pipe,
    const Hole   &hole,
    BeamData    &beam,
    byte        *buf)
{
    assert(conf_stat.contact_enabled);

    // --- Retrieve fields -------------------------------------------
    ArrayView<Vec3>  v         = beam.get_field<BeamField::v>(buf);
    ArrayView<Vec3>  omega     = beam.get_field<BeamField::omega>(buf);
    ArrayView<Vec3>  u         = beam.get_field<BeamField::u>(buf);
    ArrayView<Vec3>  theta     = beam.get_field<BeamField::theta>(buf);

    ArrayView<Vec3>  f_dyn     = beam.get_field<BeamField::f_dyn>(buf);
    ArrayView<Vec3>  m_dyn     = beam.get_field<BeamField::m_dyn>(buf);
    ArrayView<Vec3>  f_hyd     = beam.get_field<BeamField::f_hyd>(buf);
    ArrayView<Vec3>  p_ref     = beam.get_field<BeamField::p_ref>(buf);
    ArrayView<Vec2>  kappas    = hole.get_field<HoleField::kappa>(buf);
    ArrayView<Vec2>  offsets        = hole.get_field<HoleField::co>(buf);
    ArrayView<scalar> f_axial = beam.get_field<BeamField::f_axial>(buf);
     ArrayView<uint> i_pipe_to_ie_hole = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);
   
    ArrayView<scalar> rc       = pipe.get_field<PipeField::rc>(buf);

    // Contact radius index (as you had)
    ArrayView<uint> num_ic_to_ie = pipe.get_field<PipeField::num_ic_to_ie>(buf);
    uint ic = min(num_ic_to_ie[i], pipe.Nc - 1);
    const scalar r_o = rc[ic];                 // outer radius = d_o / 2

    // Curvilinear position (for your own use; kept as in your code)
    scalar s_i = pipe.calc_s(i, u[i].x(), buf);
    if (s_i <= 0) { // IMPORTANT: Removes top-node drag
        return;
    }

    // -----------------------------------------------------------------
    // 1) Normal force magnitude (use whatever you already computed)
    //    Here I just use current f_dyn as in your snippet.
    // -----------------------------------------------------------------
    uint i_hole = i_pipe_to_ie_hole[i];
    const Vec2 curvatures = hole.lerp_hole_property_from_pipe_node_position<Vec2>(kappas, s_i, i_hole, buf);
    const Vec2 co = hole.lerp_hole_property_from_pipe_node_position<Vec2>(offsets, s_i, i_hole, buf);
    scalar kappa_y = curvatures.x();
    scalar kappa_z = curvatures.y();

    //For contact normal force calculation
    scalar dS = pipe.dS_node_avg(i, buf);

    //Equivalent to Mitchell 2009, eq 5: 
    //https://onepetro.org/DC/article-abstract/24/01/62/191728/How-Good-Is-the-Torque-Drag-Model?redirectedFrom=PDF 
    scalar f_dyn_y = (f_axial[i] * kappa_y * dS + f_hyd[i].y());
    scalar f_dyn_z = (f_axial[i] * kappa_z * dS + f_hyd[i].z());
    scalar F_normal_softstring = sqrt(f_dyn_y * f_dyn_y + f_dyn_z * f_dyn_z);

    
    scalar F_normal_raw = f_dyn[i].norm();
    // Clamp raw normal force using soft-string estimate
    constexpr scalar max_factor = 5.0;
    const scalar F_normal_fric = min(F_normal_raw, max_factor * F_normal_softstring);
    if (F_normal_fric <= SMALL_SCALAR) {
        return; // no side force => no contact friction worth applying
    }
    
    // -----------------------------------------------------------------
    // 2) Geometry with hole offset
    //    Use pipe center relative to *hole center* for normal/tangent
    // -----------------------------------------------------------------
    // Co is the hole center offset in (y,z) at this s
    const scalar co_y_hole = co.x();  
    const scalar co_z_hole = co.y();

    // Pipe center in hole-centered coordinates:
    const scalar uy_rel = u[i].y() - co_y_hole;
    const scalar uz_rel = u[i].z() - co_z_hole;

    const scalar r_c = sqrt(uy_rel * uy_rel + uz_rel * uz_rel);
    if (r_c <= SMALL_SCALAR) {
        return;
    }
    
    // Radial unit vector from hole center to pipe center
    const scalar n_y = uy_rel / r_c;
    const scalar n_z = uz_rel / r_c;

    Vec3 n{0.0, n_y, n_z};

    // Contact point velocity
    const Vec3 r_contact = rc[ic] * n;
    const Vec3 vc = v[i] + omega[i].cross(r_contact);

    const scalar t_y = -n_z;
    const scalar t_z = n_y;
    Vec3 t{0.0, t_y, t_z};
    
    scalar v_tangential = vc.dot(t);

    Vec2 v_t{ vc.x(), v_tangential };

    // -----------------------------------------------------------------
    // 4) 2D "position" in axial/tangential space for Kelvin–Voigt spring
    //    Consistent with v_t definition: x_t = {u_ax, phi*r_c + theta*r_o}
    // -----------------------------------------------------------------
    const scalar x_axial   = u[i].x();
    const scalar x_tan = sqrt(uy_rel * uy_rel + uz_rel * uz_rel) + theta[i].x() * r_o;
    Vec2 x_t{ x_axial, x_tan };

    // -----------------------------------------------------------------
    // 5) Call your existing 2D soft-string friction law
    // -----------------------------------------------------------------
    Vec2 f_t{0.0, 0.0};

    apply_contact_friction_softstring(
        /* INOUT */ f_t,
        /* INOUT */ p_ref[i],
        /* IN  */   F_normal_fric,
                    x_t,
                    v_t,
        /* IN  */   conf_stat.mu_static,
                    conf_stat.mu_kinetic,
                    conf_stat.K_tangent,
                    conf_stat.C_tangent,
                    conf_stat.critical_stribeck_velocity
    );

    // -----------------------------------------------------------------
    // 6) Project 2D friction back to 3D
    // -----------------------------------------------------------------

    // Tangential unit vector t_hat in (y,z): [-n_z, n_y]

    f_dyn[i].x() += f_t.x();        // axial friction
    f_dyn[i].y() += f_t.y() * t_y;  // lateral tangent Y
    f_dyn[i].z() += f_t.y() * t_z;  // lateral tangent Z

    m_dyn[i].x() += r_o * f_t.y();  // torsional moment

}



DEVICE_FUNC inline void calc_contact_friction_soft_string(
    const uint i,
    const ConfigStatic &conf_stat,
    const Hole  &hole,
    const Pipe  &pipe,
    BeamData    &beam,
    byte        *buf)
{
    assert(conf_stat.contact_enabled);

    // --- Retrieve fields -------------------------------------------
    ArrayView<Vec3>  v         = beam.get_field<BeamField::v>(buf);
    ArrayView<Vec3>  omega     = beam.get_field<BeamField::omega>(buf);
    ArrayView<Vec3>  u         = beam.get_field<BeamField::u>(buf);
    ArrayView<Vec3>  theta     = beam.get_field<BeamField::theta>(buf);
    
    ArrayView<Vec2>  kappas    = hole.get_field<HoleField::kappa>(buf);
    ArrayView<scalar> f_axial = beam.get_field<BeamField::f_axial>(buf);
    ArrayView<Vec3> f_dyn = beam.get_field<BeamField::f_dyn>(buf);
    ArrayView<Vec3> m_dyn = beam.get_field<BeamField::m_dyn>(buf);
    ArrayView<Vec3> f_hyd = beam.get_field<BeamField::f_hyd>(buf);
    ArrayView<Vec3> f_int = beam.get_field < BeamField::f_int> (buf);
    ArrayView<uint> i_pipe_to_ie_hole = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);
    ArrayView<Vec3> p_ref = beam.get_field<BeamField::p_ref>(buf);
    ArrayView<uint> num_ic_to_ie = pipe.get_field<PipeField::num_ic_to_ie>(buf);
    uint ic = min(num_ic_to_ie[i], pipe.Nc - 1); // Contact point index corresponding to node i

    
    ArrayView<scalar> rc = pipe.get_field<PipeField::rc>(buf);
    // Compute curvature in Bishop frame ---------------------
    scalar s_i = pipe.calc_s(i, u[i].x(), buf);
    if (s_i <= 0) { // IMPORTANT: Removes top-node drag
        return;
    }
    uint i_hole = i_pipe_to_ie_hole[i];
    const Vec2 curvatures = hole.lerp_hole_property_from_pipe_node_position<Vec2>(kappas, s_i, i_hole, buf);
    scalar kappa_y = curvatures.x();
    scalar kappa_z = curvatures.y();

    //For contact normal force calculation
    scalar dS = pipe.dS_node_avg(i, buf);

    //Equivalent to Mitchell 2009, eq 5: 
    //https://onepetro.org/DC/article-abstract/24/01/62/191728/How-Good-Is-the-Torque-Drag-Model?redirectedFrom=PDF 
    f_dyn[i].y() = (f_axial[i] * kappa_y * dS + f_hyd[i].y());
    f_dyn[i].z() = (f_axial[i] * kappa_z * dS + f_hyd[i].z());

    scalar F_normal = sqrt(f_dyn[i].y() * f_dyn[i].y() + f_dyn[i].z() * f_dyn[i].z());

    // --- Velocities in 2D axial/torsional space ---------------------
    scalar V_axial = v[i].x();
    scalar Omega   = omega[i].x();        // twist about pipe axis
    scalar radius  = rc[ic];               // contact radius

    // --- 2D slip velocity ------------------------------------------
    scalar v_tan = radius * Omega;
    Vec2 v_t{V_axial, v_tan};
    
    Vec2 x_t{ u[i].x(), radius * theta[i].x()};
    Vec2 f_t{0,0}; //Contains friction force in axial and tangential direction

    apply_contact_friction_softstring(
        /* INOUT */ f_t,
        /* INOUT */ p_ref[i],
        /* IN  */ F_normal,
                  x_t,
                  v_t,   
        /* IN  */ conf_stat.mu_static,
                  conf_stat.mu_kinetic,
                  conf_stat.K_tangent,
                  conf_stat.C_tangent,
                  conf_stat.critical_stribeck_velocity
    );

    // --- Write axial friction force and torsional moment ------------
    f_dyn[i].x() = f_t.x();
    m_dyn[i].x() = radius * f_t.y(); 
}


DEVICE_FUNC inline void calc_contact_forces_circular(uint ie, const ConfigStatic &conf_stat, const Hole &hole,
                                                     const Pipe &pipe, BeamData &beam, byte *buf) {
    assert(conf_stat.contact_enabled);
    assert(hole.type == HoleSurfaceType::CIRCULAR);

    // FEM parameters
    const scalar dx = pipe.dS_e(ie, buf);
    const scalar Ap = pipe.Ap_e(ie, buf);
    const scalar Ip = pipe.Ip_e(ie, buf);
    const scalar E = conf_stat.E;
    const scalar G = conf_stat.get_G();
    const scalar k = pipe.k_s(ie, buf);
    const scalar alpha = 12.0 * E * Ip / (k * G * Ap * dx * dx);

    // Contact parameters
    const scalar K_c = conf_stat.K_normal;
    const scalar C_c = conf_stat.C_normal;

    ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);
    ArrayView<Vec3> v = beam.get_field<BeamField::v>(buf);
    ArrayView<Vec3> theta = beam.get_field<BeamField::theta>(buf);
    ArrayView<Vec3> omega = beam.get_field<BeamField::omega>(buf);
    ArrayView<Vec3> f_dyn = beam.get_field<BeamField::f_dyn>(buf);
    ArrayView<Vec3> m_dyn = beam.get_field<BeamField::m_dyn>(buf);
    ArrayView<uint> i_pipe_to_ie_hole = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);

    ArrayView<scalar> r_h = hole.get_field<HoleField::a>(buf);
    ArrayView<Vec2> co = hole.get_field<HoleField::co>(buf);
    ArrayView<scalar> s = hole.get_field<HoleField::s>(buf);

    ArrayView<scalar> S = pipe.get_field<PipeField::S>(buf);
    ArrayView<scalar> Sc = pipe.get_field<PipeField::Sc>(buf);
    ArrayView<scalar> rc = pipe.get_field<PipeField::rc>(buf);
    ArrayView<uint> num_ic_to_ie = pipe.get_field<PipeField::num_ic_to_ie>(buf);

    const scalar u1 = u[ie].x();
    const scalar u2 = u[ie].y();
    const scalar u3 = u[ie].z();
    const scalar u5 = theta[ie].y();
    const scalar u6 = theta[ie].z();
    const scalar u7 = u[ie + 1].x();
    const scalar u8 = u[ie + 1].y();
    const scalar u9 = u[ie + 1].z();
    const scalar u11 = theta[ie + 1].y();
    const scalar u12 = theta[ie + 1].z();

    const scalar v1 = v[ie].x();
    const scalar v2 = v[ie].y();
    const scalar v3 = v[ie].z();
    const scalar v4 = omega[ie].x();
    const scalar v5 = omega[ie].y();
    const scalar v6 = omega[ie].z();
    const scalar v7 = v[ie + 1].x();
    const scalar v8 = v[ie + 1].y();
    const scalar v9 = v[ie + 1].z();
    const scalar v10 = omega[ie + 1].x();
    const scalar v11 = omega[ie + 1].y();
    const scalar v12 = omega[ie + 1].z();

    // Linear interpolation for global s coordinate of contact point
    const scalar s_i = pipe.calc_s(ie, u1, buf);
    if (s_i <= 0) { // IMPORTANT: Removes top-node drag
        return;
    }

    const scalar S_i = S[ie];
    const scalar S_ip = S[ie + 1];

    uint start = num_ic_to_ie[ie];
    uint end = num_ic_to_ie[ie + 1];
    uint m = end - start;

    if (m == 0)
        return; // no contact samples in this element

    scalar S_prev = S_i;
    const uint i_h = i_pipe_to_ie_hole[ie];

    for (uint ic = start; ic < end; ++ic) {
        const uint ic_local = ic - start; // local index in [0, m-1]
        const scalar S_ic = Sc[ic];
        const scalar r_pipe = rc[ic];

        // Voronoi / half-interval weight (on-the-fly)
        const bool has_next = (ic + 1) < end; // next sample still in this element
        const scalar S_next = has_next ? Sc[ic + 1] : S_ip;

        scalar w_i;
        if (m == 1) {
            w_i = S_ip - S_i; // single sample takes entire element length
        } else {
            if (ic_local == 0)
                w_i = 0.5 * (S_next - S_i);
            else if (ic_local == m - 1)
                w_i = 0.5 * (S_ip - S_prev);
            else
                w_i = 0.5 * (S_next - S_prev);
            S_prev = S_ic;
        }

        scalar xi = max<scalar>(0.0, (S_ic - S_i) / (S_ip - S_i));
        assert(xi > -1e-14 && xi <= 1.0);

        Vec10 N_shape = shape_functions(xi, alpha, dx);

        const scalar n_1 = N_shape[0];
        const scalar n_2 = N_shape[1];
        const scalar h1_1 = N_shape[2];
        const scalar h1_2 = N_shape[3];
        const scalar h1_3 = N_shape[4];
        const scalar h1_4 = N_shape[5];
        const scalar h2_1 = N_shape[6];
        const scalar h2_2 = N_shape[7];
        const scalar h2_3 = N_shape[8];
        const scalar h2_4 = N_shape[9];

        // Interpolate kinematics (your code)
        const scalar ux = u1 * n_1 + u7 * n_2;
        const scalar uy = u2 * h1_1 + u6 * h1_2 + u8 * h1_3 + u12 * h1_4;
        const scalar uz = u3 * h1_1 - u5 * h1_2 + u9 * h1_3 - u11 * h1_4;

        const scalar vx = v1 * n_1 + v7 * n_2;
        const scalar vy = v2 * h1_1 + v6 * h1_2 + v8 * h1_3 + v12 * h1_4;
        const scalar vz = v3 * h1_1 - v5 * h1_2 + v9 * h1_3 - v11 * h1_4;
        const Vec3 v_i{vx, vy, vz};

        const scalar omega_x = v4 * n_1 + v10 * n_2;
        const scalar omega_z = v2 * h2_1 + v6 * h2_2 + v8 * h2_3 + v12 * h2_4;
        const scalar omega_y = -v3 * h2_1 + v5 * h2_2 - v9 * h2_3 + v11 * h2_4;
        const Vec3 omega_i{omega_x, omega_y, omega_z};

        // Contact geometry & gap (your code)
        const scalar s_ce = pipe.calc_sc(ic, ux, buf);
        uint ic_increment = hole.get_increment_index_pipe_node_to_hole_segment(i_h, s_ce, buf);
        uint ic_h = i_h + ic_increment;
        while (ic_increment != 0) {
            ic_increment = hole.get_increment_index_pipe_node_to_hole_segment(ic_h, s_ce, buf);
            ic_h += ic_increment;
        }
        const scalar grad_r_hole = hole.gradient_hole_property_section<scalar>(r_h, ic_h, buf);
        const Vec2 grad_co_hole = hole.gradient_hole_property_section<Vec2>(co, ic_h, buf);
        const scalar r_hole = r_h[ic_h] + grad_r_hole * (s_ce - s[ic_h]);
        const Vec2 co_hole = co[ic_h] + grad_co_hole * (s_ce - s[ic_h]);

        const Vec3 u_perp{0.0, uy, uz}; // Planar componets of displacement
        const Vec3 co_vec = Vec3{0.0, co_hole.x(), co_hole.y()};

        // Vector from hole center to pipe center
        Vec3 u_perp_rel = u_perp - co_vec;

        // In-place safe normalization: avoid division by zero if u_perp_rel == 0scalar u_perp_rel_norm =
        u_perp_rel.norm();
        scalar u_perp_rel_norm = u_perp_rel.norm();
        const Vec3 n_perp = (u_perp_rel_norm > SMALL_SCALAR) ? (u_perp_rel / u_perp_rel_norm) : Vec3{0.0, 1.0, 0.0};

        // Axial tangent (including radius gradient)
        Vec3 t_axial = (Vec3{1.0, grad_co_hole.x(), grad_co_hole.y()} + grad_r_hole * n_perp);
        
        // In-plane tangent (perpendicular in y-z plane)
        Vec3 t_inplane{0.0, n_perp[2], -n_perp[1]};
        
        // True surface normal
        Vec3 n = t_axial.cross(t_inplane).normalized();

        // Vector from pipe center to the closest in-plane wall point
        Vec3 c_perp = co_vec + r_hole * n_perp - u_perp;
        
        // shortest distance to wall from pipe center (positive if inside hole)
        scalar d_wall = c_perp.dot(n);
        
        const scalar delta = r_pipe - d_wall;
        const scalar H = heaviside(delta);

        const Vec3 vc = (v_i + omega_i.cross(r_pipe * n));
        const scalar vcn = vc.dot(n);

        // fn should be a line traction [N/m] for consistency
        const scalar fn =  (delta * K_c + C_c * vcn) * H;
        const Vec3 qc_f = -n * fn; // [N/m] line traction

        const scalar S_xi = (1.0 - xi) * S_i + xi * S_ip;
        const Vec3 x = Vec3{S_xi + ux, uy, uz};

        const Vec3 rc_arm = r_pipe * n;
        const Vec3 pc = x + rc_arm;
        const Vec3 x_node1 = Vec3{S_i + u1, u2, u3};
        const Vec3 x_node2 = Vec3{S_ip + u7, u8, u9};
        const Vec3 r1 = pc - x_node1;
        const Vec3 r2 = pc - x_node2;

        // Assemble to nodal generalized forces with weights
        f_dyn[ie] += w_i * N_shape[0] * qc_f; // force
        f_dyn[ie + 1] += w_i * N_shape[1] * qc_f;

        m_dyn[ie] += w_i * N_shape[0] * (r1.cross(qc_f)); // couple via lever arm
        m_dyn[ie + 1] += w_i * N_shape[1] * (r2.cross(qc_f));
    }
}

DEVICE_FUNC inline void calc_contact_forces_elliptical_nodewise(const uint i, const ConfigStatic &conf_stat,
                                                                const Hole &hole, const Pipe &pipe, BeamData &beam,
                                                                byte *buf) {
    assert(hole.type == HoleSurfaceType::ELLIPTICAL);
    assert(conf_stat.contact_enabled);

    const scalar mu_s = conf_stat.mu_static;
    const scalar mu_k = conf_stat.mu_kinetic;
    const scalar v_cs = conf_stat.critical_stribeck_velocity;
    const scalar K_c = conf_stat.K_normal;
    const scalar C_c = conf_stat.C_normal;

    const uint Nc_iter = conf_stat.Nc_iter;
    ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);
    ArrayView<Vec3> v = beam.get_field<BeamField::v>(buf);
    ArrayView<Vec3> omega = beam.get_field<BeamField::omega>(buf);
    ArrayView<Vec3> f_dyn = beam.get_field<BeamField::f_dyn>(buf);
    ArrayView<Vec3> m_dyn = beam.get_field<BeamField::m_dyn>(buf);
    ArrayView<uint> i_pipe_to_ie_hole = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);

    ArrayView<scalar> s = hole.get_field<HoleField::s>(buf);
    ArrayView<scalar> a_arr = hole.get_field<HoleField::a>(buf);
    ArrayView<scalar> b_arr = hole.get_field<HoleField::b>(buf);
    ArrayView<scalar> alpha_arr = hole.get_field<HoleField::alpha>(buf);
    ArrayView<Vec2> co_arr = hole.get_field<HoleField::co>(buf);

    scalar s_i = pipe.calc_s(i, u[i].x(), buf);
    if (s_i <= 0) { // IMPORTANT: Removes top-node drag
        return;
    }

    ArrayView<scalar> rc = pipe.get_field<PipeField::rc>(buf);
    ArrayView<uint> num_ic_to_ie = pipe.get_field<PipeField::num_ic_to_ie>(buf);
    uint ic = num_ic_to_ie[i]; // Contact point index corresponding to node i
    scalar r_pipe = rc[ic];

    const uint indice = i_pipe_to_ie_hole[i];

    // Get gradients, and linearly interpolate ellipse parameters
    scalar a_prime = hole.gradient_hole_property_section<scalar>(a_arr, indice, buf);
    scalar b_prime = hole.gradient_hole_property_section<scalar>(b_arr, indice, buf);
    scalar alpha_prime = hole.gradient_hole_property_section<scalar>(alpha_arr, indice, buf);
    Vec2 co_prime = hole.gradient_hole_property_section<Vec2>(co_arr, indice, buf);
    scalar a = a_arr[indice] + a_prime * (s_i - s[indice]);
    scalar b = b_arr[indice] + b_prime * (s_i - s[indice]);
    scalar alpha = alpha_arr[indice] + alpha_prime * (s_i - s[indice]);
    Vec2 co = co_arr[indice] + co_prime * (s_i - s[indice]);

    // Calculate the pipe kinematic properties
    const Vec2 u_perp{u[i].y(), u[i].z()};
    const Vec3 v_i{v[i].x(), v[i].y(), v[i].z()};
    const Vec3 omega_i{omega[i].x(), omega[i].y(), omega[i].z()};

    // Define ellipse rotation
    scalar cos_alpha = cos(alpha);
    scalar sin_alpha = sin(alpha);
    Mat2 R_e;
    R_e << cos_alpha, -sin_alpha, sin_alpha, cos_alpha;
    Mat2 R_e_prime; // Local derivative with respect to s of the ellipse rotation
    R_e_prime << -alpha_prime * sin_alpha, -alpha_prime * cos_alpha, alpha_prime * cos_alpha, -alpha_prime * sin_alpha;

    // Transform to ellipse local coordinates
    Vec2 u_perp_local = u_perp - co; // Local in-plane displacement vector
    Vec2 u_perp_rot = R_e.transpose() * u_perp_local;
    scalar sign_y = u_perp_rot.x() >= 0 ? 1.0 : -1.0;
    scalar sign_z = u_perp_rot.y() >= 0 ? 1.0 : -1.0;

    // Calculate the vector from the center of the point in the ellipse
    Vec2 wall_vec_rot = get_ellipse_quadrant_vector_new(a, b, abs(u_perp_rot.x()), abs(u_perp_rot.y()), Nc_iter);
    wall_vec_rot[0] *= sign_y;
    wall_vec_rot[1] *= sign_z;
    Vec2 c_perp = R_e * wall_vec_rot;

    // In-plane tangent of contact point (proportional to dh/dtheta)
    Vec3 t_perp{0.0, c_perp.y(), -c_perp.x()};

    // Compute axial tangent (dh/ds)
    scalar theta_ang = atan2(c_perp[1] / b, c_perp[0] / a);
    scalar cos_theta = cos(theta_ang);
    scalar sin_theta = sin(theta_ang);
    Vec2 h_e = {a * cos_theta, b * sin_theta}; // Ellipse surface vector
    Vec2 h_e_prime = {a_prime * cos_theta, b_prime * sin_theta};
    Vec2 t_axial_2d = co_prime + R_e_prime * h_e + R_e * h_e_prime;
    Vec3 t_axial{1.0, t_axial_2d.x(), t_axial_2d.y()};

    // True surface normal
    Vec3 n = t_axial.cross(t_perp).normalized();

    // Shortest distance to wall from pipe center (positive if inside hole)
    scalar d_wall = Vec3{0.0, c_perp.x(), c_perp.y()}.dot(n);

    Vec3 q_f, q_m;
    calc_contact_force_3D(q_f, q_m, mu_s, mu_k, v_cs, K_c, C_c, r_pipe, d_wall, n, v_i, omega_i);

    const Vec3 &fc = q_f;
    const Vec3 &mc = q_m;
    assert(fc.allFinite() && mc.allFinite()); // This cannot be in device code

    f_dyn[i].y() += fc[1];
    f_dyn[i].z() += fc[2];
    m_dyn[i].y() += mc[1];
    m_dyn[i].z() += mc[2];

    if (!conf_stat.enable_ax_tor_subcycle) {
        f_dyn[i].x() += fc[0];
        m_dyn[i].x() += mc[0];
    }
}

DEVICE_FUNC inline void calc_contact_forces_uneven_ellipse(uint ie, const ConfigStatic &conf_stat, const Hole &hole,
                                                           const Pipe &pipe, const BeamData &beam, byte *buf) {
    assert(conf_stat.contact_enabled);
    assert(hole.type == HoleSurfaceType::ELLIPTICAL);

    // FEM parameters
    const scalar dx = pipe.dS_e(ie, buf);
    const scalar Ap = pipe.Ap_e(ie, buf);
    const scalar Ip = pipe.Ip_e(ie, buf);
    const scalar E = conf_stat.E;
    const scalar G = conf_stat.get_G();
    const scalar k = pipe.k_s(ie, buf);
    const scalar alpha = 12.0 * E * Ip / (k * G * Ap * dx * dx);

    // Contact parameters
    const scalar K_c = conf_stat.K_normal;
    const scalar C_c = conf_stat.C_normal;
    const uint Nc_iter = conf_stat.Nc_iter;

    ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);
    ArrayView<Vec3> v = beam.get_field<BeamField::v>(buf);
    ArrayView<Vec3> theta = beam.get_field<BeamField::theta>(buf);
    ArrayView<Vec3> omega = beam.get_field<BeamField::omega>(buf);
    ArrayView<Vec3> f_dyn = beam.get_field<BeamField::f_dyn>(buf);
    ArrayView<Vec3> m_dyn = beam.get_field<BeamField::m_dyn>(buf);
    ArrayView<uint> i_pipe_to_ie_hole = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);

    ArrayView<scalar> s = hole.get_field<HoleField::s>(buf);
    ArrayView<scalar> a_arr = hole.get_field<HoleField::a>(buf);
    ArrayView<scalar> b_arr = hole.get_field<HoleField::b>(buf);
    ArrayView<scalar> alpha_arr = hole.get_field<HoleField::alpha>(buf);
    ArrayView<Vec2> co_arr = hole.get_field<HoleField::co>(buf);

    ArrayView<scalar> S = pipe.get_field<PipeField::S>(buf);
    ArrayView<scalar> Sc = pipe.get_field<PipeField::Sc>(buf);
    ArrayView<scalar> rc = pipe.get_field<PipeField::rc>(buf);
    ArrayView<uint> num_ic_to_ie = pipe.get_field<PipeField::num_ic_to_ie>(buf);

    const scalar u1 = u[ie].x();
    const scalar u2 = u[ie].y();
    const scalar u3 = u[ie].z();
    const scalar u5 = theta[ie].y();
    const scalar u6 = theta[ie].z();
    const scalar u7 = u[ie + 1].x();
    const scalar u8 = u[ie + 1].y();
    const scalar u9 = u[ie + 1].z();
    const scalar u11 = theta[ie + 1].y();
    const scalar u12 = theta[ie + 1].z();

    const scalar v1 = v[ie].x();
    const scalar v2 = v[ie].y();
    const scalar v3 = v[ie].z();
    const scalar v4 = omega[ie].x();
    const scalar v5 = omega[ie].y();
    const scalar v6 = omega[ie].z();
    const scalar v7 = v[ie + 1].x();
    const scalar v8 = v[ie + 1].y();
    const scalar v9 = v[ie + 1].z();
    const scalar v10 = omega[ie + 1].x();
    const scalar v11 = omega[ie + 1].y();
    const scalar v12 = omega[ie + 1].z();

    const scalar s_i = pipe.calc_s(ie, u1, buf);
    if (s_i <= 0) { // IMPORTANT: Removes top-node drag
        return;
    }
    const scalar S_i = S[ie];
    const scalar S_ip = S[ie + 1];

    uint start = num_ic_to_ie[ie];
    uint end = num_ic_to_ie[ie + 1];
    uint m = end - start;

    if (m == 0)
        return; // no contact samples in this element

    scalar S_prev = S_i;
    const uint i_h = i_pipe_to_ie_hole[ie];

    for (uint ic = start; ic < end; ++ic) {
        const uint ic_local = ic - start; // local index in [0, m-1]
        const scalar S_ic = Sc[ic];
        const scalar r_pipe = rc[ic];

        // Voronoi / half-interval weight (on-the-fly)
        const bool has_next = (ic + 1) < end; // next sample still in this element
        const scalar S_next = has_next ? Sc[ic + 1] : S_ip;

        scalar w_i;
        if (m == 1) {
            w_i = S_ip - S_i; // single sample takes entire element length
        } else {
            if (ic_local == 0)
                w_i = 0.5 * (S_next - S_i);
            else if (ic_local == m - 1)
                w_i = 0.5 * (S_ip - S_prev);
            else
                w_i = 0.5 * (S_next - S_prev);
            S_prev = S_ic;
        }

        scalar xi = max<scalar>(0.0, (S_ic - S_i) / (S_ip - S_i));
        assert(xi > -1e-14 && xi <= 1.0);

        Vec10 N_shape = shape_functions(xi, alpha, dx);

        const scalar n_1 = N_shape[0];
        const scalar n_2 = N_shape[1];
        const scalar h1_1 = N_shape[2];
        const scalar h1_2 = N_shape[3];
        const scalar h1_3 = N_shape[4];
        const scalar h1_4 = N_shape[5];
        const scalar h2_1 = N_shape[6];
        const scalar h2_2 = N_shape[7];
        const scalar h2_3 = N_shape[8];
        const scalar h2_4 = N_shape[9];

        const scalar ux = u1 * n_1 + u7 * n_2;
        const scalar uy = u2 * h1_1 + u6 * h1_2 + u8 * h1_3 + u12 * h1_4;
        const scalar uz = u3 * h1_1 - u5 * h1_2 + u9 * h1_3 - u11 * h1_4;

        const scalar vx = v1 * n_1 + v7 * n_2;
        const scalar vy = v2 * h1_1 + v6 * h1_2 + v8 * h1_3 + v12 * h1_4;
        const scalar vz = v3 * h1_1 - v5 * h1_2 + v9 * h1_3 - v11 * h1_4;

        const scalar omega_x = v4 * n_1 + v10 * n_2;
        const scalar omega_z = v2 * h2_1 + v6 * h2_2 + v8 * h2_3 + v12 * h2_4;
        const scalar omega_y = -v3 * h2_1 + v5 * h2_2 - v9 * h2_3 + v11 * h2_4;

        // Contact geometry & gap (your code)
        const scalar s_ce = pipe.calc_sc(ic, ux, buf);
        uint ic_increment = hole.get_increment_index_pipe_node_to_hole_segment(i_h, s_ce, buf);
        uint ic_h = i_h + ic_increment;
        while (ic_increment != 0) {
            ic_increment = hole.get_increment_index_pipe_node_to_hole_segment(ic_h, s_ce, buf);
            ic_h += ic_increment;
        }

        // Get gradients, and linearly interpolate ellipse parameters
        scalar a_prime = hole.gradient_hole_property_section<scalar>(a_arr, ic_h, buf);
        scalar b_prime = hole.gradient_hole_property_section<scalar>(b_arr, ic_h, buf);
        scalar alpha_prime = hole.gradient_hole_property_section<scalar>(alpha_arr, ic_h, buf);
        Vec2 co_prime = hole.gradient_hole_property_section<Vec2>(co_arr, ic_h, buf);

        scalar a = a_arr[ic_h] + a_prime * (s_ce - s[ic_h]);
        scalar b = b_arr[ic_h] + b_prime * (s_ce - s[ic_h]);
        scalar alpha_rot = alpha_arr[ic_h] + alpha_prime * (s_ce - s[ic_h]);
        Vec2 co = co_arr[ic_h] + co_prime * (s_ce - s[ic_h]);

        // Define ellipse rotation
        scalar cos_alpha = cos(alpha_rot);
        scalar sin_alpha = sin(alpha_rot);
        Mat2 R_e;
        R_e << cos_alpha, -sin_alpha, 
            sin_alpha, cos_alpha;
        Mat2 R_e_prime; // Local derivative with respect to s of the ellipse rotation
        R_e_prime << -alpha_prime * sin_alpha, -alpha_prime * cos_alpha, alpha_prime * cos_alpha,
            -alpha_prime * sin_alpha;

        // Transform to ellipse local coordinates
        Vec2 u_perp = {uy, uz};
        Vec2 u_perp_local = u_perp - co; // Local in-plane displacement vector
        Vec2 u_perp_rot = R_e.transpose() * u_perp_local;
        scalar sign_y = u_perp_rot.x() >= 0 ? 1.0 : -1.0;
        scalar sign_z = u_perp_rot.y() >= 0 ? 1.0 : -1.0;

        // Calculate the vector from the center of the point in the ellipse
        Vec2 wall_vec_rot = get_ellipse_quadrant_vector_new(a, b, abs(u_perp_rot.x()), abs(u_perp_rot.y()), Nc_iter);
        wall_vec_rot[0] *= sign_y;
        wall_vec_rot[1] *= sign_z;
        Vec2 c_perp = R_e * wall_vec_rot;

        // In-plane tangent of contact point (proportional to dh/dtheta)
        Vec3 t_perp{0.0, c_perp.y(), -c_perp.x()};

        // Compute axial tangent (dh/ds)
        scalar theta_ang = atan2(c_perp[1] / b, c_perp[0] / a);
        scalar cos_theta = cos(theta_ang);
        scalar sin_theta = sin(theta_ang);
        Vec2 h_e = {a * cos_theta, b * sin_theta}; // Ellipse surface vector
        Vec2 h_e_prime = {a_prime * cos_theta, b_prime * sin_theta};
        Vec2 t_axial_2d = co_prime + R_e_prime * h_e + R_e * h_e_prime;
        Vec3 t_axial{1.0, t_axial_2d.x(), t_axial_2d.y()};
        

        //Vec3 t_axial = Vec3{1.0, 0.0, 0.0};
       
        // True surface normal
        Vec3 n = t_axial.cross(t_perp).normalized();

        // Shortest distance to wall from pipe center (positive if inside hole)
        scalar d_wall = Vec3{0.0, c_perp.x(), c_perp.y()}.dot(n);
        const Vec3 v_ic{vx, vy, vz};
        const Vec3 omega_ic{omega_x, omega_y, omega_z};

        const scalar delta = r_pipe - d_wall;
        const scalar H = heaviside(delta);

        const Vec3 vc = (v_ic + omega_ic.cross(r_pipe * n));
        const scalar vcn = vc.dot(n);

        // fn should be a line traction [N/m] for consistency
        const scalar fn =  (delta * K_c + C_c * vcn) * H;
        
        const Vec3 qc_f = -n * fn; // [N/m] line traction

        const scalar S_xi = (1.0 - xi) * S_i + xi * S_ip;
        const Vec3 x = Vec3{S_xi + ux, uy, uz};

        const Vec3 rc_arm = r_pipe * n;
        const Vec3 pc = x + rc_arm;
        const Vec3 x_node1 = Vec3{S_i + u1, u2, u3};
        const Vec3 x_node2 = Vec3{S_ip + u7, u8, u9};
        const Vec3 r1 = pc - x_node1;
        const Vec3 r2 = pc - x_node2;

        // Assemble to nodal generalized forces with weights
        f_dyn[ie] += w_i * N_shape[0] * qc_f; // force
        f_dyn[ie + 1] += w_i * N_shape[1] * qc_f;

        m_dyn[ie] += w_i * N_shape[0] * (r1.cross(qc_f)); // couple via lever arm
        m_dyn[ie + 1] += w_i * N_shape[1] * (r2.cross(qc_f));
    }
}

DEVICE_FUNC inline void calc_contact_forces_bow_spring(uint ie, const ConfigStatic &conf_stat, const Hole &hole,
                                                       const Pipe &pipe, BeamData &beam, byte *buf) {
    assert(conf_stat.contact_enabled);
    assert(hole.type == HoleSurfaceType::CIRCULAR);

    // FEM parameters
    const scalar dx = pipe.dS_e(ie, buf);
    const scalar Ap = pipe.Ap_e(ie, buf);
    const scalar Ip = pipe.Ip_e(ie, buf);
    const scalar E = conf_stat.E;
    const scalar G = conf_stat.get_G();
    const scalar k = pipe.k_s(ie, buf);
    const scalar alpha = 12.0 * E * Ip / (k * G * Ap * dx * dx);
    const scalar beta = 1.0 / (1.0 + alpha);

    // Contact parameters
    const scalar mu_s = conf_stat.mu_static;
    const scalar mu_k = conf_stat.mu_kinetic;
    const scalar v_cs = conf_stat.critical_stribeck_velocity;

    // Bow spring parameters
    const scalar p_bs = conf_stat.p_bs;
    const scalar K_bs = conf_stat.K_bs;
    const scalar C_bs = conf_stat.C_bs;
    const uint N_bs = conf_stat.N_bs;
    const scalar r_bs_o = conf_stat.r_bs_outer;

    ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);
    ArrayView<Vec3> v = beam.get_field<BeamField::v>(buf);
    ArrayView<Vec3> theta = beam.get_field<BeamField::theta>(buf);
    ArrayView<Vec3> omega = beam.get_field<BeamField::omega>(buf);
    ArrayView<Vec3> f_dyn = beam.get_field<BeamField::f_dyn>(buf);
    ArrayView<Vec3> m_dyn = beam.get_field<BeamField::m_dyn>(buf);
    ArrayView<uint> i_pipe_to_ie_hole = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);

    ArrayView<scalar> r_h = hole.get_field<HoleField::a>(buf);
    ArrayView<Vec2> co = hole.get_field<HoleField::co>(buf);
    ArrayView<scalar> s = hole.get_field<HoleField::s>(buf);

    ArrayView<scalar> S = pipe.get_field<PipeField::S>(buf);
    ArrayView<scalar> Sbs = pipe.get_field<PipeField::Sbs>(buf);
    const scalar r_p = pipe.get_field<PipeField::ro>(buf)[ie];

    ArrayView<uint> num_ibs_to_ie = pipe.get_field<PipeField::num_ibs_to_ie>(buf);

    const scalar u1 = u[ie].x();
    const scalar u2 = u[ie].y();
    const scalar u3 = u[ie].z();
    const scalar u5 = theta[ie].y();
    const scalar u6 = theta[ie].z();
    const scalar u7 = u[ie + 1].x();
    const scalar u8 = u[ie + 1].y();
    const scalar u9 = u[ie + 1].z();
    const scalar u11 = theta[ie + 1].y();
    const scalar u12 = theta[ie + 1].z();

    const scalar v1 = v[ie].x();
    const scalar v2 = v[ie].y();
    const scalar v3 = v[ie].z();
    const scalar v4 = omega[ie].x();
    const scalar v5 = omega[ie].y();
    const scalar v6 = omega[ie].z();
    const scalar v7 = v[ie + 1].x();
    const scalar v8 = v[ie + 1].y();
    const scalar v9 = v[ie + 1].z();
    const scalar v10 = v[ie + 1].x();
    const scalar v11 = v[ie + 1].y();
    const scalar v12 = v[ie + 1].z();

    // Linear interpolation for global s coordinate of contact point
    const scalar s_i = pipe.calc_s(ie, u1, buf);
    if (s_i < 0) { // IMPORTANT: Removes top-node drag
        return;
    }

    const scalar S_i = S[ie];
    const scalar S_ip = S[ie + 1];
    const scalar dS = S_ip - S_i;

    const uint i_h = i_pipe_to_ie_hole[ie];

    for (uint ibs = num_ibs_to_ie[ie]; ibs < num_ibs_to_ie[ie + 1]; ibs++) { // For all contact points in element
        const scalar S_ic = Sbs[ibs];

        // NOTE!! We must have xi > 0 (xi <= 1.0 is ok). This is because of the limits of the heaviside function.
        scalar xi = max(SMALL_SCALAR, (S_ic - S_i) / (S_ip - S_i));
        assert(xi > 0.0 && xi <= 1.0);

        Vec10 N_shape = shape_functions(xi, alpha, dx);

        const scalar n_1 = N_shape[0];
        const scalar n_2 = N_shape[1];
        const scalar h1_1 = N_shape[2];
        const scalar h1_2 = N_shape[3];
        const scalar h1_3 = N_shape[4];
        const scalar h1_4 = N_shape[5];
        const scalar h2_1 = N_shape[6];
        const scalar h2_2 = N_shape[7];
        const scalar h2_3 = N_shape[8];
        const scalar h2_4 = N_shape[9];

        const scalar ux = u1 * n_1 + u7 * n_2;
        const scalar uy = u2 * h1_1 + u6 * h1_2 + u8 * h1_3 + u12 * h1_4;
        const scalar uz = u3 * h1_1 - u5 * h1_2 + u9 * h1_3 - u11 * h1_4;

        const scalar vx = v1 * n_1 + v7 * n_2;
        const scalar vy = v2 * h1_1 + v6 * h1_2 + v8 * h1_3 + v12 * h1_4;
        const scalar vz = v3 * h1_1 - v5 * h1_2 + v9 * h1_3 - v11 * h1_4;

        const scalar omega_x = v4 * n_1 + v10 * n_2;
        const scalar omega_z = v2 * h2_1 + v6 * h2_2 + v8 * h2_3 + v12 * h2_4;
        const scalar omega_y = -v3 * h2_1 + v5 * h2_2 - v9 * h2_3 + v11 * h2_4;

        // Planar componets of displacement
        const Vec2 u_perp{uy, uz};
        const Vec3 v_i{vx, vy, vz};
        const Vec3 omega_i{omega_x, omega_y, omega_z};
        scalar s_ce = pipe.calc_sbs(ibs, ux, buf);

        // Finds the correct hole index for the contact point within the pipe element
        uint ic_increment = hole.get_increment_index_pipe_node_to_hole_segment(i_h, s_ce, buf);
        uint ic_h = i_h + ic_increment;
        while (ic_increment != 0) {
            ic_increment = hole.get_increment_index_pipe_node_to_hole_segment(ic_h, s_ce, buf);
            ic_h += ic_increment;
        }

        // Extract gradients of the hole with respect to s
        scalar grad_r_hole = hole.gradient_hole_property_section<scalar>(r_h, ic_h, buf);
        Vec2 grad_co_hole = hole.gradient_hole_property_section<Vec2>(co, ic_h, buf);

        // Use the gradients to calculate linear interpolation
        scalar r_hole = r_h[ic_h] + grad_r_hole * (s_ce - s[ic_h]);
        Vec2 co_hole = co[ic_h] + grad_co_hole * (s_i - s[ic_h]);

        // Vector from hole center to pipe center
        Vec2 u_perp_rel = u_perp - co_hole;

        const scalar r_bs_i = max(conf_stat.r_bs_inner, r_p);
        const scalar r_diff = r_bs_i - r_p;

        // Calculate center of bow spring
        Vec2 e_r = u_perp_rel.normalized();
        Vec2 r_cent = (u_perp_rel.norm() - r_diff) * e_r;

        Vec3 f_bs, m_bs;
        calc_bow_spring_centralizer_forces_vector(f_bs, m_bs, r_cent, r_bs_o, r_hole, N_bs, K_bs, C_bs, p_bs, mu_s,
                                                  mu_k, v_cs, v_i, omega_i);

        assert(f_bs.allFinite() && m_bs.allFinite());

        const scalar f_x = f_bs[0];
        const scalar f_y = f_bs[1];
        const scalar f_z = f_bs[2];
        const scalar m_x = m_bs[0];
        const scalar m_y = m_bs[1];
        const scalar m_z = m_bs[2];

        f_dyn[ie].x() += f_x * (1 - xi);
        f_dyn[ie].y() += beta * f_y * (2 * powi(xi, 3) - 3 * powi(xi, 2) + (-1 + 1.0 / beta) * (1 - xi) + 1) +
                         6 * beta * m_z * (powi(xi, 2) - xi) / dS;
        f_dyn[ie].z() += beta * f_z * (2 * powi(xi, 3) - 3 * powi(xi, 2) + (-1 + 1.0 / beta) * (1 - xi) + 1) -
                         6 * beta * m_y * (powi(xi, 2) - xi) / dS;
        m_dyn[ie].x() += m_x * (1 - xi);
        m_dyn[ie].y() +=
            -dS * beta * f_z *
                (powi(xi, 3) - 2 * powi(xi, 2) + xi + (1.0 / 2.0) * (-1 + 1.0 / beta) * (-powi(xi, 2) + xi)) +
            beta * m_y * (3 * powi(xi, 2) - 4 * xi + (-1 + 1.0 / beta) * (1 - xi) + 1);
        m_dyn[ie].z() +=
            dS * beta * f_y *
                (powi(xi, 3) - 2 * powi(xi, 2) + xi + (1.0 / 2.0) * (-1 + 1.0 / beta) * (-powi(xi, 2) + xi)) +
            beta * m_z * (3 * powi(xi, 2) - 4 * xi + (-1 + 1.0 / beta) * (1 - xi) + 1);
        f_dyn[ie + 1].x() += f_x * xi;
        f_dyn[ie + 1].y() += beta * f_y * (-2 * powi(xi, 3) + 3 * powi(xi, 2) + xi * (-1 + 1.0 / beta)) -
                             6 * beta * m_z * (powi(xi, 2) - xi) / dS;
        f_dyn[ie + 1].z() += beta * f_z * (-2 * powi(xi, 3) + 3 * powi(xi, 2) + xi * (-1 + 1.0 / beta)) +
                             6 * beta * m_y * (powi(xi, 2) - xi) / dS;
        m_dyn[ie + 1].x() += m_x * xi;
        m_dyn[ie + 1].y() +=
            -dS * beta * f_z * (powi(xi, 3) - powi(xi, 2) + (1.0 / 2.0) * (-1 + 1.0 / beta) * (powi(xi, 2) - xi)) +
            beta * m_y * (3 * powi(xi, 2) + xi * (-1 + 1.0 / beta) - 2 * xi);
        f_dyn[ie + 1].z() +=
            dS * beta * f_y * (powi(xi, 3) - powi(xi, 2) + (1.0 / 2.0) * (-1 + 1.0 / beta) * (powi(xi, 2) - xi)) +
            beta * m_z * (3 * powi(xi, 2) + xi * (-1 + 1.0 / beta) - 2 * xi);
    }
}

DEVICE_FUNC inline void calc_contact_forces(uint ie, const ConfigStatic &conf_stat, const Hole &hole, const Pipe &pipe,
                                            BeamData &beam, byte *buf) {

    if (!conf_stat.contact_enabled) {
        return;
    }

    if (hole.type == HoleSurfaceType::CIRCULAR) {
        calc_contact_forces_circular(ie, conf_stat, hole, pipe, beam, buf);
    } else {
        calc_contact_forces_uneven_ellipse(ie, conf_stat, hole, pipe, beam, buf);
        assert(hole.type == HoleSurfaceType::ELLIPTICAL);
    }
}

} // namespace curvlin