#include "pipe-solver/pipe-solver-curvlin/pipe-solver-interface-curvlin.hpp"
#include "pipe-solver/pipe-solver-curvlin/numerics/constant-data.hpp"
#include "pipe-solver/pipe-solver-curvlin/numerics/pre-derived-integration.hpp"
#include "pipe-solver/pipe-solver-curvlin/pipe-solver-interface-curvlin.hpp"

namespace curvlin {




DEVICE_FUNC inline void calc_element_residual( uint ie, const ConfigStatic &conf_stat, const uint top_node,
                                              const BCsData &bcs, BeamData &beam, const FluidData &fluid, const Pipe &pipe,
                                              const Hole &hole, byte *buf) {

    switch (conf_stat.curvilinear_integration_type) {
        case CurvilinearIntegrationType::PRE_DERIVED:
            calc_element_inner_forces_prederived(ie, conf_stat, bcs, beam, pipe, hole, buf);
            //if (conf_stat.fluid_dynamics_enabled) {
            //    calc_element_fluid_forces_prederived(ie, conf_stat, beam, fluid, pipe, hole, buf);
            //}
            break;
        case CurvilinearIntegrationType::GAUSS_LEGENDRE:
            calc_element_forces_gauss_legendre( ie, conf_stat, bcs, beam, fluid, pipe, hole, buf);
            break;
        default:
            assert(false);
    }

    calc_contact_forces(ie, conf_stat, hole, pipe, beam, buf);
    
    
    if (conf_stat.string_type == StringType::CASING_STRING && pipe.Nbs > 0) { 
        calc_contact_forces_bow_spring(ie, conf_stat, hole, pipe, beam, buf);
    }

}


DEVICE_FUNC inline void calc_element_residual_ax_tor( uint ie, const ConfigStatic &conf_stat, const uint top_node,
                                              const BCsData &bcs, BeamData &beam, const FluidData &fluid, const Pipe &pipe,
                                              const Hole &hole, byte *buf) {

    const scalar dx = pipe.dS_e(ie, buf);
    assert(dx > SMALL_SCALAR);
    const scalar Ap = pipe.Ap_e(ie, buf);
    const scalar Ip = pipe.Ip_e(ie, buf);
    const scalar E = conf_stat.E;
    const scalar G = conf_stat.get_G();
    const scalar u_top = bcs.get_bcs_field(buf).u_top_np;
    scalar beta = conf_stat.rayleigh_damping_enabled ? conf_stat.beta : 0.0;
    
    ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);
    ArrayView<Vec3> v = beam.get_field<BeamField::v>(buf);
    ArrayView<Vec3> theta = beam.get_field<BeamField::theta>(buf);
    ArrayView<Vec3> omega = beam.get_field<BeamField::omega>(buf);
    ArrayView<Vec3> f_int = beam.get_field<BeamField::f_int>(buf);
    ArrayView<Vec3> m_int = beam.get_field<BeamField::m_int>(buf);

    // Precompute coefficients
    const scalar EA_dx = Ap * E / dx;
    const scalar GIp2_dx = 2.0 * G * Ip / dx;
    
    // Axial
    const scalar du = (u[ie].x() - u_top) - (u[ie + 1].x() - u_top); // = u[ie].x() - u[ie+1].x()
    const scalar dv = v[ie].x() - v[ie + 1].x();

    const scalar fac_fx = EA_dx * (du + beta * dv);
    f_int[ie].x()     += fac_fx;
    f_int[ie + 1].x() += -fac_fx;

    // Torsion
    const scalar dtheta = theta[ie].x() - theta[ie + 1].x();
    const scalar domega = omega[ie].x() - omega[ie + 1].x();

    const scalar fac_mx = GIp2_dx * (dtheta + beta * domega);
    m_int[ie].x()     += fac_mx;
    m_int[ie + 1].x() += -fac_mx;

}



void assemble_even_ax_tor_cpu(const ConfigStatic &cs, uint top_node, const BCsData &bcs,
                              BeamData &beam, const FluidData &fluid, const Pipe &pipe, const Hole &hole, byte *buf)
{
    const uint N = pipe.N;
    uint ie;
    for_each_element_cpu_even(ie) {
        if (ie >= top_node && ie + 1 < pipe.N) {
            calc_element_residual_ax_tor(ie, cs, top_node, bcs, beam, fluid, pipe, hole, buf);
        }
    }
}


void assemble_odd_ax_tor_cpu(const ConfigStatic &cs, uint top_node, const BCsData &bcs,
                             BeamData &beam, const FluidData &fluid, const Pipe &pipe, const Hole &hole, byte *buf)
{
    const uint N = pipe.N;
    uint ie;
    for_each_element_cpu_odd(ie) {
        if (ie >= top_node && ie + 1 < pipe.N) {
            calc_element_residual_ax_tor(ie, cs, top_node, bcs, beam, fluid, pipe, hole, buf);
        }
    }
}


DEVICE_FUNC inline void calc_element_contact( uint ie, const ConfigStatic &conf_stat, const uint top_node,
                                              BeamData &beam, const Pipe &pipe,
                                              const Hole &hole, byte *buf) {


    //Elementwise contact
    if (conf_stat.contact_enabled) { 
        calc_contact_forces(ie, conf_stat, hole, pipe, beam, buf);
    }

    if (conf_stat.string_type == StringType::CASING_STRING) { 
        calc_contact_forces_bow_spring(ie, conf_stat, hole, pipe, beam, buf);
    }

}


DEVICE_FUNC inline void integrate_central_diff(
    const scalar dt, scalar &u, scalar &v, scalar &a, const scalar &m,
    const scalar &f_hyd, const scalar &f_dyn, const scalar &f_int,
    const scalar alpha)
{
    // Add -beta * f_int for stiffness-proportional damping
    a = (f_hyd + f_dyn - f_int - alpha * m * v) / m;
    v += dt * a;
    u += dt * v;
}
DEVICE_FUNC inline void calculate_contact_friction(
    scalar dt_fast,
    Vec3 &f_dyn,            // contact-only accumulators; y/z hold normal from slow pass
    Vec3 &m_dyn,
    scalar v_ax,            // axial velocity
    scalar a_x,             // axial acceleration (current)
    scalar omega_x,         // torsional velocity
    scalar alpha_x,         // torsional angular acceleration (current)
    scalar m_ax,            // effective axial mass at node
    scalar Jx,              // torsional inertia at node
    scalar mu_s,
    scalar mu_k,
    scalar v_cs,
    scalar r_pipe)
{
    // --- Normal load magnitude from slow pass (already assembled into f_dyn.y/z):
    const scalar fn = sqrt(f_dyn.y()*f_dyn.y() + f_dyn.z()*f_dyn.z());
    if (fn <= 1e-12) { f_dyn.x() = 0.0; m_dyn.x() = 0.0; return; }

    // --- Slip kinematics (magnitudes built from axial & torsional components):
    const scalar v_theta = r_pipe * omega_x;
    const scalar v_slip  = sqrt(v_ax*v_ax + v_theta*v_theta);

    // --- Static caps:
    const scalar Fcap = mu_s * fn;
    const scalar Mcap = Fcap * r_pipe;


    // --- SLIP: kinetic Stribeck (rate weakening allowed), always dissipative
    // NOTE: use decaying Stribeck (exp(-|v|/v_cs)):
    const scalar mu  = mu_k + (mu_s - mu_k) * exp(- v_slip / (v_cs + SMALL_SCALAR));
    const scalar ft  = mu * fn;   // tangential magnitude at contact

    // Decompose along axial and torsional directions (consistent with v components):
    const scalar inv = 1.0 / (v_slip + SMALL_SCALAR);
    const scalar F_new = -ft * (v_ax     * inv);            // axial friction force
    const scalar T_new = -ft * (v_theta  * inv) * r_pipe;   // torsional friction torque

    // Final safety clamp to static envelope:
    f_dyn.x() = (Fcap > 0) ? max(-Fcap, min(F_new, Fcap)) : 0.0;
    m_dyn.x() = (Mcap > 0) ? max(-Mcap, min(T_new, Mcap)) : 0.0;
}




DEVICE_FUNC inline void initialize_forces(const uint i, const bool at_subcycling, ArrayView<Vec3> f_int, ArrayView<Vec3> f_dyn, 
                                                        ArrayView<Vec3> m_int, ArrayView<Vec3> m_dyn) { 
    f_int[i].x() = 0.0;

    f_dyn[i].y() = 0.0;
    f_int[i].y() = 0.0;

    f_dyn[i].z() = 0.0;
    f_int[i].z() = 0.0;
    
    m_int[i].x() = 0.0;

    m_dyn[i].y() = 0.0;
    m_int[i].y() = 0.0;

    m_dyn[i].z() = 0.0;
    m_int[i].z() = 0.0;

    if (!at_subcycling){
        f_dyn[i].x() = 0.0;
        m_dyn[i].x() = 0.0;
    }
}


DEVICE_FUNC inline void update_indices_node(const uint i,const uint N, scalar ui_x, ArrayView<uint> indices,
                                             const Hole &hole, const Pipe &pipe, byte *buf) {
    const scalar s_i = pipe.calc_s(i, ui_x, buf);
    indices[i] += hole.get_increment_index_pipe_node_to_hole_segment(indices[i], s_i, buf);
    assert(indices[i] < hole.N_hole + 2); //Allow for outside of hole?
    assert(hole.get_increment_index_pipe_node_to_hole_segment(indices[i], s_i, buf) == 0);
}


#ifdef __CUDACC__
__global__ void initialize_forces_gpu(byte *buf) {
    assert(blockIdx.y == 0 && blockIdx.z == 0 && blockDim.y == 1 && blockDim.z == 1);
    const uint i = blockDim.x * blockIdx.x + threadIdx.x; 
    BeamData &beam = cd_d.beam_field;
    const FluidData &fluid = cd_d.fluid_field;
    const Pipe &pipe = cd_d.pipe;
    const Hole &hole = cd_d.hole;
    const BCsData &bcs = cd_d.bc_data;
    const ConfigStatic &conf_stat = cd_d.conf_stat;
    bool at_subcycling = conf_stat.enable_ax_tor_subcycle;
    
    ArrayView<Vec3> f_int = beam.get_field<BeamField::f_int>(buf);
    ArrayView<Vec3> f_dyn = beam.get_field<BeamField::f_dyn>(buf);
    ArrayView<Vec3> m_int = beam.get_field<BeamField::m_int>(buf);
    ArrayView<Vec3> m_dyn = beam.get_field<BeamField::m_dyn>(buf);
    ArrayView<uint> indices = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);
    
    const uint top_node = cd_d.top_node;
    const uint N = pipe.N;
    
    // Initialize forces and indices for even nodes
    if (i >= top_node && i < N) {
        initialize_forces(i, at_subcycling, f_int, f_dyn, m_int, m_dyn);
    }   
    
}
#endif

#ifdef __CUDACC__
__global__ void assemble_odd_gpu(byte *buf) {
    assert(blockIdx.y == 0 && blockIdx.z == 0 && blockDim.y == 1 && blockDim.z == 1);
    const uint ie = 2 * (blockDim.x * blockIdx.x + threadIdx.x) + 1; // ie is always odd
    assert(ie % 2 == 1);
    
    BeamData &beam = cd_d.beam_field;
    const FluidData &fluid = cd_d.fluid_field;
    const Hole &hole = cd_d.hole;
    const BCsData &bcs = cd_d.bc_data;
    const ConfigStatic &conf_stat = cd_d.conf_stat;
    const Pipe &pipe = cd_d.pipe;
    const uint top_node = cd_d.top_node;
    const uint N = pipe.N;
    
    // Calculate residuals for odd-numbered elements
    if (ie >= top_node && ie < N - 1) {
        calc_element_residual(ie, conf_stat, top_node, bcs, beam, fluid, pipe, hole, buf);
    }    
}
#endif




#ifdef __CUDACC__
__global__ void initialize_forces_ax_tor_gpu(byte *buf) {
    assert(blockIdx.y == 0 && blockIdx.z == 0 && blockDim.y == 1 && blockDim.z == 1);
    const uint i = blockDim.x * blockIdx.x + threadIdx.x; 
    BeamData &beam = cd_d.beam_field;
    const FluidData &fluid = cd_d.fluid_field;
    const Pipe &pipe = cd_d.pipe;
    const Hole &hole = cd_d.hole;
    const BCsData &bcs = cd_d.bc_data;
    
    ArrayView<Vec3> f_int = beam.get_field<BeamField::f_int>(buf);
    ArrayView<Vec3> f_dyn = beam.get_field<BeamField::f_dyn>(buf);
    ArrayView<Vec3> m_int = beam.get_field<BeamField::m_int>(buf);
    ArrayView<Vec3> m_dyn = beam.get_field<BeamField::m_dyn>(buf);
    
    const uint top_node = cd_d.top_node;
    const uint N = pipe.N;
    
    // Initialize forces and indices for even nodes
    if (i >= top_node && i < N) {
        f_int[i].x() = 0.0;
        m_int[i].x() = 0.0;

        f_dyn[i].y() = 0.0;
        f_dyn[i].z() = 0.0;
    }   
    
}
#endif


#ifdef __CUDACC__
__global__ void assemble_odd_ax_tor_gpu(byte *buf) {
    assert(blockIdx.y == 0 && blockIdx.z == 0 && blockDim.y == 1 && blockDim.z == 1);
    const uint ie = 2 * (blockDim.x * blockIdx.x + threadIdx.x) + 1; // ie is always odd
    assert(ie % 2 == 1);
    
    BeamData &beam = cd_d.beam_field;
    const FluidData &fluid = cd_d.fluid_field;
    const Hole &hole = cd_d.hole;
    const BCsData &bcs = cd_d.bc_data;
    const ConfigStatic &conf_stat = cd_d.conf_stat;
    const Pipe &pipe = cd_d.pipe;
    const uint top_node = cd_d.top_node;
    const uint N = pipe.N;
    
    
    // Calculate residuals for odd-numbered elements
    if (ie >= top_node && ie < N - 1) {
        calc_element_residual_ax_tor(ie, conf_stat, top_node, bcs, beam, fluid, pipe, hole, buf);
    }    
}
#endif

void initialize_forces_and_assemble_odd_cpu(const uint top_node, const ConfigStatic &conf_stat, const BCsData &bcs, BeamData &beam, 
                                                          const FluidData &fluid, const Pipe &pipe,const Hole &hole, byte *buf) {

    ArrayView<Vec3> f_int = beam.get_field<BeamField::f_int>(buf);
    ArrayView<Vec3> f_dyn = beam.get_field<BeamField::f_dyn>(buf);
    ArrayView<Vec3> m_int = beam.get_field<BeamField::m_int>(buf);
    ArrayView<Vec3> m_dyn = beam.get_field<BeamField::m_dyn>(buf);
    bool at_subcycling = conf_stat.enable_ax_tor_subcycle;

    const uint N = pipe.N;
    uint i, ie;

#pragma omp parallel for
    for_each_node_cpu(i) {
        initialize_forces(i, at_subcycling, f_int, f_dyn, m_int, m_dyn);
    }

#pragma omp parallel for
    for_each_element_cpu_odd(ie) {
        calc_element_residual( ie, conf_stat, top_node, bcs, beam, fluid, pipe, hole, buf);
    }
}


void initialize_forces_and_assemble_odd_ax_tor_cpu(const uint top_node, const ConfigStatic &conf_stat, const BCsData &bcs, BeamData &beam, 
                                                          const FluidData &fluid, const Pipe &pipe, const Hole &hole, byte *buf){

    ArrayView<Vec3> f_int = beam.get_field<BeamField::f_int>(buf);
    ArrayView<Vec3> f_dyn = beam.get_field<BeamField::f_dyn>(buf);
    ArrayView<Vec3> m_int = beam.get_field<BeamField::m_int>(buf);
    ArrayView<Vec3> m_dyn = beam.get_field<BeamField::m_dyn>(buf);
    
     
    const uint N = pipe.N;
    uint i, ie;

#pragma omp parallel for
    for_each_node_cpu(i) {
        f_int[i].x() = 0.0;
        m_int[i].x() = 0.0;

        f_dyn[i].y() = 0.0;
        f_dyn[i].z() = 0.0;
    }

#pragma omp parallel for
    for_each_element_cpu_odd(ie) {
        calc_element_residual_ax_tor( ie, conf_stat, top_node, bcs, beam, fluid, pipe, hole, buf);
    }
}


void assemble_even_ax_tor_cpu(const uint top_node, const ConfigStatic &conf_stat, const BCsData &bcs, BeamData &beam, 
                                                          const FluidData &fluid, const Pipe &pipe,const Hole &hole, byte *buf) {

    ArrayView<Vec3> f_int = beam.get_field<BeamField::f_int>(buf);
    ArrayView<Vec3> f_dyn = beam.get_field<BeamField::f_dyn>(buf);
    ArrayView<Vec3> m_int = beam.get_field<BeamField::m_int>(buf);
    ArrayView<Vec3> m_dyn = beam.get_field<BeamField::m_dyn>(buf);
    
     
    const uint N = pipe.N;
    uint ie;

#pragma omp parallel for
    for_each_element_cpu_even(ie) {
        calc_element_residual_ax_tor( ie, conf_stat, top_node, bcs, beam, fluid, pipe, hole, buf);
    }
}


#ifdef __CUDACC__
__global__ void integrate_time_gpu(byte *buf) {
    assert(blockIdx.y == 0 && blockIdx.z == 0 && blockDim.y == 1 && blockDim.z == 1);
    const uint i = blockDim.x * blockIdx.x + threadIdx.x; // ie is always even
    const ConfigStatic &conf_stat = cd_d.conf_stat;
    const Pipe &pipe = cd_d.pipe;
    const scalar dt = conf_stat.dt;
    const uint N = pipe.N;
    const uint top_node = cd_d.top_node;
    
    
    BeamData &beam = cd_d.beam_field;
    const Hole &hole = cd_d.hole;
    ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);
    ArrayView<Vec3> v = beam.get_field<BeamField::v>(buf);
    ArrayView<Vec3> a = beam.get_field<BeamField::a>(buf);
    ArrayView<Vec3> theta = beam.get_field<BeamField::theta>(buf);
    ArrayView<Vec3> omega = beam.get_field<BeamField::omega>(buf);
    ArrayView<Vec3> alpha = beam.get_field<BeamField::alpha>(buf);
    ArrayView<uint> indices = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);
    scalar alpha_rayleigh = conf_stat.rayleigh_damping_enabled ? conf_stat.alpha : 0.0;

    
    const ArrayView<scalar> m = beam.get_field<BeamField::m>(buf);
    const ArrayView<scalar> mfy = beam.get_field<BeamField::mfy>(buf);
    const ArrayView<scalar> mfz = beam.get_field<BeamField::mfz>(buf);
    const ArrayView<scalar> Jr = beam.get_field<BeamField::Jr>(buf);
    const ArrayView<scalar> Jx = beam.get_field<BeamField::Jx>(buf);
    const ArrayView<Vec3> f_dyn = beam.get_field<BeamField::f_dyn>(buf);
    const ArrayView<Vec3> f_int = beam.get_field<BeamField::f_int>(buf);
    const ArrayView<Vec3> f_hyd = beam.get_field<BeamField::f_hyd>(buf);
    const ArrayView<Vec3> m_dyn = beam.get_field<BeamField::m_dyn>(buf);
    const ArrayView<Vec3> m_int = beam.get_field<BeamField::m_int>(buf);
    const ArrayView<Vec3> m_hyd = beam.get_field<BeamField::m_hyd>(buf);
    
    // Calculate residuals for even-numbered elements
    if (i >= top_node && i < N) {
        integrate_central_diff(dt, u[i].x(), v[i].x(), a[i].x(), m[i], f_hyd[i].x(), f_dyn[i].x(), f_int[i].x(), alpha_rayleigh);
        integrate_central_diff(dt, u[i].y(), v[i].y(), a[i].y(), m[i] + mfy[i], f_hyd[i].y(), f_dyn[i].y(), f_int[i].y(), alpha_rayleigh);
        integrate_central_diff(dt, u[i].z(), v[i].z(), a[i].z(), m[i] + mfz[i], f_hyd[i].z(), f_dyn[i].z(), f_int[i].z(), alpha_rayleigh);
        integrate_central_diff(dt, theta[i].x(), omega[i].x(), alpha[i].x(), Jx[i], m_hyd[i].x(), m_dyn[i].x(), m_int[i].x(), alpha_rayleigh);
        integrate_central_diff(dt, theta[i].y(), omega[i].y(), alpha[i].y(), Jr[i], m_hyd[i].y(), m_dyn[i].y(), m_int[i].y(), alpha_rayleigh);
        integrate_central_diff(dt, theta[i].z(), omega[i].z(), alpha[i].z(), Jr[i], m_hyd[i].z(), m_dyn[i].z(), m_int[i].z(), alpha_rayleigh);
        update_indices_node(i, N, u[i].x(), indices, hole, pipe, buf);
    }
}
#endif


#ifdef __CUDACC__
__global__ void assemble_even_gpu(byte *buf) {
    assert(blockIdx.y == 0 && blockIdx.z == 0 && blockDim.y == 1 && blockDim.z == 1);
    const uint ie = 2 * (blockDim.x * blockIdx.x + threadIdx.x); // ie is always even
    assert(ie % 2 == 0);

    const ConfigStatic &conf_stat = cd_d.conf_stat;
    const BCsData &bcs = cd_d.bc_data;
    const Pipe &pipe = cd_d.pipe;
    const Hole &hole = cd_d.hole;
    const FluidData &fluid = cd_d.fluid_field;
    BeamData &beam = cd_d.beam_field;

    const uint N = pipe.N;
    const uint top_node = cd_d.top_node;
    const uint Ne = N - 1;
    
    // Calculate residuals for even-numbered elements
    if (ie >= top_node && ie < Ne) {
        calc_element_residual(ie, conf_stat, top_node, bcs, beam, fluid, pipe, hole, buf);
    }
}
#endif


#ifdef __CUDACC__
__global__ void assemble_even_ax_tor_gpu(byte *buf) {
    assert(blockIdx.y == 0 && blockIdx.z == 0 && blockDim.y == 1 && blockDim.z == 1);
    const uint ie = 2 * (blockDim.x * blockIdx.x + threadIdx.x); // ie is always even
    assert(ie % 2 == 0);

    const ConfigStatic &conf_stat = cd_d.conf_stat;
    const BCsData &bcs = cd_d.bc_data;
    const Pipe &pipe = cd_d.pipe;
    const Hole &hole = cd_d.hole;
    const FluidData &fluid = cd_d.fluid_field;
    BeamData &beam = cd_d.beam_field;

    const uint N = pipe.N;
    const uint top_node = cd_d.top_node;
    const uint Ne = N - 1;
    
    // Calculate residuals for even-numbered elements
    if (ie >= top_node && ie < Ne) {
        calc_element_residual_ax_tor(ie, conf_stat, top_node, bcs, beam, fluid, pipe, hole, buf);
    }
}
#endif

#ifdef __CUDACC__
__global__ void calc_contact_even_gpu(byte *buf) {
    assert(blockIdx.y == 0 && blockIdx.z == 0 && blockDim.y == 1 && blockDim.z == 1);
    const uint ie = 2 * (blockDim.x * blockIdx.x + threadIdx.x); // ie is always even
    assert(ie % 2 == 0);

    const ConfigStatic &conf_stat = cd_d.conf_stat;
    const Pipe &pipe = cd_d.pipe;
    const Hole &hole = cd_d.hole;
    BeamData &beam = cd_d.beam_field;

    const uint N = pipe.N;
    const uint top_node = cd_d.top_node;
    const uint Ne = N - 1;
    
    // Calculate residuals for even-numbered elements
    if (ie >= top_node && ie < Ne) {
        calc_element_contact(ie, conf_stat, top_node, beam, pipe, hole, buf);
    }
}
#endif

#ifdef __CUDACC__
__global__ void calc_contact_odd_gpu(byte *buf) {
    assert(blockIdx.y == 0 && blockIdx.z == 0 && blockDim.y == 1 && blockDim.z == 1);
    const uint ie = 2 * (blockDim.x * blockIdx.x + threadIdx.x) + 1; // ie is always odd
    assert(ie % 2 == 1);

    const ConfigStatic &conf_stat = cd_d.conf_stat;
    const Pipe &pipe = cd_d.pipe;
    const Hole &hole = cd_d.hole;
    BeamData &beam = cd_d.beam_field;

    const uint N = pipe.N;
    const uint top_node = cd_d.top_node;
    const uint Ne = N - 1;
    
    // Calculate residuals for even-numbered elements
    if (ie >= top_node && ie < Ne) {
        calc_element_contact(ie, conf_stat, top_node, beam, pipe, hole, buf);
    }
}
#endif




#ifdef __CUDACC__
__global__ void integrate_time_lateral_gpu(byte *buf) {
    assert(blockIdx.y == 0 && blockIdx.z == 0 && blockDim.y == 1 && blockDim.z == 1);
    const uint i = blockDim.x * blockIdx.x + threadIdx.x; // ie is always even
    const ConfigStatic &conf_stat = cd_d.conf_stat;
    const Pipe &pipe = cd_d.pipe;
    const Hole &hole = cd_d.hole;
    const scalar dt = conf_stat.dt;
    const uint N = pipe.N;
    const uint top_node = cd_d.top_node;
    scalar alpha_rayleigh = conf_stat.rayleigh_damping_enabled ? conf_stat.alpha : 0.0;

    BeamData &beam = cd_d.beam_field;
    ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);
    ArrayView<Vec3> v = beam.get_field<BeamField::v>(buf);
    ArrayView<Vec3> a = beam.get_field<BeamField::a>(buf);
    ArrayView<Vec3> theta = beam.get_field<BeamField::theta>(buf);
    ArrayView<Vec3> omega = beam.get_field<BeamField::omega>(buf);
    ArrayView<Vec3> alpha = beam.get_field<BeamField::alpha>(buf);
    ArrayView<uint> indices = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);
    
    const ArrayView<scalar> rc = pipe.get_field<PipeField::rc>(buf);
    const ArrayView<uint> num_ic_to_ie = pipe.get_field<PipeField::num_ic_to_ie>(buf);
    

    const ArrayView<scalar> m = beam.get_field<BeamField::m>(buf);
    const ArrayView<scalar> mfy = beam.get_field<BeamField::mfy>(buf);
    const ArrayView<scalar> mfz = beam.get_field<BeamField::mfz>(buf);
    const ArrayView<scalar> Jr = beam.get_field<BeamField::Jr>(buf);
    
    ArrayView<Vec3> f_dyn = beam.get_field<BeamField::f_dyn>(buf);
    ArrayView<Vec3> m_dyn = beam.get_field<BeamField::m_dyn>(buf);
    
    const ArrayView<Vec3> f_int = beam.get_field<BeamField::f_int>(buf);
    const ArrayView<Vec3> m_int = beam.get_field<BeamField::m_int>(buf);
    const ArrayView<Vec3> f_hyd = beam.get_field<BeamField::f_hyd>(buf);
    const ArrayView<Vec3> m_hyd = beam.get_field<BeamField::m_hyd>(buf);
    
    // Calculate residuals for even-numbered elements
    if (i >= top_node && i < N) {
            integrate_central_diff(dt, u[i].y(), v[i].y(), a[i].y(), m[i] + mfy[i], f_hyd[i].y(), f_dyn[i].y(), f_int[i].y(), alpha_rayleigh);
            integrate_central_diff(dt, u[i].z(), v[i].z(), a[i].z(), m[i] + mfz[i], f_hyd[i].z(), f_dyn[i].z(), f_int[i].z(), alpha_rayleigh);
            integrate_central_diff(dt, theta[i].y(), omega[i].y(), alpha[i].y(), Jr[i], m_hyd[i].y(), m_dyn[i].y(), m_int[i].y(), alpha_rayleigh);
            integrate_central_diff(dt, theta[i].z(), omega[i].z(), alpha[i].z(), Jr[i], m_hyd[i].z(), m_dyn[i].z(), m_int[i].z(), alpha_rayleigh);
            update_indices_node(i, N, u[i].x(), indices, hole, pipe, buf);
        }
}
#endif

#ifdef __CUDACC__
__global__ void integrate_time_ax_tor_gpu(scalar dt, byte *buf) {
    assert(blockIdx.y == 0 && blockIdx.z == 0 && blockDim.y == 1 && blockDim.z == 1);
    const uint i = blockDim.x * blockIdx.x + threadIdx.x; // ie is always even
    const ConfigStatic &conf_stat = cd_d.conf_stat;
    const Pipe &pipe = cd_d.pipe;
    const Hole &hole = cd_d.hole;
    const uint N = pipe.N;
    const uint top_node = cd_d.top_node;
    scalar alpha_rayleigh = conf_stat.rayleigh_damping_enabled ? conf_stat.alpha : 0.0;
    
    BeamData &beam = cd_d.beam_field;
    ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);
    ArrayView<Vec3> v = beam.get_field<BeamField::v>(buf);
    ArrayView<Vec3> a = beam.get_field<BeamField::a>(buf);
    ArrayView<Vec3> theta = beam.get_field<BeamField::theta>(buf);
    ArrayView<Vec3> omega = beam.get_field<BeamField::omega>(buf);
    ArrayView<Vec3> alpha = beam.get_field<BeamField::alpha>(buf);
    ArrayView<uint> indices = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);
    
    const ArrayView<scalar> rc = pipe.get_field<PipeField::rc>(buf);
    const ArrayView<uint> num_ic_to_ie = pipe.get_field<PipeField::num_ic_to_ie>(buf);
    

    const ArrayView<scalar> m = beam.get_field<BeamField::m>(buf);
    const ArrayView<scalar> Jx = beam.get_field<BeamField::Jx>(buf);
    
    ArrayView<Vec3> f_dyn = beam.get_field<BeamField::f_dyn>(buf);
    ArrayView<Vec3> m_dyn = beam.get_field<BeamField::m_dyn>(buf);
    
    const ArrayView<Vec3> f_int = beam.get_field<BeamField::f_int>(buf);
    const ArrayView<Vec3> m_int = beam.get_field<BeamField::m_int>(buf);
    const ArrayView<Vec3> f_hyd = beam.get_field<BeamField::f_hyd>(buf);
    const ArrayView<Vec3> m_hyd = beam.get_field<BeamField::m_hyd>(buf);
    
    // Calculate residuals for even-numbered elements
    if (i >= top_node && i < N) {
        calc_contact_friction_soft_string(i, conf_stat, hole, pipe, beam, buf);

        integrate_central_diff(dt, u[i].x(), v[i].x(), a[i].x(), m[i], f_hyd[i].x(), f_dyn[i].x(), f_int[i].x(), alpha_rayleigh);
        integrate_central_diff(dt, theta[i].x(), omega[i].x(), alpha[i].x(), Jx[i], m_hyd[i].x(), m_dyn[i].x(), m_int[i].x(), alpha_rayleigh);
        update_indices_node(i, N, u[i].x(), indices, hole, pipe, buf);
    }
}
#endif


void integrate_time_ax_tor_cpu(scalar dt, const ConfigStatic &conf_stat, uint top_node, BeamData &beam,
                               const Pipe &pipe, const Hole &hole, byte *buf)
{
    uint N = pipe.N;
    scalar alpha_rayleigh = conf_stat.rayleigh_damping_enabled ? conf_stat.alpha : 0.0;
    

    ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);
    ArrayView<Vec3> v = beam.get_field<BeamField::v>(buf);
    ArrayView<Vec3> a = beam.get_field<BeamField::a>(buf);
    ArrayView<Vec3> theta = beam.get_field<BeamField::theta>(buf);
    ArrayView<Vec3> omega = beam.get_field<BeamField::omega>(buf);
    ArrayView<Vec3> alpha = beam.get_field<BeamField::alpha>(buf);
    ArrayView<uint> indices = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);
    
    const ArrayView<scalar> rc = pipe.get_field<PipeField::rc>(buf);
    const ArrayView<uint> num_ic_to_ie = pipe.get_field<PipeField::num_ic_to_ie>(buf);
    

    const ArrayView<scalar> m = beam.get_field<BeamField::m>(buf);
    const ArrayView<scalar> Jx = beam.get_field<BeamField::Jx>(buf);
    
    ArrayView<Vec3> f_dyn = beam.get_field<BeamField::f_dyn>(buf);
    ArrayView<Vec3> m_dyn = beam.get_field<BeamField::m_dyn>(buf);
    
    const ArrayView<Vec3> f_int = beam.get_field<BeamField::f_int>(buf);
    const ArrayView<Vec3> m_int = beam.get_field<BeamField::m_int>(buf);
    const ArrayView<Vec3> f_hyd = beam.get_field<BeamField::f_hyd>(buf);
    const ArrayView<Vec3> m_hyd = beam.get_field<BeamField::m_hyd>(buf);
    

    uint i;
    #pragma omp parallel for
    for_each_node_cpu(i) {
        calc_contact_friction_soft_string(i, conf_stat, hole, pipe, beam, buf);

        integrate_central_diff(dt, u[i].x(), v[i].x(), a[i].x(), m[i], f_hyd[i].x(), f_dyn[i].x(), f_int[i].x(), alpha_rayleigh);
        integrate_central_diff(dt, theta[i].x(), omega[i].x(), alpha[i].x(), Jx[i], m_hyd[i].x(), m_dyn[i].x(), m_int[i].x(), alpha_rayleigh);
        update_indices_node(i, N, u[i].x(), indices, hole, pipe, buf);
    }
}


// Integrate ONLY lateral (u.y,u.z) and tilts (theta.y,theta.z) once per coarse step
void integrate_time_lateral_cpu(const ConfigStatic &conf_stat, uint top_node, BeamData &beam,
                                const Pipe &pipe, const Hole &hole, byte *buf)
{
    const uint N = pipe.N;
    const scalar dt = conf_stat.dt;

    ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);
    ArrayView<Vec3> v = beam.get_field<BeamField::v>(buf);
    ArrayView<Vec3> a = beam.get_field<BeamField::a>(buf);
    ArrayView<Vec3> theta = beam.get_field<BeamField::theta>(buf);
    ArrayView<Vec3> omega = beam.get_field<BeamField::omega>(buf);
    ArrayView<Vec3> alpha = beam.get_field<BeamField::alpha>(buf);
    ArrayView<uint> indices = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);
    scalar alpha_rayleigh = conf_stat.rayleigh_damping_enabled ? conf_stat.alpha : 0.0;

    
    const ArrayView<scalar> m = beam.get_field<BeamField::m>(buf);
    const ArrayView<scalar> mfy = beam.get_field<BeamField::mfy>(buf);
    const ArrayView<scalar> mfz = beam.get_field<BeamField::mfz>(buf);
    const ArrayView<scalar> Jr = beam.get_field<BeamField::Jr>(buf);
    const ArrayView<Vec3> f_dyn = beam.get_field<BeamField::f_dyn>(buf);
    const ArrayView<Vec3> f_int = beam.get_field<BeamField::f_int>(buf);
    const ArrayView<Vec3> f_hyd = beam.get_field<BeamField::f_hyd>(buf);
    const ArrayView<Vec3> m_dyn = beam.get_field<BeamField::m_dyn>(buf);
    const ArrayView<Vec3> m_int = beam.get_field<BeamField::m_int>(buf);
    const ArrayView<Vec3> m_hyd = beam.get_field<BeamField::m_hyd>(buf);
    
    
    uint i;
    #pragma omp parallel for
    for_each_node_cpu(i) {
        if (i >= top_node && i < N) {
            integrate_central_diff(dt, u[i].y(), v[i].y(), a[i].y(), m[i] + mfy[i], f_hyd[i].y(), f_dyn[i].y(), f_int[i].y(), alpha_rayleigh);
            integrate_central_diff(dt, u[i].z(), v[i].z(), a[i].z(), m[i] + mfz[i], f_hyd[i].z(), f_dyn[i].z(), f_int[i].z(), alpha_rayleigh);
            integrate_central_diff(dt, theta[i].y(), omega[i].y(), alpha[i].y(), Jr[i], m_hyd[i].y(), m_dyn[i].y(), m_int[i].y(), alpha_rayleigh);
            integrate_central_diff(dt, theta[i].z(), omega[i].z(), alpha[i].z(), Jr[i], m_hyd[i].z(), m_dyn[i].z(), m_int[i].z(), alpha_rayleigh);
            update_indices_node(i, N, u[i].x(), indices, hole, pipe, buf);
        }
    }
}


void integrate_time_cpu(const ConfigStatic &conf_stat, const uint top_node, BeamData &beam,
                                      const Pipe &pipe, const Hole &hole,
                                      byte *buf){
    
    const uint N = pipe.N;
    const scalar dt = conf_stat.dt;
    
    ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);
    ArrayView<Vec3> v = beam.get_field<BeamField::v>(buf);
    ArrayView<Vec3> a = beam.get_field<BeamField::a>(buf);
    ArrayView<Vec3> theta = beam.get_field<BeamField::theta>(buf);
    ArrayView<Vec3> omega = beam.get_field<BeamField::omega>(buf);
    ArrayView<Vec3> alpha = beam.get_field<BeamField::alpha>(buf);
    ArrayView<uint> indices = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);
    scalar alpha_rayleigh = conf_stat.rayleigh_damping_enabled ? conf_stat.alpha : 0.0;


    
    const ArrayView<scalar> m = beam.get_field<BeamField::m>(buf);
    const ArrayView<scalar> mfy = beam.get_field<BeamField::mfy>(buf);
    const ArrayView<scalar> mfz = beam.get_field<BeamField::mfz>(buf);
    const ArrayView<scalar> Jr = beam.get_field<BeamField::Jr>(buf);
    const ArrayView<scalar> Jx = beam.get_field<BeamField::Jx>(buf);
    const ArrayView<Vec3> f_dyn = beam.get_field<BeamField::f_dyn>(buf);
    const ArrayView<Vec3> f_int = beam.get_field<BeamField::f_int>(buf);
    const ArrayView<Vec3> f_hyd = beam.get_field<BeamField::f_hyd>(buf);
    const ArrayView<Vec3> m_dyn = beam.get_field<BeamField::m_dyn>(buf);
    const ArrayView<Vec3> m_int = beam.get_field<BeamField::m_int>(buf);
    const ArrayView<Vec3> m_hyd = beam.get_field<BeamField::m_hyd>(buf);
    
    uint i;

    #pragma omp parallel for
    for_each_node_cpu(i) {

        integrate_central_diff(dt, u[i].x(), v[i].x(), a[i].x(), m[i], f_hyd[i].x(), f_dyn[i].x(), f_int[i].x(), alpha_rayleigh);
        integrate_central_diff(dt, u[i].y(), v[i].y(), a[i].y(), m[i] + mfy[i], f_hyd[i].y(), f_dyn[i].y(), f_int[i].y(), alpha_rayleigh);
        integrate_central_diff(dt, u[i].z(), v[i].z(), a[i].z(), m[i] + mfz[i], f_hyd[i].z(), f_dyn[i].z(), f_int[i].z(), alpha_rayleigh);
        integrate_central_diff(dt, theta[i].x(), omega[i].x(), alpha[i].x(), Jx[i], m_hyd[i].x(), m_dyn[i].x(), m_int[i].x(), alpha_rayleigh);
        integrate_central_diff(dt, theta[i].y(), omega[i].y(), alpha[i].y(), Jr[i], m_hyd[i].y(), m_dyn[i].y(), m_int[i].y(), alpha_rayleigh);
        integrate_central_diff(dt, theta[i].z(), omega[i].z(), alpha[i].z(), Jr[i], m_hyd[i].z(), m_dyn[i].z(), m_int[i].z(), alpha_rayleigh);
        update_indices_node(i, N, u[i].x(), indices, hole, pipe, buf);
    }

}

void assemble_even_cpu(const ConfigStatic &conf_stat, const uint top_node, const BCsData &bcs, BeamData &beam,
                                      const FluidData &fluid, const Pipe &pipe, const Hole &hole,
                                      byte *buf){
    const uint N = pipe.N;
    uint ie;
    for_each_element_cpu_even(ie) {
        calc_element_residual(ie, conf_stat, top_node, bcs, beam, fluid, pipe, hole, buf);
    }
 
}


 
void initialize_forces_and_assemble_odd(const uint grid_dim, const uint block_dim, const Config &config, const uint top_node, const BCsData &bc, BeamData &beam, 
                                                const FluidData &fluid, const Pipe &pipe,const Hole &hole, byte *buf) {
    if (config.use_gpu) {
#ifdef __CUDACC__
        CUDA_CHECK_LAST_ERROR();
        initialize_forces_gpu<<<grid_dim, block_dim>>>(buf);   
        assemble_odd_gpu<<<grid_dim, block_dim>>>(buf);
        CUDA_CHECK_LAST_ERROR();
        
#endif
    } else {
        initialize_forces_and_assemble_odd_cpu(top_node, config.conf_stat, bc, beam, fluid, pipe, hole, buf);
    }
}



void assemble_even(const uint grid_dim, const uint block_dim, 
                                    const Config &config, const uint top_node, const BCsData &bc, BeamData &beam, 
                                    const FluidData &fluid, const Pipe &pipe,const Hole &hole, byte *buf) {
    if (config.use_gpu) {
#ifdef __CUDACC__
        CUDA_CHECK_LAST_ERROR();
        assemble_even_gpu<<<grid_dim, block_dim>>>(buf);
        CUDA_CHECK_LAST_ERROR();
#endif
    } else {
        assemble_even_cpu(config.conf_stat, top_node, bc, beam, fluid, pipe, hole, buf);
    }
}


void initialize_forces_and_assemble_odd_ax_tor(const uint grid_dim, const uint block_dim, const Config &config, const uint top_node, const BCsData &bc, BeamData &beam, 
                                                const FluidData &fluid, const Pipe &pipe,const Hole &hole, byte *buf) {
    if (config.use_gpu) {
#ifdef __CUDACC__
        CUDA_CHECK_LAST_ERROR();
        initialize_forces_ax_tor_gpu<<<grid_dim, block_dim>>>(buf);
        assemble_odd_ax_tor_gpu<<<grid_dim, block_dim>>>(buf);
        CUDA_CHECK_LAST_ERROR();
#endif
    } else {
        initialize_forces_and_assemble_odd_ax_tor_cpu(top_node, config.conf_stat, bc,  beam, fluid, pipe, hole, buf);
    }
}





void assemble_even_ax_tor(const uint grid_dim, const uint block_dim, 
                                    const Config &config, const uint top_node, const BCsData &bcs, BeamData &beam, 
                                     const FluidData &fluid, const Pipe &pipe,const Hole &hole, byte *buf) {
    if (config.use_gpu) {
#ifdef __CUDACC__
        CUDA_CHECK_LAST_ERROR();
        assemble_even_ax_tor_gpu<<<grid_dim, block_dim>>>(buf);
        CUDA_CHECK_LAST_ERROR();
#endif
    } else {
        assemble_even_ax_tor_cpu(config.conf_stat, top_node, bcs, beam, fluid, pipe, hole, buf);
    }
}


void calc_contact_forces_nodewise_cpu(const ConfigStatic &conf_stat, const uint top_node,
    const Pipe &pipe, const Hole &hole, BeamData &beam, byte *buf){
    
        
    uint N = pipe.N;
    uint i;

#pragma omp parallel for
        for_each_node_cpu(i) { 
            calc_contact_friction_nodewise_ax_tor(i, conf_stat, pipe, hole, beam, buf);
        } 
     
}


#ifdef __CUDACC__
__global__ void calc_contact_forces_nodewise_gpu(byte *buf) {
    assert(blockIdx.y == 0 && blockIdx.z == 0 && blockDim.y == 1 && blockDim.z == 1);
    const uint i = blockDim.x * blockIdx.x + threadIdx.x; // ie is always even
    
    BeamData &beam = cd_d.beam_field;
    const ConfigStatic &conf_stat = cd_d.conf_stat;
    const Pipe &pipe = cd_d.pipe;
    const Hole &hole = cd_d.hole;
    const uint N = pipe.N;
    const uint top_node = cd_d.top_node;
    
    if (i >= top_node && i < N) {   
            calc_contact_friction_nodewise_ax_tor(i, conf_stat, pipe, hole, beam, buf);
    }
}
#endif


void calc_nodal_friction_forces(const uint grid_dim, const uint block_dim, const Config &config, 
        const uint top_node,  const Pipe &pipe, const Hole &hole, BeamData &beam, byte *buf){
    if (!config.conf_stat.contact_enabled){
        return;
    }
     
    if (config.use_gpu) {
        #ifdef __CUDACC__
        CUDA_CHECK_LAST_ERROR();
        calc_contact_forces_nodewise_gpu<<<grid_dim, block_dim>>>(buf);
        CUDA_CHECK_LAST_ERROR();
        #endif
    } else {
        calc_contact_forces_nodewise_cpu(config.conf_stat, top_node, pipe, hole, beam, buf);
    }
}




void integrate_time(const uint grid_dim, const uint block_dim, 
                                    const Config &config, const uint top_node, BeamData &beam, 
                                    const Pipe &pipe, const Hole &hole, byte *buf) {
    if (config.use_gpu) {
#ifdef __CUDACC__
        CUDA_CHECK_LAST_ERROR();
        integrate_time_gpu<<<grid_dim, block_dim>>>(buf);
        CUDA_CHECK_LAST_ERROR();
#endif
    } else {
        integrate_time_cpu(config.conf_stat, top_node, beam, pipe, hole, buf);
    }
}

void integrate_time_ax_tor(const uint grid_dim, const uint block_dim, scalar dt,
                                    const Config &config, const uint top_node, BeamData &beam,  
                                    const Pipe &pipe,const Hole &hole, byte *buf) {
    if (config.use_gpu) {
#ifdef __CUDACC__
        CUDA_CHECK_LAST_ERROR();
        integrate_time_ax_tor_gpu<<<grid_dim, block_dim>>>(dt, buf);
        CUDA_CHECK_LAST_ERROR();
#endif
    } else {
        integrate_time_ax_tor_cpu(dt, config.conf_stat, top_node, beam, pipe, hole, buf);
    }
}

void integrate_time_lateral(const uint grid_dim, const uint block_dim, 
                                    const Config &config, const uint top_node, BeamData &beam,  
                                    const Pipe &pipe,const Hole &hole, byte *buf) {
    if (config.use_gpu) {
#ifdef __CUDACC__
        CUDA_CHECK_LAST_ERROR();
        integrate_time_lateral_gpu<<<grid_dim, block_dim>>>(buf);
        CUDA_CHECK_LAST_ERROR();
#endif
    } else {
        integrate_time_lateral_cpu(config.conf_stat, top_node, beam, pipe, hole, buf);
    }
}

void initialize_beam_segment_solution(uint start_node, uint end_node, const Hole &hole, const Pipe &pipe,
                                      BeamData &beam, const BCsData &bcs, byte *buf) {

    const BCs &bcs_field = bcs.get_bcs_field(buf);

    const scalar u_top = bcs_field.u_top_np;
    const scalar v_top = bcs_field.v_top_np;
    const scalar a_top = bcs_field.a_top_np;
    const scalar theta_top = bcs_field.theta_top_np;
    const scalar omega_top = bcs_field.omega_top_np;
    const scalar omega_dot_top = bcs_field.alpha_top_np;

    ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);
    ArrayView<Vec3> v = beam.get_field<BeamField::v>(buf);
    ArrayView<Vec3> a = beam.get_field<BeamField::a>(buf);
    ArrayView<Vec3> theta = beam.get_field<BeamField::theta>(buf);
    ArrayView<Vec3> omega = beam.get_field<BeamField::omega>(buf);
    ArrayView<Vec3> alpha = beam.get_field<BeamField::alpha>(buf);
    ArrayView<uint> i_pipe_to_ie_hole = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);
    
    for (uint i = start_node; i < end_node; i++) {
        u[i].x() = u_top;
        u[i].y() = 0.0;
        u[i].z() = 0.0;
        theta[i].x() = theta_top;
        theta[i].y() = 0.0;
        theta[i].z() = 0.0;

        v[i].x() = v_top;
        v[i].y() = 0.0;
        v[i].z() = 0.0;
        omega[i].x() = omega_top;
        omega[i].y() = 0.0;
        omega[i].z() = 0.0;

        a[i].x() = a_top;
        a[i].y() = 0.0;
        a[i].z() = 0.0;
        alpha[i].x() = omega_dot_top;
        alpha[i].y() = 0.0;
        alpha[i].z() = 0.0;

        const scalar s_i = pipe.calc_s(i, u_top, buf);
        const uint i_hole = hole.get_increment_index_pipe_node_to_hole_segment(0, s_i, buf);
        assert(i_hole == 0 || i_hole == 1);
        i_pipe_to_ie_hole[i] = i_hole;

    }
}




} // namespace curvlin
