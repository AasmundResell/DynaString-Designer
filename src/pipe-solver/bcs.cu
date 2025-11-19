#include "bcs.hpp"
#include "pipe-solver/pipe-solver-curvlin/numerics/constant-data.hpp"

BCsData create_bcs(ArenaBump &arena_h) {
    BCsData bcs{};
    arena_h.allocate<BCs>(&bcs.bcs_field, 1);
    
    bcs.get_bcs_field(arena_h.buf) = BCs{};
    return bcs;
}

DEVICE_FUNC  void update_top_kinematics(scalar dt, BCs &bcs_field, scalar v_top_target,
                                         scalar omega_top_target) {
    // To alter top condition, update v_top_input/omega_top_input
    bcs_field.u_top_n = bcs_field.u_top_np;
    bcs_field.v_top_n = bcs_field.v_top_np;
    bcs_field.a_top_n = bcs_field.a_top_np;
    bcs_field.u_top_np = bcs_field.u_top_n + dt * bcs_field.v_top_n + dt *dt *bcs_field.a_top_n / 2.0;
    bcs_field.v_top_np = v_top_target;
    bcs_field.a_top_np = 2 / dt * (bcs_field.v_top_np - bcs_field.v_top_n) - bcs_field.a_top_n;
    //assert(bcs_field.u_top_np >= 0.0);

    bcs_field.theta_top_n = bcs_field.theta_top_np;
    bcs_field.omega_top_n = bcs_field.omega_top_np;
    bcs_field.alpha_top_n = bcs_field.alpha_top_np;
    bcs_field.theta_top_np =
        bcs_field.theta_top_n + dt * bcs_field.omega_top_n + dt * dt * bcs_field.alpha_top_np / 2.0;

    bcs_field.omega_top_np = omega_top_target;

    bcs_field.alpha_top_np = 2 / dt * (bcs_field.omega_top_np - bcs_field.omega_top_n) - bcs_field.alpha_top_n;
    
}


/*--------------------------------------------------------------------
Update the index of the first active node based on how much of the
string has been fed into the pipe. 
--------------------------------------------------------------------*/
void update_top_node(Config &config, ConfigDynamic &conf_dyn, const Pipe &pipe, const vector<PipeComponent> &pipe_assembly) {
    
    scalar &block_position = conf_dyn.block_position;
    scalar &feed_length = conf_dyn.feed_length;
    uint &top_component = config.top_component;

    const scalar dt = config.conf_stat.dt;
    assert(abs(conf_dyn.v_top * dt) <= pipe.dS_min);

    block_position -= conf_dyn.v_top * dt;
    feed_length += conf_dyn.v_top * dt; 
    if (block_position <= 0) {  
        if (top_component == 0) {
            conf_dyn.v_top_target = 0.0;
            if (config.conf_stat.v_top_input > 0.0) {
                printf("Warning: Pipe assembly is fully fed into the hole, setting top velocity to zero\n");
                config.pipe_fully_fed = true;
                config.conf_stat.v_top_input = 0.0;
            }
            block_position = 0.0;
        } else {
            // Block hoisted up to feed next component
            top_component--;
            block_position = pipe_assembly[top_component].L;
        }
    }
    // Current component pulled out of the hole
    else if (block_position > pipe_assembly[top_component].L) { 
        if (top_component == pipe_assembly.size() - 1) {
            conf_dyn.v_top_target = 0.0;
            if (config.conf_stat.v_top_input < 0.0) {
                printf("Warning: Pipe assembly is fully pulled out of the hole, setting top velocity to zero\n");
                config.pipe_fully_pulled = true;
                config.conf_stat.v_top_input = 0.0;
            }
            block_position = pipe_assembly[top_component].L;
        }
        else { 
            //Block hoisted down to retrieve next component
            block_position = 0;
            top_component++;
        }
    }

    config.top_node_new  = pipe_assembly[config.top_component].i_top_node;
    assert(config.top_node_new < pipe.N);
    return;
}

uint set_initial_top_component(const vector<PipeComponent> &pipe_assembly, scalar L_outside) {
    // Start from bottom (highest index) and work up
    scalar L_accumulated = 0.0;
    for (int i = 0; i < pipe_assembly.size(); i++) {
        L_accumulated += pipe_assembly[i].L;
        if (L_accumulated > L_outside) {
            return i;  // This component is partially inserted
        }
    }
    
    // If we've fed in more than total length, return top component (index 0)
    return pipe_assembly.size() - 1;
}
scalar set_initial_block_position(const uint top_component, const vector<PipeComponent> &pipe_assembly, scalar L_outside) {
    // Find how much of the first active component is sticking out
    scalar L_accumulated = 0.0;
    for (uint i = 0; i < top_component; i++) {
        L_accumulated += pipe_assembly[i].L;
    }


    return L_outside - L_accumulated;
}

void set_pipe_feed_length_initial(Config &config, ConfigDynamic &conf_dyn, BCsData &bcs, const Pipe &pipe, const vector<PipeComponent> &pipe_assembly, byte *buf) {
    const scalar L_string_depth_initial = config.conf_stat.L_string_depth_initial;
    BCs &bcs_field = bcs.get_bcs_field(buf);
    conf_dyn.feed_length = L_string_depth_initial;
    bcs_field.u_top_np = L_string_depth_initial;
    update_top_kinematics(config.conf_stat.dt, bcs_field, config.conf_stat.v_top_input, config.conf_stat.omega_top_input);
    
    //No acceleration as initial condition
    bcs_field.a_top_np = 0.0;
    bcs_field.alpha_top_np = 0.0; 


    const scalar L_outside = pipe.L_tot - L_string_depth_initial;
    config.top_component = set_initial_top_component(pipe_assembly, L_outside);
    conf_dyn.block_position = set_initial_block_position(config.top_component, pipe_assembly, L_outside);
    config.top_node = pipe_assembly[config.top_component].i_top_node;
    config.top_node_new = config.top_node;
}


namespace curvlin {

DEVICE_FUNC void update_bc_force_dynamic(const uint top_node, const BCsData &bcs,
    const ConfigStatic &conf_stat, const Pipe &pipe, BitRockData &bit_rock_data, 
    BeamData &beam, byte *buf) {
    const uint N = pipe.N;
    ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);
    ArrayView<Vec3> v = beam.get_field<BeamField::v>(buf);

    ArrayView<Vec3> theta = beam.get_field<BeamField::theta>(buf);
    ArrayView<Vec3> omega = beam.get_field<BeamField::omega>(buf);
    ArrayView<Vec3> alpha = beam.get_field<BeamField::alpha>(buf);
    
    ArrayView<Vec3> f_int = beam.get_field<BeamField::f_int>(buf);
    ArrayView<Vec3> f_dyn = beam.get_field<BeamField::f_dyn>(buf);
    ArrayView<Vec3> f_hyd = beam.get_field<BeamField::f_hyd>(buf);
    ArrayView<Vec3> m_dyn = beam.get_field<BeamField::m_dyn>(buf);
    ArrayView<Vec3> m_hyd = beam.get_field<BeamField::m_hyd>(buf);
    ArrayView<Vec3> m_int = beam.get_field<BeamField::m_int>(buf);

    //These could technically be moved to static forces!
    scalar WOB = conf_stat.WOB_constant;
    scalar TOB = conf_stat.TOB_constant;
    
    if (conf_stat.bc_bit_rock_type != BC_BitRockType::NO_BIT) { 

        BitRock &bit_rock = bit_rock_data.get_bit_rock_field(buf);
        
        scalar v_bit = v[N - 1].x();   
        scalar u_bit = u[N - 1].x();
        scalar omega_bit = omega[N - 1].x();
        
        bit_rock.s_blade = u_bit;        
        bit_rock.theta_old = bit_rock.theta;
        scalar theta_bit = theta[N - 1].x();
        bit_rock.theta = theta_bit;
        bit_rock.omega = omega_bit;


        update_bit_kinematics(conf_stat, bit_rock_data, buf);
        
        if (conf_stat.bc_rock_cutting_analytical){
            // Update bit kinematics and calculate current depth of cut
            cut_rock_surface(conf_stat, bit_rock_data, buf);
        }
        else {
            cut_rock_surface_PDE(conf_stat, bit_rock_data, buf);
        }


        scalar f_reaction = f_dyn[N - 1].x() + f_hyd[N - 1].x() - WOB - f_int[N - 1].x();
        scalar m_reaction = m_dyn[N - 1].x() + m_hyd[N - 1].x() - TOB - m_int[N - 1].x();

        // Calculate current bit-rock forces based on updated depth of cut
        if (conf_stat.bc_bit_rock_type == BC_BitRockType::DETOURNAY){
            calculate_bit_rock_Detournay_w_reaction(conf_stat, bit_rock, v_bit, omega_bit, f_reaction, m_reaction);
        } else if (conf_stat.bc_bit_rock_type == BC_BitRockType::DETOURNAY_REGULARIZED){
            calculate_bit_rock_Detournay_regularized(conf_stat, bit_rock, v_bit);
        } else if (conf_stat.bc_bit_rock_type == BC_BitRockType ::TUCKER_WANG) {
            calculate_bit_rock_Tucker_Wang(conf_stat, bit_rock, v_bit);
        } else if (conf_stat.bc_bit_rock_type == BC_BitRockType ::TUCKER_WANG_TORSIONAL) {
            //This uses the reaction WOB to calculate the torque
            calculate_bit_rock_Tucker_Wang_torsional(conf_stat, bit_rock, f_reaction);
        }
        

        WOB += bit_rock.WOB;
        TOB += bit_rock.TOB;
    }
    f_int[N - 1].x() += WOB;
    m_int[N - 1].x() += TOB;

    if (conf_stat.bc_top_kinematics_type == BC_TopKinematicsType::NEUMANN) {

        const scalar omega_top = omega[top_node].x();
        const scalar theta_top = theta[top_node].x();
        const scalar alpha_top = alpha[top_node].x(); //angular acceleration at top node is not known here

        const scalar Kp = conf_stat.PID_Kp; 
        const scalar Ki = conf_stat.PID_Ki; 
        const scalar Kd = conf_stat.PID_Kd; 

        const BCs &bcs_field = bcs.get_bcs_field(buf);

        //PID Controller
        scalar T_top = Kp * (bcs_field.omega_top_np - omega_top) + Ki * (bcs_field.theta_top_np - theta_top) + Kd * (bcs_field.alpha_top_np - alpha_top);

        // apply to internal force vector at top node (sign convention: add negative to dof)
        m_int[top_node].x() -= T_top;
    }

}



#ifdef __CUDACC__
__global__ void set_bc_force_gpu( byte *buf) {
    assert(blockIdx.y == 0 && blockIdx.z == 0 && blockDim.y == 1 && blockDim.z == 1);
    const uint idx = threadIdx.x + blockIdx.x * blockDim.x;
    const Pipe &pipe = cd_d.pipe;
    const ConfigStatic &conf_stat = cd_d.conf_stat;
    BeamData &beam = cd_d.beam_field;
    BitRockData &bit_rock_data = cd_d.bit_rock_data;
    if (idx == 0) {
        update_bc_force_dynamic(cd_d.top_node, cd_d.bc_data, conf_stat, pipe, bit_rock_data, beam, buf);
    }
}
#endif




DEVICE_FUNC void update_bc_kinematic_lateral(const uint top_node, const Pipe &pipe, BeamData &beam, byte *buf) {

    const uint N = pipe.N; 
    ArrayView<Vec3> a = beam.get_field<BeamField::a>(buf);
    ArrayView<Vec3> v = beam.get_field<BeamField::v>(buf);
    ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);

    const ArrayView<uint> i_pipe_to_ie_hole = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);

    uint i = top_node;
    while (i < N && i_pipe_to_ie_hole[i] <= 1) {
        // Clamp lateral kinematics for nodes outside the hole
        u[i].y() = 0.0;
        u[i].z() = 0.0;
        v[i].y() = 0.0;
        v[i].z() = 0.0;
        a[i].y() = 0.0;
        a[i].z() = 0.0;
        i++;
    }
}

DEVICE_FUNC void update_bc_kinematic_ax_tor(const scalar dt, const uint top_node, scalar v_top_target, scalar omega_top_target, const ConfigStatic &conf_stat,
                                    BCsData &bcs, const Pipe &pipe, BeamData &beam, byte *buf) {

    const uint N = pipe.N;
    BCs &bcs_field = bcs.get_bcs_field(buf);
    update_top_kinematics(dt, bcs_field, v_top_target, omega_top_target);
    
    ArrayView<Vec3> a = beam.get_field<BeamField::a>(buf);
    ArrayView<Vec3> v = beam.get_field<BeamField::v>(buf);
    ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);
    ArrayView<Vec3> alpha = beam.get_field<BeamField::alpha>(buf);
    ArrayView<Vec3> omega = beam.get_field<BeamField::omega>(buf);
    ArrayView<Vec3> theta = beam.get_field<BeamField::theta>(buf);

    const ArrayView<uint> i_pipe_to_ie_hole = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);


    // NOTE: For now we always set axial kinematics directly
    scalar u_top_target_np = min(bcs_field.u_top_np, pipe.L_tot);
    scalar v_top_target_np = bcs_field.v_top_np;
    scalar a_top_target_np = bcs_field.a_top_np;
    a[top_node].x() = a_top_target_np;
    v[top_node].x() = v_top_target_np;
    u[top_node].x() = u_top_target_np;

    if (conf_stat.bc_top_kinematics_type == BC_TopKinematicsType::DIRICHLET) {
        scalar theta_top_target_np = bcs_field.theta_top_np;
        scalar omega_dot_top_target_np = bcs_field.alpha_top_np;
        scalar omega_top_target_np = bcs_field.omega_top_np; 

        alpha[top_node].x() = omega_dot_top_target_np;
        theta[top_node].x() = theta_top_target_np;
        omega[top_node].x() = omega_top_target_np;
    }


    if (conf_stat.bc_bottom_axial_kinematics == BC_BottomAxialKinematics::FIXED){
        u[N - 1].x() = conf_stat.L_string_depth_initial;
        v[N - 1].x() = 0.0;
    }

}

DEVICE_FUNC void update_bc_kinematic(const uint top_node, scalar v_top_target, scalar omega_top_target, const ConfigStatic &conf_stat,
                                    BCsData &bcs, const Pipe &pipe, BeamData &beam, byte *buf) {

    update_bc_kinematic_ax_tor(conf_stat.dt, top_node, v_top_target, omega_top_target, conf_stat, bcs, pipe, beam, buf);

    update_bc_kinematic_lateral(top_node, pipe, beam, buf);

}


#ifdef __CUDACC__
__global__ void update_bc_kinematic_gpu(scalar v_top_target, scalar omega_top_target, byte *buf) {
    assert(blockIdx.y == 0 && blockIdx.z == 0 && blockDim.y == 1 && blockDim.z == 1);
    const uint idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx == 0) {
        update_bc_kinematic(cd_d.top_node, v_top_target, omega_top_target, cd_d.conf_stat, cd_d.bc_data, cd_d.pipe, cd_d.beam_field, buf);
    }
}
#endif

#ifdef __CUDACC__
__global__ void update_bc_kinematic_ax_tor_gpu(scalar dt, scalar v_top_target, scalar omega_top_target, byte *buf) {
    assert(blockIdx.y == 0 && blockIdx.z == 0 && blockDim.y == 1 && blockDim.z == 1);
    const uint idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx == 0) {
        update_bc_kinematic_ax_tor(dt, cd_d.top_node, v_top_target, omega_top_target, cd_d.conf_stat, cd_d.bc_data, cd_d.pipe, cd_d.beam_field, buf);
    }
}
#endif

#ifdef __CUDACC__
__global__ void update_bc_kinematic_lateral_gpu(byte *buf) {
    assert(blockIdx.y == 0 && blockIdx.z == 0 && blockDim.y == 1 && blockDim.z == 1);
    const uint idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx == 0) {
        update_bc_kinematic_lateral(cd_d.top_node, cd_d.pipe, cd_d.beam_field, buf);
    }
}
#endif



void set_bc_kinematic(const Config &config, const ConfigDynamic &conf_dyn, const Pipe &pipe, BeamData &beam,  BCsData &bcs, byte *buf) {
    if (config.use_gpu) {
#ifdef __CUDACC__
        constexpr uint GRID_DIM_MIN = 1;
        constexpr uint BLOCK_DIM_MIN = 8;
        update_bc_kinematic_gpu<<<GRID_DIM_MIN, BLOCK_DIM_MIN>>>(conf_dyn.v_top_target, conf_dyn.omega_top_target, buf);

        CUDA_CHECK_LAST_ERROR();
#endif
    } else {
        update_bc_kinematic(config.top_node, conf_dyn.v_top_target, conf_dyn.omega_top_target, config.conf_stat, bcs, pipe, beam, buf);
    }
}

void set_bc_kinematic_ax_tor(scalar dt, const Config &config, const ConfigDynamic &conf_dyn, const Pipe &pipe, BeamData &beam,  BCsData &bcs, byte *buf) {
    if (config.use_gpu) {
#ifdef __CUDACC__
        constexpr uint GRID_DIM_MIN = 1;
        constexpr uint BLOCK_DIM_MIN = 8;
        update_bc_kinematic_ax_tor_gpu<<<GRID_DIM_MIN, BLOCK_DIM_MIN>>>(dt, conf_dyn.v_top_target, conf_dyn.omega_top_target, buf);

        CUDA_CHECK_LAST_ERROR();
#endif
    } else {
        update_bc_kinematic_ax_tor(dt, config.top_node, conf_dyn.v_top_target, conf_dyn.omega_top_target, config.conf_stat, bcs, pipe, beam, buf);
    }
}


void set_bc_kinematic_lateral(const Config &config, const Pipe &pipe, BeamData &beam, byte *buf) {
    if (config.use_gpu) {
#ifdef __CUDACC__
        constexpr uint GRID_DIM_MIN = 1;
        constexpr uint BLOCK_DIM_MIN = 8;
        update_bc_kinematic_lateral_gpu<<<GRID_DIM_MIN, BLOCK_DIM_MIN>>>(buf);
        CUDA_CHECK_LAST_ERROR();
#endif
    } else {
        update_bc_kinematic_lateral(config.top_node, pipe, beam, buf);
    }
}


void set_bc_force_dynamic(const Config &config, const Pipe &pipe, BeamData &beam, BitRockData &bit_rock_data,
                          const BCsData &bcs, byte *buf) {

    if (config.use_gpu) {
#ifdef __CUDACC__
        CUDA_CHECK_LAST_ERROR();
        constexpr uint GRID_DIM_MIN = 1;
        constexpr uint BLOCK_DIM_MIN = 8;
        set_bc_force_gpu<<<GRID_DIM_MIN, BLOCK_DIM_MIN>>>(buf);
        CUDA_CHECK_LAST_ERROR();
#endif
    } else {
        update_bc_force_dynamic(config.top_node, bcs, config.conf_stat, pipe, bit_rock_data, beam, buf);
    }
}



} // namespace curvlin


namespace corot {

void set_bc_force_dynamic(const uint N, const ConfigStatic &conf_stat, BeamData &beam, const Hole &hole, byte *buf) {

    if (abs(conf_stat.WOB_constant) > 0.0 || abs(conf_stat.TOB_constant) > 0.0) {
        Vec3 wob_local(-conf_stat.WOB_constant, 0.0, 0.0);
        Vec3 tob_local(conf_stat.TOB_constant, 0.0, 0.0);
        const Quaternion &orientation_bottom = hole.get_field<HoleField::q>(buf)[N - 1];
        Vec3 wob_global = orientation_bottom.rotate_vector(wob_local);
        Vec3 tob_global = orientation_bottom.rotate_vector(tob_local);

        beam.f_ext_trans[N - 1] += wob_global;
        beam.f_ext_rot[N - 1] += tob_global;
    }
}



void set_bc_top(bool first_call, const uint top_node, ConfigStatic &conf_stat, BCs &bcs_field, const Pipe &pipe,
                       const Hole &hole, BeamData &beam, byte *buf) {

    const scalar u_top_np = bcs_field.u_top_np;
    const scalar u_top_n = bcs_field.u_top_n;
    const scalar v_top_np = bcs_field.v_top_np;
    const scalar a_top_n = bcs_field.a_top_n;
    const scalar a_top_np = bcs_field.a_top_np;

    const scalar theta_top_n = bcs_field.theta_top_n;
    const scalar theta_top_np = bcs_field.theta_top_np;
    const scalar omega_top_np = bcs_field.omega_top_np;
    const scalar alpha_top_n = bcs_field.alpha_top_n;
    const scalar alpha_top_np = bcs_field.alpha_top_np;

    assert(pipe.N == beam.d_trans.size() && pipe.N == beam.d_rot.size() && pipe.N == beam.v_trans.size() &&
           pipe.N == beam.v_rot.size());

    Quaternion orientation_top = hole.get_field<HoleField::q>(buf)[2];

    orientation_top.exponential_map_body_frame(Vec3{theta_top_np, 0.0, 0.0});
    if (is_close(theta_top_np, 0.0)) {
        assert(hole.get_field<HoleField::q>(buf)[2].to_matrix().isApprox(orientation_top.to_matrix()));
    }

    Vec3 t_top = hole.get_field<HoleField::q>(buf)[2].to_matrix().col(0);
    beam.d_rot[top_node] = orientation_top;
    beam.v_rot[top_node] = {omega_top_np, 0, 0};
    beam.d_trans[top_node] = t_top * u_top_np;

    if (conf_stat.check_energy_balance) {
        beam.delta_d_rot[top_node] = t_top * (theta_top_np - theta_top_n);
        beam.delta_d_trans[top_node] = t_top * (u_top_np - u_top_n);
    }
    beam.v_trans[top_node] = t_top * v_top_np;

    if (first_call) {
        beam.a_trans[top_node] = a_top_np * t_top;
        beam.a_rot[top_node] = {alpha_top_n, 0.0, 0.0};
        beam.f_ext_trans[top_node] = beam.M[top_node] * beam.a_trans[top_node] + beam.f_int_trans[top_node];
        beam.f_ext_rot[top_node] = beam.J_u[top_node].x() * beam.a_rot[top_node] + beam.f_int_trans[top_node];
    } else {
        beam.a_trans[top_node] = a_top_np * t_top;
        beam.a_rot[top_node] = {alpha_top_np, 0.0, 0.0};
    }
}

} // namespace corot