#include "pipe-solver-curvlin.hpp"
#include "pipe-solver-interface-curvlin.hpp"
#include <iomanip>


namespace curvlin {

constexpr uint BLOCK_DIM_DEFAULT = 32;

PipeSolver::PipeSolver(Config &config, ConfigDynamic &conf_dyn, const Pipe &pipe, const vector<PipeComponent> &pipe_assembly, const Hole &hole, 
    BCsData &bcs, BitRockData &bit_rock_data, ArenaBump &arena_h, ArenaBump &arena_d) {
    const uint N = pipe.N;
    calc_dt_and_inertia_scaling(config, pipe, arena_h.buf);

    
    set_pipe_feed_length_initial(config, conf_dyn, bcs, pipe, pipe_assembly, arena_h.buf);

    beam = create_and_allocate_beam(N,  arena_h);

    //Temporary: fluid field is defined in the middle of pipe elements for now
    //We add one at each end (pump and nozzle pressure)
    fluid = create_and_allocate_fluid(N + 1, arena_h);
    
    m_vec.resize(N);
    J_vec.resize(N);
    assemble_mass_vectors(config, pipe, hole, arena_h.buf);

    //calc_contact_stiffness_and_damping(config, pipe.N, beam.get_field<BeamField::m>(arena_h.buf), arena_h.buf);

    initialize_beam_segment_solution(0, pipe.N, hole, pipe, beam, bcs, arena_h.buf);

    uint *indices = beam.get_field<BeamField::i_pipe_to_ie_hole>(arena_h.buf).data;
    hole.initalize_pipe_nodes_to_hole_segments(pipe.N, config, indices, pipe.get_field<PipeField::S>(arena_h.buf), arena_h.buf);
    
    calc_fluid_newtonian(config.conf_stat, fluid, hole, pipe, beam, config.top_node, arena_h);

    /*Setup complete. Copy entire buffer from host to device*/
#ifdef __CUDACC__
    if (config.use_gpu) {
        constant_data_setup(arena_d.buf, config.top_node, config.conf_stat, beam, fluid, pipe, hole, bcs, bit_rock_data);
        ArenaBump::copy_all(arena_d, arena_h);

        /*Experiment with these*/
        grid_dim_nodal = (N + BLOCK_DIM_DEFAULT - 1) / BLOCK_DIM_DEFAULT;
        
    }
#endif
}


void PipeSolver::update_constant_data_gpu(const Config &config, const Pipe &pipe, const Hole &hole, const BCsData &bc_data, const BitRockData &bit_rock_data, ArenaBump &arena_d) {
#ifdef __CUDACC__
    constant_data_setup(arena_d.buf, config.top_node, config.conf_stat, beam, fluid, pipe, hole, bc_data, bit_rock_data);
#endif
}

void PipeSolver::step(Config &config, ConfigDynamic &conf_dyn, const Pipe &pipe, const vector<PipeComponent> &pipe_assembly, const Hole &hole, BCsData &bcs, BitRockData &bit_rock_data, ArenaBump &arena_h, ArenaBump &arena_d) {
    
    if (config.save_csv && config.n % config.n_write == 0) {
        save_csv(config, conf_dyn, pipe, hole, bcs, bit_rock_data, arena_h, arena_d);
        printf("\n---------------------------------\n"
               "csv saved for: n = %lu, t = %.5f"
               "\n---------------------------------\n",
               config.n, config.t);
    }

    if (config.conf_stat.curvilinear_integration_type == CurvilinearIntegrationType::SOFT_STRING) {
        step_solver_soft_string(config, conf_dyn, pipe, pipe_assembly, hole, bcs, bit_rock_data, arena_h, arena_d);
    } else {
        step_solver(config, conf_dyn, pipe, pipe_assembly, hole, bcs, bit_rock_data, arena_h, arena_d);
    }
}


void PipeSolver::step_solver(Config &config, ConfigDynamic &conf_dyn, const Pipe &pipe, const vector<PipeComponent> &pipe_assembly, const Hole &hole, BCsData &bc_data, 
                            BitRockData &bit_rock_data, ArenaBump &arena_h, ArenaBump &arena_d) {
    const bool use_gpu = config.use_gpu;
    byte *buf = use_gpu ? arena_d.buf : arena_h.buf;
    
    const uint top_node = config.top_node;
    
    if (config.reassemble_mass_vectors) {
        assemble_mass_vectors(config, pipe, hole, arena_h.buf);
        config.reassemble_mass_vectors = false;

        if (use_gpu) { //Copy mass vector fields to device 
            copy_beam_field_to_dst<curvlin::BeamField::m>(arena_d, arena_h);
            copy_beam_field_to_dst<curvlin::BeamField::Jx>(arena_d, arena_h);
            copy_beam_field_to_dst<curvlin::BeamField::Jr>(arena_d, arena_h);
        }
    }
    
    if (config.n % config.static_load_update_interval == 0) {

        if (use_gpu) { //Copy fields to host
            copy_current_pipe_feed_to_host(arena_h, arena_d);
        }
        
        /*==============================================
        Update static loads and fluid fields 
        geometry changes
        ==============================================*/
        
        calc_fluid_newtonian(config.conf_stat, fluid, hole, pipe, beam, top_node, arena_h);
        if (config.conf_stat.fluid_dynamics_enabled) {
            update_fluid_mass(config.conf_stat, top_node, pipe, hole, beam, arena_h);
        }

        calc_static_loads(config.conf_stat, beam, fluid, hole, pipe, top_node, arena_h);
        
        if (use_gpu) { //Copy fields to device
           copy_static_force_and_fluid_fields_to_device(arena_h, arena_d);
        }
    }

    /*==============================================
    Initialize force vectors and add forces for
    odd elements
    ==============================================*/
    initialize_forces_and_assemble_odd(grid_dim_nodal, BLOCK_DIM_DEFAULT, config, top_node, bc_data, beam, fluid, pipe, hole, buf);
    
    
    /*==============================================
    Add forces for even elements
    ==============================================*/
    assemble_even(grid_dim_nodal, BLOCK_DIM_DEFAULT, config, top_node, bc_data, beam, fluid, pipe, hole, buf);
        

    if (config.conf_stat.enable_ax_tor_subcycle) {

        for (uint i_at = 0; i_at < config.conf_stat.n_at; ++i_at) {

            
            /*==============================================
            Initialize force vectors and add forces for
            odd elements
            ==============================================*/
            initialize_forces_and_assemble_odd_ax_tor(grid_dim_nodal, BLOCK_DIM_DEFAULT, config, top_node, bc_data, beam, fluid, pipe, hole, buf);
    
            
            assemble_even_ax_tor(grid_dim_nodal, BLOCK_DIM_DEFAULT, config, top_node, bc_data, beam, fluid, pipe, hole, buf);
            
            /*==============================================
            Update the bit-rock model and set the force 
            boundary conditions in the bottom
            ==============================================*/    
            set_bc_force_dynamic(config, pipe, beam, bit_rock_data, bc_data, buf);
            

            /*==============================================
            Integrates axial and torsional dynamics in time
            ==============================================*/    
            integrate_time_ax_tor(grid_dim_nodal, BLOCK_DIM_DEFAULT, config.conf_stat.dt_at,config, top_node, beam, pipe, hole, buf);

            /*==============================================
            Set axial and torsional kinematic boundary conditions
            ==============================================*/
            set_bc_kinematic_ax_tor(config.conf_stat.dt_at, config, conf_dyn, pipe, beam, bc_data, buf);

            /*========================================================
            Update controller inputs (host side)
            =========================================================*/
            update_control_input(config.conf_stat.dt_at, config, conf_dyn, pipe, pipe_assembly, bc_data, arena_h, arena_d);

        }

        /*==============================================
        Integrates lateral dynamics in time and update the pipe to hole indices
        ==============================================*/
        integrate_time_lateral(grid_dim_nodal, BLOCK_DIM_DEFAULT, config, top_node, beam, pipe, hole, buf);

        /*========================================================
        Update controller inputs and sets the top node (host side)
        =========================================================*/
        set_bc_kinematic_lateral(config, pipe, beam, buf);
    
    } else {

        /*==============================================
        Calculate friction forces in each node
        ==============================================*/
        calc_nodal_friction_forces(grid_dim_nodal, BLOCK_DIM_DEFAULT, config, top_node, pipe, hole, beam, buf);
    

        /*==============================================
        Update the bit-rock model and set the force 
        boundary conditions in the bottom
        ==============================================*/    
        set_bc_force_dynamic(config, pipe, beam, bit_rock_data, bc_data, buf);


        /*==============================================
        Integrates lateral dynamics in time and update the pipe to hole indices
        ==============================================*/
        integrate_time(grid_dim_nodal, BLOCK_DIM_DEFAULT, config, top_node, beam, pipe, hole, buf);

        
        /*==============================================
        Set all kinematic boundary conditions
        ==============================================*/
        set_bc_kinematic(config, conf_dyn, pipe, beam, bc_data, buf);
        
        
        /*========================================================
        Update controller inputs (host side)
        =========================================================*/
        update_control_input(config.conf_stat.dt, config, conf_dyn, pipe, pipe_assembly, bc_data, arena_h, arena_d);
    }
        
    /*========================================================
    Update top node and top component (host side)
    =========================================================*/        
    update_top_node(config, conf_dyn, pipe, pipe_assembly);

    /*========================================================
    Update the pipe component and if the top node has changed,
    also set the top node on device
    =========================================================*/
    if (config.top_node_new != config.top_node) {
        update_pipe_feed(config, pipe, hole, bc_data, bit_rock_data, arena_h, arena_d);
    }
    
}

void PipeSolver::step_solver_soft_string(Config &config, ConfigDynamic &conf_dyn, const Pipe &pipe, const vector<PipeComponent> &pipe_assembly, const Hole &hole, BCsData &bc_data, 
                            BitRockData &bit_rock_data, ArenaBump &arena_h, ArenaBump &arena_d) {
    const bool use_gpu = config.use_gpu;
    byte *buf = use_gpu ? arena_d.buf : arena_h.buf;
    
    const uint top_node = config.top_node;
    
    if (config.reassemble_mass_vectors) {
        assemble_mass_vectors(config, pipe, hole, arena_h.buf);
        config.reassemble_mass_vectors = false;

        if (use_gpu) { //Copy mass vector fields to device 
            copy_beam_field_to_dst<curvlin::BeamField::m>(arena_d, arena_h);
            copy_beam_field_to_dst<curvlin::BeamField::Jx>(arena_d, arena_h);
            copy_beam_field_to_dst<curvlin::BeamField::Jr>(arena_d, arena_h);
        }
    }
    
    if (config.n % config.static_load_update_interval == 0) {

        if (use_gpu) { //Copy fields to host
            copy_current_pipe_feed_to_host(arena_h, arena_d);
        }
        
        /*==============================================
        Update static loads and fluid fields 
        geometry changes
        ==============================================*/
        
        calc_fluid_newtonian(config.conf_stat, fluid, hole, pipe, beam, top_node, arena_h);
        if (config.conf_stat.fluid_dynamics_enabled) {
            update_fluid_mass(config.conf_stat, top_node, pipe, hole, beam, arena_h);
        }

        calc_static_loads(config.conf_stat, beam, fluid, hole, pipe, top_node, arena_h);

        calc_softstring_axial_tension(pipe, beam, config.conf_stat, arena_h.buf);
    
        if (use_gpu) { //Copy fields to device
           copy_static_force_and_fluid_fields_to_device(arena_h, arena_d);
        }
    }

    
    /*==============================================
    Initialize force vectors and add forces for
    odd elements
    ==============================================*/
    initialize_forces_and_assemble_odd_ax_tor(grid_dim_nodal, BLOCK_DIM_DEFAULT, config, top_node, bc_data, beam, fluid, pipe, hole, buf);
    
    /*==============================================
    Add forces for even elements
    ==============================================*/        
    assemble_even_ax_tor(grid_dim_nodal, BLOCK_DIM_DEFAULT, config, top_node, bc_data, beam, fluid, pipe, hole, buf);
    
    /*==============================================
    Update the bit-rock model and set the force 
    boundary conditions in the bottom
    ==============================================*/    
    set_bc_force_dynamic(config, pipe, beam, bit_rock_data, bc_data, buf);
            
    /*==============================================
    Integrates axial and torsional dynamics in time
    ==============================================*/    
    integrate_time_ax_tor(grid_dim_nodal, BLOCK_DIM_DEFAULT, config.conf_stat.dt, config, top_node, beam, pipe, hole, buf);

    /*==============================================
    Set axial and torsional kinematic boundary conditions
    ==============================================*/
    set_bc_kinematic_ax_tor(config.conf_stat.dt, config, conf_dyn, pipe, beam, bc_data, buf);

    /*========================================================
    Update controller inputs (host side)
    =========================================================*/
    update_control_input(config.conf_stat.dt, config, conf_dyn, pipe, pipe_assembly, bc_data, arena_h, arena_d);
        
    /*========================================================
    Update top node and top component (host side)
    =========================================================*/        
    update_top_node(config, conf_dyn, pipe, pipe_assembly);

    /*========================================================
    Update the pipe component and if the top node has changed,
    also set the top node on device
    =========================================================*/
    if (config.top_node_new != config.top_node) {
        update_pipe_feed(config, pipe, hole, bc_data, bit_rock_data, arena_h, arena_d);
    }
    
}


void PipeSolver::update_control_input(scalar dt, Config &config, ConfigDynamic &conf_dyn, const Pipe &pipe, const vector<PipeComponent> &pipe_assembly,
const BCsData &bc_data, ArenaBump &arena_h, ArenaBump &arena_d) {
    const bool use_gpu = config.use_gpu;
    byte *buf = use_gpu ? arena_d.buf : arena_h.buf;
    constexpr scalar v_ramp_rate = 0.1; //m/s^2
    constexpr scalar omega_ramp_rate = 10.0; //rad/s^2
    
    // Calculate target angular velocity change with ramping
    scalar current_omega = conf_dyn.omega_top_target;
    scalar target_omega = config.conf_stat.omega_top_input;
    scalar max_omega_change = omega_ramp_rate * dt;
    
    if (current_omega < target_omega) {
        conf_dyn.omega_top_target = min(current_omega + max_omega_change, target_omega);
    } else if (current_omega > target_omega) {
        conf_dyn.omega_top_target = max(current_omega - max_omega_change, target_omega);
    }
    
    // Calculate target velocity change with ramping
    scalar current_v = conf_dyn.v_top_target;
    scalar target_v = config.conf_stat.v_top_input;
    scalar max_change = v_ramp_rate * dt;
    
    if (current_v < target_v) {
        conf_dyn.v_top_target = min(current_v + max_change, target_v);
    } else if (current_v > target_v) {
        conf_dyn.v_top_target = max(current_v - max_change, target_v);
    }
}


void PipeSolver::update_pipe_feed(Config &config, const Pipe &pipe, const Hole &hole,
                                             const BCsData &bc_data, const BitRockData &bit_rock_data, ArenaBump &arena_h, ArenaBump &arena_d) {
    /* Called every time a new pipe component is added or removed which
    extends or reduces the pipe domain.*/
    if (config.use_gpu) {
#ifdef __CUDACC__
        if (config.top_node_new < config.top_node ){
            assert(arena_h.mem_type == MemType::HOST && arena_d.mem_type == MemType::DEVICE);
            copy_beam_field_to_dst<BeamField::u>(arena_h, arena_d);
            copy_beam_field_to_dst<BeamField::v>(arena_h, arena_d);
            copy_beam_field_to_dst<BeamField::a>(arena_h, arena_d);
            copy_beam_field_to_dst<BeamField::theta>(arena_h, arena_d);
            copy_beam_field_to_dst<BeamField::omega>(arena_h, arena_d);
            copy_beam_field_to_dst<BeamField::alpha>(arena_h, arena_d);
            copy_beam_field_to_dst<BeamField::i_pipe_to_ie_hole>(arena_h, arena_d);
            copy_bcs_to_dst(arena_h, arena_d, bc_data);    
        }
        // Update constant data with new top node
        constant_data_setup(arena_d.buf, config.top_node_new, config.conf_stat, beam, fluid, pipe, hole, bc_data, bit_rock_data);
#endif
    }   
    if (config.top_node_new < config.top_node){
        initialize_beam_segment_solution(config.top_node_new, config.top_node, hole, pipe, beam, bc_data, arena_h.buf);
        
         //Copy mass related fields to device
    
        if (config.use_gpu){
            copy_beam_kinematics_to_device(arena_h, arena_d);
        }
    }
    config.top_node = config.top_node_new;
    
    // Mass vectors need to be reassembled since 
    // top-drive mass is added to new top node
    config.reassemble_mass_vectors = true;

} 

void PipeSolver::assemble_mass_vectors(Config &config, const Pipe &pipe, const Hole &hole,
                           byte *buf) {
    const ConfigStatic &conf_stat = config.conf_stat;

    const uint N = pipe.N;

    std::fill(m_vec.begin(), m_vec.end(), 0.0);
    std::fill(J_vec.begin(), J_vec.end(), Vec3::Zero());

    create_mass_vectors(m_vec, J_vec, config, pipe, buf);
    
    ArrayView<scalar> m = beam.get_field<BeamField::m>(buf);
    ArrayView<scalar> Jx = beam.get_field<BeamField::Jx>(buf);
    ArrayView<scalar> Jr = beam.get_field<BeamField::Jr>(buf);

#pragma omp parallel for
    for(uint i = 0; i < N; i++) {
        m[i] = m_vec[i];
        Jx[i] = J_vec[i].x();
        Jr[i] = J_vec[i].y();
    }

    //Add top drive mass to top node
    //m[config.top_node] += conf_stat.m_td; 
    Jx[config.top_node] += conf_stat.J_td;


    if (conf_stat.string_type == StringType::COILED_TUBING ||
        conf_stat.string_type == StringType::DRILL_STRING) {
            m[N - 1] += conf_stat.m_br; //Add mass of drill bit to last node

            if (conf_stat.J_br < SMALL_SCALAR){
                //Estimate rotary inertia of drill bit as a solid cylinder
                scalar ro = conf_stat.r_br;
                scalar J_br_estimated = 0.5 * conf_stat.m_br * ro * ro;
                Jx[N - 1] += J_br_estimated;
            }
            else{
                Jx[N - 1] += conf_stat.J_br; //Add rotary inertia of drill bit to last node
            }
    }
}

void PipeSolver::save_csv(Config &config, ConfigDynamic &conf_dyn, const Pipe &pipe_h, const Hole &hole_h, const BCsData &bcs, const BitRockData &bit_rock_data, ArenaBump &arena_h, ArenaBump &arena_d) {

    if (config.use_gpu) {
        /*--------------------------------------------------------------------
        Copy data back to host if GPU is used
        --------------------------------------------------------------------*/
        copy_beam_field_to_dst<BeamField::u>(arena_h, arena_d);
        copy_beam_field_to_dst<BeamField::v>(arena_h, arena_d);
        copy_beam_field_to_dst<BeamField::a>(arena_h, arena_d);
        copy_beam_field_to_dst<BeamField::f_int>(arena_h, arena_d);
        copy_beam_field_to_dst<BeamField::f_dyn>(arena_h, arena_d);
        copy_beam_field_to_dst<BeamField::f_hyd>(arena_h, arena_d);
        copy_beam_field_to_dst<BeamField::theta>(arena_h, arena_d);
        copy_beam_field_to_dst<BeamField::omega>(arena_h, arena_d);
        copy_beam_field_to_dst<BeamField::alpha>(arena_h, arena_d);
        copy_beam_field_to_dst<BeamField::m_int>(arena_h, arena_d);
        copy_beam_field_to_dst<BeamField::m_dyn>(arena_h, arena_d);
        copy_beam_field_to_dst<BeamField::m_hyd>(arena_h, arena_d);
        copy_beam_field_to_dst<BeamField::i_pipe_to_ie_hole>(arena_h, arena_d);

        
        //Copy entire fluid field
        uint begin, end;
        fluid.get_field_begin_end(&begin, &end);
        ArenaBump::copy_slice(arena_d, arena_h, begin, end);
        
        if (config.write_mass) {
            copy_beam_field_to_dst<BeamField::m>(arena_h, arena_d);
            copy_beam_field_to_dst<BeamField::mfy>(arena_h, arena_d);
            copy_beam_field_to_dst<BeamField::mfz>(arena_h, arena_d);
            copy_beam_field_to_dst<BeamField::Jx>(arena_h, arena_d);
            copy_beam_field_to_dst<BeamField::Jr>(arena_h, arena_d);
        }
        
        if (config.conf_stat.bc_bit_rock_type != BC_BitRockType::NO_BIT) {
                copy_bit_rock_to_dst(arena_h, arena_d, bit_rock_data);
        }
    }

    update_drilling_quantities_curvlin(config, conf_dyn, pipe_h, arena_h, bit_rock_data);

    if (config.output_global_frame) {
        save_csv_global(config, conf_dyn, pipe_h, hole_h, bcs, arena_h);
    } else {
        save_csv_curvlin(config, conf_dyn, pipe_h, hole_h, bcs, arena_h);
    }

    if (config.conf_stat.fluid_dynamics_enabled)
        save_csv_fluid(config, pipe_h, config.top_node, arena_h);
    
}


void PipeSolver::save_csv_curvlin(Config &config, const ConfigDynamic &conf_dyn, const Pipe &pipe, const Hole &hole, const BCsData &bcs, ArenaBump &arena_h) {

    using namespace std;
    uint i;
    assert(config.save_csv && config.n % config.n_write == 0);
    const string output_subdir = config.create_output_subdir();

    save_csv_header_file(pipe.N, config, conf_dyn, output_subdir, bcs, arena_h.buf);
   
    /*--------------------------------------------------------------------
    Save beam output
    --------------------------------------------------------------------*/
    {
        const string beam_file = output_subdir + "/beam.csv";
        ofstream ost{beam_file};
        const uint top_node = 0; //Want to print everything
        const uint N = pipe.N;
        const scalar L_feed = conf_dyn.feed_length;
        const ArrayView<Vec3> u = beam.get_field<BeamField::u>(arena_h.buf);
        const ArrayView<Vec3> v = beam.get_field<BeamField::v>(arena_h.buf);
        const ArrayView<Vec3> a = beam.get_field<BeamField::a>(arena_h.buf);
        const ArrayView<Vec3> f_int = beam.get_field<BeamField::f_int>(arena_h.buf);
        const ArrayView<Vec3> f_dyn = beam.get_field<BeamField::f_dyn>(arena_h.buf);
        const ArrayView<Vec3> f_hyd = beam.get_field<BeamField::f_hyd>(arena_h.buf);
        const ArrayView<Vec3> theta = beam.get_field<BeamField::theta>(arena_h.buf);
        const ArrayView<Vec3> omega = beam.get_field<BeamField::omega>(arena_h.buf);
        const ArrayView<Vec3> alpha = beam.get_field<BeamField::alpha>(arena_h.buf);
        const ArrayView<Vec3> m_int = beam.get_field<BeamField::m_int>(arena_h.buf);
        const ArrayView<Vec3> m_dyn = beam.get_field<BeamField::m_dyn>(arena_h.buf);
        const ArrayView<Vec3> m_hyd = beam.get_field<BeamField::m_hyd>(arena_h.buf);
        const ArrayView<uint> i_pipe_to_ie_hole = beam.get_field<BeamField::i_pipe_to_ie_hole>(arena_h.buf);
        
        // --- Precompute effective axial tension if needed ---
        vector<scalar> fx_tension;
        
        if (config.write_misc_quantities) {
            fx_tension.resize(N, 0.0);
            scalar sum_axial = 0.0;
            for (uint j = 0; j < N; j++) {
                sum_axial += f_int[j].x();
                fx_tension[j] = -sum_axial;
            }
            
        }
        save_csv_curvlin_header(config, ost);
        constexpr uint w = 11;

        for_each_node_cpu(i) {
            const scalar s = pipe.calc_s(i, u[i].x(), arena_h.buf);
            const int ie_h = i_pipe_to_ie_hole[i];
            ost << setw(w) << s << ", " << setw(w) << ie_h << ", " << setw(w) << u[i].x() - L_feed
                                                  << ", " << setw(w) << u[i].y() << ", " << setw(w) << u[i].z() 
                << ", " << setw(w) << theta[i].x() << ", " << setw(w) << theta[i].y() << ", " << setw(w) << theta[i].z()
                << ", " << setw(w) << v[i].x() << ", " << setw(w) << v[i].y() << ", " << setw(w) << v[i].z() 
                << ", " << setw(w) << omega[i].x() << ", " << setw(w) << omega[i].y() << ", " << setw(w) << omega[i].z() 
                << ", " << setw(w) << a[i].x() << ", " << setw(w) << a[i].y() << ", " << setw(w) << a[i].z() 
                << ", " << setw(w) << alpha[i].x() << ", " << setw(w) << alpha[i].y() << ", " << setw(w) << alpha[i].z() 
                << ", " << setw(w) << f_int[i].x() << ", " << setw(w) << f_int[i].y() << ", " << setw(w) << f_int[i].z()
                << ", " << setw(w) << m_int[i].x() << ", " << setw(w) << m_int[i].y() << ", " << setw(w) << m_int[i].z()
                << ", " << setw(w) << f_dyn[i].x() << ", " << setw(w) << f_dyn[i].y() << ", " << setw(w) << f_dyn[i].z()
                << ", " << setw(w) << m_dyn[i].x() << ", " << setw(w) << m_dyn[i].y() << ", " << setw(w) << m_dyn[i].z()
                << ", " << setw(w) << f_hyd[i].x() << ", " << setw(w) << f_hyd[i].y() << ", " << setw(w) << f_hyd[i].z()
                << ", " << setw(w) << m_hyd[i].x() << ", " << setw(w) << m_hyd[i].y() << ", " << setw(w) << m_hyd[i].z();

            if (config.write_mass) {
                const ArrayView<scalar> m = beam.get_field<BeamField::m>(arena_h.buf);
                const ArrayView<scalar> mfy = beam.get_field<BeamField::mfy>(arena_h.buf);
                const ArrayView<scalar> mfz = beam.get_field<BeamField::mfz>(arena_h.buf);
                const ArrayView<scalar> Jx = beam.get_field<BeamField::Jx>(arena_h.buf);
                const ArrayView<scalar> Jr = beam.get_field<BeamField::Jr>(arena_h.buf);
        
                ost << ", " << setw(w) << m[i]  << ", " << setw(w) << Jx[i] 
                    << ", " << setw(w) << Jr[i];
            }
            if (config.write_misc_quantities) { 
                // --- Writing nodewise quantities ---
                ost << ", " << setw(w) << fx_tension[i];

                scalar a_r  = compute_radial_acceleration(i, pipe, hole, arena_h.buf);
                scalar a_t  = compute_tangential_acceleration(i, pipe, hole, arena_h.buf);
                ost << ", " << setw(w) << a_r << ", " << setw(w) << a_t;

                scalar stand_off = compute_standoff(i, pipe, hole, arena_h.buf);
                ost << ", " << setw(w) << stand_off;

                
                if (i < N - 1){
                    // --- Writing elementwise quantities ---
                    const scalar xi = 0.5; // Midpoint of the element
                    scalar sigma_vm = compute_vm_stress_curvlin(i, xi, pipe, hole, config, arena_h.buf);
                    ost << ", " << setw(w) << sigma_vm;
                    
                    // --- Linear Contact Force [N/m] ---
                    scalar Fy = f_dyn[i].y();
                    scalar Fz = f_dyn[i].z();
                    scalar F_i = sqrt(Fy * Fy + Fz * Fz);
                    scalar dS = pipe.dS_node_avg(i, arena_h.buf);
                    scalar q_c = F_i / dS; // [N/m]
                    ost << ", " << setw(w) << q_c;
                }
                else { //Dummy values for last node
                    ost << ", " << setw(w) << 0.0; //No stress at last node
                    ost << ", " << setw(w) << 0.0; //No contact force at last node
                }

            }
            ost << endl; ost << endl;
        }

        if (config.write_mass) {
            config.write_mass = false; // Only written when changed
        }
    }

    /*--------------------------------------------------------------------
    Save bit rock model
    --------------------------------------------------------------------*/
    // if (sim_data.bit_rock != nullptr) {
    //     assert(config.bit_rock_enabled);
    //     const BitRock &bit_rock = *sim_data.bit_rock;
    //     const string bit_rock_file = output_subdir + "bit-rock.csv";
    //     ofstream ost{bit_rock_file};

    //     if (!ost) {
    //         throw runtime_error{"Failed to open output file: " + bit_rock_file + "\n"};
    //     }
    //     ost << "N_cells, n_blades, theta, ptr, s_blade\n";
    //     ost << bit_rock.N_cells() << ", " << bit_rock.n_blades << ", " << bit_rock.theta << ", " << bit_rock.ptr
    //         << ", " << bit_rock.s_blade << "\n";
    //     ost << "cut depths absolute position:\n";
    //     for (uint i = 0; i < bit_rock.N_cells(); i++) {
    //         ost << bit_rock.cut_depths[i];
    //         if (i < bit_rock.N_cells() - 1)
    //             ost << ", ";
    //     }
    //     ost << "\n";
    // }
}

void PipeSolver::save_csv_global(Config &config, const ConfigDynamic &conf_dyn, const Pipe &pipe, const Hole &hole, const BCsData &bcs, ArenaBump &arena_h) {
    using namespace std;
    assert(config.save_csv && config.n % config.n_write == 0);

    const string output_subdir = config.create_output_subdir();

    save_csv_header_file(pipe.N, config, conf_dyn, output_subdir, bcs, arena_h.buf);

    /*--------------------------------------------------------------------
    Save beam output
    --------------------------------------------------------------------*/
    {
        const string beam_file = output_subdir + "/beam.csv";
        ofstream ost{beam_file};

        if (!ost) {
            throw runtime_error{"Failed to open output file: " + beam_file + "\n"};
        }

        save_csv_global_header(config, ost);
        const ArrayView<Vec3> u = beam.get_field<BeamField::u>(arena_h.buf);
        const ArrayView<Vec3> v = beam.get_field<BeamField::v>(arena_h.buf);
        const ArrayView<Vec3> theta = beam.get_field<BeamField::theta>(arena_h.buf);
        const ArrayView<Vec3> omega = beam.get_field<BeamField::omega>(arena_h.buf);
        const ArrayView<uint> i_pipe_to_ie_hole = beam.get_field<BeamField::i_pipe_to_ie_hole>(arena_h.buf);

        const ArrayView<scalar> S = pipe.get_field<PipeField::S>(arena_h.buf);
        const ArrayView<scalar> s = hole.get_field<HoleField::s>(arena_h.buf);
        const ArrayView<Vec3> x = hole.get_field<HoleField::x>(arena_h.buf);
        const ArrayView<Quaternion> q = hole.get_field<HoleField::q>(arena_h.buf);
        const scalar L_pipe = pipe.L_tot;

        assert(L_pipe >= config.conf_stat.L_string_depth_initial);

        Quaternion q_identity;
        q_identity.from_matrix(Mat3::Identity());
        vector<Vec3> X;
        const uint N = pipe.N;
        X.resize(N);

        const Vec3 t_top = hole.get_field<HoleField::q>(arena_h.buf)[0].to_matrix().col(0);
        for (uint i = 0; i < N; i++) {
            X[i] = t_top * (-L_pipe + S[i]);
            // Check origo
            assert(i != N - 1 || (is_close(X[i][0], 0.0) && is_close(X[i][1], 0.0) && is_close(X[i][2], 0.0)));
        }

        Mat3 Rx, Ry, Rz;
        for (uint i = 0; i < N; i++) {
            const int ie_h = i_pipe_to_ie_hole[i];

            const scalar &ux = u[i].x();
            const scalar &uy = u[i].y();
            const scalar &uz = u[i].z();
            const scalar &thetax = theta[i].x();
            const scalar &thetay = theta[i].y();
            const scalar &thetaz = theta[i].z();

            const scalar &vx = v[i].x();
            const scalar &vy = v[i].y();
            const scalar &vz = v[i].z();
            const scalar &omegax = omega[i].x();
            const scalar &omegay = omega[i].y();
            const scalar &omegaz = omega[i].z();

            const scalar s_i = pipe.calc_s(i, ux, arena_h.buf);
            const scalar &s_hole_i = s[ie_h];
            const scalar &s_hole_ip = s[ie_h + 1];
            const scalar t = (s_i - s_hole_i) / (s_hole_ip - s_hole_i);

            const Quaternion &q_hole_ih = q[ie_h];
            const Quaternion &q_hole_ihp = q[ie_h + 1];
            const Quaternion q_i = Quaternion::lerp(q_hole_ih, q_hole_ihp, t);

            // Using similar notation as article

            // Global vector of hole centerline position at pipe node
            const Vec3 x_pipe_holecenter =
                hole.lerp_hole_property_from_pipe_node_position<Vec3>(x, s_i, ie_h, arena_h.buf);

            const Vec3 u_l = Vec3{0.0, uy, uz};

            const Vec3 x_pipe = x_pipe_holecenter + q_i.rotate_vector(u_l); // Displacements global frame

            const Mat3 R_i = q_i.to_matrix();

            const Vec3 u_i = x_pipe - X[i];

            // Following relations from Thiago Ritto thesis (Chapter 3)
            const Vec3 v_i = q_i.rotate_vector(Vec3{vx, vy, vz});

            // clang-format off
            Rz << cos(thetaz), -sin(thetaz), 0,
                  sin(thetaz),  cos(thetaz), 0,
                  0,            0,           1;
            
            Ry << cos(thetay), 0, sin(thetay),
                  0,      1,       0,
                 -sin(thetay), 0, cos(thetay);

            Rx << 1, 0,            0, 
                  0, cos(thetax), -sin(thetax),
                  0, sin(thetax),  cos(thetax);
            // clang-format on

            const Mat3 R_pipe = Rz * Ry * Rx;
            const Mat3 U = R_i * R_pipe;

            const Vec3 omega_u = Vec3{omegax + thetay * omegaz, cos(thetax) * omegay - sin(thetax) * omegaz,
                                      sin(thetax) * omegay + cos(thetax) * omegaz}; // Eq 3.7

            save_csv_global_line(ost, X[i], u_i, U, v_i, omega_u);
        }
    }
}


void PipeSolver::save_csv_fluid(Config &config, const Pipe &pipe, const uint top_node, ArenaBump &arena_h) {

    using namespace std;
    assert(config.save_csv && config.n % config.n_write == 0);


    const uint Nf = fluid.Nf;
    const uint N = pipe.N;
    const string output_subdir = config.get_current_output_subdir();
    /*--------------------------------------------------------------------
    Save fluid output
    --------------------------------------------------------------------*/
    {
        const string fluid_file = output_subdir + "/fluid.csv";
        ofstream ost{fluid_file};
        
        save_csv_fluid_header(config, ost);
        constexpr uint w = 11;

        ArrayView<Vec3> u = beam.get_field<BeamField::u>(arena_h.buf);
        ArrayView<scalar> p_i = fluid.get_field<FluidField::p_i>(arena_h.buf);
        ArrayView<scalar> p_o = fluid.get_field<FluidField::p_o>(arena_h.buf);
        ArrayView<scalar> v_i = fluid.get_field<FluidField::v_i>(arena_h.buf);
        ArrayView<scalar> v_o = fluid.get_field<FluidField::v_o>(arena_h.buf);
        ArrayView<scalar> dp_i = fluid.get_field<FluidField::dp_i>(arena_h.buf);
        ArrayView<scalar> dp_o = fluid.get_field<FluidField::dp_o>(arena_h.buf);
        
        scalar s_ie;
        for (uint i = top_node; i < Nf; i++) {
            if (i < N - 1){
                const scalar s_i = pipe.calc_s(i, u[i].x(), arena_h.buf);
                const scalar s_ip = pipe.calc_s(i+1, u[i+1].x(), arena_h.buf);
                s_ie = (s_i + s_ip) / 2.0;
            }
            else {
                s_ie = pipe.calc_s(N - 1, u[N - 1].x(), arena_h.buf);
            }

            ost << setw(w) << s_ie << ", " << setw(w) << p_i[i] << ", " << setw(w) << p_o[i] << ", "
                << setw(w) << v_i[i] << ", " << setw(w) << v_o[i] << ", "
                << setw(w) << dp_i[i] << ", " << setw(w) << dp_o[i];

            ost << endl;
        }
    }
}

void PipeSolver::copy_current_pipe_feed_to_host(ArenaBump &arena_h, ArenaBump &arena_d){
    assert(arena_h.mem_type == MemType::HOST && arena_d.mem_type == MemType::DEVICE);
    copy_beam_field_to_dst<BeamField::u>(arena_h, arena_d);
    copy_beam_field_to_dst<BeamField::i_pipe_to_ie_hole>(arena_h, arena_d);
}
void PipeSolver::copy_static_force_and_fluid_fields_to_device(ArenaBump &arena_h, ArenaBump &arena_d){
    assert(arena_h.mem_type == MemType::HOST && arena_d.mem_type == MemType::DEVICE);
    
    copy_beam_field_to_dst<curvlin::BeamField::f_hyd>(arena_d, arena_h);

    uint begin, end;
    fluid.get_field_begin_end(&begin, &end);
    ArenaBump::copy_slice(arena_d, arena_h, begin, end);
    
}

void PipeSolver::copy_beam_kinematics_to_device(ArenaBump &arena_h, ArenaBump &arena_d){
    assert(arena_h.mem_type == MemType::HOST && arena_d.mem_type == MemType::DEVICE);
    /*=========================================
    Copy u, v, a and i_pipe_to_ie_hole to device
    ==========================================*/

    //Utilize that u, v, and a are stored sequentially in the buffer
    uint u_offset = beam.offsets[(uint)BeamField::u].offset;
    uint a_offset = beam.offsets[(uint)BeamField::a].offset;
    uint a_count = beam.offsets[(uint)BeamField::a].count;
    ArenaBump::copy_slice(arena_d, arena_h, u_offset, a_offset + a_count * sizeof(scalar));
    
    copy_beam_field_to_dst<curvlin::BeamField::i_pipe_to_ie_hole>(arena_d, arena_h);

}


void PipeSolver::copy_bcs_to_dst(ArenaBump &arena_dst, const ArenaBump &arena_src, const BCsData &bcs) {
    assert(arena_dst.mem_type == MemType::HOST);
    /*=========================================
    Copy bcs to host
    ==========================================*/
        
    const uint offset = bcs.bcs_field.offset;
    ArenaBump::copy_slice(arena_dst, arena_src, offset, offset + sizeof(BCs));
}

void PipeSolver::copy_bit_rock_to_dst(ArenaBump &arena_h, const ArenaBump &arena_d, const BitRockData &bit_rock_data) {
    assert(arena_h.mem_type == MemType::HOST);

    // 1. Copy BitRock struct
    const uint offset = bit_rock_data.bit_rock_field.offset;
    ArenaBump::copy_slice(arena_h, arena_d, offset, offset + sizeof(BitRock));

    // 2. Copy cut_depths array
    uint cut_depths_offset = bit_rock_data.cut_depths_offset.offset;
    const uint cut_depths_count = bit_rock_data.cut_depths_offset.count;
    ArenaBump::copy_slice(
        arena_h,
        arena_d,
        cut_depths_offset,
        cut_depths_offset + cut_depths_count * sizeof(scalar)
    );
}

void PipeSolver::update_drilling_quantities_curvlin(Config &config, ConfigDynamic &conf_dyn, const Pipe &pipe, const ArenaBump &arena_h, const BitRockData &bit_rock_data) const {

    const ConfigStatic &conf_stat = config.conf_stat;
    const uint top_node = config.top_node;
    const uint N = pipe.N;
    
    if (conf_stat.bc_bit_rock_type != BC_BitRockType::NO_BIT) {
        BitRock &br = bit_rock_data.get_bit_rock_field(arena_h.buf);
        // ======= ROP Calculation======
        {

            scalar s_blade = br.s_blade;
            scalar s_max = conf_dyn.s_max;
            scalar t_now = config.t;
            scalar dt_elapsed = t_now - conf_dyn.t_ROP;
            if (dt_elapsed <= 0.0) dt_elapsed = conf_stat.dt;
            
            scalar ds_pos = std::max((scalar)0.0, s_blade - s_max);
            conf_dyn.ROP = ds_pos / dt_elapsed;
               
            // update bookkeeping only when bit advanced
            if (s_blade > s_max) {
                conf_dyn.s_max = s_blade;
                conf_dyn.t_ROP = t_now;
            }
            
        }
        conf_dyn.WOB = br.WOB;
        conf_dyn.TOB = br.TOB;

    } else {
        // fallback: kinematic last-node axial velocity (instantaneous)
        conf_dyn.ROP = std::max((scalar)0, beam.get_field<curvlin::BeamField::v>(arena_h.buf)[N - 1].x());

    }


    // ======= Hook Load/WOB & Top torque/TOB Calculation ======
    {
        const ArrayView<Vec3> u = beam.get_field<curvlin::BeamField::u>(arena_h.buf);
        const ArrayView<Vec3> theta = beam.get_field<curvlin::BeamField::theta>(arena_h.buf);

        scalar dS = pipe.dS_e(config.top_node, arena_h.buf);
        scalar A = pipe.Ap_e(config.top_node, arena_h.buf);
        scalar J = 2 * pipe.Ip_e(config.top_node, arena_h.buf);
        scalar E = config.conf_stat.E;
        scalar G = config.conf_stat.get_G();

        scalar f_hook_load = E * A * (u[top_node + 1].x() - u[top_node].x()) / dS; // Hook load force estimate
        scalar m_top_torque = G * J * (theta[top_node + 1].x() - theta[top_node].x()) / dS; // Top torque estimate
        conf_dyn.hook_load = f_hook_load; // tension positive
        conf_dyn.top_drive_torque = m_top_torque;

        //NOTE: This way to measure WOB and TOB is not the actual b-r response, but likely more similar to measurement subs
        //dS = pipe.dS_node_avg(N - 1, arena_h.buf);
        //A = pipe.Ap_e(N - 2, arena_h.buf);
        //J = 2 * pipe.Ip_e(N - 2, arena_h.buf);
        //scalar WOB = - E * A * (u[N - 1].x() - u[N - 2].x()) / dS; // WOB estimate
        //scalar TOB = - G * J * (theta[N - 1].x() - theta[N - 2].x()) / dS; // TOB estimate
        //conf_dyn.WOB = max(WOB, 0.0);
        //conf_dyn.TOB = max(TOB, 0.0);
    }


    conf_dyn.v_top =
            beam.get_field<curvlin::BeamField::v>(arena_h.buf)[config.top_node].x();
    conf_dyn.omega_top =
            beam.get_field<curvlin::BeamField::omega>(arena_h.buf)[config.top_node].x();

    conf_dyn.omega_bit = beam.get_field<curvlin::BeamField::omega>(arena_h.buf)[N - 1].x();


    scalar A_bit = M_PI * conf_stat.r_br * conf_stat.r_br;
    conf_dyn.MSE = conf_dyn.WOB / A_bit + conf_dyn.TOB * conf_dyn.omega_bit / (A_bit * conf_dyn.v_top + SMALL_SCALAR);
}

void PipeSolver::compute_generalized_strains(const Vec12 &U, const Vec10 &N, const Vec10 &dN, scalar kappa_y, scalar kappa_z,
    Vec3 &gamma, Vec3 &kappa) const {

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
        
    scalar tx = u4 * n_1 + u10 * n_2;
    scalar v = u2 * h1_1 + u6 * h1_2 + u8 * h1_3 + u12 * h1_4;
    scalar w = u3 * h1_1 - u5 * h1_2 + u9 * h1_3 - u11 * h1_4;
    scalar tz = u2 * h2_1 + u6 * h2_2 + u8 * h2_3 + u12 * h2_4;
    scalar ty = -u3 * h2_1 + u5 * h2_2 - u9 * h2_3 + u11 * h2_4;
        
    scalar u_s = u1 * dn_1 + u7 * dn_2;
    scalar tx_s = u4 * dn_1 + u10 * dn_2;
    scalar v_s = u2 * dh1_1 + u6 * dh1_2 + u8 * dh1_3 + u12 * dh1_4;
    scalar w_s = u3 * dh1_1 - u5 * dh1_2 + u9 * dh1_3 - u11 * dh1_4;
    scalar tz_s = u2 * dh2_1 + u6 * dh2_2 + u8 * dh2_3 + u12 * dh2_4;
    scalar ty_s = -u3 * dh2_1 + u5 * dh2_2 - u9 * dh2_3 + u11 * dh2_4;

    // // GEBT generalized translational strains
    gamma.x() = u_s + 0.5 * (u_s * u_s + v_s * v_s + w_s * w_s) + kappa_y * w + kappa_z * v;
    gamma.y() = v_s - tz;
    gamma.z() = w_s + ty;

    // GEBT generalized bending strains
    kappa.x() = tx_s - ty_s * tz;
    kappa.y() = ty_s + kappa_y + tx_s * tz;
    kappa.z() = tz_s - kappa_z - tx_s * ty;
}

scalar PipeSolver::compute_vm_stress_curvlin(const uint ie, scalar xi, const Pipe &pipe, const Hole &hole,
    const Config &config, const byte *buf) const {

    const ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);
    const ArrayView<Vec3> theta = beam.get_field<BeamField::theta>(buf);
    const ArrayView<uint> i_pipe_to_ie_hole = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);

    uint i_hole = i_pipe_to_ie_hole[ie];
    uint ip_hole = i_pipe_to_ie_hole[ie + 1];

    const scalar Le = pipe.dS_e(ie, buf);
    const scalar A = pipe.Ap_e(ie, buf);
    const scalar R = pipe.get_field<PipeField::ro>(buf)[ie]; // Outer radius
    const scalar k = pipe.k_s(ie, buf);
    const scalar I = pipe.Ip_e(ie, buf);
    const scalar E = config.conf_stat.E;
    const scalar G = config.conf_stat.get_G();
    const scalar alpha = 12.0 * E * I / (k * G * A * Le * Le);
    
    const scalar s_i = pipe.calc_s(ie, u[ie].x(), buf);
    const scalar s_ip = pipe.calc_s(ie + 1, u[ie + 1].x(), buf);

    ArrayView<Vec2> kappas = hole.get_field<HoleField::kappa>(buf);
    const Vec2 curv_i = hole.lerp_hole_property_from_pipe_node_position<Vec2>(kappas, s_i, i_hole, buf);
    const Vec2 curv_ip = hole.lerp_hole_property_from_pipe_node_position<Vec2>(kappas, s_ip, ip_hole, buf);
    const Vec2 curvatures = (curv_i + curv_ip) / 2; // Elemental curvature
    scalar kappa_y = curvatures.x();
    scalar kappa_z = curvatures.y();
    
    // Use shape functions and their derivatives
    Vec12 U;
    U << u[ie].x(), u[ie].y(), u[ie].z(), theta[ie].x(), theta[ie].y(), theta[ie].z(), u[ie + 1].x(),
    u[ie + 1].y(), u[ie + 1].z(), theta[ie + 1].x(), theta[ie + 1].y(), theta[ie + 1].z();
    Vec10 N_funcs = shape_functions(xi, alpha, Le);
    Vec10 dN_funcs = shape_function_derivatives(xi, alpha, Le);
    Vec3 gamma, kappa;
    compute_generalized_strains(U, N_funcs, dN_funcs, kappa_y, kappa_z, gamma, kappa);
    
    // --- Von Mises Stress [Pa] ---
    return von_mises_stress_from_strains(gamma, kappa, E, G, R);
}



scalar PipeSolver::compute_radial_acceleration(const uint i, const Pipe &pipe, const Hole &hole, const byte *buf) const {
    const ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);
    const ArrayView<Vec3> a = beam.get_field<BeamField::a>(buf);
    const ArrayView<uint> i_pipe_to_ie_hole = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);    
    const ArrayView<Vec2> co_vec = hole.get_field<HoleField::co>(buf);

    scalar uy_i = u[i].y();
    scalar uz_i = u[i].z();
    scalar ay_i = a[i].y();
    scalar az_i = a[i].z();
    
    uint i_h = i_pipe_to_ie_hole[i];
    
    scalar s_i = pipe.calc_s(i, u[i].x(), buf);
    
    Vec2 co = hole.lerp_hole_property_from_pipe_node_position(co_vec, s_i, i_h, buf);

    Vec2 u_cent = Vec2(uy_i, uz_i) - co; // Vector from hole center to pipe center
    scalar u_cent_norm = u_cent.norm();
    Vec2 e_u_cent = u_cent / (u_cent_norm + SMALL_SCALAR); // Avoid div by zero

    // Radial acceleration (projection of acceleration on radial direction)
    // Project acceleration on radial direction and convert from m/s^2 to g
    scalar a_r = ay_i * e_u_cent.x() + az_i * e_u_cent.y();
    constexpr scalar g_acc = 9.80665;
    return fabs(a_r) / g_acc;
}

scalar PipeSolver::compute_tangential_acceleration(const uint i, const Pipe &pipe, const Hole &hole, const byte *buf) const {
    const ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);
    const ArrayView<Vec3> a = beam.get_field<BeamField::a>(buf);
    const ArrayView<uint> i_pipe_to_ie_hole = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);    
    const ArrayView<Vec2> co_vec = hole.get_field<HoleField::co>(buf);

    scalar uy_i = u[i].y();
    scalar uz_i = u[i].z();
    scalar ay_i = a[i].y();
    scalar az_i = a[i].z();
    
    uint i_h = i_pipe_to_ie_hole[i];
    
    scalar s_i = pipe.calc_s(i, u[i].x(), buf);
    
    Vec2 co = hole.lerp_hole_property_from_pipe_node_position(co_vec, s_i, i_h, buf);

    Vec2 u_cent = Vec2(uy_i, uz_i) - co; // Vector from ellipse center to pipe center
    scalar u_cent_norm = u_cent.norm();
    Vec2 e_u_cent = u_cent / (u_cent_norm + SMALL_SCALAR); // Avoid div by zero
    
    // Tangential acceleration (projection of acceleration on tangential direction)
    scalar a_t = - ay_i * e_u_cent.y() + az_i * e_u_cent.x();
    constexpr scalar g_acc = 9.80665;
    return fabs(a_t) / g_acc;
}

scalar PipeSolver::compute_standoff(const uint i, const Pipe &pipe, const Hole &hole, const byte *buf) const {
    const ArrayView<Vec3> u = beam.get_field<BeamField::u>(buf);
    const ArrayView<uint> i_pipe_to_ie_hole = beam.get_field<BeamField::i_pipe_to_ie_hole>(buf);    
    const ArrayView<scalar> a_vec = hole.get_field<HoleField::a>(buf);
    const ArrayView<scalar> b_vec = hole.get_field<HoleField::b>(buf);
    const ArrayView<Vec2> co_vec = hole.get_field<HoleField::co>(buf);
    scalar ux_i = u[i].x();
    scalar uy_i = u[i].y();
    scalar uz_i = u[i].z();
    
    uint i_h = i_pipe_to_ie_hole[i];
    
    scalar s_i = pipe.calc_s(i, ux_i, buf);
    scalar a_h = hole.lerp_hole_property_from_pipe_node_position(a_vec, s_i, i_h, buf);
    scalar b_h = hole.lerp_hole_property_from_pipe_node_position(b_vec, s_i, i_h, buf);
    
    Vec2 co = hole.lerp_hole_property_from_pipe_node_position(co_vec, s_i, i_h, buf);
    scalar r_p = pipe.ro_n_max(i, buf);
    
    Vec2 u_cent = Vec2(uy_i, uz_i) - co; // Vector from ellipse center to pipe center
    scalar d_norm = u_cent.norm();
    Vec2 d = u_cent / (d_norm + SMALL_SCALAR); // Avoid div by zero
    
    // Distance from ellipse center to ellipse boundary in direction d
    scalar r_ellipse = (a_h * b_h) / sqrt(pow(b_h * d.x(), 2) + pow(a_h * d.y(), 2));
    scalar so_dist = r_ellipse - r_p - d_norm;
    return 100.0 * so_dist / (r_ellipse - r_p);
}

} // namespace curvlin