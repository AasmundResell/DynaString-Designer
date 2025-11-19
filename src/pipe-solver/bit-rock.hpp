#pragma once
#include "misc/config.hpp"
#include "misc/includes.hpp"

struct BitRock {
    uint N_cells{0};                     // Number of cells (runtime size)
    scalar theta{0.0};                           /*Orientation of the drill head[rad] */
    scalar omega{0.0};                          /*Angular velocity of the drill head[rad/s] */
    scalar s_blade{0.0};                         /*Absolute depth measured from the top*/
    scalar d_cut{0.0};                           /*Instantanous cut-depth*/
    scalar ds_cut_last{0.0};                     /*Axial advance of cut rock used for ROP*/
    scalar WOB{0.0}, TOB{0.0};                   /*Weight on bit and torque on bit*/
    scalar Z0{0.0};                             /*Vertical position of the bit*/
    uint ptr{0};                                 /*pointer to first cell*/
    
    // Store previous timestep values for delta calculations
    scalar theta_old{0.0};                       /*Previous orientation for delta calculation*/
    scalar s_blade_old{0.0};                     /*Previous blade position for delta calculation*/

    
    DEVICE_FUNC scalar cell_size_rad(uint n_blades) const { return 2 * M_PI / (n_blades * N_cells); }

};

struct BitRockData {
    Offset<BitRock> bit_rock_field;
    Offset<scalar> cut_depths_offset; // Offset for cut depths array
    

    DEVICE_FUNC ArrayView<scalar> get_cut_depths(byte *buf) const {
        return {buf, cut_depths_offset};
    }
    DEVICE_FUNC const ArrayView<scalar> get_cut_depths(const byte *buf) const {
        return {buf, cut_depths_offset};
    }

    DEVICE_FUNC BitRock &get_bit_rock_field(byte *buf) const {
        ArrayView<BitRock> bit_rock_field_view = {buf, bit_rock_field};
        assert(bit_rock_field_view.count == 1);
        return bit_rock_field_view[0];
    }
    DEVICE_FUNC const BitRock &get_bit_rock_field(const byte *buf) const {
        ArrayView<BitRock> bit_rock_field_view = {buf, bit_rock_field};
        assert(bit_rock_field_view.count == 1);
        return bit_rock_field_view[0];
    }


};

BitRockData create_bit_rock(Config &config, ArenaBump &arena_h);

DEVICE_FUNC void update_bit_kinematics(const ConfigStatic &conf_stat, BitRockData &bit_rock, byte *buf);

DEVICE_FUNC void cut_rock_surface(const ConfigStatic &conf_stat, BitRockData &bit_rock, byte *buf);

DEVICE_FUNC void cut_rock_surface_PDE(const ConfigStatic &conf_stat, BitRockData &bit_rock, byte *buf);

DEVICE_FUNC void calculate_bit_rock_Detournay(const ConfigStatic &conf_stat, BitRock &br, 
                                              const scalar v_bit);

DEVICE_FUNC void calculate_bit_rock_Detournay_w_reaction(const ConfigStatic &conf_stat,
                                              BitRock &br,
                                              const scalar v_bit, const scalar omega_bit,
                                              const scalar F_reaction, const scalar T_reaction);
                                              
DEVICE_FUNC void calculate_bit_rock_Detournay_regularized(const ConfigStatic &conf_stat, BitRock &br, 
                                              const scalar v_bit);

DEVICE_FUNC void calculate_bit_rock_Tucker_Wang(const ConfigStatic &conf_stat, BitRock &br, const scalar v_bit);

DEVICE_FUNC void calculate_bit_rock_Tucker_Wang_torsional(const ConfigStatic &conf_stat, BitRock &br,
                                                          const scalar v_bit);