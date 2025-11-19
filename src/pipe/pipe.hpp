#pragma once
#include "misc/config.hpp"
#include "misc/includes.hpp"

struct EccentricMass {
    // Eccentric mass properties
    scalar S_ecc; // Center position of the eccentric mass along the pipe component
    scalar L_ecc; // Total (axial) length of the eccentric mass
    scalar r_ecc; // Radial distance of the eccentric mass center from the pipe center
};

struct Centralizer {
    scalar S_cent;
    CentralizerType type;
};

struct PipeComponent {

    // Pipe component properties
    ComponentType type;
    string name;         // (Unique) name of the pipe component
    scalar L;            // Length of the pipe component
    scalar S_global;     // Global S position of the start of the pipe component
    scalar Do;           // Base outer diameter
    scalar Di;           // Base inner diameter
    scalar L_tool;       // (optional) Length of the tool joint
    scalar D_tool;       // (optional) Diameter of the tool joint
    scalar D_stabilizer; // (optional) Diameter of the stabilizer
    scalar S_stabilizer; // (optional) Position of the stabilizer along the component
    scalar L_stabilizer; // (optional) Length of the stabilizer
    scalar L_sensor;     // (optional) Length of the MWD sensor section
    scalar S_sensor;     // (optional) Position of the MWD sensor section along the component

    bool has_tool_joints = false;
    bool has_sensors = false;
    bool has_damping = false;
    bool has_steering = false;
    bool has_stabilizer = false;
    bool has_mass_imbalance = false;

    SteeringMode steering_mode = SteeringMode::NONE;
    vector<EccentricMass> eccentric_masses;

    // Computational properties
    uint i_top_node; // Index of the top node in the component
    uint i_top_poly; // Top polygon index in the component
};

using Polygon = vector<Vec2>;

struct PipeRenderComponents {
    vector<Polygon> polygons;
    vector<uint> arr_ie;  /*size= polygons.size() + 1*/
    vector<uint> arr_ip;  // ID of corresponding pipe component
    vector<scalar> arr_S; /*size= polygons.size() + 1*/
};

enum class PipeField {
    S,             /*Size (N) Reference coordinates of FEM nodes. Defines the pipe discretization*/
    ro,            /*Size (N - 1) Outer radii of pipe for all elements*/
    ri,            /*Size (N - 1) Inner radii of pipe for all elements*/
    ec,            /*Size (N - 1) Eccentric mass span as fraction of the element [0,1]*/
    Sc,            /*Size (Nc) Reference coordinates of contact points along the pipe*/
    rc,            /*Size (Nc) Contact point radii along the pipe*/
    num_ic_to_ie,  /*Size (N) First contact point index in each element*/
    num_ibs_to_ie, /*Size (N) First bow-spring contact point index in each element*/
    num_ir_to_ie,  /*Size (N) First Riemann invariant index in each element*/
    Sbs,           /*Size (Nbs) Reference coordinates of bow-spring contact points along the pipe*/
    COUNT
};

// clang-format off
template <PipeField array> struct PipeFieldTraits;
/*The variable type of each field*/
template <> struct PipeFieldTraits<PipeField::S> { using variable_type = scalar; };
template <> struct PipeFieldTraits<PipeField::ro> { using variable_type = scalar; };
template <> struct PipeFieldTraits<PipeField::ri> { using variable_type = scalar; };
template <> struct PipeFieldTraits<PipeField::ec> { using variable_type = scalar; };
template <> struct PipeFieldTraits<PipeField::Sc> { using variable_type = scalar; };
template <> struct PipeFieldTraits<PipeField::rc> { using variable_type = scalar; };
template <> struct PipeFieldTraits<PipeField::num_ic_to_ie> { using variable_type = uint; };
template <> struct PipeFieldTraits<PipeField::num_ibs_to_ie> { using variable_type = uint; };
template <> struct PipeFieldTraits<PipeField::num_ir_to_ie> { using variable_type = uint; };
template <> struct PipeFieldTraits<PipeField::Sbs> { using variable_type = scalar; };



struct Pipe {  
    
    OffsetUntyped offsets[(uint)PipeField::COUNT];
    scalar L_tot;  /*Total undeformed length of the pipe */
    uint N;        /*Number of pipe nodes*/
    uint Nc;       /*Number of contact points*/
    uint Nr;      /*Number of Riemann invariants*/
    uint Nbs;      /*Number of bow-spring contact points*/
    scalar dS_min; /*Minimum element length */
    scalar nu;     /*poisson ratio*/

    template <PipeField field>
    DEVICE_FUNC ArrayView<typename PipeFieldTraits<field>::variable_type> get_field(byte *buf) const {
        static_assert((uint)field < (uint)PipeField::COUNT);
        using T = typename PipeFieldTraits<field>::variable_type;
        OffsetUntyped uoffset = offsets[(uint)field];
        Offset<T> offset = {.offset = uoffset.offset, .count = uoffset.count};
            assert(offset.offset > 0 && offset.count > 0);
        return {buf, offset};
    }

    template <PipeField field>
    DEVICE_FUNC const ArrayView<typename PipeFieldTraits<field>::variable_type> get_field(const byte *buf) const {
        static_assert((uint)field < (uint)PipeField::COUNT);
        using T = typename PipeFieldTraits<field>::variable_type;
        OffsetUntyped uoffset = offsets[(uint)field];
        Offset<T> offset = {.offset = uoffset.offset, .count = uoffset.count};
        assert(offset.offset > 0);
        assert(offset.count > 0);
        return {buf, offset};
    }

    DEVICE_FUNC uint get_Ne() const { return N - 1; }

    /*Finds the index of thee last node that is smaller than s,
    measured from the beginning of the string*/

    /*The curvilinear coordinate of a node on the string measured from the top of the hole*/
    DEVICE_FUNC scalar calc_s(uint i, scalar ux_i, const byte *buf) const;

    /*The curvilinear coordinate of a contact point on the string measured from the top of the hole*/
    DEVICE_FUNC scalar calc_sc(uint i, scalar ux_i, const byte *buf) const;

    /*The curvilinear coordinate of a bow-spring contact point on the string measured from the top of the hole*/
    DEVICE_FUNC scalar calc_sbs(uint ibs, scalar ux_i, const byte * buf) const;

    /*Length of an element*/
    DEVICE_FUNC scalar dS_e(uint ie, const byte *buf) const;

    /*Maximum outer radius of a node*/
    DEVICE_FUNC scalar ro_n_max(uint i, const byte *buf) const;

    /*Minimum outer radius of a node*/
    DEVICE_FUNC scalar ro_n_min(uint i, const byte *buf) const;

    /*Solid pipe area of an element*/
    DEVICE_FUNC scalar Ap_e(uint ie, const byte *buf) const;

    /*Inner fluid area of an element*/
    DEVICE_FUNC scalar Af_e(uint ie, const byte *buf) const;

    /*Second area moment of inertia of solid pipe of an element*/
    DEVICE_FUNC scalar Ip_e(uint ie, const byte *buf) const;

    /*Second area moment of inertia of inner fluid of an element*/
    DEVICE_FUNC scalar If_e(uint ie, const byte *buf) const;

    /*Radial eccentricity of the element mass center*/
    DEVICE_FUNC scalar r_ec(uint ie, scalar outer_radial_frac, const byte * buf) const;

    /*Shear correction coefficient for circular pipes*/
    DEVICE_FUNC scalar k_s(uint ie, const byte *buf) const;

    /*The distance between two element centroids (Half element length if first or last element is selected)*/
    DEVICE_FUNC scalar dS_node_avg(uint i, const byte *buf) const;

    /*Finds the index of the first node to the left of the reference coordinate S defined on [0, L_tot], 
    only used when last previous index is known*/
    DEVICE_FUNC uint find_node_index_from_curve_length(const scalar S, const byte *buf, const uint i_prev) const;

    /*Finds the index of the first node to the left of the reference coordinate S defined on [0, L_tot]*/
    uint find_intial_top_node_from_curve_length(const scalar S_i, const byte * buf) const;


};

Pipe pipe_create(Config &config, vector<PipeComponent> &pipe_assembly, PipeRenderComponents *render_components,
                 vector<Centralizer> &centralizers, ArenaBump &arena_h);

void save_pipe_csv(const Config &config, const Pipe &pipe, const byte *buf);

#include "pipe.inl"