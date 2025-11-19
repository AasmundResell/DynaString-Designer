
#pragma once
#include "misc/config.hpp"
#include "misc/includes.hpp"
#include "misc/utils.hpp"

class Hole;

struct HoleSection {

    // Hole section properties
    string name;                                                      // (Unique) name of the hole section
    scalar depth{0.0};                                                // (required) Start depth of the hole section
    HoleSurfaceType surface_type;                                     // (required) Type of the hole surface
    HoleOffsetBuildType offset_build_type{HoleOffsetBuildType::NONE}; // (optional) Type of the hole offset build
    HoleCrossSectionBuildType cross_section_build_type{
        HoleCrossSectionBuildType::CONSTANT}; // (optional) Type of the hole cross section build
    scalar D;                                 // (required circular) Base diameter
    scalar a_mean;                            // (required elliptical) Ellipse parameter a
    scalar b_mean;                            // (required elliptical) Ellipse parameter b
    scalar dalpha{0.0};          // (optional)[rad/m] Uniform rotation angle change per hole length for ellipse surface
    scalar pitch{0.0};           // (optional) Pitch for hole offset spiral
    scalar diameter_spiral{0.0}; // (optional) Diameter for hole offset spiral
    scalar diameter_drift{
        std::numeric_limits<scalar>::max()}; // (optional) Diameter upper limit to restrict the drift of the hole center
    scalar max_curvature{0.005};             // (optional) Maximum allowed curvature for the hole center curve, used to
                                             // restrict the pitch of the spiral if needed

    // RANDOM FIELD Parameters (Corresponding parameters above are used as the mean values)
    scalar std_pr_radial{SMALL_SCALAR}; // (optional) Standard deviation percentage of the mean value for the radial
                                        // parameters in the random field
    scalar std_pr_dalpha{
        SMALL_SCALAR}; // (optional) Standard deviation percentage of the mean value for dalpha in the random field
    scalar std_pr_pitch{
        SMALL_SCALAR}; // (optional) Standard deviation percentage of the mean value for the pitch in the random field
    scalar std_pr_diameter_spiral{SMALL_SCALAR}; // (optional) Standard deviation percentage of the mean value for the
                                                 // spiral diameter in the random field
    scalar std_pr_off_noise{SMALL_SCALAR}; // (optional) Standard deviation percentage of the mean value of the mean
                                           // spiral diameter for the noise in the offset vector
    scalar cor_radial{SMALL_SCALAR}; // (optional) Correlation length for the radial parameters in the random field
    scalar cor_dalpha{SMALL_SCALAR}; // (optional) Correlation length for the dalpha parameter in the random field
    scalar cor_pitch{SMALL_SCALAR};  // (optional) Correlation length for the pitch parameter in the random field
    scalar cor_diameter_spiral{
        SMALL_SCALAR};                  // (optional) Correlation length for the spiral diameter in the random field
    scalar cor_off_noise{SMALL_SCALAR}; // (optional) Correlation length for the noise in the offset vector
};

enum class HoleField {
    // N_hole + 2 is the number of hole nodes + two ghost nodes to handle the drill floor
    s,     /*(N_hole + 2) Curvilinear coordinates describing the Hole discretization*/
    x,     /*(N_hole + 2) Coordinates of centre of well. Defines the Hole discretization*/
    q,     /*(N_hole + 2) Quaternions of the orientation of the hole nodes*/
    g,     /*(N_hole + 2) The gravity vectors in the local frame for each element*/
    kappa, /*(N_hole + 2) The curvatures, for y and z directions respectively */
    a,     /*(N_hole + 2) constants in the ellipse equation x²/a² + y²/b² = 1*/
    b,     /*(N_hole + 2) constants in the ellipse equation x²/a² + y²/b² = 1*/
    alpha, /*(N_hole + 2) Rotation of the elliptical section*/
    co,    /*(N_hole + 2) Offsets from hole section center*/
    COUNT
};

// clang-format off
template <HoleField array> struct HoleFieldTraits;
template <> struct HoleFieldTraits<HoleField::s> { using variable_type = scalar; };
template <> struct HoleFieldTraits<HoleField::x> { using variable_type = Vec3; };
template <> struct HoleFieldTraits<HoleField::q> { using variable_type = Quaternion; };
template <> struct HoleFieldTraits<HoleField::g> { using variable_type = Vec3; };
template <> struct HoleFieldTraits<HoleField::kappa> { using variable_type = Vec2; };
template <> struct HoleFieldTraits<HoleField::a> { using variable_type = scalar; };
template <> struct HoleFieldTraits<HoleField::b> { using variable_type = scalar; };
template <> struct HoleFieldTraits<HoleField::alpha> { using variable_type = scalar; };
template <> struct HoleFieldTraits<HoleField::co> { using variable_type = Vec2; };


class Hole {
  public:
    uint N_hole;
    HoleSurfaceType type;
    OffsetUntyped offsets[(uint)HoleField::COUNT];

    template <HoleField field>
    DEVICE_FUNC ArrayView<typename HoleFieldTraits<field>::variable_type> get_field(byte *buf) const {
        static_assert((uint)field < (uint)HoleField::COUNT);
        using T = typename HoleFieldTraits<field>::variable_type;
        OffsetUntyped uoffset = offsets[(uint)field];
        Offset<T> offset = {.offset = uoffset.offset, .count = uoffset.count};
        assert(offset.offset > 0 && offset.count > 0);
        return {buf, offset};
    }

    template <HoleField field>
    DEVICE_FUNC const ArrayView<typename HoleFieldTraits<field>::variable_type> get_field(const byte *buf) const {
        static_assert((uint)field < (uint)HoleField::COUNT);
        using T = typename HoleFieldTraits<field>::variable_type;
        OffsetUntyped uoffset = offsets[(uint)field];
        Offset<T> offset = {.offset = uoffset.offset, .count = uoffset.count};
        assert(offset.offset > 0 && offset.count > 0);
        return {buf, offset};
    }

    DEVICE_FUNC scalar A_n(uint i, const byte *buf) const {
        assert(type == HoleSurfaceType::CIRCULAR);
        assert(i < N_hole);
        ArrayView<scalar> a = get_field<HoleField::a>(buf);
        return M_PI * powi(a[i], 2);
    }

    DEVICE_FUNC scalar get_L(const byte *buf) const {
        ArrayView<scalar> s = get_field<HoleField::s>(buf);
        return s[N_hole + 1]; // length of s array is N_hole + 2
    }

    template <typename T>
    DEVICE_FUNC inline T lerp_hole_property_from_pipe_node_position(const ArrayView<T> v, const scalar s_i, uint i_hole, const byte *buf) const {
        assert(s_i <= this->get_L(buf) + 1.0e-3); //Allow for slightly outside of hole?
        //assert(get_increment_index_pipe_node_to_hole_segment(i_hole, s_i, buf) == 0);

        //Ensure interpolation within the last segment 
        if (i_hole > N_hole) {
            i_hole = N_hole - 1; 
        }
        ArrayView<scalar> s = get_field<HoleField::s>(buf);
        scalar ds = s[i_hole + 1] - s[i_hole];

        const scalar s_hole_i = s[i_hole];
        const T v_interp = v[i_hole] + (v[i_hole + 1] - v[i_hole]) * (s_i - s_hole_i) / ds; /*Linear interpolation*/
        return v_interp;
    }

    template <typename T>
    DEVICE_FUNC inline T gradient_hole_property_section(const ArrayView<T> v, uint i_hole, const byte *buf) const {
        // Ensure within bounds
        if (i_hole >= N_hole) {
            i_hole = N_hole - 1;
        }
        ArrayView<scalar> s = get_field<HoleField::s>(buf);
        scalar ds = s[i_hole + 1] - s[i_hole];
        // Gradient: (v[i+1] - v[i]) / (s[i+1] - s[i])
        return (v[i_hole + 1] - v[i_hole]) / ds;
    }

    DEVICE_FUNC int get_increment_index_pipe_node_to_hole_segment(uint ih, const scalar s_i, const byte *buf) const {
        assert(ih < N_hole + 1); // check bounds
        ArrayView<scalar> s = get_field<HoleField::s>(buf);
    
        if (s_i < s[ih]) {
            return -1;
        } else if (s_i <= s[ih + 1]) {
            return 0;
        } else {
            assert(s_i > s[ih + 1]); 
            return 1;
        }
    }

    DEVICE_FUNC inline scalar calc_pipe_element_hole_radius(scalar s_i, scalar s_ip, int i_hole, int ip_hole, const byte *buf) const {
        /* Calculates the equivalent hole radius for a pipe element
            by averaging the hole radius at the two nodes */
        ArrayView<scalar> a = get_field<HoleField::a>(buf);
        ArrayView<scalar> b = get_field<HoleField::b>(buf);
        // Possible fix if we dont want if statements: Use ellipse calculation for circle as well
        if (type == HoleSurfaceType::CIRCULAR) {
            const scalar r_i = lerp_hole_property_from_pipe_node_position<scalar>(a, s_i, i_hole, buf);
            const scalar r_ip = lerp_hole_property_from_pipe_node_position<scalar>(a, s_ip, ip_hole, buf);
            return (r_i + r_ip) / 2;
        } else if (type == HoleSurfaceType::ELLIPTICAL) {
            const scalar a_i = lerp_hole_property_from_pipe_node_position<scalar>(a, s_i, i_hole, buf);
            const scalar a_ip = lerp_hole_property_from_pipe_node_position<scalar>(a, s_ip, ip_hole, buf);
            const scalar b_i = lerp_hole_property_from_pipe_node_position<scalar>(b, s_i, i_hole, buf);
            const scalar b_ip = lerp_hole_property_from_pipe_node_position<scalar>(b, s_ip, ip_hole, buf);
            const scalar a_avg = (a_i + a_ip) / 2.0;
            const scalar b_avg = (b_i + b_ip) / 2.0;
            return sqrt(a_avg * b_avg); // Equivalent area formula: A = pi * a * b
        } else {
            assert(false);
            return 0.0;
        }
    }


void initalize_pipe_nodes_to_hole_segments(const uint N, const Config & config, uint * indices, const ArrayView<scalar> S, const byte * buf) const;

  private:
    int get_increment_index_pipe_node_to_hole_segment_initial(uint ih, const scalar s_i, const byte * buf) const;
    

    DEVICE_FUNC int find_s_index(scalar s, int i_hole) const {
        assert(false); // Not implemented yet
        return 0;
    }

    // void add_hole_segment(const scalar r, const Mat3 &R);
};

Hole hole_create_from_vector(HoleTrajectoryType trajectory_type, uint N_hole, vector<scalar> &MD, vector<scalar> &inclinations, vector<scalar> &azimuths, ArenaBump &arena_h);

Hole hole_create_from_linear(HoleTrajectoryType trajectory_type, uint N_hole, vector<scalar> &MD,
                             scalar inclination_top, scalar inclination_build, scalar azimuth_top, scalar azimuth_build,
                             scalar L_total, scalar L_front, scalar L_end, ArenaBump &arena_h);
void create_hole_surface(Config &config, Hole &hole, const vector<HoleSection> &hole_sections, ArenaBump &arena_h);
void save_hole_csv(const Config &config, const Hole &hole, ArenaBump &arena_h);
