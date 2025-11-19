#pragma once
#include "misc/includes.hpp"
#include "pipe-solver/pipe-solver-curvlin/beam-curvlin.hpp"
#include "pipe-solver/pipe-solver-curvlin/fluid-curvlin.hpp"

constexpr scalar display_margin = 0.1;

enum class PlotSpatialVariable {
    DISPLACEMENT_TRANS = 0,
    DISPLACEMENT_ROT,
    VELOCITY_TRANS,
    VELOCITY_ROT,
    ACCELERATION_TRANS,
    ACCELERATION_ROT,
    FORCE_INT,
    MOMENT_INT,
    FORCE_EXT_DYN,
    MOMENT_EXT_DYN,
    FORCE_EXT_STAT,
    MOMENT_EXT_STAT,
    COUNT
};

enum class MiscPlotSpatialVariable {
    AXIAL_TENSION = 0,
    EFFECTIVE_TENSION,
    EFFECTIVE_TORQUE,
    RADIAL_ACCELERATION,
    TANGENTIAL_ACCELERATION,
    AXIAL_STRAIN,
    STANDOFF_RATIO,
    VON_MISES_STRESS,
    LINEAR_CONTACT_FORCE,
    TANGENTIAL_CONTACT_FORCE,
    COUNT
};

enum class FluidPlotSpatialVariable {
    PRESSURE_INNER = 0,
    PRESSURE_OUTER,
    VELOCITY_INNER,
    VELOCITY_OUTER,
    dPRESSURE_INNER,
    dPRESSURE_OUTER,
    COUNT
};

enum class ContourPlotVariable {
    NONE = 0,
    AXIAL_TENSION,
    EFFECTIVE_TENSION,
    EFFECTIVE_TORQUE,
    STANDOFF_RATIO,
    AXIAL_STRAIN,
    RADIAL_ACCELERATION,
    TANGENTIAL_ACCELERATION,
    VON_MISES_STRESS,
    LINEAR_CONTACT_FORCE,
    TANGENTIAL_CONTACT_FORCE,
    PRESSURE_INNER,
    PRESSURE_OUTER,
    VELOCITY_INNER,
    VELOCITY_OUTER,
    COUNT
};

// Number of spatial graphs and misc graphs
uint constexpr N_SG = (uint)PlotSpatialVariable::COUNT * 3;
uint constexpr N_MG = (uint)MiscPlotSpatialVariable::COUNT;
uint constexpr N_FG = (uint)FluidPlotSpatialVariable::COUNT;

enum class Direction {
    X = 0,
    Y,
    Z,
};

constexpr array<const char *, 3> directions_strings = {"x", "y", "z"};

// Labels/units now owned by each SpatialGraph instance (to avoid global ordering issues)

struct SpatialGraph {
    bool show_graph_plot = false;
    bool auto_size = true;
    PlotSpatialVariable variable;
    Direction direction;
    vector<scalar> data;  // Pointer to the data array for this graph
    float minimal_window; // Minimal window size for the graph
    float y_min = 0.0;
    float y_max = 0.0;
    float s_min = 0.0;
    float s_max = 0.0;
    float data_min = std::numeric_limits<float>::max();
    float data_max = std::numeric_limits<float>::lowest();

    // human-readable items owned by the graph instance
    string symbol; // short symbol used in keys, e.g. "u", "f_int"
    string label;  // display name, e.g. "Displacement"
    string unit;   // unit string, e.g. "[m]"

    SpatialGraph(PlotSpatialVariable var, Direction dir, float min_window, const string &symbol_, const string &label_,
                 const string &unit_)
        : variable(var), direction(dir), minimal_window(min_window), symbol(symbol_), label(label_), unit(unit_) {}

    string get_variable_string() const;

    string get_y_label() const;

    void reset_scale();

    void update_auto_scale(uint index);
};

struct MiscSpatialGraph {
    MiscPlotSpatialVariable variable;
    ContourPlotVariable contour_variable; // Corresponding contour variable
    vector<scalar> data;                  // Pointer to the data array for this graph
    const bool elementwise;
    bool show_graph_plot = false;
    float y_min = 0, y_max = 0;
    float contour_min = 0, contour_max = 0;
    float s_min = 0, s_max = 0;
    float data_min;
    float data_max;

    bool auto_size = true;
    string label;
    string colorbar_label; // Use a short version for the colorbar
    string unit;
    MiscSpatialGraph(MiscPlotSpatialVariable var, ContourPlotVariable contour_var, bool elementwise_,
                     const string &label_, const string &colorbar_label_, const string &unit_,
                     float min_data = std::numeric_limits<float>::max(),
                     float max_data = std::numeric_limits<float>::lowest())
        : variable(var), contour_variable(contour_var), elementwise(elementwise_), label(label_),
          colorbar_label(colorbar_label_), unit(unit_), data_min(min_data), data_max(max_data) {}

    string get_graph_label() const;
    string get_colorbar_label() const;
    void reset_scale();
};

struct FluidSpatialGraph {
    FluidPlotSpatialVariable variable;
    ContourPlotVariable contour_variable; // Corresponding contour variable
    vector<scalar> data;                  // Pointer to the data array for this graph
    bool show_graph_plot = false;
    float y_min = 0, y_max = 0;
    float contour_min = 0, contour_max = 0;
    float s_min = 0, s_max = 0;
    float data_min = std::numeric_limits<float>::max();
    float data_max = std::numeric_limits<float>::lowest();

    float minimal_window; // Minimal window size for the graph

    bool auto_size = true;
    string label;
    string colorbar_label; // Use a short version for the colorbar
    string unit;
    FluidSpatialGraph(FluidPlotSpatialVariable var, ContourPlotVariable contour_var, const string &label_,
                      const string &colorbar_label_, const string &unit_, float minimal_window_ = 0.1f)
        : variable(var), contour_variable(contour_var), label(label_), colorbar_label(colorbar_label_), unit(unit_),
          minimal_window(minimal_window_) {}

    string get_graph_label() const;
    string get_colorbar_label() const;
    void update_auto_scale(uint index);
    void reset_scale();
};
