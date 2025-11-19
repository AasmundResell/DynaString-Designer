#pragma once
#include "misc/includes.hpp"
#include "pipe-solver/pipe-solver-curvlin/beam-curvlin.hpp"

enum class DrillingVariable {
    BLOCK_POSITION,
    HOOK_LOAD,
    TOP_DRIVE_TORQUE,
    WOB,
    TOB,
    ROP,
    FEED_RATE,
    OMEGA_BIT,
    OMEGA_TOP_DRIVE,
    MSE,
    COUNT
};

constexpr array<const char *, (uint)DrillingVariable::COUNT> drilling_time_variable_name_array = {
    "Block Position",      "Hook Load", "Top Drive Torque", "Weight on Bit",   "Torque on Bit",
    "Rate of Penetration", "Feed Rate", "Omega Bit",        "Omega Top Drive", "Mechanical Specific Energy"};

constexpr array<const char *, (uint)DrillingVariable::COUNT> drilling_variable_unit_array = {
    "[m]", "[kN]", "[kNm]", "[kN]", "[kNm]", "[m/s]", "[m/s]", "[RPM]", "[RPM]", "[J/m^3]"};
struct HolePlotData {
    uint N_hole;
    vector<scalar> s;
    vector<scalar> a;
    vector<scalar> b;
    vector<scalar> kappa_y;
    vector<scalar> kappa_z;
    vector<scalar> offset_y;
    vector<scalar> offset_z;
    vector<scalar> alpha;

    scalar y_min_curvature = 0.0;
    scalar y_max_curvature = 0.0;
    scalar y_min_radius = 0.0;
    scalar y_max_radius = 0.0;
    scalar y_min_offsets = 0.0;
    scalar y_max_offsets = 0.0;
    scalar y_min_rotation = 0.0;
    scalar y_max_rotation = 0.0;

    void allocate(const Hole hole, const byte *buf);
};

static constexpr float MAX_FPS = 60.0f;
static constexpr float SAFETY_MARGIN = 1.5f; // 50% extra space
constexpr float TIME_WINDOW_PLOT = 10.0f;    // For the key quantities window
constexpr float TIME_PADDING_PLOT = 1.0f;    // 1 second padding

struct DrillingQuantityGraph {
    DrillingVariable variable;
    string label;
    bool show = false;
    bool auto_scale = true;
    bool initialized = false;
    float value_min = 0.0f;
    float value_max = 0.0f;
    float value_min_stamp = 0.0f;
    float value_max_stamp = 0.0f;

    // Calculate buffer size based on time window and FPS
    static constexpr size_t BUFFER_SIZE = static_cast<size_t>(TIME_WINDOW_PLOT * MAX_FPS * SAFETY_MARGIN);

    std::array<float, BUFFER_SIZE> times;
    std::array<float, BUFFER_SIZE> values;
    size_t current_index = 0;
    size_t data_size = 0;
    size_t plot_size = 0;
    size_t plot_start_idx = 0;

    DrillingQuantityGraph(DrillingVariable var, string y_label) : variable(var), label(y_label) {
        value_min = std::numeric_limits<float>::max();
        value_max = std::numeric_limits<float>::lowest();
    };

    void update_plot_limits(float current_time);
    void update_incremental_limits(float new_value, float sample_time);
};

struct PlotGroup {
    string title;
    string label;
    bool show;
    float scale_factor;
    vector<DrillingQuantityGraph> graphs;
    vector<bool> active;
    float group_min = std::numeric_limits<float>::max();
    float group_max = std::numeric_limits<float>::lowest();
    float prev_min = std::numeric_limits<float>::max();
    float prev_max = std::numeric_limits<float>::lowest();
    float min_window;

    // --- Added: control group-level smooth shrinking and padding ---
    static constexpr float group_padding = 0.10f; // 10% padding on both sides of current data range
    static constexpr float shrink_alpha = 0.25f;  // smoothing factor when shrinking (0..1], larger -> faster shrink

    PlotGroup(const string &plot_title, const string &y_label, float scale, float min_win,
              const vector<DrillingVariable> &vars, const vector<string> &labels)
        : title(plot_title), label(y_label), scale_factor(scale), show(false), min_window(min_win) {
        // Create graphs for each variable
        for (size_t i = 0; i < vars.size(); i++) {
            DrillingQuantityGraph graph(vars[i], labels[i]);
            graphs.emplace_back(graph);
            active.push_back(false);
        }
    }

    void update_group_limits();
};

struct OrbitalPlot {
    scalar S_plot;
    uint node;
    bool is_recording;
    vector<pair<scalar, scalar>> trail_points;
};

struct BitRockPlot {
    uint N_surface_points;

    static constexpr float PLOT_MARGIN = 0.1f;
    static constexpr float DECAY_RATE = 0.01f;

    std::vector<float> surface_x;
    std::vector<float> surface_y;

    float x_min, x_max, y_min, y_max;
    float deepest_cut = 0.0f;
    float max_depth = std::numeric_limits<float>::max();

    BitRockPlot() {
        x_min = 0.0f;
        x_max = 0.0f;
        y_min = 0.0f;
        y_max = 0.0f;
    }

    void init_surface_and_limits(const uint n_blades, const scalar initial_surface_depth, uint n_surface_points) {

        N_surface_points = n_surface_points;
        surface_x.resize(N_surface_points);
        surface_y.resize(N_surface_points);

        float d_theta = 2 * M_PI / n_blades / (N_surface_points - 1);
        for (size_t i = 0; i < N_surface_points; i++) {
            float x = i * d_theta;
            surface_x[i] = x;
            surface_y[i] = (float)initial_surface_depth;
        }
        x_min = 0.0f;
        x_max = 2 * M_PI / n_blades;
        y_min = (float)initial_surface_depth;
        y_max = (float)initial_surface_depth;
    }

    // Update surface_y from ArrayView<scalar> cut_depths
    void update_plot_and_limits(float s_blade, const ArrayView<scalar> &cut_depths);
};

struct FrequencyResponseData {
    std::vector<std::vector<float>> time_series; // Time-series data for each point
    std::vector<float> frequencies;              // Frequency axis
    std::vector<std::vector<float>> amplitudes;  // FFT amplitudes for each point
    size_t buffer_size;                          // Size of the circular buffer
    size_t current_index;                        // Current index in the buffer

    FrequencyResponseData(size_t num_points, size_t buffer_sizing)
        : time_series(num_points, std::vector<float>(buffer_sizing, 0.0f)),
          amplitudes(num_points, std::vector<float>(buffer_sizing / 2, 0.0f)), buffer_size(buffer_sizing),
          current_index(0) {}
};
