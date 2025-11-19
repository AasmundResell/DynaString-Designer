#include "plot-utils.hpp"

string SpatialGraph::get_variable_string() const {
    string variable_string = symbol;
    variable_string += "_" + string(directions_strings[(uint)direction]);
    return variable_string;
}

string SpatialGraph::get_y_label() const {
    string y_label = label;
    y_label += " " + unit;
    return y_label;
}

void SpatialGraph::reset_scale() {
    data_min = std::numeric_limits<float>::max();
    data_max = std::numeric_limits<float>::lowest();
    s_min = std::numeric_limits<float>::max();
    s_max = std::numeric_limits<float>::lowest();
}

void SpatialGraph::update_auto_scale(uint index) {
    // Adjust this value to control shrinking speed
    constexpr float decay_rate_graph = 0.99995f;
    float value = data[index];
    // Decay the current bounds
    data_min = data_min * decay_rate_graph + value * (1.0f - decay_rate_graph);
    data_max = data_max * decay_rate_graph + value * (1.0f - decay_rate_graph);

    // Still need immediate response to values outside current range
    if (value < data_min)
        data_min = value;
    if (value > data_max)
        data_max = value;

    // Add margins for display
    float range = max(data_max - data_min, minimal_window);
    y_min = data_min - (range * display_margin);
    y_max = data_max + (range * display_margin);
}

string MiscSpatialGraph::get_graph_label() const {
    return label + " " + unit;
}

string MiscSpatialGraph::get_colorbar_label() const {
    return colorbar_label + " " + unit;
}

void MiscSpatialGraph::reset_scale() {
    data_min = std::numeric_limits<float>::max();
    data_max = std::numeric_limits<float>::lowest();
    s_min = std::numeric_limits<float>::max();
    s_max = std::numeric_limits<float>::lowest();
}

string FluidSpatialGraph::get_graph_label() const {
    return label + " " + unit;
}

string FluidSpatialGraph::get_colorbar_label() const {
    return colorbar_label + " " + unit;
}

void FluidSpatialGraph::update_auto_scale(uint index) {
    // Adjust this value to control shrinking speed
    constexpr float decay_rate_graph = 0.99995f;
    float value = data[index];
    // Decay the current bounds
    data_min = data_min * decay_rate_graph + value * (1.0f - decay_rate_graph);
    data_max = data_max * decay_rate_graph + value * (1.0f - decay_rate_graph);

    // Still need immediate response to values outside current range
    if (value < data_min)
        data_min = value;
    if (value > data_max)
        data_max = value;

    // Add margins for display
    float range = max(data_max - data_min, minimal_window);
    y_min = data_min - (range * display_margin);
    y_max = data_max + (range * display_margin);
}

void FluidSpatialGraph::reset_scale() {
    data_min = std::numeric_limits<float>::max();
    data_max = std::numeric_limits<float>::lowest();
    s_min = std::numeric_limits<float>::max();
    s_max = std::numeric_limits<float>::lowest();
}
