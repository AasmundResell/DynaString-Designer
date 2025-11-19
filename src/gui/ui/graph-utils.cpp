#include "graph-utils.hpp"

void HolePlotData::allocate(const Hole hole, const byte *buf) {

    N_hole = hole.N_hole;
    s.resize(N_hole);
    a.resize(N_hole);
    b.resize(N_hole);
    kappa_y.resize(N_hole);
    kappa_z.resize(N_hole);
    offset_y.resize(N_hole);
    offset_z.resize(N_hole);
    alpha.resize(N_hole);
    const ArrayView<scalar> s_hole = hole.get_field<HoleField::s>(buf);
    const ArrayView<scalar> r_a = hole.get_field<HoleField::a>(buf);
    const ArrayView<scalar> r_b = hole.get_field<HoleField::b>(buf);
    const ArrayView<Vec2> kappas = hole.get_field<HoleField::kappa>(buf);
    const ArrayView<Vec2> offsets = hole.get_field<HoleField::co>(buf);
    const ArrayView<scalar> alphas = hole.get_field<HoleField::alpha>(buf);

    for (uint i = 0; i < N_hole; i++) {
        s[i] = s_hole[i + 2];
        a[i] = r_a[i + 2];
        b[i] = r_b[i + 2];
        kappa_y[i] = kappas[i + 2].x();
        kappa_z[i] = kappas[i + 2].y();
        offset_y[i] = offsets[i + 2].x();
        offset_z[i] = offsets[i + 2].y();
        alpha[i] = alphas[i + 2] * (180.0f / M_PI); // Convert to degrees
    }

    scalar y_min =
        std::min(*std::min_element(kappa_y.begin(), kappa_y.end()), *std::min_element(kappa_z.begin(), kappa_z.end()));
    scalar y_max =
        std::max(*std::max_element(kappa_y.begin(), kappa_y.end()), *std::max_element(kappa_z.begin(), kappa_z.end()));
    scalar padding = 0.1f * (y_max - y_min);
    y_min_curvature = y_min - padding;
    y_max_curvature = y_max + padding;

    y_min = std::min(*std::min_element(offset_y.begin(), offset_y.end()),
                     *std::min_element(offset_z.begin(), offset_z.end()));
    y_max = std::max(*std::max_element(offset_y.begin(), offset_y.end()),
                     *std::max_element(offset_z.begin(), offset_z.end()));

    padding = 0.1f * (y_max - y_min);
    y_min_offsets = y_min - padding;
    y_max_offsets = y_max + padding;

    y_min = std::min(*std::min_element(a.begin(), a.end()), *std::min_element(b.begin(), b.end()));
    y_max = std::max(*std::max_element(a.begin(), a.end()), *std::max_element(b.begin(), b.end()));
    padding = 0.1f * (y_max - y_min);
    y_min_radius = y_min - padding;
    y_max_radius = y_max + padding;

    y_min = *std::min_element(alpha.begin(), alpha.end());
    y_max = *std::max_element(alpha.begin(), alpha.end());
    padding = 0.1f * (y_max - y_min);
    y_min_rotation = y_min - padding;
    y_max_rotation = y_max + padding;
}

void PlotGroup::update_group_limits() {
    // Compute raw group min/max from visible graph bounds
    float raw_min = std::numeric_limits<float>::max();
    float raw_max = std::numeric_limits<float>::lowest();
    bool any_shown = false;

    for (uint i = 0; i < graphs.size(); ++i) {
        DrillingQuantityGraph &graph = graphs[i];
        if (!graph.show)
            continue;
        raw_min = std::min(raw_min, graph.value_min);
        raw_max = std::max(raw_max, graph.value_max);
        any_shown = true;
    }

    if (!any_shown) {
        return;
    }

    float range = raw_max - raw_min;
    // center of raw data
    float center = 0.5f * (raw_max + raw_min);
    // ensure we never shrink below min_window (use actual range if larger)
    float effective_range = (range > 0.0f) ? range : 0.0f;
    effective_range = std::max(effective_range, min_window);
    float pad = group_padding * effective_range;
    float half = 0.5f * effective_range;
    float target_min = center - half - pad;
    float target_max = center + half + pad;

    // Initialize on first use
    if (!(group_min <= group_max)) {
        group_min = target_min;
        group_max = target_max;
        prev_min = group_min;
        prev_max = group_max;
        return;
    }

    // Smooth both expansion and contraction with a single smoothing alpha
    const float alpha = std::clamp(shrink_alpha, 0.0f, 1.0f);
    group_min = group_min * (1.0f - alpha) + target_min * alpha;
    group_max = group_max * (1.0f - alpha) + target_max * alpha;

    prev_min = group_min;
    prev_max = group_max;
}

void BitRockPlot::update_plot_and_limits(float s_blade, const ArrayView<scalar> &cut_depths) {

    for (size_t i = 0; i < N_surface_points; i++) {
        surface_y[i] = (float)cut_depths[i];
    }

    // Find min/max values in the cut_depths array
    float surface_min = std::numeric_limits<float>::max();
    float surface_max = std::numeric_limits<float>::lowest();

    for (size_t i = 0; i < N_surface_points - 1; i++) {
        float y_val = (float)cut_depths[i];
        surface_y[i] = y_val;
        surface_min = min(surface_min, y_val);
        surface_max = max(surface_max, y_val);
    }

    // Apply periodic boundary condition
    surface_y[N_surface_points - 1] = surface_y[0];

    // Calculate the combined range including both surface and bit position
    float data_min = min(surface_min, s_blade);
    float data_max = max(surface_max, s_blade);

    // Calculate the range and ensure minimum window size
    float data_range = data_max - data_min;
    float min_range = 0.1f; // Minimum window size in meters
    if (data_range < min_range) {
        float center = (data_min + data_max) * 0.5f;
        data_min = center - min_range * 0.5f;
        data_max = center + min_range * 0.5f;
        data_range = min_range;
    }

    // Apply small centered padding around the actual data range
    float padding = PLOT_MARGIN * data_range;
    float target_min = data_min - padding;
    float target_max = data_max + padding;

    // Apply decay to smooth transitions when shrinking the view
    // When expanding (target bounds exceed current bounds), update immediately
    // When shrinking (target bounds are within current bounds), apply decay
    if (target_min < y_min) {
        y_min = target_min; // Expand immediately downward
    } else {
        y_min = y_min * DECAY_RATE + target_min * (1.0f - DECAY_RATE); // Decay toward target
    }

    if (target_max > y_max) {
        y_max = target_max; // Expand immediately upward
    } else {
        y_max = y_max * DECAY_RATE + target_max * (1.0f - DECAY_RATE); // Decay toward target
    }
}

void DrillingQuantityGraph::update_plot_limits(float current_time) {
    // Default
    plot_start_idx = 0;
    plot_size = 0;

    if (data_size == 0)
        return;

    // newest sample index (current_index points to next write position)
    size_t newest = (current_index + BUFFER_SIZE - 1) % BUFFER_SIZE;
    size_t j = newest;
    size_t count = 0;

    // Count how many most-recent samples are inside the time window
    for (size_t k = 0; k < data_size; ++k) {
        float t = times[j];
        if ((current_time - t) <= TIME_WINDOW_PLOT) {
            ++count;
            // move to previous sample
            j = (j == 0) ? BUFFER_SIZE - 1 : j - 1;
        } else {
            break;
        }
    }

    if (count > 0) {
        plot_size = count;
        plot_start_idx = (j + 1) % BUFFER_SIZE; // first index inside the window
    } else {
        // No samples lie inside the time window: show nothing
        plot_size = 0;
        plot_start_idx = 0;
    }
}

void DrillingQuantityGraph::update_incremental_limits(float new_value, float sample_time) {
    if (!auto_scale) // do nothing if not autoscaling
        return;

    // Fast expand: if new sample expands bounds, update and set stamp
    bool expanded = false;
    if (new_value < value_min - SMALL_SCALAR) {
        value_min = new_value;
        value_min_stamp = sample_time;
        expanded = true;
    }
    if (new_value > value_max + SMALL_SCALAR) {
        value_max = new_value;
        value_max_stamp = sample_time;
        expanded = true;
    }
    if (expanded) {
        return; // no shrink work needed this call
    }

    // If extrema are older than the visible window, recompute from visible samples.
    const float now = sample_time;
    bool recompute_min = (now - value_min_stamp) > TIME_WINDOW_PLOT;
    bool recompute_max = (now - value_max_stamp) > TIME_WINDOW_PLOT;

    if (!recompute_min && !recompute_max) {
        return; // nothing to do
    }

    // Determine window to scan: prefer the visible plot window, else last data_size samples
    size_t start_idx;
    size_t scan_count;
    if (plot_size > 0) {
        start_idx = plot_start_idx;
        scan_count = plot_size;
    } else {
        scan_count = data_size;
        start_idx = (current_index + BUFFER_SIZE - data_size) % BUFFER_SIZE;
    }

    if (scan_count == 0) {
        return;
    }

    float wmin = std::numeric_limits<float>::max();
    float wmax = std::numeric_limits<float>::lowest();
    size_t idx_of_min = start_idx;
    size_t idx_of_max = start_idx;

    for (size_t k = 0; k < scan_count; ++k) {
        size_t idx = (start_idx + k) % BUFFER_SIZE;
        float v = values[idx];
        if (v < wmin) {
            wmin = v;
            idx_of_min = idx;
        }
        if (v > wmax) {
            wmax = v;
            idx_of_max = idx;
        }
    }

    // Update min/max and stamps only for the ones we needed to recompute
    if (recompute_min) {
        value_min = wmin;
        value_min_stamp = times[idx_of_min];
    }
    if (recompute_max) {
        value_max = wmax;
        value_max_stamp = times[idx_of_max];
    }

    // Note: padding is applied elsewhere (UI_Manager) so we keep raw bounds here.
}