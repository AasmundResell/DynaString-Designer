#pragma once
#include "sim-tools/gui/includes-GL.hpp"
#include "sim-tools/utils.hpp"

namespace sito {

inline glm::vec3 get_color_from_colormap(float value, float lim_min, float lim_max, ColorMapType type,
                                         bool grey_outside_limits = true) {
    // Normalize the value between 0 and 1
    // assert(value >= min_value && value <= max_value);
    float normalized = (!is_close(lim_min, lim_max)) ? (value - lim_min) / (lim_max - lim_min) : 0.5f;
    assert(isfinite(normalized));
    // Check if the value is outside the limits
    if (normalized < 0.0f || normalized > 1.0f) {
        if (grey_outside_limits) {
            return glm::vec3(0.5f, 0.5f, 0.5f); // Return grey for out-of-range values
        } else {
            normalized = glm::clamp(normalized, 0.0f, 1.0f);
        }
    }

    switch (type) {
    case ColorMapType::RAINBOW:
        // Rainbow color map
        if (normalized < 0.25f) {
            return glm::vec3(0.0f, 4.0f * normalized, 1.0f);
        } else if (normalized < 0.5f) {
            return glm::vec3(0.0f, 1.0f, 1.0f + 4.0f * (0.25f - normalized));
        } else if (normalized < 0.75f) {
            return glm::vec3(4.0f * (normalized - 0.5f), 1.0f, 0.0f);
        } else {
            return glm::vec3(1.0f, 1.0f + 4.0f * (0.75f - normalized), 0.0f);
        }

        // Add more cases for different color maps here in the future

    default:
        assert(false);
        return glm::vec3(0.5f, 0.5f, 0.5f);
    }
}

} // namespace sito