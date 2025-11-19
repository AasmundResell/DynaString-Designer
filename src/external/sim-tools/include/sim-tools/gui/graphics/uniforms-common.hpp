#pragma once
#include "lighting.hpp"
#include "sim-tools/gui/includes-GL.hpp"

namespace sito {
// Grouping uniforms making it easier to pass them around
struct UniformsCommon {
    glm::mat4 projection;
    glm::mat4 view;
    glm::vec3 view_pos;
    glm::vec3 front, right, up;
    PointLight point_light; // Extend this when we implement more than a single light

    glm::vec3 ambient_color;
};
} // namespace sito