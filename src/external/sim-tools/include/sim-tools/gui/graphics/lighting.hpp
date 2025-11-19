#pragma once
#include "sim-tools/gui/includes-GL.hpp"
namespace sito {
struct PointLight {
    glm::vec3 pos;
    glm::vec3 color;
    float intensity;
};
} // namespace sito