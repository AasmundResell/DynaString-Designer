#pragma once
#include "sim-tools/gui/includes-GL.hpp"

namespace sito {

/*================================================================================
 Define various Vertex types here
==================================================================================*/

struct VertexPos3Normal3 {
    glm::vec3 pos;
    glm::vec3 normal;
};

struct VertexPos3Color3Normal3 {
    glm::vec3 pos;
    glm::vec3 color;
    glm::vec3 normal;
};

struct VertexPos3Color4Normal3 {
    glm::vec3 pos;
    glm::vec4 color;
    glm::vec3 normal;
};

struct VertexPos2Color3 {
    glm::vec2 pos;
    glm::vec3 color;
};

template <typename VertexType> void set_vertex_attrib_pointers(uint *next_attrib_index, bool instanced);

} // namespace sito