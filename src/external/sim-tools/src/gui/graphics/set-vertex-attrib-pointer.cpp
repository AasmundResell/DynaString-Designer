#include "sim-tools/gui/graphics/vertices.hpp"

namespace sito {

template <typename T> inline void set_pointer(uint count, uint offset, uint *next_attrib_index, bool instanced) {
    glVertexAttribPointer(*next_attrib_index, count, GL_FLOAT, GL_FALSE, sizeof(T), (void *)((size_t)offset));
    glEnableVertexAttribArray(*next_attrib_index);
    if (instanced) {
        glVertexAttribDivisor(*next_attrib_index, 1);
    }
    (*next_attrib_index)++;
}

template <> void set_vertex_attrib_pointers<glm::vec2>(uint *next_attrib_index, bool instanced) {
    set_pointer<glm::vec2>(2, 0, next_attrib_index, instanced);
}

template <> void set_vertex_attrib_pointers<glm::vec3>(uint *next_attrib_index, bool instanced) {
    set_pointer<glm::vec3>(3, 0, next_attrib_index, instanced);
}

template <> void set_vertex_attrib_pointers<glm::mat4>(uint *next_attrib_index, bool instanced) {
    static_assert(sizeof(glm::mat4) == 4 * sizeof(glm::vec4));
    for (uint i = 0; i < 4; i++) {
        set_pointer<glm::mat4>(4, i * sizeof(glm::vec4), next_attrib_index, instanced);
    }
}

template <> void set_vertex_attrib_pointers<VertexPos3Normal3>(uint *next_attrib_index, bool instanced) {
    set_pointer<VertexPos3Normal3>(3, offsetof(VertexPos3Normal3, pos), next_attrib_index, instanced);
    set_pointer<VertexPos3Normal3>(3, offsetof(VertexPos3Normal3, normal), next_attrib_index, instanced);
}

template <> void set_vertex_attrib_pointers<VertexPos2Color3>(uint *next_attrib_index, bool instanced) {
    set_pointer<VertexPos2Color3>(2, offsetof(VertexPos2Color3, pos), next_attrib_index, instanced);
    set_pointer<VertexPos2Color3>(3, offsetof(VertexPos2Color3, color), next_attrib_index, instanced);
}

template <> void set_vertex_attrib_pointers<VertexPos3Color3Normal3>(uint *next_attrib_index, bool instanced) {

    set_pointer<VertexPos3Color3Normal3>(3, offsetof(VertexPos3Color3Normal3, pos), next_attrib_index, instanced);
    set_pointer<VertexPos3Color3Normal3>(3, offsetof(VertexPos3Color3Normal3, color), next_attrib_index, instanced);
    set_pointer<VertexPos3Color3Normal3>(3, offsetof(VertexPos3Color3Normal3, normal), next_attrib_index, instanced);
}

template <> void set_vertex_attrib_pointers<VertexPos3Color4Normal3>(uint *next_attrib_index, bool instanced) {

    set_pointer<VertexPos3Color4Normal3>(3, offsetof(VertexPos3Color4Normal3, pos), next_attrib_index, instanced);
    set_pointer<VertexPos3Color4Normal3>(4, offsetof(VertexPos3Color4Normal3, color), next_attrib_index, instanced);
    set_pointer<VertexPos3Color4Normal3>(3, offsetof(VertexPos3Color4Normal3, normal), next_attrib_index, instanced);
}

} // namespace sito