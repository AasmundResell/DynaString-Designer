#pragma once
#include "sim-tools/gui/includes-GL.hpp"
#include "vertices.hpp"

namespace sito {
template <typename VertexType> struct VertexBuffer {
    uint VBO;
    uint VAO;
    vector<VertexType> vertices;

    uint get_count() const { return vertices.size(); }

    void init(uint VAO_, uint count, uint *next_attrib_pointer, bool instanced, GLenum usage) {
        assert(vertices.empty());
        assert(VAO == 0 && VBO == 0); // call default constructor first
        VAO = VAO_;

        vertices.resize(count);

        glBindVertexArray(VAO);
        glGenBuffers(1, &VBO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);

        assert(usage == GL_DYNAMIC_DRAW || usage == GL_STATIC_DRAW);
        glBufferData(GL_ARRAY_BUFFER, sizeof(VertexType) * vertices.size(), nullptr, usage);
        set_vertex_attrib_pointers<VertexType>(next_attrib_pointer, instanced);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    void upload_data(uint begin = 0, int end = -1) {
        if (end == -1) {
            end = vertices.size();
        }
        glBindVertexArray(VAO);
        assert(end >= 0 && end <= vertices.size() && begin < end);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, sizeof(VertexType) * begin, sizeof(VertexType) * (end - begin),
                        &vertices[begin]);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }
};

} // namespace sito