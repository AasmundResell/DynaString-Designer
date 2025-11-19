#include "sim-tools/gui/graphics/graphics-mesh.hpp"

namespace sito {

void GraphicsMesh::upload_to_gpu() {
    assert(VAO == 0 && VBO == 0 && EBO == 0);
    assert(!indices.empty() && !vertices.empty());

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(uint), indices.data(), GL_STATIC_DRAW);

    uint next_attrib_pointer = 0;
    set_vertex_attrib_pointers<Vertex>(&next_attrib_pointer, false);

    glBindVertexArray(0);

    // Optional: Unbind buffers to avoid state leakage
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void GraphicsMesh::draw(Shader &shader) const {
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

} // namespace sito
