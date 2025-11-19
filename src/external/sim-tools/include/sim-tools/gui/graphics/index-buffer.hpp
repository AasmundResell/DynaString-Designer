#include "sim-tools/gui/includes-GL.hpp"

namespace sito {
struct IndexBuffer {
    uint EBO;
    uint VAO;
    vector<uint> indices;

    uint count() const { return indices.size(); }

    void init(uint VAO_, uint count) {
        assert(indices.empty());
        assert(VAO == 0 && EBO == 0); // call default constructor first
        VAO = VAO_;

        indices.resize(count);

        glBindVertexArray(VAO);
        glGenBuffers(1, &EBO);
        glBindVertexArray(0);
    }
    void upload_data(GLenum usage) {
        assert(!indices.empty());
        glBindVertexArray(VAO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint) * indices.size(), indices.data(), usage);
        glBindVertexArray(0);
    }
};

} // namespace sito