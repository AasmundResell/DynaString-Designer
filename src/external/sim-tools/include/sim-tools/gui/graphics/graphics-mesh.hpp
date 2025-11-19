#pragma once
//#include "../../vendor/assimp/include/assimp/scene.h"
#include "sim-tools/gui/graphics/shader.hpp"
#include "sim-tools/gui/includes-GL.hpp"
#include "vertices.hpp"

namespace sito {

class GraphicsMesh {
  public:
    uint VAO, VBO, EBO;

    using Vertex = VertexPos3Color3Normal3;

    vector<Vertex> vertices;
    vector<uint> indices;

    // Mesh(const vector<Vertex> &vertices, const vector<uint> &indices);

    void upload_to_gpu();

    void draw(Shader &shader) const;
};

} // namespace sito
