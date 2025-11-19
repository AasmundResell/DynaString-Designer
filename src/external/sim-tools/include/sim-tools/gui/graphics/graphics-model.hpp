#pragma once
#include "assimp/Importer.hpp"
#include "assimp/postprocess.h"
#include "assimp/scene.h"
#include "graphics-mesh.hpp"
#include "sim-tools/gui/includes-GL.hpp"

namespace sito {
class GraphicsModel {
  public:
    using Vertex = GraphicsMesh::Vertex;
    vector<GraphicsMesh> meshes;

    void draw(Shader &shader) const;

    void load(const path &path);
    void merge_meshes();

    void upload_to_gpu();

  private:
    void process_node(const aiNode *node, const aiScene *scene);
    GraphicsMesh process_mesh(const aiMesh *mesh, const aiScene *scene);
};

class ModelManager {
  public:
    static GraphicsModel &get_model(const std::filesystem::path &path);
    static void clear_cache();

  private:
    static std::unordered_map<std::string, std::unique_ptr<GraphicsModel>> model_cache;
};

} // namespace sito
