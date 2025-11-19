#include "sim-tools/gui/graphics/graphics-model.hpp"

namespace sito {

void GraphicsModel::draw(Shader &shader) const {
    assert(!meshes.empty()); // might be removed if it makes sense
    for (const GraphicsMesh &mesh : meshes)
        mesh.draw(shader);
}

void GraphicsModel::load(const path &path) {
    assert(meshes.size() == 0); // This should be zero when called

    Assimp::Importer importer;
    const aiScene *scene =
        importer.ReadFile(path.string(), aiProcess_Triangulate | aiProcess_GenNormals | aiProcess_FlipUVs);

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        THROW_RUNTIME_ERROR("Error loading assimp scene with path " + path.string() +
                            ", error message: " + importer.GetErrorString());
    }
    process_node(scene->mRootNode, scene);
};
void GraphicsModel::process_node(const aiNode *node, const aiScene *scene) {
    /*Process all meshes in this nodes*/
    meshes.reserve(node->mNumMeshes);
    for (uint i = 0; i < node->mNumMeshes; i++) {
        const uint meshID = node->mMeshes[i];
        const aiMesh *mesh = scene->mMeshes[meshID];
        meshes.emplace_back(process_mesh(mesh, scene));
    }
    /*Recursively process all child nodes*/
    for (uint i = 0; i < node->mNumChildren; i++) {
        const aiNode *child_node = node->mChildren[i];
        process_node(child_node, scene);
    }
}
GraphicsMesh GraphicsModel::process_mesh(const aiMesh *mesh, const aiScene *scene) {
    vector<Vertex> vertices;
    vertices.reserve(mesh->mNumVertices);
    vector<uint> indices;

    aiMaterial *material = scene->mMaterials[mesh->mMaterialIndex];
    aiColor3D diffuse_color(0.5f, 0.5f, 0.5f);                           // Default to grey
    material->Get(AI_MATKEY_COLOR_DIFFUSE, diffuse_color) == AI_SUCCESS; // If material is defined, get that color

    /*A rotation matrix rotating 90 deg about x. This ensures that geomery imported from
    Blender isn't flipped. Keep in mind that the glm constructor may be confusing,
    e1, e2, e3 are here the three columns of the matrix*/

    // clang-format off
  const glm::mat3 R(glm::vec3(1, 0, 0), //e1
                    glm::vec3(0, 0, 1), //e2
                    glm::vec3(0,-1, 0)); //e3
    // clang-format on

    for (uint i = 0; i < mesh->mNumVertices; i++) {
        Vertex vertex;
        vertex.pos.x = mesh->mVertices[i].x;
        vertex.pos.y = mesh->mVertices[i].y;
        vertex.pos.z = mesh->mVertices[i].z;
        vertex.pos = R * vertex.pos;

        if (mesh->HasNormals()) {
            vertex.normal.x = mesh->mNormals[i].x;
            vertex.normal.y = mesh->mNormals[i].y;
            vertex.normal.z = mesh->mNormals[i].z;
            vertex.normal = R * vertex.normal;
        }
        if (mesh->HasVertexColors(0)) {
            // Vertex colors takes precedence over material color
            vertex.color.r = mesh->mColors[0][i].r;
            vertex.color.g = mesh->mColors[0][i].g;
            vertex.color.b = mesh->mColors[0][i].b;
        } else {
            vertex.color = {diffuse_color.r, diffuse_color.g, diffuse_color.b};
        }
        vertices.push_back(vertex);
    }
    for (uint i = 0; i < mesh->mNumFaces; i++) {
        const aiFace face = mesh->mFaces[i];
        for (uint j = 0; j < face.mNumIndices; j++) {
            indices.push_back(face.mIndices[j]);
        }
    }
    return GraphicsMesh{.vertices = vertices, .indices = indices};
}

void GraphicsModel::upload_to_gpu() {
    for (GraphicsMesh &mesh : meshes) {
        mesh.upload_to_gpu();
    }
}

void GraphicsModel::merge_meshes() {
    // Merge all meshes into one mesh
    vector<Vertex> vertices;
    vector<uint> indices;
    uint num_vertices_tot = 0;
    uint num_indices_tot = 0;
    for (const GraphicsMesh &mesh : meshes) {
        num_vertices_tot += mesh.vertices.size();
        num_indices_tot += mesh.indices.size();
    }
    vertices.reserve(num_vertices_tot);
    indices.reserve(num_indices_tot);

    for (const GraphicsMesh &mesh : meshes) {
        const uint offset = vertices.size();
        for (const Vertex &vertex : mesh.vertices) {
            vertices.push_back(vertex);
        }
        for (const uint index : mesh.indices) {
            indices.push_back(index + offset);
        }
    }
    meshes.clear();
    meshes.push_back(GraphicsMesh{.vertices = vertices, .indices = indices});
}

std::unordered_map<std::string, std::unique_ptr<GraphicsModel>> ModelManager::model_cache;

GraphicsModel &ModelManager::get_model(const std::filesystem::path &path) {
    const std::string path_str = path.string();
    auto it = model_cache.find(path_str);

    if (it == model_cache.end()) {
        // Model not in cache, load it
        std::unique_ptr<GraphicsModel> model = std::make_unique<GraphicsModel>();
        model->load(path);
        it = model_cache.emplace(path_str, std::move(model)).first;
    }

    return *it->second;
}

void ModelManager::clear_cache() {
    model_cache.clear();
}

} // namespace sito
