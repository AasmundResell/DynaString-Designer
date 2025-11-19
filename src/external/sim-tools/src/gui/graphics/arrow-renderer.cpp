#include "sim-tools/gui/graphics/arrow-renderer.hpp"
#include "sim-tools/gui/graphics/graphics-model.hpp"

namespace sito {

void ArrowRenderer::init(uint num_arrows) {
    assert(vb_arrow.vertices.empty() && ib_arrow.indices.empty());
    vb_arrow = {};
    ib_arrow = {};
    shader = {};

    const path shader_dir = path(sim_tools_home_dir) / "resources" / "shaders";
    shader.create("phong-arrows-instanced.vs", "phong.fs", shader_dir);

    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);
    uint next_attrib_pointer = 0;

    /*Load model and copy it over to the vertex and index buffer*/
    GraphicsModel arrow_model;
    const path model_file = path(sim_tools_home_dir) / "resources" / "models" / "arrow.obj";
    arrow_model.load(model_file);
    assert(arrow_model.meshes.size() == 1);

    const uint vertex_count = arrow_model.meshes[0].vertices.size();
    vb_arrow.init(VAO, vertex_count, &next_attrib_pointer, false, GL_STATIC_DRAW);

    for (uint i = 0; i < vertex_count; i++) {
        const auto &vertex_model = arrow_model.meshes[0].vertices[i];
        vb_arrow.vertices[i] = {.pos = vertex_model.pos, .normal = vertex_model.normal};
    }

    vb_instance_models.init(VAO, num_arrows, &next_attrib_pointer, true, GL_DYNAMIC_DRAW);
    vb_instance_colors.init(VAO, num_arrows, &next_attrib_pointer, true, GL_DYNAMIC_DRAW);

    vb_arrow.upload_data();

    const uint index_count = arrow_model.meshes[0].indices.size();
    ib_arrow.init(VAO, index_count);
    ib_arrow.indices = arrow_model.meshes[0].indices;
    ib_arrow.upload_data(GL_STATIC_DRAW);

    glBindVertexArray(0);
}

void ArrowRenderer::upload_instance_models() {
    assert(vb_arrow.get_count() > 0);
    vb_instance_models.upload_data();
}

void ArrowRenderer::upload_instance_colors() {
    assert(vb_instance_colors.get_count() > 0);
    vb_instance_colors.upload_data();
}

void ArrowRenderer::draw(const UniformsCommon &uniforms_common) {
    glBindVertexArray(VAO);
    shader.use();
    shader.set_vec3("view_pos", uniforms_common.view_pos);
    shader.set_vec3("light_pos", uniforms_common.point_light.pos);
    shader.set_vec3("color_light", uniforms_common.point_light.color);

    shader.set_mat4("view", uniforms_common.view);
    shader.set_mat4("projection", uniforms_common.projection);

    glDrawElementsInstanced(GL_TRIANGLES, ib_arrow.count(), GL_UNSIGNED_INT, (void *)0, vb_instance_models.get_count());
    glBindVertexArray(0);
}

void ArrowRenderer::set_color_uniform(const glm::vec3 &color) {
    assert(vb_instance_colors.get_count() > 0);
    std::fill(vb_instance_colors.vertices.begin(), vb_instance_colors.vertices.end(), color);
    upload_instance_colors();
}

} // namespace sito