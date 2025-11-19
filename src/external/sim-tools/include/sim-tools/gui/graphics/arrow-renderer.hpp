#pragma once
#include "index-buffer.hpp"
#include "shader.hpp"
#include "uniforms-common.hpp"
#include "vertex-buffer.hpp"

namespace sito {
class ArrowRenderer {
    uint VAO;
    Shader shader;

  public:
    using Vertex = sito::VertexPos3Normal3;
    VertexBuffer<Vertex> vb_arrow;
    VertexBuffer<glm::mat4> vb_instance_models;
    VertexBuffer<glm::vec3> vb_instance_colors;

    IndexBuffer ib_arrow;

    void init(uint num_arrows);

    void upload_instance_models();
    void upload_instance_colors();

    void draw(const UniformsCommon &uniforms_common);

    /*Sets unform color and uploads to gpu*/
    void set_color_uniform(const glm::vec3 &color);

    uint get_count() const {
        assert(vb_instance_models.get_count() > 0);
        return vb_instance_models.get_count();
    }

    void set_arrow_color(uint i, const glm::vec3 &color) {
        assert(i < vb_instance_models.get_count());
        vb_instance_colors.vertices[i] = color;
    }

    void set_arrow(uint i, const glm::vec3 &pos_base, const glm::vec3 &dir, float scale_thickness) {
        assert(i < vb_instance_models.get_count());

        glm::mat4 model = glm::mat4(1);
        model = glm::translate(model, pos_base);

        // We assume that the model arrow points in the direction e1 = [1 0 0]
        const float arrow_length = glm::length(dir);
        const float angle_rad = acos(dir[0] / arrow_length); // Cosinus sentence applied to vectors dir and e1
        const glm::vec3 axis = glm::normalize(glm::cross(glm::vec3(1, 0, 0), dir)); // Rotation axis

        model = glm::rotate(model, angle_rad, axis);

        glm::vec3 scale = glm::vec3(arrow_length, scale_thickness, scale_thickness);
        model = glm::scale(model, scale);

        vb_instance_models.vertices[i] = model;
    }
};
} // namespace sito