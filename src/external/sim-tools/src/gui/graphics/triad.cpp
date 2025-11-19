#include "sim-tools/gui/graphics/triad.hpp"
#include "sim-tools/gui/graphics/uniforms-common.hpp"
#include "sim-tools/utils.hpp"

namespace sito {

void Triad::load() {
    assert(triad_model.meshes.empty());
    const path model_file = path(sim_tools_home_dir) / "resources" / "models" / "triad.obj";

    // triad_model.load(model_file);
    triad_model = ModelManager::get_model(model_file);
    triad_model.merge_meshes();
    triad_model.upload_to_gpu();
}

float Triad::find_L_char_triad_model() const {

    // JUST SETTING THE ASSUMED VALUE
    const float L_char_triad_model = 1.2;

    assert(L_char_triad_model > 1000 * SMALL_FLOAT);
    return L_char_triad_model;
}

void Triad::draw(Shader &shader) const {
    assert(!triad_model.meshes.empty()); //
    // Clear depth buffer to ignore previously rendered geometry
    glClear(GL_DEPTH_BUFFER_BIT);

    triad_model.draw(shader);
}

void TriadOrigin::create(float L_char_triad_world) {
    triad.load();
    model_matrix = glm::mat4(1.0f);
    const float L_char_triad = triad.find_L_char_triad_model();
    const float scale_factor = L_char_triad_world / L_char_triad;
    model_matrix = glm::scale(model_matrix, glm::vec3(scale_factor));
}

void TriadOrigin::draw(Shader &shader) const {
    assert(glGetUniformLocation(shader.ID, "model") != -1); // Check that the shader used has "model" unform defined
    shader.use();
    shader.set_mat4("model", model_matrix);
    triad.draw(shader);
}

void TriadScreen::draw(Shader &shader, TextRenderer &text_renderer, const UniformsCommon &uniforms_common,
                       float screen_width, float screen_height) {

    constexpr float scale_factor = 0.03f;

    float aspect = screen_width / screen_height;
    float right = aspect;
    float left = 0.0f;
    constexpr float near_plane = 0.0f;
    constexpr float far_plane = 3.0f;
    constexpr float bottom = 0.0f;
    constexpr float top = 1.0f;
    constexpr float edge_dist = 0.12f;

    const glm::vec3 &camera_pos = uniforms_common.view_pos;
    const glm::vec3 &cam_e1 = uniforms_common.front;
    const glm::vec3 &cam_e2 = uniforms_common.right;
    const glm::vec3 &cam_e3 = uniforms_common.up;

    float h_ratio = std::max(0.01f, screen_height / sito::VIRTUAL_HEIGHT);
    float fov_y = 2.0f * std::atan(h_ratio * std::tan(glm::radians(60.0f) * 0.5f));

    float d = 0.5f; // distance forward along camera front
    float f = 1.0f / std::tan(fov_y * 0.5f);

    constexpr float TRIAD_OFFSET_X_PX = 120.0f; // distance from left edge
    constexpr float TRIAD_OFFSET_Y_PX =
        120.0f; // distance from bottom edge
                // Hide triad if requested anchor exceeds half the screen in either direction
    if (TRIAD_OFFSET_X_PX > 0.5f * screen_width || TRIAD_OFFSET_Y_PX > 0.5f * screen_height) {
        return;
    }
    // Convert pixel offsets to NDC anchors ( -1 .. +1 )
    const float anchor_x_ndc = -1.0f + 2.0f * (TRIAD_OFFSET_X_PX / screen_width);
    const float anchor_y_ndc = -1.0f + 2.0f * (TRIAD_OFFSET_Y_PX / screen_height);
    float Xv = anchor_x_ndc * d * aspect / f;
    float Yv = anchor_y_ndc * d / f;

    // Convert from camera (view) space to world space via camera basis
    glm::vec3 trans = camera_pos + d * cam_e1 + Xv * cam_e2 + Yv * cam_e3;
    fov_y = std::clamp(fov_y, glm::radians(5.0f), glm::radians(120.0f));
    glm::mat4 projection = glm::perspective(fov_y, aspect, near_plane, far_plane);

    glm::mat4 model = glm::mat4(1);
    model = glm::translate(model, trans);
    model = glm::scale(model, glm::vec3(scale_factor));

    if (label_mode == TriadLabelMode::TVD_NE) {
        // clang-format off
    glm::mat4 rot = glm::mat4(
        0, 0, -1, 0,  // X column (E)
        0, 1,  0, 0,  // Y column (N)
        1, 0,  0, 0,  // Z column (TVD)
        0, 0,  0, 1
    );
        // clang-format on
        model = model * rot;
    }
    shader.set_mat4("projection", projection);
    shader.set_mat4("model", model);
    triad.draw(shader);

    for (uint dim = 0; dim < 3; dim++) {
        glm::vec3 model_space_pos(0.0f);
        constexpr scalar factor = 1.5f;
        model_space_pos[dim] = factor * 1.0f;
        glm::vec4 letter_pos_clip_space = projection * uniforms_common.view * model * glm::vec4(model_space_pos, 1.0f);
        glm::vec3 ndc = glm::vec3(letter_pos_clip_space) / letter_pos_clip_space.w;

        const float x_virtual = 0.5f * (ndc.x + 1.0f) * screen_width;
        const float y_virtual = 0.5f * (ndc.y + 1.0f) * screen_height;
        text_renderer.draw(labels[dim], x_virtual, y_virtual, 0.8f, screen_width, screen_height);
    }
};

} // namespace sito