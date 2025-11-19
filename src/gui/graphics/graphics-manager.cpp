#include "graphics-manager.hpp"
#include "gui/frame-data.hpp"
#include "gui/includes-imgui.hpp"
#include "misc/includes.hpp"

GraphicsManager::GraphicsManager(const Config &config, const ConfigGraphics &config_graphics, const Hole &hole,
                                 const Pipe &pipe, const PipeRenderComponents &pipe_render_components,
                                 const vector<PipeComponent> &pipe_assembly, const byte *buf) {

    uniforms_common.point_light = {
        .pos = {4.0f, 5.0f, 0.0f},
        .color = {1.0f, 1.0f, 1.0f},
        .intensity = 1.0f,
    };

    uniforms_common.ambient_color = {0.2f, 0.2f, 0.2f};
    float fov = 60.0f; // Field of view angle
    glm::mat4 projection = glm::perspective(glm::radians(fov), (float)frame_data.width / (float)frame_data.height,
                                            frame_data.near_plane, frame_data.far_plane);

    {
        const path shader_dir = path(gui_dir) / "graphics" / "shaders";
        shader_pipe = {};
        shader_pipe.create("pipe-shader.vs", "pipe-shader.fs", shader_dir);
        shader_pipe.use();
        shader_pipe.set_vec3("light_color", uniforms_common.point_light.color);
        shader_pipe.set_vec3("ambient_color", uniforms_common.ambient_color);
        float specular_strength = 0.8f; // Shininess (between 0 and 1)
        shader_pipe.set_float("specular_strength", specular_strength);
        shader_pipe.set_mat4("projection", projection);
    }

    {
        const path shader_dir = path(gui_dir) / "graphics" / "shaders";
        shader_hole = {};
        shader_hole.create("hole-shader.vs", "hole-shader.fs", shader_dir);
        shader_hole.use();
        shader_hole.set_vec3("light_color", uniforms_common.point_light.color);
        shader_hole.set_vec3("ambient_color", uniforms_common.ambient_color);
        float specular_strength = 0.1f; // Shininess (between 0 and 1)
        shader_hole.set_float("specular_strength", specular_strength);
        shader_hole.set_mat4("projection", projection);
    }

    {
        const path shader_sim_tools_dir = path(sim_tools_home_dir) / "resources/shaders";
        shader_default = {};
        shader_default.create("phong.vs", "phong.fs", shader_sim_tools_dir);
    }

    {
        const float L_char_triad_world = 0.5;
        hole_renderer = make_unique<HoleRenderer>(hole, buf, L_char_triad_world);
    }

    pipe_renderer = make_unique<PipeRenderer>(config, pipe, pipe_render_components, pipe_assembly, buf);

    {
        const float L_char_triad_world = 1.0; // 1 meter
        triad_origin.create(L_char_triad_world);
        triad_screen.create(sito::TriadLabelMode::TVD_NE);
    }

    camera = {};
    setup_camera();
    create_multisampled_frame_buffer_on_window_resize();

    colorbar = {};
    colorbar.create(config_graphics.color_map_type, sito::VIRTUAL_WIDTH, sito::VIRTUAL_HEIGHT);

    text_renderer = TextRenderer{}; // just setting default values for easier debugging.
    text_renderer.create();

    drill_bit_model.load("../resources/models/tricone-oil-drill-bit.obj");
    drill_bit_model.merge_meshes();
    d_drill_bit =
        pipe_assembly.back().Do; // Using the actual drill bit diameter looks weird with current drill bit 3D model

    assert(drill_bit_model.meshes.size() == 1); // This should be true

    drill_bit_model.upload_to_gpu();
}

void GraphicsManager::create_multisampled_frame_buffer_on_window_resize() { // CHANGED
    if (framebuffer != 0)
        return; // allocate once only
    const int w = sito::VIRTUAL_WIDTH;
    const int h = sito::VIRTUAL_HEIGHT;
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    glGenTextures(1, &texture_colorbuffer_multi_sampled);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, texture_colorbuffer_multi_sampled);
    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, 4, GL_RGB, w, h, GL_TRUE);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE,
                           texture_colorbuffer_multi_sampled, 0);
    glGenRenderbuffers(1, &rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, rbo);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, 4, GL_DEPTH24_STENCIL8, w, h);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);
    glGenFramebuffers(1, &intermediate_FBO);
    glBindFramebuffer(GL_FRAMEBUFFER, intermediate_FBO);
    glGenTextures(1, &texture_colorbuffer);
    glBindTexture(GL_TEXTURE_2D, texture_colorbuffer);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture_colorbuffer, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void GraphicsManager::render(GLFWwindow *window, Config &config, ConfigGraphics &config_graphics,
                             const ConfigDynamic &conf_dyn, const curvlin::PipeSolver *solver_curvlin, const Hole &hole,
                             const Pipe &pipe, const vector<PipeComponent> &pipe_assembly, const vector<Vec3> &x_pipe,
                             const vector<Quaternion> &q_pipe, const array<MiscSpatialGraph, N_MG> misc_spatial_graphs,
                             const array<FluidSpatialGraph, N_FG> fluid_spatial_graphs, const byte *buf) {
    const uint top_node = config.top_node;
    ImVec2 viewport_panel_size = ImGui::GetContentRegionAvail();

    frame_data.height = static_cast<uint>(viewport_panel_size.y);
    frame_data.width = static_cast<uint>(viewport_panel_size.x);

    check_if_mouse_is_over_graphics_window(viewport_panel_size);

    if (frame_data.mouse_is_over_graphics_window) {
        sito::update_camera(frame_data.fps_mode, frame_data.delta_time, &frame_data.delta_cursor_x_frame,
                            &frame_data.delta_cursor_y_frame, &frame_data.scroll_y_offset, frame_data.is_pressed,
                            &camera, config_graphics.radial_scale);
    }

    // camera.set_viewport_size(frame_data.width, frame_data.height); // should this be viewport panel size or global
    //* size?

    // Bind the multisampled framebuffer and set the viewport
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    // glViewport(0, 0, viewport_panel_size.x, viewport_panel_size.y);
    glViewport(0, 0, sito::VIRTUAL_WIDTH, sito::VIRTUAL_HEIGHT);

    // Background color
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Enable depth testing
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    // Clamp to avoid domain issues if window gets extremely small
    float h_ratio = std::max(0.01f, viewport_panel_size.y / sito::VIRTUAL_HEIGHT);
    float fov_y = 2.0f * std::atan(h_ratio * std::tan(glm::radians(60.0f) * 0.5f));
    // Optional: clamp min/max fov_y to sane limits
    fov_y = std::clamp(fov_y, glm::radians(5.0f), glm::radians(120.0f));

    const float aspect = viewport_panel_size.x / viewport_panel_size.y;
    uniforms_common.projection = glm::perspective(fov_y, aspect, frame_data.near_plane, frame_data.far_plane);

    // Setuniforms used throughout the rendering
    uniforms_common.point_light.pos = camera.position; // light follows camera
    uniforms_common.view_pos = camera.position;
    // uniforms_common.projection =
    //     glm::perspective(glm::radians(60.0f), static_cast<float>(viewport_panel_size.x / viewport_panel_size.y),
    //                      frame_data.near_plane, frame_data.far_plane);
    uniforms_common.view = camera.get_view_matrix();
    uniforms_common.right = camera.right;
    uniforms_common.up = camera.up;
    uniforms_common.front = camera.front;

    { // Pipe rendering
        shader_pipe.use();
        shader_pipe.set_mat4("view", uniforms_common.view);
        shader_pipe.set_mat4("model", glm::mat4(1.0f));
        shader_pipe.set_mat4("projection", uniforms_common.projection);
        shader_pipe.set_vec3("view_pos", uniforms_common.view_pos);

        shader_pipe.set_vec3("light_dir", uniforms_common.front);
        shader_pipe.set_vec3("light_color", uniforms_common.point_light.color);
        shader_pipe.set_vec3("ambient_color", uniforms_common.ambient_color);
        pipe_renderer->draw(config, config_graphics, solver_curvlin, pipe, hole, pipe_assembly, x_pipe, q_pipe,
                            misc_spatial_graphs, fluid_spatial_graphs, buf);
    }

    { // Hole rendering
        shader_hole.use();
        shader_hole.set_mat4("view", uniforms_common.view);
        shader_hole.set_mat4("model", glm::mat4(1.0f));
        shader_hole.set_mat4("projection", uniforms_common.projection);
        shader_hole.set_vec3("view_pos", uniforms_common.view_pos);
        shader_hole.set_vec3("light_pos", uniforms_common.point_light.pos);
        shader_hole.set_vec4("object_color", glm::vec4(0.02f, 0.1f, 0.4f, 0.6f));
        hole_renderer->draw_walls(hole, config_graphics.wireframe_mode, config_graphics.radial_scale, buf);
    }

    if (config_graphics.show_rock_fill) {
        hole_renderer->draw_fill(hole, buf, config_graphics.radial_scale, config_graphics.s_hole_depth);
    }

    // Set default values of shader_default
    shader_default.use();
    shader_default.set_mat4("view", uniforms_common.view);
    shader_default.set_mat4("projection", uniforms_common.projection);
    shader_default.set_vec3("view_pos", uniforms_common.view_pos);
    shader_default.set_vec3("light_pos", uniforms_common.point_light.pos);
    shader_default.set_vec3("light_color", uniforms_common.point_light.color);

    if (config.conf_stat.string_type == StringType::DRILL_STRING) {
        // Draw drill bit
        const glm::mat4 model = calc_model_matrix_drill_bit(x_pipe, q_pipe, config_graphics.radial_scale);
        shader_default.set_mat4("model", model);
        shader_default.set_mat3("normal_matrix", glm::mat3(glm::transpose(glm::inverse(model))));
        drill_bit_model.draw(shader_default);
    }

    // Draw triads
    {
        shader_default.set_mat4("model", glm::mat4(1.0f));
        shader_default.set_mat3("normal_matrix", glm::mat3(1.0f)); // Identity matrix in this case);
    }
    triad_origin.draw(shader_default);
    triad_screen.draw(shader_default, text_renderer, uniforms_common, frame_data.width, frame_data.height);

    if (config_graphics.show_hole_triads) {
        hole_renderer->draw_triads(shader_default, uniforms_common.view, uniforms_common.projection,
                                   config_graphics.hole_triad_scale);
    }

    if (config_graphics.show_pipe_triads) {
        pipe_renderer->draw_triads(top_node, pipe.N, x_pipe, q_pipe, shader_default, uniforms_common.view,
                                   uniforms_common.projection, config_graphics.pipe_triad_scale);
    }

    if (config_graphics.contour_variable != ContourPlotVariable::NONE) {

        colorbar.draw(config_graphics.draw_horizontal_colorbar, config_graphics.contour_val_min,
                      config_graphics.contour_val_max, config_graphics.contour_lim_min, config_graphics.contour_lim_max,
                      text_renderer, config_graphics.colorbar_title, frame_data.width, frame_data.height);
    }
    // Resolve multisampled buffer to intermediate buffer
    glBindFramebuffer(GL_READ_FRAMEBUFFER, framebuffer);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, intermediate_FBO);
    // glBlitFramebuffer(0, 0, viewport_panel_size.x, viewport_panel_size.y, 0, 0, viewport_panel_size.x,
    //                   viewport_panel_size.y, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glBlitFramebuffer(0, 0, sito::VIRTUAL_WIDTH, sito::VIRTUAL_HEIGHT, 0, 0, sito::VIRTUAL_WIDTH, sito::VIRTUAL_HEIGHT,
                      GL_COLOR_BUFFER_BIT, GL_LINEAR); // use LINEAR to reduce jaggies
                                                       // .
    // Unbind the framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    call_imgui_camera_control_slider(config_graphics, conf_dyn.feed_length, viewport_panel_size, pipe, x_pipe, buf);
}

void GraphicsManager::call_imgui_camera_control_slider(ConfigGraphics &config_graphics, const scalar S_feed,
                                                       const ImVec2 &viewport_panel_size, const Pipe &pipe,
                                                       const vector<Vec3> &x_pipe, const byte *buf) {

    // Calculate available space and reserve space for slider
    float slider_height = ImGui::GetFrameHeightWithSpacing(); // Get proper height including padding
    ImVec2 graphics_size = ImVec2(viewport_panel_size.x, viewport_panel_size.y - slider_height);

    ImGui::Text("MD [m]");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 20.0f);

    assert(config_graphics.s_camera >= 0.0f);

    const uint N = pipe.N;

    float S_max_slider = max(0.0f, (float)(S_feed - SMALL_SCALAR));

    if (ImGui::SliderFloat("##camera_s", &config_graphics.s_camera, SMALL_SCALAR, S_max_slider, "%.2f")) {

        scalar S_pos = min(pipe.L_tot, config_graphics.s_camera + pipe.L_tot - S_feed);

        uint i_node = min(N - 2, pipe.find_intial_top_node_from_curve_length(S_pos, buf));

        ArrayView<scalar> S = pipe.get_field<PipeField::S>(buf);
        const scalar S_i = S[i_node];
        const scalar S_ip = S[i_node + 1];
        const glm::vec3 x_i = eigen_vec3_to_glm_vec3(x_pipe[i_node]);
        const glm::vec3 x_ip = eigen_vec3_to_glm_vec3(x_pipe[i_node + 1]);

        const float xi_poly = (S_pos - S_i) / (S_ip - S_i);
        assert(xi_poly >= 0 && xi_poly <= 1);
        const glm::vec3 x_centre = x_i + xi_poly * (x_ip - x_i);

        const glm::vec3 tangent = glm::normalize(x_ip - x_i);
        constexpr glm::vec3 global_up = glm::vec3(0.0f, 0.0f, 1.0f);

        // Create triad using Gram-Schmidt process
        glm::vec3 normal = global_up - glm::dot(global_up, tangent) * tangent;
        float n2 = glm::dot(normal, normal);
        if (n2 < 1e-6f) {
            // tangent is parallel or anti-parallel to global_up; pick an arbitrary orthogonal vector
            normal = glm::vec3(0.0f, 1.0f, 0.0f);
        } else {
            normal = glm::normalize(normal);
        }
        const glm::vec3 camera_direction = glm::normalize(glm::cross(tangent, normal));

        const float offset = config_graphics.radial_scale * 10.0f; // Camera position always normal to pipe
        camera.position = x_centre - offset * camera_direction;

        // Make the camera look in the direction of the pipe
        glm::vec3 direction = glm::normalize(x_centre - camera.position);
        camera.front = direction;

        // Ensure the camera's up direction is always in the global z-direction
        camera.right = glm::normalize(glm::cross(camera.front, global_up));
        camera.up = glm::normalize(glm::cross(camera.right, camera.front));

        // Update camera's internal angles to match the new orientation
        camera.pitch = glm::degrees(asin(direction.z));
        camera.yaw = glm::degrees(atan2(direction.y, direction.x));
    }
    ImGui::Image((void *)(intptr_t)texture_colorbuffer, graphics_size, ImVec2(0, 1), ImVec2(1, 0));
}

void GraphicsManager::check_if_mouse_is_over_graphics_window(ImVec2 viewport_panel_size) {
    assert(frame_data.imgui_frame_active); // Otherwise it's illegal to call imgui functions
    ImVec2 window_pos = ImGui::GetWindowPos();
    ImVec2 mouse_pos = ImGui::GetMousePos();
    frame_data.mouse_is_over_graphics_window =
        ImGui::IsWindowHovered() && mouse_pos.x >= window_pos.x && mouse_pos.x < window_pos.x + viewport_panel_size.x &&
        mouse_pos.y >= window_pos.y && mouse_pos.y < window_pos.y + viewport_panel_size.y;
    if (frame_data.mouse_is_over_graphics_window) {
        frame_data.mouse_is_over_viewport = false; // Allow camera control when mouse is over 3D view
    }
}

glm::mat4 GraphicsManager::calc_model_matrix_drill_bit(const vector<Vec3> &x_pipe, const vector<Quaternion> &q_pipe,
                                                       float radial_scale) {
    glm::mat4 model(1.0f);
    const glm::vec3 pos = eigen_vec3_to_glm_vec3(x_pipe.back());
    model = glm::translate(model, pos);

    const Mat3 R = q_pipe.back().to_matrix();
    glm::mat4 rotation4x4(1.0f);
    for (uint i = 0; i < 3; ++i) {
        rotation4x4[i] = glm::vec4(R.col(i).x(), R.col(i).y(), R.col(i).z(), 0.0f);
    }

    model *= rotation4x4;                                                      // global rotation
    model = glm::rotate(model, -(float)M_PI / 2, glm::vec3(0.0f, 0.0f, 1.0f)); // local model rotation

    float base_size = 0.005 * d_drill_bit;
    float scaled_size = base_size * radial_scale; // Scale the model based on the drill bit size

    // Note, the bit-rock model "x-axis" points in the global Y-direction which is why we scale X and Z here
    model = glm::scale(model, glm::vec3(scaled_size, base_size * std::pow(radial_scale, 0.8), scaled_size));
    return model;
}

void GraphicsManager::setup_camera() {
    camera.mouse_sensitivity_translation = 0.05f;
    camera.pan_sensitivity = 0.05f;
    camera.fov_radians = glm::radians(45.0f);
    camera.position = {20.0f, 20.0f, 0.0f};
    camera.front = glm::normalize(-camera.position);
    camera.pitch = glm::degrees(asin(camera.front.z));
    camera.yaw = glm::degrees(atan2(camera.front.y, camera.front.x));
    camera.update_camera_vectors();
}

GraphicsManager::~GraphicsManager() {
    if (framebuffer != 0) {
        glDeleteFramebuffers(1, &framebuffer);
        glDeleteFramebuffers(1, &intermediate_FBO);
        glDeleteTextures(1, &texture_colorbuffer);
        glDeleteTextures(1, &texture_colorbuffer_multi_sampled);
        glDeleteRenderbuffers(1, &rbo);
    }
}