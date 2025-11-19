#include "hole-renderer.hpp"
#include <cmath>

HoleRenderer::HoleRenderer(const Hole &hole, const byte *buf, const float L_char_triad_world)
    : N_hole(hole.N_hole), radial_scale(1.0) {
    scalar *arr_alpha = hole.get_field<HoleField::alpha>(buf).data;
    vertex_buffer.resize(get_vertex_count());
    index_buffer.resize(get_index_count());
    create_indices(arr_alpha);
    setup_buffers();

    update_wall_vertex_buffer(hole, buf);

    hole_triads.resize(N_hole);
    triads_positions.resize(N_hole);
    triads_orientations.resize(N_hole);

    for (uint i = 0; i < N_hole; i++) {
        hole_triads[i].load();

        /* NOTE: i + 2 is because we skip the ghost values */
        Mat3 R = hole.get_field<HoleField::q>(buf)[i + 2].to_matrix();
        triads_positions[i] = eigen_vec3_to_glm_vec3(hole.get_field<HoleField::x>(buf)[i + 2]);

        // Convert Eigen matrix to GLM matrix
        glm::mat4 rotation_glm = glm::mat4(1.0f);
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                rotation_glm[col][row] = R(row, col);
            }
        }
        triads_orientations[i] = rotation_glm;
    }
}

void HoleRenderer::update_wall_vertex_buffer(const Hole &hole, const byte *buf) {
    assert(hole.type == HoleSurfaceType::CIRCULAR || hole.type == HoleSurfaceType::ELLIPTICAL);

    const ArrayView<Vec3> arr_x = hole.get_field<HoleField::x>(buf);
    const ArrayView<Quaternion> arr_q = hole.get_field<HoleField::q>(buf);
    const ArrayView<Vec2> arr_co = hole.get_field<HoleField::co>(buf);
    const ArrayView<scalar> arr_alpha = hole.get_field<HoleField::alpha>(buf);
    const ArrayView<scalar> arr_a = hole.get_field<HoleField::a>(buf);
    const ArrayView<scalar> arr_b = hole.get_field<HoleField::b>(buf);

    /*--------------------------------------------------
    -----  The elliptical surface is defined by: -------
    -----      1 = (x / a)^2 + (y / b)^2         -------
    ----------------------------------------------------
        A point on the elliptical surface is found by:
        x = a * cos(t)
        y = b * sin(t)
        The normal is proportional to the gradient of the surface
        grad = (x / a^2, y / b^2) = (cos(t) / a, sin(t) / b)
    --------------------------------------------------*/
    for (uint i = 2; i < N_hole + 2; ++i) { // Skipping ghost values
        const Vec3 x = arr_x[i];
        const Quaternion q = arr_q[i];
        const Mat3 R = q.to_matrix();
        const Vec3 normal = R.col(1);
        const Vec3 binormal = R.col(2);
        const scalar a = arr_a[i] * radial_scale;
        const scalar b = arr_b[i] * radial_scale;
        const Vec2 co = arr_co[i] * radial_scale;
        const scalar alpha = arr_alpha[i];

        const Vec3 normal_rotated = cos(alpha) * normal - sin(alpha) * binormal;
        const Vec3 binormal_rotated = sin(alpha) * normal + cos(alpha) * binormal;
        const Vec3 offset_center = x + co.x() * normal + co.y() * binormal;

        for (uint j = 0; j < NUM_POINTS_CIRCUMF; j++) {
            // const scalar angle = j * 2.0 * M_PI / (NUM_POINTS_CIRCUMF - 1.0);
            const scalar angle = j * 2.0 * M_PI / NUM_POINTS_CIRCUMF;
            const scalar cos_angle = cos(angle);
            const scalar sin_angle = sin(angle);

            const Vec3 point = offset_center + (a * cos_angle * normal_rotated + b * sin_angle * binormal_rotated);
            const Vec3 point_normal = (cos_angle * normal_rotated / a + sin_angle * binormal_rotated / b).normalized();

            const uint vertex_index = (i - 2) * NUM_POINTS_CIRCUMF + j;
            vertex_buffer[vertex_index].point = eigen_vec3_to_glm_vec3(point);
            vertex_buffer[vertex_index].normal = eigen_vec3_to_glm_vec3(point_normal);
        }
    }
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, vertex_buffer.size() * sizeof(Vertex), vertex_buffer.data());
}

void HoleRenderer::update_fill_vertex_buffer(const Hole &hole, const byte *buf, scalar fill_depth, uint N_current) {

    const ArrayView<Vec3> arr_x = hole.get_field<HoleField::x>(buf);
    const ArrayView<Quaternion> arr_q = hole.get_field<HoleField::q>(buf);
    const ArrayView<scalar> arr_a = hole.get_field<HoleField::a>(buf);
    const ArrayView<scalar> arr_b = hole.get_field<HoleField::b>(buf);

    constexpr scalar radial_fill_fraction = 0.9; // Fraction of the radius to fill

    for (uint i = N_current + 2; i < N_hole + 2; ++i) {
        const Vec3 x = arr_x[i];
        const Quaternion q = arr_q[i];
        const Mat3 R = q.to_matrix();
        const Vec3 normal = R.col(1);
        const Vec3 binormal = R.col(2);
        const scalar a = arr_a[i] * radial_scale * radial_fill_fraction;
        const scalar b = arr_b[i] * radial_scale * radial_fill_fraction;

        for (uint j = 0; j < NUM_POINTS_CIRCUMF; j++) {
            const scalar angle = j * 2.0 * M_PI / NUM_POINTS_CIRCUMF;
            const scalar cos_angle = cos(angle);
            const scalar sin_angle = sin(angle);

            const Vec3 point = x + (a * cos_angle * normal + b * sin_angle * binormal);
            const Vec3 point_normal = (cos_angle * normal / a + sin_angle * binormal / b).normalized();

            const uint vertex_index = (i - N_current - 2) * NUM_POINTS_CIRCUMF + j;
            vertex_buffer[vertex_index].point = eigen_vec3_to_glm_vec3(point);
            vertex_buffer[vertex_index].normal = eigen_vec3_to_glm_vec3(point_normal);
        }
    }

    float offset = N_current * NUM_POINTS_CIRCUMF * sizeof(Vertex);
    float size = (N_hole - N_current) * NUM_POINTS_CIRCUMF * sizeof(Vertex);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferSubData(GL_ARRAY_BUFFER, offset, size, vertex_buffer.data());
}

void HoleRenderer::create_indices(const scalar *arr_theta) {
    const uint index_count = get_index_count();
    constexpr uint segments = NUM_POINTS_CIRCUMF; // No need to subtract 1
    index_buffer.resize(index_count);
    uint index_index = 0;

    for (uint i = 2; i < N_hole + 1; ++i) {
        scalar theta_current = arr_theta[i];
        scalar theta_next = arr_theta[i + 1];
        scalar theta_diff = theta_next - theta_current;

        // Normalize theta_diff to [-π, π]
        if (theta_diff > M_PI)
            theta_diff -= 2 * M_PI;
        if (theta_diff < -M_PI)
            theta_diff += 2 * M_PI;

        // Calculate vertex offset based on rotation
        int vertex_offset = round((theta_diff * segments) / (2 * M_PI));

        for (uint j = 0; j < segments; ++j) {
            uint current = (i - 2) * segments + j;
            uint next = (i - 2) * segments + ((j + 1) % segments); // Wrap around to first vertex

            // Calculate offset for next row connections
            uint j_offset = (j + vertex_offset + segments) % segments;
            uint next_row = (i - 1) * segments + j_offset;
            uint next_row_next = (i - 1) * segments + ((j_offset + 1) % segments);

            // Add the two triangles
            index_buffer[index_index++] = current;
            index_buffer[index_index++] = next_row;
            index_buffer[index_index++] = next;

            index_buffer[index_index++] = next;
            index_buffer[index_index++] = next_row;
            index_buffer[index_index++] = next_row_next;
        }
    }
}

void HoleRenderer::setup_buffers() {
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertex_buffer.size() * sizeof(Vertex), vertex_buffer.data(), GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, index_buffer.size() * sizeof(uint), index_buffer.data(), GL_STATIC_DRAW);

    // This should be improved! Use alignof of the Vertex struct
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)(sizeof(glm::vec3)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}

void HoleRenderer::draw_walls(const Hole &hole, bool wireframe_mode, float new_scale, const byte *buf) {
    if (abs(new_scale - this->radial_scale) > 10E-6) {
        this->radial_scale = (scalar)new_scale;
        update_wall_vertex_buffer(hole, buf);
    }

    glBindVertexArray(VAO);
    const uint index_count = get_index_count();

    // Enable depth testing but don't write to depth buffer for transparent objects
    glDepthMask(GL_FALSE);

    if (wireframe_mode) {
        glLineWidth(3.0f);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDrawElements(GL_TRIANGLES, index_count, GL_UNSIGNED_INT, 0);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glLineWidth(1.0f);
    } else {
        // Transparent rendering
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // First pass: render back faces
        glEnable(GL_CULL_FACE);
        glCullFace(GL_FRONT);
        glDepthMask(GL_FALSE);
        glDrawElements(GL_TRIANGLES, index_count, GL_UNSIGNED_INT, 0);

        // Second pass: render front faces
        glCullFace(GL_BACK);
        glDepthMask(GL_TRUE);
        glDrawElements(GL_TRIANGLES, index_count, GL_UNSIGNED_INT, 0);

        // Reset states
        glDisable(GL_CULL_FACE);
        glDisable(GL_BLEND);
    }

    // Reset depth mask
    glDepthMask(GL_TRUE);
    glBindVertexArray(0);
}

void HoleRenderer::draw_fill(const Hole &hole, const byte *buf, float new_scale, scalar s_depth) {

    uint N_current = 0;
    for (uint i = 2; i < N_hole + 2; ++i) {
        if (hole.get_field<HoleField::s>(buf)[i] > s_depth) {
            N_current = i - 2; // Subtract 2 to account for ghost values
            break;
        }
    }

    // Update if scale is changed OR if the fill index is changed is changed
    if (abs(new_scale - this->radial_scale) > 10E-6 || N_current != N_hole_fill) {
        N_hole_fill = N_current;
        this->radial_scale = (scalar)new_scale;
        update_wall_vertex_buffer(hole, buf);
    }

    update_fill_vertex_buffer(hole, buf, s_depth, N_current);

    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, get_index_count(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

void HoleRenderer::draw_triads(Shader &shader, const glm::mat4 &view_matrix, const glm::mat4 &proj_matrix,
                               float L_char_triad_world) {
    shader.use(); // Add this to ensure shader is bound

    shader.set_mat4("view", view_matrix);
    shader.set_mat4("projection", proj_matrix);

    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);

    constexpr float L_char_triad_model = 1.2;
    assert(L_char_triad_model > 1000 * SMALL_FLOAT);
    const float scale_factor_triads = L_char_triad_world / L_char_triad_model;

    for (uint i = 0; i < N_hole; i++) {
        // Build transformation matrix: Scale -> Rotate -> Translate
        glm::mat4 model_matrix_triads = glm::mat4(1.0f);
        model_matrix_triads = glm::translate(model_matrix_triads, triads_positions[i]);
        model_matrix_triads = model_matrix_triads * triads_orientations[i];
        model_matrix_triads = glm::scale(model_matrix_triads, glm::vec3(scale_factor_triads));

        shader.set_mat4("model", model_matrix_triads);
        hole_triads[i].draw(shader);
    }
}

HoleRenderer::~HoleRenderer() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
}