#include "sim-tools/gui/graphics/camera.hpp"

namespace sito {
void Camera::position_camera(const glm::vec3 &pos, const glm::vec3 &target) {

    fov_radians = 45.0f * M_PI / 180;
    position = pos;
    front = glm::normalize(target - pos);
    pitch = glm::degrees(glm::asin(glm::clamp(front.z, -1.0f, 1.0f)));
    pitch = std::clamp(pitch, PITCH_MIN, PITCH_MAX);
    yaw = glm::degrees(glm::atan(front.y, front.x));
    assert(std::isfinite(yaw));
    update_camera_vectors();
}

void Camera::process_keyboard(CameraMovement movement, float delta_time) {
    float velocity = speed * delta_time;
    if (movement == CameraMovement::FORWARD)
        position += front * velocity;
    else if (movement == CameraMovement::BACKWARD)
        position -= front * velocity;
    else if (movement == CameraMovement::LEFT)
        position -= right * velocity;
    else if (movement == CameraMovement::RIGHT)
        position += right * velocity;
    else if (movement == CameraMovement::UP)
        position += up * velocity;
    else {
        assert(movement == CameraMovement::DOWN);
        position -= up * velocity;
    }
}
void Camera::process_mouse_pan(float xoffset, float yoffset) {
    xoffset *= pan_sensitivity;
    yoffset *= pan_sensitivity;

    position -= right * xoffset;
    position -= up * yoffset;
    update_camera_vectors();
}
void Camera::process_mouse_movement_rotation(float xoffset, float yoffset, bool drag) {
    xoffset *= mouse_sensitivity_rotation;
    yoffset *= mouse_sensitivity_rotation;

    if (drag) {
        yaw -= xoffset;
        pitch += yoffset;
    } else {
        yaw -= xoffset;
        pitch += yoffset;
    }

    if (yaw < 0) {
        assert(yaw > -360.0f);
        yaw += 360.0f;
    } else if (yaw > 360.0f) {
        assert(yaw < 2 * 360.0f);
        yaw -= 360.0f;
    }
    pitch = std::clamp(pitch, PITCH_MIN, PITCH_MAX);

    update_camera_vectors();
}
void Camera::process_mouse_scroll(float yoffset) {
    /*Multiplying the value with itself so that scroll gets less sensitive when you are zoomed in.*/
    fov_radians -= fov_radians * scroll_sensitivity * yoffset;
    fov_radians = std::clamp(fov_radians, FOV_MIN, FOV_MAX);
}

void Camera::process_mouse_movement_translation(float yoffset) {
    position -= front * yoffset * mouse_sensitivity_translation;
}

void Camera::update_camera_vectors() {

    front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    front.y = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    front.z = sin(glm::radians(pitch));
    front = glm::normalize(front);
    right = glm::normalize(glm::cross(front, WORLD_UP));
    up = glm::cross(right, front);
    assert(is_closef(glm::length(up), 1.0f));
}

void update_camera(bool fps_mode, float delta_time, float *delta_cursor_x_frame, float *delta_cursor_y_frame,
                   float *scroll_y_offset, const bitset<(uint)Button::COUNT> &is_pressed, Camera *camera,
                   const float radial_scale) {

    if (fps_mode) {
        camera->process_mouse_movement_rotation(*delta_cursor_x_frame, *delta_cursor_y_frame, false);
    } else if (is_pressed[(uint)Button::LEFT_MOUSE]) {
        camera->process_mouse_movement_rotation(*delta_cursor_x_frame, *delta_cursor_y_frame, true);
    } else if (is_pressed[(uint)Button::MIDDLE_MOUSE]) {
        camera->process_mouse_pan(*delta_cursor_x_frame, *delta_cursor_y_frame);
    } else if (is_pressed[(uint)Button::RIGHT_MOUSE]) {
        camera->process_mouse_movement_translation(*delta_cursor_y_frame * radial_scale);
    }
    // reset mouse changes, this is added up from the mouse position callback
    *delta_cursor_x_frame = 0.0f;
    *delta_cursor_y_frame = 0.0f;
    camera->process_mouse_scroll(*scroll_y_offset);
    *scroll_y_offset = 0.0f;

    if (is_pressed[(uint)Button::W])
        camera->process_keyboard(CameraMovement::FORWARD, delta_time);
    if (is_pressed[(uint)Button::S])
        camera->process_keyboard(CameraMovement::BACKWARD, delta_time);
    if (is_pressed[(uint)Button::A])
        camera->process_keyboard(CameraMovement::LEFT, delta_time);
    if (is_pressed[(uint)Button::D])
        camera->process_keyboard(CameraMovement::RIGHT, delta_time);
    if (is_pressed[(uint)Button::SPACE])
        camera->process_keyboard(CameraMovement::UP, delta_time);
    if (is_pressed[(uint)Button::LEFT_SHIFT])
        camera->process_keyboard(CameraMovement::DOWN, delta_time);
}

} // namespace sito