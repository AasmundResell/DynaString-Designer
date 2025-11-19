#pragma once
#include "sim-tools/gui/includes-GL.hpp"
#include <bitset>
using std::bitset;

namespace sito {
// enum class CameraMovement {
//     FORWARD,
//     BACKWARD,
//     LEFT,
//     RIGHT,
//     UP,
//     DOWN
// };

// class Camera {
//     static constexpr float SPEED = 3.0f;
//     inline static const glm::vec3 WORLD_UP{0, 0, 1};
//     static constexpr float PITCH_MAX = 89.0f;
//     static constexpr float PITCH_MIN = -89.0f;
//     static constexpr float MOUSE_SENSITIVITY = 0.1f;
//     static constexpr float SCROLL_SENSITIVITY = 0.1f;

//   public:
//     glm::vec3 position;
//     /*Changed the convention from learnopengl.com, since I want z to point up*/
//     glm::vec3 front; // -y
//     glm::vec3 right; // x
//     glm::vec3 up;    // z

//     float yaw, pitch; // euler angles
//     float zoom;

//     void position_camera(const glm::vec3 &pos, const glm::vec3 &target) {

//         zoom = 45.0f;
//         position = pos;
//         front = glm::normalize(target - pos);
//         pitch = glm::degrees(glm::asin(glm::clamp(front.z, -1.0f, 1.0f)));
//         pitch = std::clamp(pitch, PITCH_MIN, PITCH_MAX);
//         yaw = glm::degrees(glm::atan(front.y, front.x));
//         assert(std::isfinite(yaw));
//         update_camera_vectors();
//     }

//     glm::mat4 get_view_matrix() const { return glm::lookAt(position, position + front, up); }

//     void process_keyboard(CameraMovement direction, float delta_time) {
//         float delta_distance = SPEED * delta_time;
//         if (direction == CameraMovement::FORWARD)
//             position += front * delta_distance;
//         if (direction == CameraMovement::BACKWARD)
//             position -= front * delta_distance;
//         if (direction == CameraMovement::LEFT)
//             position -= right * delta_distance;
//         if (direction == CameraMovement::RIGHT)
//             position += right * delta_distance;
//         if (direction == CameraMovement::UP)
//             position += up * delta_distance;
//         if (direction == CameraMovement::DOWN)
//             position -= up * delta_distance;
//     }
//     void process_mouse_pan(float xoffset, float yoffset, bool constrain_pitch = true) {
//         xoffset *= MOUSE_SENSITIVITY;
//         yoffset *= MOUSE_SENSITIVITY;
//         yaw -= xoffset;
//         if (yaw < 0) {
//             assert(yaw > -360.0f);
//             yaw += 360.0f;
//         } else if (yaw > 360.0f) {
//             assert(yaw < 2 * 360.0f);
//             yaw -= 360.0f;
//         }
//         // yaw = fmod(yaw, 360.0f);
//         pitch += yoffset;
//         if (constrain_pitch) {
//             pitch = std::clamp(pitch, PITCH_MIN, PITCH_MAX);
//         }

//         update_camera_vectors();
//     }
//     void process_mouse_scroll(float yoffset) { zoom = std::clamp(zoom - SCROLL_SENSITIVITY * yoffset, 1.0f, 45.0f); }

//   private:
//     void update_camera_vectors() {
//         front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
//         front.y = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
//         front.z = sin(glm::radians(pitch));
//         front = glm::normalize(front);
//         right = glm::normalize(glm::cross(front, WORLD_UP));
//         up = glm::cross(right, front);
//         assert(is_closef(glm::length(up), 1.0f));
//     }
// };

#pragma once
#include "sim-tools/gui/includes-GL.hpp"
#include <bitset>

enum class CameraMovement {
    FORWARD = 0,
    BACKWARD,
    LEFT,
    RIGHT,
    UP,
    DOWN
};

class Camera {
    inline static const glm::vec3 WORLD_UP{0, 0, 1};
    static constexpr float PITCH_MAX = 89.0f;
    static constexpr float PITCH_MIN = -89.0f;

    float FOV_MIN = 5.0f * M_PI / 180;
    float FOV_MAX = 45.0f * M_PI / 180;

  public:
    static constexpr float SPEED_DEFAULT = 4.0f;
    static constexpr float MOUSE_SENSITIVITY_ROTATION_DEFAULT = 0.2f;
    static constexpr float MOUSE_SENSITIVITY_TRANSLATION_DEFAULT = 0.008f;
    static constexpr float PAN_SENSITIVITY_DEFAULT = 0.003f;
    static constexpr float SCROLL_SENSITIVITY_DEFAULT = 0.1f;
    /*Override these in the client code at runtime if needed.*/
    float speed = SPEED_DEFAULT;
    float mouse_sensitivity_rotation = MOUSE_SENSITIVITY_ROTATION_DEFAULT;
    float mouse_sensitivity_translation = MOUSE_SENSITIVITY_TRANSLATION_DEFAULT;
    float pan_sensitivity = PAN_SENSITIVITY_DEFAULT;
    float scroll_sensitivity = SCROLL_SENSITIVITY_DEFAULT;

    glm::vec3 position;

    glm::vec3 front;
    glm::vec3 right;
    glm::vec3 up;

    float yaw, pitch;  // euler angles
    float fov_radians; /*Field of view (FOV) in radians. Used to adjust "zoom"*/

    void position_camera(const glm::vec3 &pos, const glm::vec3 &target);

    glm::mat4 get_view_matrix() const { return glm::lookAt(position, position + front, up); }

    void process_keyboard(CameraMovement movement, float delta_time);
    void process_mouse_pan(float xoffset, float yoffset);

    void process_mouse_movement_rotation(float xoffset, float yoffset, bool drag);

    void process_mouse_scroll(float yoffset);

    /*Will move the camera forwards and backwards depending on yoffset*/
    void process_mouse_movement_translation(float yoffset);

    void update_camera_vectors();
};

void update_camera(bool fps_mode, float delta_time, float *delta_cursor_x_frame, float *delta_cursor_y_frame,
                   float *scroll_y_offset, const bitset<(uint)Button::COUNT> &is_pressed, Camera *camera,
                   const float radial_scale);

} // namespace sito