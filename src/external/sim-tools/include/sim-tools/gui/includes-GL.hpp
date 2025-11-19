#pragma once
//#include "external/glm/glm.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#pragma GCC diagnostic pop
#include "sim-tools/includes.hpp"
#include <GL/glew.h>
#include <GLFW/glfw3.h>

constexpr float SMALL_FLOAT = 1e-4;

constexpr double DELTA_TIME_60_FPS_SEC = 1.0 / 60;

enum class Button {
    LEFT_MOUSE = 0,
    MIDDLE_MOUSE,
    RIGHT_MOUSE,
    W,
    S,
    A,
    D,
    E,
    Q,
    F,
    SPACE,
    LEFT_SHIFT,
    COUNT // Number of buttons
};

inline bool is_closef(float a, float b, float tol = SMALL_FLOAT) {
    return abs(a - b) < tol;
}

inline glm::vec3 eigen_vec3_to_glm_vec3(const Vec3 &v) {
    return {(float)v.x(), (float)v.y(), (float)v.z()};
}
inline Vec3 glm_vec3_to_eigen_vec3(const glm::vec3 &v) {
    return {(scalar)v.x, (scalar)v.y, (scalar)v.z};
}

inline void print_glm_vec2(glm::vec2 v) {
    printf("[%.2f, %.2f]", v.x, v.y);
}

inline void print_glm_vec3(glm::vec3 v) {
    printf("[%.2f, %.2f, %.2f]", v.x, v.y, v.z);
}
inline void print_glm_vec4(glm::vec4 v) {
    printf("[%.2f, %.2f, %.2f, %.2f]", v.x, v.y, v.z, v.w);
}

inline void APIENTRY gl_debug_output(GLenum source, GLenum type, unsigned int id, GLenum severity, GLsizei length,
                                     const char *message, const void *userParam) {
    // ignore non-significant error/warning codes
    if (id == 1 || id == 131169 || id == 131185 || id == 131218 || id == 131204) {
        cout << "Ignoring GL error with id: " << id << "\n";
        return;
    }

    std::cout << "---------------"
              << "\n";
    std::cout << "Debug message (" << id << "): " << message << "\n";

    switch (source) {
    case GL_DEBUG_SOURCE_API:
        std::cout << "Source: API";
        break;
    case GL_DEBUG_SOURCE_WINDOW_SYSTEM:
        std::cout << "Source: Window System";
        break;
    case GL_DEBUG_SOURCE_SHADER_COMPILER:
        std::cout << "Source: Shader Compiler";
        break;
    case GL_DEBUG_SOURCE_THIRD_PARTY:
        std::cout << "Source: Third Party";
        break;
    case GL_DEBUG_SOURCE_APPLICATION:
        std::cout << "Source: Application";
        break;
    case GL_DEBUG_SOURCE_OTHER:
        std::cout << "Source: Other";
        break;
    }
    std::cout << "\n";

    switch (type) {
    case GL_DEBUG_TYPE_ERROR:
        std::cout << "Type: Error";
        break;
    case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
        std::cout << "Type: Deprecated Behaviour";
        break;
    case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:
        std::cout << "Type: Undefined Behaviour";
        break;
    case GL_DEBUG_TYPE_PORTABILITY:
        std::cout << "Type: Portability";
        break;
    case GL_DEBUG_TYPE_PERFORMANCE:
        std::cout << "Type: Performance";
        break;
    case GL_DEBUG_TYPE_MARKER:
        std::cout << "Type: Marker";
        break;
    case GL_DEBUG_TYPE_PUSH_GROUP:
        std::cout << "Type: Push Group";
        break;
    case GL_DEBUG_TYPE_POP_GROUP:
        std::cout << "Type: Pop Group";
        break;
    case GL_DEBUG_TYPE_OTHER:
        std::cout << "Type: Other";
        break;
    }
    std::cout << "\n";

    switch (severity) {
    case GL_DEBUG_SEVERITY_HIGH:
        std::cout << "Severity: high";
        break;
    case GL_DEBUG_SEVERITY_MEDIUM:
        std::cout << "Severity: medium";
        break;
    case GL_DEBUG_SEVERITY_LOW:
        std::cout << "Severity: low";
        break;
    case GL_DEBUG_SEVERITY_NOTIFICATION:
        std::cout << "Severity: notification";
        break;
    }
    std::cout << "\n";
    std::cout << "\n";
    assert(false);
}
