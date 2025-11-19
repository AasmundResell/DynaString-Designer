#pragma once
#include "shader.hpp"
#include "sim-tools/gui/includes-GL.hpp"

namespace sito {

struct Character {
    uint textureID;
    glm::ivec2 size;    /*size of glyph*/
    glm::ivec2 bearing; /*Offset from baseline to left/top of glyph*/
    uint advance;       /*Horizontal offset to advance to next glyph*/
};

struct TextRenderer {
    map<GLchar, Character> characters;
    uint VAO, VBO;
    Shader shader_text;
    glm::mat4 projection;
    float screen_width_old = 0.0f, screen_height_old = 0.0f;

    void create();

    void draw(const string &text, float x_normalized, float y_normalized, float scale, uint screen_width,
              uint screen_height, const glm::vec3 &text_color = glm::vec3{0, 0, 0});

    void update_projection(float screen_width, float screen_height);
};

} // namespace sito