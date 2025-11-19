#include "sim-tools/gui/graphics/text-renderer.hpp"
#include <ft2build.h>
#include FT_FREETYPE_H

namespace sito {

void TextRenderer::create() {
    assert(characters.empty());
    const path shader_dir = path(sim_tools_home_dir) / "resources" / "shaders";
    shader_text.create("text.vs", "text.fs", shader_dir);

    /*================================================================================
    Adapted from https://learnopengl.com/In-Practice/Text-Rendering
    ==================================================================================*/
    FT_Library ft;
    if (FT_Init_FreeType(&ft)) {
        THROW_RUNTIME_ERROR("Could not init FreeType library");
    }
    const path font_path = path(sim_tools_home_dir) / "resources" / "fonts" / "DejaVuSans.ttf";
    FT_Face face;
    if (FT_New_Face(ft, font_path.c_str(), 0, &face)) {
        THROW_RUNTIME_ERROR("Failed to load font: " + font_path.string());
    }

    FT_Set_Pixel_Sizes(face, 0, 24);       // setting width=0 lets it dynamically calculate width
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1); // Disable byte alignment restriction (haven't fully looked into why)
    /*Load 128 ASCII characters*/
    for (u_char c = 0; c < 128; c++) {
        if (FT_Load_Char(face, c, FT_LOAD_RENDER)) {
            THROW_RUNTIME_ERROR("Failed to load glyph: " + c);
        }
        uint texture;
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        /*Create texture from loaded character*/
        glTexImage2D(GL_TEXTURE_2D, 0,
                     GL_RED, // Didn't fully look into this
                     face->glyph->bitmap.width, face->glyph->bitmap.rows, 0, GL_RED, GL_UNSIGNED_BYTE,
                     face->glyph->bitmap.buffer);
        /*Set texture options*/
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        const Character character{.textureID = texture,
                                  .size = {face->glyph->bitmap.width, face->glyph->bitmap.rows},
                                  .bearing = {face->glyph->bitmap_left, face->glyph->bitmap_top},
                                  .advance = (uint)face->glyph->advance.x};
        characters.insert(pair{c, character});
    }
    glBindTexture(GL_TEXTURE_2D, 0);
    FT_Done_Face(face);
    FT_Done_FreeType(ft);

    /*Set up quads to draw the texture on on the gpu*/
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 6 * 4, NULL, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void TextRenderer::draw(const string &text, float x, float y, float scale, uint virtual_width, uint virtual_height,
                        const glm::vec3 &text_color) {
    if (text.empty())
        return;
    assert(characters.size() == 128);

    shader_text.use();

    // Projection matrix for virtual canvas
    glm::mat4 projection = glm::ortho(0.0f, (float)virtual_width, 0.0f, (float)virtual_height);
    shader_text.set_mat4("projection", projection);

    shader_text.set_vec3("text_color", text_color);
    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(VAO);

    glEnable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glClear(GL_DEPTH_BUFFER_BIT); // Optional

    for (char c : text) {
        const Character ch = characters[c];
        const float xpos = x + ch.bearing.x * scale;
        const float ypos = y - (ch.size.y - ch.bearing.y) * scale;
        const float w = ch.size.x * scale;
        const float h = ch.size.y * scale;
        // clang-format off
        const float vertices[6][4] = {
            {xpos,     ypos + h, 0.0f, 0.0f},
            {xpos,     ypos,     0.0f, 1.0f},
            {xpos + w, ypos,     1.0f, 1.0f},

            {xpos,     ypos + h, 0.0f, 0.0f},
            {xpos + w, ypos,     1.0f, 1.0f},
            {xpos + w, ypos + h, 1.0f, 0.0f}};
        // clang-format on

        glBindTexture(GL_TEXTURE_2D, ch.textureID);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glDrawArrays(GL_TRIANGLES, 0, 6);

        // Advance cursor
        x += (ch.advance / 64) * scale;
    }
    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);

    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ZERO);
}

void TextRenderer::update_projection(float screen_width, float screen_height) {
    if (!is_close(screen_width, screen_width_old) || !is_close(screen_height, screen_height_old)) {
        glm::mat4 projection = glm::ortho(0.0f, screen_width, 0.0f, screen_height);
        shader_text.use();
        shader_text.set_mat4("projection", projection);
        screen_width_old = screen_width;
        screen_height_old = screen_height;
    }
}

} // namespace sito
