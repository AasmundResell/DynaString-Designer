#include "sim-tools/gui/graphics/colorbar.hpp"
#include "sim-tools/gui/graphics/colormap.hpp"

namespace sito {

/*================================================================================
 The colorbar will be defined relative to the virtual screen space of 1920x1080. It will
 be transformed to device coordinates in the shader
==================================================================================*/

constexpr uint NUM_BINS = 8;
constexpr uint COLORBAR_HEIGHT = 0.25 * VIRTUAL_HEIGHT;
constexpr uint BIN_HEIGHT = COLORBAR_HEIGHT / NUM_BINS;
constexpr uint COLORBAR_WIDTH = BIN_HEIGHT;
uint NUM_VERTICES_LINES = 2 * (2 + NUM_BINS + 1);

// BOTTOMLEFT CONVENTION
constexpr uint COLORBAR_X = 0.8 * VIRTUAL_WIDTH;
constexpr uint COLORBAR_Y = 0.5 * VIRTUAL_HEIGHT;

constexpr float HORIZONTAL_ASPECT = 10.0f; // width:height ratio
constexpr uint HORIZONTAL_COLORBAR_WIDTH = 0.25 * VIRTUAL_WIDTH;
constexpr uint HORIZONTAL_BIN_WIDTH = HORIZONTAL_COLORBAR_WIDTH / NUM_BINS;
constexpr uint HORIZONTAL_COLORBAR_X = 0.5 * (VIRTUAL_WIDTH - HORIZONTAL_COLORBAR_WIDTH);
constexpr uint HORIZONTAL_COLORBAR_Y = 0.25 * VIRTUAL_HEIGHT; // Lower on screen

void Colorbar::create(ColorMapType color_map_type_, uint screen_width, uint screen_height) {
    assert(VAO_colors == 0 && VAO_lines == 0);
    assert(vb_colors.vertices.size() == 0 && vb_lines.vertices.size() == 0);
    vb_colors = {};
    vb_lines = {};
    color_map_type = color_map_type_;
    {
        const path shader_path = path(sim_tools_home_dir) / "resources" / "shaders";
        shader_colors.create("colorbar-colors.vs", "colorbar-colors.fs", shader_path);
        shader_lines.create("colorbar-lines.vs", "colorbar-lines.fs", shader_path);
    }

    {
        /*================================================================================
         Create vertices for the colors in the colorbar
        ==================================================================================*/
        glGenVertexArrays(1, &VAO_colors);
        glBindVertexArray(VAO_colors);
        uint next_attrib_pointer = 0;
        vb_colors.init(VAO_colors, 6 * NUM_BINS, &next_attrib_pointer, false, GL_DYNAMIC_DRAW);
        glBindVertexArray(0);
    }
    {
        /*================================================================================
         Create vertices for the lines in the colorbar
        ==================================================================================*/
        glGenVertexArrays(1, &VAO_lines);
        glBindVertexArray(VAO_lines);
        uint next_attrib_pointer = 0;
        vb_lines.init(VAO_lines, NUM_VERTICES_LINES, &next_attrib_pointer, false, GL_STATIC_DRAW);
        glBindVertexArray(0);
    }
}

void Colorbar::set_vertices(uint screen_width_, uint screen_height_) {
    if (screen_width != screen_width_ || screen_height != screen_height_ || last_horizontal) {
        screen_width = screen_width_;
        screen_height = screen_height_;
        // Center the vertical colorbar in height
        anchor_point.x = screen_width + COLORBAR_X - VIRTUAL_WIDTH;
        anchor_point.y = 0.5f * screen_height - 0.5f * COLORBAR_HEIGHT;
    }

    {
        std::vector<Vertex> &vertices_colors = vb_colors.vertices;
        assert(vertices_colors.size() == 6 * NUM_BINS);
        for (uint i = 0; i < NUM_BINS; i++) {
            float y0 = anchor_point.y + i * BIN_HEIGHT;
            float y1 = anchor_point.y + (i + 1) * BIN_HEIGHT;

            float x0 = anchor_point.x;
            float x1 = anchor_point.x + COLORBAR_WIDTH;

            glm::vec2 p0 = glm::vec2(x0, y0);
            glm::vec2 p1 = glm::vec2(x1, y0);
            glm::vec2 p2 = glm::vec2(x1, y1);
            glm::vec2 p3 = glm::vec2(x0, y1);

            vertices_colors[6 * i + 0].pos = p0;
            vertices_colors[6 * i + 1].pos = p1;
            vertices_colors[6 * i + 2].pos = p2;
            vertices_colors[6 * i + 3].pos = p0;
            vertices_colors[6 * i + 4].pos = p2;
            vertices_colors[6 * i + 5].pos = p3;
        }
        vb_colors.upload_data();
    }
    {
        std::vector<glm::vec2> &vertices_lines = vb_lines.vertices;
        vertices_lines.clear();
        vertices_lines.reserve(NUM_VERTICES_LINES);
        float y0 = anchor_point.y;
        float y1 = anchor_point.y + COLORBAR_HEIGHT;

        float x0 = anchor_point.x;
        float x1 = anchor_point.x + COLORBAR_WIDTH;

        // vertical lines
        vertices_lines.push_back(glm::vec2(x0, y0));
        vertices_lines.push_back(glm::vec2(x0, y1));
        vertices_lines.push_back(glm::vec2(x1, y0));
        vertices_lines.push_back(glm::vec2(x1, y1));

        float offset_x = VIRTUAL_WIDTH - screen_width_;
        float offset_y = VIRTUAL_HEIGHT - screen_height_;
        // horizontal bin lines
        for (uint i = 0; i < NUM_BINS + 1; i++) {

            float y = anchor_point.y + i * BIN_HEIGHT;
            vertices_lines.push_back(glm::vec2(anchor_point.x, y));
            vertices_lines.push_back(glm::vec2(anchor_point.x + COLORBAR_WIDTH, y));
        }
        assert(vertices_lines.size() == NUM_VERTICES_LINES);
        vb_lines.upload_data();
    }
}

void Colorbar::draw(const bool draw_horizontal, float val_min, float val_max, float lim_min, float lim_max,
                    TextRenderer &text_renderer, const std::string &title, uint screen_width_, uint screen_height_) {

    if (draw_horizontal) {
        if (!last_horizontal) {
            // Orientation changed: force color update
            lim_min_old = std::numeric_limits<float>::quiet_NaN();
            lim_max_old = std::numeric_limits<float>::quiet_NaN();
        }
        set_vertices_horizontal(screen_width_, screen_height_);
        update_colors_horizontal(val_min, val_max, lim_min, lim_max);
        last_horizontal = true;
    } else {

        if (last_horizontal) {
            // Orientation changed: force color update
            lim_min_old = std::numeric_limits<float>::quiet_NaN();
            lim_max_old = std::numeric_limits<float>::quiet_NaN();
        }
        set_vertices(screen_width_, screen_height_);
        update_colors(val_min, val_max, lim_min, lim_max);
        last_horizontal = false;
    }

    // --- Set up orthographic projection for 2D overlays ---
    // Use the actual window size, since your vertices are in screen pixel coordinates
    glm::mat4 projection =
        glm::ortho(0.0f, static_cast<float>(screen_width_), 0.0f, static_cast<float>(screen_height_));

    glDisable(GL_DEPTH_TEST);

    // Draw colorbar colors
    glBindVertexArray(VAO_colors);
    shader_colors.use();
    shader_colors.set_mat4("projection", projection); // <-- Set projection matrix!
    glDrawArrays(GL_TRIANGLES, 0, vb_colors.get_count());

    // Draw colorbar lines glBindVertexArray(VAO_lines);
    glBindVertexArray(VAO_lines);
    shader_lines.use();
    shader_lines.set_mat4("projection", projection); // <-- Set projection matrix!
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);       /*wireframe mode*/
    glLineWidth(2.f);
    glDrawArrays(GL_LINES, 0, vb_lines.get_count());
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); /*Restore solid fill mode*/

    glBindVertexArray(0);
    glEnable(GL_DEPTH_TEST);

    if (draw_horizontal) {
        // Draw text
        float text_scale = 0.7f;
        float char_width = 10.0f * 1.3f * text_scale;
        float title_width = title.size() * char_width;
        float title_x = anchor_point.x + 0.5f * HORIZONTAL_COLORBAR_WIDTH - 0.5f * title_width;
        float title_y = anchor_point.y + BIN_HEIGHT + 20.0f;
        text_renderer.draw(title, title_x, title_y, 1.3f * text_scale, screen_width_, screen_height_);

        // Tick labels
        const float val_low = lim_min;
        const float val_high = lim_max;
        assert(val_high >= val_low);

        constexpr float SCIENTIFIC_NOTATION_LIMIT_ABS_MAX = 10000;
        constexpr float SCIENTIFIC_NOTATION_LIMIT_ABS_MIN = 0.01;
        const float max_abs = std::max(std::abs(val_low), std::abs(val_high));
        const float min_abs = std::min(std::abs(val_low), std::abs(val_high));
        const bool use_scientific_notation = (max_abs > SCIENTIFIC_NOTATION_LIMIT_ABS_MAX) ||
                                             (min_abs > 0.0f && min_abs < SCIENTIFIC_NOTATION_LIMIT_ABS_MIN);

        for (uint i = 0; i < NUM_BINS + 1; i += 2) { // Only every other bin
            const float xi = i / (float)NUM_BINS;
            const float val = val_low + xi * (val_high - val_low);

            std::stringstream ss;
            if (use_scientific_notation) {
                ss << std::scientific << std::setprecision(3) << val;
            } else {
                ss << std::fixed << std::setprecision(1) << val;
            }
            // Draw below the colorbar
            float label_x = anchor_point.x + i * HORIZONTAL_BIN_WIDTH - 10.0f;
            float label_y = anchor_point.y - 18.0f;
            text_renderer.draw(ss.str(), label_x, label_y, text_scale, screen_width_, screen_height_);
        }
    } else {
        /*================================================================================
        Draw text for vertical colorbar
        ==================================================================================*/

        // Title position (above colorbar, offset by 5 virtual pixels)
        float text_scale = 0.7f;
        float char_width = 10.0f * 1.3f * text_scale; // 1.3f is the scale used in draw()
        float title_width = title.size() * char_width;
        float title_x = anchor_point.x + 0.5f * COLORBAR_WIDTH - 0.5f * title_width;
        float title_y = anchor_point.y + COLORBAR_HEIGHT + 20.0f;
        text_renderer.draw(title, title_x, title_y, 1.3f * text_scale, screen_width_, screen_height_);

        // Tick labels
        // The color mapping is fixed to the provided display limits [lim_min, lim_max].
        // Draw tick labels across the limit range so the text updates when limits are changed.
        const float val_low = lim_min;
        const float val_high = lim_max;
        assert(val_high >= val_low);

        constexpr float SCIENTIFIC_NOTATION_LIMIT_ABS_MAX = 10000;
        constexpr float SCIENTIFIC_NOTATION_LIMIT_ABS_MIN = 0.01;
        const float max_abs = std::max(std::abs(val_low), std::abs(val_high));
        const float min_abs = std::min(std::abs(val_low), std::abs(val_high));
        // Only use scientific when values are very large OR when the smallest magnitude is
        // non-zero and below the small threshold. Avoid triggering on exact zero.
        const bool use_scientific_notation = (max_abs > SCIENTIFIC_NOTATION_LIMIT_ABS_MAX) ||
                                             (min_abs > 0.0f && min_abs < SCIENTIFIC_NOTATION_LIMIT_ABS_MIN);

        for (uint i = 0; i < NUM_BINS + 1; i++) {
            const float xi = i / (float)NUM_BINS;
            const float val = val_low + xi * (val_high - val_low);

            std::stringstream ss;
            if (use_scientific_notation) {
                ss << std::scientific << std::setprecision(3) << val;
            } else {
                ss << std::fixed << std::setprecision(1) << val;
            }
            text_renderer.draw(ss.str(), anchor_point.x + 2.0f + COLORBAR_WIDTH,
                               anchor_point.y - 5.0f * text_scale + i * BIN_HEIGHT, text_scale, screen_width_,
                               screen_height_);
        }
    }
}

void Colorbar::update_colors(float val_min, float val_max, float lim_min, float lim_max) {
    assert(isfinite(val_min) && isfinite(val_max));
    assert(val_max >= val_min);
    assert(lim_max >= lim_min);

    // Only update when display limits change (colors are always mapped to [lim_min, lim_max]).
    if (lim_min == lim_min_old && lim_max == lim_max_old) {
        return;
    }
    lim_min_old = lim_min;
    lim_max_old = lim_max;

    // Map colors across the fixed display limits
    const float val_low = lim_min;
    const float val_high = lim_max;

    for (uint i = 0; i < NUM_BINS; i++) {
        float xi_i = i / (float)NUM_BINS;
        float xi_ip = (i + 1) / (float)NUM_BINS;

        float val_i = val_low + xi_i * (val_high - val_low);
        float val_ip = val_low + xi_ip * (val_high - val_low);

        glm::vec3 color_i;
        glm::vec3 color_ip;

        // Always map into the display limits, clamping outside values to edges so the bar
        // consistently spans lim_min..lim_max.
        color_i = get_color_from_colormap(std::clamp(val_i, lim_min, lim_max), lim_min, lim_max, color_map_type, true);
        color_ip =
            get_color_from_colormap(std::clamp(val_ip, lim_min, lim_max), lim_min, lim_max, color_map_type, true);

        // Creating a quad out of two triangles, where the shared vertices are duplicated
        vb_colors.vertices[6 * i + 0].color = color_i;
        vb_colors.vertices[6 * i + 1].color = color_i;
        vb_colors.vertices[6 * i + 2].color = color_ip;
        vb_colors.vertices[6 * i + 3].color = color_i;
        vb_colors.vertices[6 * i + 4].color = color_ip;
        vb_colors.vertices[6 * i + 5].color = color_ip;
    }
    vb_colors.upload_data();
}

void Colorbar::set_vertices_horizontal(uint screen_width_, uint screen_height_) {
    if (screen_width != screen_width_ || screen_height != screen_height_ || !last_horizontal) {
        screen_width = screen_width_;
        screen_height = screen_height_;
        // Center the horizontal colorbar in width
        anchor_point.x = 0.5f * screen_width - 0.5f * HORIZONTAL_COLORBAR_WIDTH;
        anchor_point.y = screen_height + HORIZONTAL_COLORBAR_Y - VIRTUAL_HEIGHT;
    }

    {
        std::vector<Vertex> &vertices_colors = vb_colors.vertices;
        assert(vertices_colors.size() == 6 * NUM_BINS);
        for (uint i = 0; i < NUM_BINS; i++) {
            float x0 = anchor_point.x + i * HORIZONTAL_BIN_WIDTH;
            float x1 = anchor_point.x + (i + 1) * HORIZONTAL_BIN_WIDTH;

            float y0 = anchor_point.y;
            float y1 = anchor_point.y + BIN_HEIGHT;

            glm::vec2 p0 = glm::vec2(x0, y0);
            glm::vec2 p1 = glm::vec2(x1, y0);
            glm::vec2 p2 = glm::vec2(x1, y1);
            glm::vec2 p3 = glm::vec2(x0, y1);

            vertices_colors[6 * i + 0].pos = p0;
            vertices_colors[6 * i + 1].pos = p1;
            vertices_colors[6 * i + 2].pos = p2;
            vertices_colors[6 * i + 3].pos = p0;
            vertices_colors[6 * i + 4].pos = p2;
            vertices_colors[6 * i + 5].pos = p3;
        }
        vb_colors.upload_data();
    }
    {
        std::vector<glm::vec2> &vertices_lines = vb_lines.vertices;
        vertices_lines.clear();
        vertices_lines.reserve(2 * (2 + NUM_BINS + 1));
        float x0 = anchor_point.x;
        float x1 = anchor_point.x + HORIZONTAL_COLORBAR_WIDTH;

        float y0 = anchor_point.y;
        float y1 = anchor_point.y + BIN_HEIGHT;

        // horizontal lines
        vertices_lines.push_back(glm::vec2(x0, y0));
        vertices_lines.push_back(glm::vec2(x1, y0));
        vertices_lines.push_back(glm::vec2(x0, y1));
        vertices_lines.push_back(glm::vec2(x1, y1));

        // vertical bin lines
        for (uint i = 0; i < NUM_BINS + 1; i++) {
            float x = anchor_point.x + i * HORIZONTAL_BIN_WIDTH;
            vertices_lines.push_back(glm::vec2(x, y0));
            vertices_lines.push_back(glm::vec2(x, y1));
        }
        vb_lines.upload_data();
    }
}

void Colorbar::update_colors_horizontal(float val_min, float val_max, float lim_min, float lim_max) {
    assert(isfinite(val_min) && isfinite(val_max));
    assert(val_max >= val_min);
    assert(lim_max >= lim_min);

    if (lim_min == lim_min_old && lim_max == lim_max_old) {
        return;
    }
    lim_min_old = lim_min;
    lim_max_old = lim_max;

    const float val_low = lim_min;
    const float val_high = lim_max;

    for (uint i = 0; i < NUM_BINS; i++) {
        float xi_i = i / (float)NUM_BINS;
        float xi_ip = (i + 1) / (float)NUM_BINS;

        float val_i = val_low + xi_i * (val_high - val_low);
        float val_ip = val_low + xi_ip * (val_high - val_low);

        glm::vec3 color_i =
            get_color_from_colormap(std::clamp(val_i, lim_min, lim_max), lim_min, lim_max, color_map_type, true);
        glm::vec3 color_ip =
            get_color_from_colormap(std::clamp(val_ip, lim_min, lim_max), lim_min, lim_max, color_map_type, true);

        vb_colors.vertices[6 * i + 0].color = color_i;
        vb_colors.vertices[6 * i + 1].color = color_ip;
        vb_colors.vertices[6 * i + 2].color = color_ip;
        vb_colors.vertices[6 * i + 3].color = color_i;
        vb_colors.vertices[6 * i + 4].color = color_ip;
        vb_colors.vertices[6 * i + 5].color = color_i;
    }
    vb_colors.upload_data();
}

} // namespace sito
