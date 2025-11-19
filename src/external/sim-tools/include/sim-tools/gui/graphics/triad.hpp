#pragma once
#include "graphics-model.hpp"
#include "text-renderer.hpp"
#include "uniforms-common.hpp"
namespace sito {

enum class TriadLabelMode {
    XYZ,   // Default
    TVD_NE // TVD, North, East
};

struct Triad {
    GraphicsModel triad_model;
    void load();
    float find_L_char_triad_model() const;
    void draw(Shader &shader) const;
};

class TriadOrigin {
    Triad triad;
    glm::mat4 model_matrix;

  public:
    /*L_char_triad_world is the length of the triad in world space*/
    void create(float L_char_triad_world);
    void draw(Shader &shader) const;
};

class TriadScreen {
    Triad triad;
    TriadLabelMode label_mode; // Default
    std::array<const char *, 3> labels;

  public:
    void create(TriadLabelMode label_mode_ = TriadLabelMode::XYZ) {
        triad.load();
        label_mode = label_mode_;

        constexpr std::array<const char *, 3> xyz_labels = {"x", "y", "z"};
        constexpr std::array<const char *, 3> tvdne_labels = {"TVD", "N", "E"};
        labels = (label_mode == TriadLabelMode::TVD_NE) ? tvdne_labels : xyz_labels;
    }

    void draw(Shader &shader, TextRenderer &text_renderer, const UniformsCommon &uniforms_common, float screen_width,
              float screen_height);
};

} // namespace sito