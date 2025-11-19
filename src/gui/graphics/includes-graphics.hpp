#pragma once
#include "gui/plot-utils.hpp"
#include "sim-tools/gui/graphics/camera.hpp"
#include "sim-tools/gui/graphics/colorbar.hpp"
#include "sim-tools/gui/graphics/colormap.hpp"
#include "sim-tools/gui/graphics/graphics-model.hpp"
#include "sim-tools/gui/graphics/shader.hpp"
#include "sim-tools/gui/graphics/text-renderer.hpp"
#include "sim-tools/gui/graphics/triad.hpp"
#include "sim-tools/gui/graphics/uniforms-common.hpp"
#include "sim-tools/gui/includes-GL.hpp"

using sito::Camera;
using sito::Colorbar;
using sito::ColorMapType;
using sito::GraphicsModel;
using sito::Shader;
using sito::TextRenderer;
using sito::Triad;
using sito::TriadOrigin;
using sito::TriadScreen;
using sito::UniformsCommon;

struct ConfigGraphics {
    bool wireframe_mode = false;
    bool show_hole_triads = false;
    bool show_pipe_triads = false;
    bool show_rock_fill = false;
    bool contour_auto_limits = true;
    bool draw_horizontal_colorbar = false;
    bool grey_outside_limits = true;
    float hole_triad_scale = 0.5f;
    float pipe_triad_scale = 0.5f;
    float radial_scale = 1.0f;
    float s_camera = 0.0f;
    scalar s_hole_depth = 0.0f;

    ContourPlotVariable contour_variable = ContourPlotVariable::NONE;
    string colorbar_title = "";
    ColorMapType color_map_type = ColorMapType::RAINBOW;
    float contour_lim_min = 0, contour_lim_max = 0; /*Min and max limit values of colors*/
    float contour_val_min = 0, contour_val_max = 0; /*Min and max values of the field that is displayed*/
};