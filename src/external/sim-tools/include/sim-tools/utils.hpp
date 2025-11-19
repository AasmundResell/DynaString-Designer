#pragma once
#include "includes.hpp"
#include "sim-tools/gui/includes-GL.hpp"

namespace sito {

// Virtual screen dimensions for drawing GUI elements
// These make the objects resolution independent
// and keep their aspect ratio constant
constexpr int VIRTUAL_WIDTH = 1920;
constexpr int VIRTUAL_HEIGHT = 1080;

enum class ColorMapType {
    RAINBOW
};

struct BoundingBox {
    Vec3 x_min, x_max;
    scalar get_max_dim() const {
        assert(x_max.x() >= x_min.x());
        assert(x_max.y() >= x_min.y());
        assert(x_max.z() >= x_min.z());
        const scalar dimension_max = (x_max - x_min).maxCoeff();
        assert(dimension_max > SMALL_SCALAR);
        return dimension_max;
    }
};
} // namespace sito