#ifndef COLOR_H
#define COLOR_H

#include "interval.h"
#include "vec3.h"

using color = vec3;

inline double linear_to_gamma(double linear_component)
{
    if (linear_component > 0)
    {
        return std::sqrt(linear_component);
    }
    return 0;
}

void write_color(std::ostream &out, const color &pixel_color, int samples_per_pixel)
{
    auto r = pixel_color.x();
    auto g = pixel_color.y();
    auto b = pixel_color.z();

    // Divide the color by the number of samples and apply gamma correction.
    auto scale = 1.0 / samples_per_pixel;
    r = linear_to_gamma(r * scale);
    g = linear_to_gamma(g * scale);
    b = linear_to_gamma(b * scale);

    // Clamp the values to [0, 1] range to avoid overflow
    r = interval(0.0, 1.0).clamp(r);
    g = interval(0.0, 1.0).clamp(g);
    b = interval(0.0, 1.0).clamp(b);

    // Translate the [0,1] component values to the byte range [0,255].
    int rbyte = static_cast<int>(256 * r);
    int gbyte = static_cast<int>(256 * g);
    int bbyte = static_cast<int>(256 * b);

    // Write out the pixel color components.
    out << rbyte << ' ' << gbyte << ' ' << bbyte << '\n';
}

#endif