#ifndef COLOR_H
#define COLOR_H

#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "interval.h"
#include "vec3.h"
#include "external/stb_image_write.h"

using color = vec3;

inline double clamp(double x, double min, double max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

inline double linear_to_gamma(double linear_component)
{
    if (linear_component > 0)
        return std::sqrt(linear_component);

    return 0;
}

inline color tone_map(const color &hdr_color)
{
    // Reinhard tone mapping
    return hdr_color / (hdr_color + color(1.0, 1.0, 1.0));
}

inline color normalize_energy(const color &energy)
{
    double max_energy = std::max(energy.x(), std::max(energy.y(), energy.z()));
    if (max_energy > 1.0)
    {
        return energy / max_energy;
    }
    return energy;
}

void write_image(std::ostream &out, std::vector<color> &hdr_image, int image_width, int image_height, bool is_hdr)
{
    if (is_hdr)
    {
        std::vector<float> hdr_data(hdr_image.size() * 3);
        for (size_t i = 0; i < hdr_image.size(); ++i)
        {
            hdr_data[i * 3 + 0] = hdr_image[i].x();
            hdr_data[i * 3 + 1] = hdr_image[i].y();
            hdr_data[i * 3 + 2] = hdr_image[i].z();
        }

        stbi_write_hdr("output.hdr", image_width, image_height, 3, hdr_data.data());
    }
    else
    {
        std::vector<color> ldr_image(hdr_image.size());
        for (size_t i = 0; i < hdr_image.size(); ++i)
        {
            ldr_image[i] = tone_map(hdr_image[i]);
        }
        
        for (const auto &pixel : ldr_image)
        {
            int ir = static_cast<int>(256 * clamp(pixel.x(), 0.0, 0.999));
            int ig = static_cast<int>(256 * clamp(pixel.y(), 0.0, 0.999));
            int ib = static_cast<int>(256 * clamp(pixel.z(), 0.0, 0.999));
            out << ir << ' ' << ig << ' ' << ib << '\n';
        }
    }
}

#endif