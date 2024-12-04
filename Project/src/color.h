#ifndef COLOR_H
#define COLOR_H

#include "interval.h"
#include "vec3.h"

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

// In color.h or utilities.h
vec3 wavelength_to_rgb(double wavelength)
{
    double R = 0.0, G = 0.0, B = 0.0;

    if (wavelength >= 380 && wavelength <= 440)
    {
        R = -(wavelength - 440) / (440 - 380);
        G = 0.0;
        B = 1.0;
    }
    else if (wavelength > 440 && wavelength <= 490)
    {
        R = 0.0;
        G = (wavelength - 440) / (490 - 440);
        B = 1.0;
    }
    else if (wavelength > 490 && wavelength <= 510)
    {
        R = 0.0;
        G = 1.0;
        B = -(wavelength - 510) / (510 - 490);
    }
    else if (wavelength > 510 && wavelength <= 580)
    {
        R = (wavelength - 510) / (580 - 510);
        G = 1.0;
        B = 0.0;
    }
    else if (wavelength > 580 && wavelength <= 645)
    {
        R = 1.0;
        G = -(wavelength - 645) / (645 - 580);
        B = 0.0;
    }
    else if (wavelength > 645 && wavelength <= 780)
    {
        R = 1.0;
        G = 0.0;
        B = 0.0;
    }

    // Intensity factor for wavelengths at the vision limits
    double factor = 0.0;
    if (wavelength >= 380 && wavelength <= 420)
    {
        factor = 0.3 + 0.7 * (wavelength - 380) / (420 - 380);
    }
    else if (wavelength > 420 && wavelength <= 700)
    {
        factor = 1.0;
    }
    else if (wavelength > 700 && wavelength <= 780)
    {
        factor = 0.3 + 0.7 * (780 - wavelength) / (780 - 700);
    }

    R *= factor;
    G *= factor;
    B *= factor;

    // Gamma correction
    R = pow(R, 0.8);
    G = pow(G, 0.8);
    B = pow(B, 0.8);

    return vec3(R, G, B);
}

// void write_color(std::ostream &out, const color &pixel_color)
// {
//     auto r = pixel_color.x();
//     auto g = pixel_color.y();
//     auto b = pixel_color.z();

//     // Replace NaN components with zero.
//     if (r != r)
//         r = 0.0;
//     if (g != g)
//         g = 0.0;
//     if (b != b)
//         b = 0.0;

//     // Apply a linear to gamma transform for gamma 2
//     r = linear_to_gamma(r);
//     g = linear_to_gamma(g);
//     b = linear_to_gamma(b);

//     // Translate the [0,1] component values to the byte range [0,255].
//     static const interval intensity(0.000, 0.999);
//     int rbyte = int(256 * intensity.clamp(r));
//     int gbyte = int(256 * intensity.clamp(g));
//     int bbyte = int(256 * intensity.clamp(b));

//     // Write out the pixel color components.
//     out << rbyte << ' ' << gbyte << ' ' << bbyte << '\n';
// }

// In camera.h or a suitable location

void write_image(std::ostream &out, std::vector<color> &hdr_image)
{
    // Apply tone mapping
    std::vector<color> ldr_image(hdr_image.size());
    for (size_t i = 0; i < hdr_image.size(); ++i)
    {
        ldr_image[i] = tone_map(hdr_image[i]);
    }

    // Save the LDR image to a file
    for (const auto &pixel : ldr_image)
    {
        int ir = static_cast<int>(256 * clamp(pixel.x(), 0.0, 0.999));
        int ig = static_cast<int>(256 * clamp(pixel.y(), 0.0, 0.999));
        int ib = static_cast<int>(256 * clamp(pixel.z(), 0.0, 0.999));
        out << ir << ' ' << ig << ' ' << ib << '\n';
    }
}

#endif