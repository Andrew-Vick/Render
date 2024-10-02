#ifndef RTW_STB_IMAGE_H
#define RTW_STB_IMAGE_H

// Disable strict warnings for this header from the Microsoft Visual C++ compiler.
#ifdef _MSC_VER
#pragma warning(push, 0)
#endif

#define STB_IMAGE_IMPLEMENTATION
#define STBI_FAILURE_USERMSG
#include "external/stb_image.h"

#include <cstdlib>
#include <iostream>
#include <vector>

class rtw_image
{
public:
    rtw_image() {}

    rtw_image(const char *image_filename)
    {
        load_image(image_filename);
    }

    ~rtw_image()
    {
        delete[] bdata;
        STBI_FREE(fdata);
    }

    bool load_image(const std::string &filename)
    {
        // Try to load as HDR first
        if (load_hdr(filename))
        {
            is_hdr = true;
            return true;
        }

        // If HDR load fails, try to load as LDR
        if (load_ldr(filename))
        {
            is_hdr = false;
            return true;
        }

        std::cerr << "ERROR: Could not load image file '" << filename << "'.\n";
        return false;
    }

    int width() const { return (fdata == nullptr && bdata == nullptr) ? 0 : image_width; }
    int height() const { return (fdata == nullptr && bdata == nullptr) ? 0 : image_height; }

    const float *pixel_data_hdr(int x, int y) const
    {
        // Return the address of the three RGB float values of the pixel at x,y. If there is no image
        // data, returns magenta.
        static float magenta[] = {1.0f, 0.0f, 1.0f};
        if (fdata == nullptr)
            return magenta;

        x = clamp(x, 0, image_width);
        y = clamp(y, 0, image_height);

        return fdata + y * bytes_per_scanline + x * bytes_per_pixel;
    }

    const unsigned char *pixel_data_ldr(int x, int y) const
    {
        // Return the address of the three RGB bytes of the pixel at x,y. If there is no image
        // data, returns magenta.
        static unsigned char magenta[] = {255, 0, 255};
        if (bdata == nullptr)
            return magenta;

        x = clamp(x, 0, image_width);
        y = clamp(y, 0, image_height);

        return bdata + y * bytes_per_scanline + x * bytes_per_pixel;
    }

    bool is_hdr_image() const { return is_hdr; }

private:
    const int bytes_per_pixel = 3;
    float *fdata = nullptr;         // Linear floating point pixel data (HDR)
    unsigned char *bdata = nullptr; // Linear 8-bit pixel data (LDR)
    int image_width = 0;            // Loaded image width
    int image_height = 0;           // Loaded image height
    int bytes_per_scanline = 0;
    bool is_hdr = false; // Flag to indicate if the image is HDR

    bool load_hdr(const std::string &filename)
    {
        // Loads the HDR image data from the given file name. Returns true if the load succeeded.
        auto n = bytes_per_pixel; // Dummy out parameter: original components per pixel
        fdata = stbi_loadf(filename.c_str(), &image_width, &image_height, &n, bytes_per_pixel);
        if (fdata == nullptr)
            return false;

        bytes_per_scanline = image_width * bytes_per_pixel;
        return true;
    }

    bool load_ldr(const std::string &filename)
    {
        // Loads the LDR image data from the given file name. Returns true if the load succeeded.
        auto n = bytes_per_pixel; // Dummy out parameter: original components per pixel
        bdata = stbi_load(filename.c_str(), &image_width, &image_height, &n, bytes_per_pixel);
        if (bdata == nullptr)
            return false;

        bytes_per_scanline = image_width * bytes_per_pixel;
        return true;
    }

    static int clamp(int x, int low, int high)
    {
        // Return the value clamped to the range [low, high).
        if (x < low)
            return low;
        if (x < high)
            return x;
        return high - 1;
    }
};

// Restore MSVC compiler warnings
#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif