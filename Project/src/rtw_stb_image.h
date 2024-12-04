#ifndef RTW_STB_IMAGE_H
#define RTW_STB_IMAGE_H

#define STB_IMAGE_IMPLEMENTATION
#define STBI_FAILURE_USERMSG
#include "external/stb_image.h"

#include <cstdlib>
#include <iostream>

class rtw_image
{
public:
    rtw_image(const char *filename)
    {
        // Determine if the image is HDR or LDR based on file extension or content
        if (is_hdr_file(filename))
        {
            load_hdr_image(filename);
        }
        else
        {
            load_ldr_image(filename);
        }
    }

    const float *pixel_data(int i, int j) const
    {
        // Return pointer to the pixel data at (i, j)
        return &data[(j * width_ + i) * channels_];
    }

    int width() const { return width_; }
    int height() const { return height_; }
    int channels() const { return channels_; }

private:
    int width_, height_, channels_;
    std::vector<float> data; // Pixel data in linear floating-point format

    void load_hdr_image(const char *filename)
    {
        float *hdr_data = stbi_loadf(filename, &width_, &height_, &channels_, 3);
        if (!hdr_data)
        {
            throw std::runtime_error("Failed to load HDR image");
        }
        channels_ = 3;
        data.assign(hdr_data, hdr_data + (width_ * height_ * channels_));
        stbi_image_free(hdr_data);
    }

    void load_ldr_image(const char *filename)
    {
        unsigned char *ldr_data = stbi_load(filename, &width_, &height_, &channels_, 3);
        if (!ldr_data)
        {
            throw std::runtime_error("Failed to load LDR image");
        }
        channels_ = 3;
        data.resize(width_ * height_ * channels_);
        for (int i = 0; i < width_ * height_ * channels_; ++i)
        {
            float value = ldr_data[i] / 255.0f;
            data[i] = srgb_to_linear(value);
        }
        stbi_image_free(ldr_data);
    }

    bool is_hdr_file(const char *filename) const
    {
        std::string fname(filename);
        return ends_with(fname, ".hdr") || ends_with(fname, ".exr");
    }

    float srgb_to_linear(float c) const
    {
        if (c <= 0.04045f)
            return c / 12.92f;
        else
            return powf((c + 0.055f) / 1.055f, 2.4f);
    }

    bool ends_with(const std::string &str, const std::string &suffix) const
    {
        return str.size() >= suffix.size() &&
               str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
    }
};

#endif