#ifndef POST_PROCESS_H
#define POST_PROCESS_H

#include "color.h"
#include <vector>
#include <numeric> // For std::accumulate
#include <cmath>   // For std::sqrt and std::exp

class post_process
{
public:
    // Apply bloom and glare effects to an HDR image
    // Adjusted apply_bloom function
    void apply_bloom(std::vector<color> &hdr_image, int width, int height)
    {
        const double threshold = 100.0;       // Lower threshold to include more lights
        const double bloom_intensity = 0.01; // Adjust as needed
        const double glare_intensity = 0.05; // Adjust as needed

        // Step 1: Calculate maximum luminance (optional adjustment)
        // double max_luminance = calculate_max_luminance(hdr_image);

        // Step 2: Extract bright areas
        std::vector<color> bright_pass = extract_bright_areas(hdr_image, threshold);

        // Step 3: Apply Gaussian blur for bloom effect
        std::vector<color> blurred_image = gaussian_blur(bright_pass, width, height, 10);

        // Step 4: Create star-shaped kernel for glare and apply it
        const int kernel_size = 31; // Increased size for longer streaks
        std::vector<double> star_kernel = create_star_kernel(kernel_size);
        std::vector<color> glare_image = convolve(bright_pass, star_kernel, width, height, kernel_size);

        // Step 5: Combine effects without scaling by max_luminance
        combine_effects(hdr_image, blurred_image, glare_image, bloom_intensity, glare_intensity);
    }

private:
    // Calculate the maximum luminance in the image
    double calculate_max_luminance(const std::vector<color> &image)
    {
        double max_luminance = 0.0;
        for (const auto &pixel : image)
        {
            double luminance = 0.2126 * pixel.x() + 0.7152 * pixel.y() + 0.0722 * pixel.z();
            if (luminance > max_luminance)
                max_luminance = luminance;
        }
        return max_luminance;
    }

    // Extract bright areas based on a luminance threshold
    std::vector<color> extract_bright_areas(const std::vector<color> &image, double threshold)
    {
        std::vector<color> bright_pass(image.size());
        for (size_t i = 0; i < image.size(); ++i)
        {
            double luminance = image[i].length();
            bright_pass[i] = (luminance > threshold) ? image[i] : color(0, 0, 0);
        }
        return bright_pass;
    }

    // Combine bloom and glare effects into the original image
    void combine_effects(
        std::vector<color> &image,
        const std::vector<color> &blurred_image,
        const std::vector<color> &glare_image,
        double bloom_intensity,
        double glare_intensity)
    {
        for (size_t i = 0; i < image.size(); ++i)
        {
            // Removed intensity_factor to make effects more prominent
            image[i] += blurred_image[i] * bloom_intensity;
            image[i] += glare_image[i] * glare_intensity;
        }
    }

    // Adjusted create_star_kernel function
    std::vector<double> create_star_kernel(int kernel_size)
    {
        std::vector<double> star_kernel(kernel_size * kernel_size, 0.0);
        int center = kernel_size / 2;

        // Increase weights along streak directions
        for (int i = 0; i < kernel_size; ++i)
        {
            star_kernel[center * kernel_size + i] = 1.0;                 // Horizontal
            star_kernel[i * kernel_size + center] = 1.0;                 // Vertical
            star_kernel[i * kernel_size + i] += 1.0;                     // Diagonal
            star_kernel[(kernel_size - 1 - i) * kernel_size + i] += 1.0; // Anti-diagonal
        }

        // Adjust Gaussian falloff for longer streaks
        double sigma = kernel_size / 10.0; // Smaller sigma for slower falloff
        for (int y = 0; y < kernel_size; ++y)
        {
            for (int x = 0; x < kernel_size; ++x)
            {
                int dx = x - center;
                int dy = y - center;
                double distance = std::sqrt(dx * dx + dy * dy);
                double weight = std::exp(-(distance * distance) / (2 * sigma * sigma));
                star_kernel[y * kernel_size + x] *= weight;
            }
        }

        // Normalize the kernel
        double sum = std::accumulate(star_kernel.begin(), star_kernel.end(), 0.0);
        for (double &value : star_kernel)
        {
            value /= sum;
        }

        return star_kernel;
    }

    // Convolve an image with a given kernel
    std::vector<color> convolve(
        const std::vector<color> &image,
        const std::vector<double> &kernel,
        int width,
        int height,
        int kernel_size)
    {
        std::vector<color> output(image.size());
        int k_center = kernel_size / 2;

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                color sum(0, 0, 0);
                for (int ky = 0; ky < kernel_size; ++ky)
                {
                    int py = y + ky - k_center;
                    if (py < 0 || py >= height)
                        continue;

                    for (int kx = 0; kx < kernel_size; ++kx)
                    {
                        int px = x + kx - k_center;
                        if (px < 0 || px >= width)
                            continue;

                        double weight = kernel[ky * kernel_size + kx];
                        sum += image[py * width + px] * weight;
                    }
                }
                output[y * width + x] = sum;
            }
        }
        return output;
    }

    // Apply a Gaussian blur to the image
    std::vector<color> gaussian_blur(const std::vector<color> &image, int width, int height, int radius)
    {
        // Horizontal and vertical passes
        std::vector<color> temp_image(image.size());
        std::vector<color> blurred_image(image.size());

        // Generate Gaussian kernel
        std::vector<double> kernel = create_gaussian_kernel(radius);

        // Horizontal blur
        horizontal_blur(image, temp_image, width, height, kernel, radius);

        // Vertical blur
        vertical_blur(temp_image, blurred_image, width, height, kernel, radius);

        return blurred_image;
    }

    // Horizontal blur pass
    void horizontal_blur(
        const std::vector<color> &input,
        std::vector<color> &output,
        int width,
        int height,
        const std::vector<double> &kernel,
        int radius)
    {
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                color sum(0, 0, 0);
                double weight_sum = 0.0;

                for (int k = -radius; k <= radius; ++k)
                {
                    int sample_x = x + k;
                    if (sample_x >= 0 && sample_x < width)
                    {
                        double weight = kernel[k + radius];
                        sum += input[y * width + sample_x] * weight;
                        weight_sum += weight;
                    }
                }
                output[y * width + x] = sum / weight_sum;
            }
        }
    }

    // Vertical blur pass
    void vertical_blur(
        const std::vector<color> &input,
        std::vector<color> &output,
        int width,
        int height,
        const std::vector<double> &kernel,
        int radius)
    {
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                color sum(0, 0, 0);
                double weight_sum = 0.0;

                for (int k = -radius; k <= radius; ++k)
                {
                    int sample_y = y + k;
                    if (sample_y >= 0 && sample_y < height)
                    {
                        double weight = kernel[k + radius];
                        sum += input[sample_y * width + x] * weight;
                        weight_sum += weight;
                    }
                }
                output[y * width + x] = sum / weight_sum;
            }
        }
    }

    // Create a Gaussian kernel
    std::vector<double> create_gaussian_kernel(int radius)
    {
        const double sigma = radius / 2.0;
        std::vector<double> kernel(2 * radius + 1);
        double sum = 0.0;

        for (int i = -radius; i <= radius; ++i)
        {
            double value = std::exp(-(i * i) / (2 * sigma * sigma));
            kernel[i + radius] = value;
            sum += value;
        }

        // Normalize the kernel
        for (double &value : kernel)
        {
            value /= sum;
        }

        return kernel;
    }
};

#endif
