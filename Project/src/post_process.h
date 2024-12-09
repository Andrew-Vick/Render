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
    // Apply bloom and glare effects to an HDR image
    void apply_bloom(std::vector<color> &hdr_image, int width, int height)
    {
        const double threshold = 20; // Lowered threshold for HDR
        std::vector<color> bright_pass = extract_bright_areas(hdr_image, threshold);

        int blur_radius = 2; // Increased blur radius
        std::vector<color> blurred_image = gaussian_blur(bright_pass, width, height, blur_radius);

        int kernel_size = 11; // Larger kernel for more prominent streaks
        std::vector<double> star_kernel = create_star_kernel(kernel_size);
        std::vector<color> glare_image = convolve(bright_pass, star_kernel, width, height, kernel_size);

        combine_effects(hdr_image, blurred_image, glare_image);
    }

private:
    // Extract bright areas based on a luminance threshold
    std::vector<color> extract_bright_areas(const std::vector<color> &image, double threshold)
    {
        std::vector<color> bright_pass(image.size());
        for (size_t i = 0; i < image.size(); ++i)
        {
            double luminance = image[i].length();
            double intensity = smoothstep(threshold, threshold + 10.0, luminance);
            bright_pass[i] = image[i] * intensity;
        }
        return bright_pass;
    }

    double smoothstep(double edge0, double edge1, double x)
    {
        x = (x - edge0) / (edge1 - edge0);
        x = (x < 0.0) ? 0.0 : (x > 1.0) ? 1.0 : x;
        return x * x * (3 - 2 * x);
    }

    // Combine bloom and glare effects into the original image
    void combine_effects(std::vector<color> &image, const std::vector<color> &blurred_image, const std::vector<color> &glare_image)
    {
        double bloom_intensity = 0.5; // Adjust as needed
        double glare_intensity = 0.7; // Adjust as needed
        for (size_t i = 0; i < image.size(); i++)
        {
            image[i] += bloom_intensity * blurred_image[i] + glare_intensity * glare_image[i];
            ;
        }
    }

    // Create a star-shaped kernel for glare effect
    std::vector<double> create_star_kernel(int kernel_size)
    {
        std::vector<double> star_kernel(kernel_size * kernel_size, 0.0);
        int center = kernel_size / 2;

        // Randomly select directions for streaks
        std::vector<std::pair<int, int>> directions = {
            {1, 0}, {0, 1}, {1, 1}, {-1, 1}, {-1, 0}, {0, -1}, {-1, -1}, {1, -1}};

        // Shuffle directions to randomize
        std::random_shuffle(directions.begin(), directions.end());

        // Use a subset of directions for randomness
        int num_streaks = 4; // Adjust for more or fewer streaks
        for (int d = 0; d < num_streaks; ++d)
        {
            int dx = directions[d].first;
            int dy = directions[d].second;

            for (int i = -center; i <= center; ++i)
            {
                int x = center + i * dx;
                int y = center + i * dy;
                if (x >= 0 && x < kernel_size && y >= 0 && y < kernel_size)
                {
                    star_kernel[y * kernel_size + x] = 1.0;
                }
            }
        }

        // Adjust Gaussian falloff for the kernel
        double sigma = kernel_size / 4.0; // Adjust sigma for desired falloff
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

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                color sum(0, 0, 0);
                for (int ky = 0; ky < kernel_size; ky++)
                {
                    int py = y + ky - k_center;
                    if (py < 0 || py >= height)
                        continue;

                    for (int kx = 0; kx < kernel_size; kx++)
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
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                color sum(0, 0, 0);
                double weight_sum = 0.0;

                for (int k = -radius; k <= radius; k++)
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
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                color sum(0, 0, 0);
                double weight_sum = 0.0;

                for (int k = -radius; k <= radius; k++)
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

        for (int i = -radius; i <= radius; i++)
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