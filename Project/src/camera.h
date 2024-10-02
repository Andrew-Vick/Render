#ifndef CAMERA_H
#define CAMERA_H

#include <atomic>
#include <future>
#include <mutex>
#include <thread>

#include "hittable.h"
#include "material.h"

/**
 * @brief A camera class for rendering images.
 * For motion blur, currently this has a simple representation of motion blur
 * To allow for sequence of images to be rendered where the camera or world moves between frames
 * then we would have to add a method to HITTABLE that would allow for all animated objects to setup
 * thier motion during the frame.
 */

class camera
{
public:
  /* Public Camera Parameters Here */
  double aspect_ratio = 1.0;  // Ratio of image width over height
  int image_width = 100;      // Rendered image width in pixel count
  int samples_per_pixel = 10; // Count for random samples for each pixel
  int max_depth = 10;         // Maximum number of ray bounces into scene
  color background;           // Scene background color

  double vfov = 90;                  // Vertical field-of-view in degrees
  point3 lookfrom = point3(0, 0, 0); // Point camera is looking from
  point3 lookat = point3(0, 0, -1);  // Point camera is looking at
  vec3 vup = vec3(0, 1, 0);          // Camera-relative "up" direction

  double defocus_angle = 0; // Variation angle of rays through each pixel
  double focus_dist = 10;   // Distance from camera lookfrom point to plane of perfect focus


/**
 * OLD RENDER CHUNK METHOD -- MULTI THREAD
 * may not need this
 */
  // void render_chunk(const hittable& world, int start_row, int end_row, std::vector<color> &image_data)
  // {
  //   for (int j = start_row; j < end_row; ++j)
  //   {
  //     for (int i = 0; i < image_width; ++i)
  //     {
  //       color pixel_color(0, 0, 0);
  //       for (int s = 0; s < samples_per_pixel; ++s)
  //       {
  //         auto u = (i + random_double()) / (image_width - 1);
  //         auto v = (j + random_double()) / (image_height - 1);
  //         ray r = get_ray(u, v);
  //         pixel_color += ray_color(r, max_depth, world);
  //       }
  //       image_data[j * image_width + i] = pixel_color;
  //     }
  //   }
  // }


/**
 * ORIGINAL RENDER METHOD -- SINGLE THREAD
 */
  // void render(const hittable &world)
  // {
  //   initialize();

  //   std::cout << "P3\n"
  //             << image_width << ' ' << image_height << "\n255\n";

  //   for (int j = 0; j < image_height; j++)
  //   {
  //     std::clog << "\rScanlines remaining: " << (image_height - j) << ' ' << std::flush;
  //     for (int i = 0; i < image_width; i++)
  //     {

  //       color pixel_color(0, 0, 0); // CHANGING VALUES HERE GIVE A HUE

  //       for (int sample = 0; sample < samples_per_pixel; sample++)
  //       {
  //         ray r = get_ray(i, j);
  //         pixel_color += ray_color(r, max_depth, world);
  //       }
  //       write_color(std::cout, pixel_samples_scale * pixel_color);
  //     }
  //   }

  //   std::clog << "\rDone.                 \n";
  // }

/**
 * NEW RENDER METHOD -- MULTI THREAD
 * 
 * This method will render the image in parallel using multiple threads.
 * The image is divided into chunks of rows, and each thread renders a chunk.
 * 
 * IMPROVEMENTS:
 * - add a dynamic task queue to allow for more efficient thread management and load balancing
 * 
 * pseudocode for dynamic task queue:
 * 1. Create task struct (e.g. tile coordinates 16x16 pixels) -- expiriment with different sizes, too small and overhead is too high, too large and load balancing is poor
 * 2. init task queue
 * 3. replace chunk processing with a loop that 
 *   - Locks the queue
 *   - checks if queue is empty
 *   - if not, pops a task from the queue
 *   - unlocks the queue
 *   - processes the task
 * 4. Need thread pool management, where threads are created once and then reused
 */
  std::mutex output_mutex;
  std::atomic<int> lines_rendered{0};

  void render(const hittable &world)
  {
    initialize();

    int num_threads = std::thread::hardware_concurrency();
    int rows_per_thread = image_height / num_threads;

    std::vector<std::future<void>> futures;
    std::vector<color> image_data(image_width * image_height);

    auto render_chunk = [&](int start_row, int end_row)
    {
      for (int j = start_row; j < end_row; ++j)
      {
        for (int i = 0; i < image_width; ++i)
        {
          color pixel_color(0, 0, 0);
          for (int sample = 0; sample < samples_per_pixel; ++sample)
          {
            ray r = get_ray(i, j);
            pixel_color += ray_color(r, max_depth, world);
          }
          image_data[j * image_width + i] = pixel_samples_scale * pixel_color;
        }
        lines_rendered++;
        {
          std::lock_guard<std::mutex> lock(output_mutex);
          int percentage = static_cast<int>(100.0 * lines_rendered / image_height);
          std::clog << "\rRendering: " << percentage << "% complete" << std::flush;
        }
      }
    };

    for (int t = 0; t < num_threads; ++t)
    {
      int start_row = t * rows_per_thread;
      int end_row = (t == num_threads - 1) ? image_height : start_row + rows_per_thread;

      futures.push_back(std::async(std::launch::async, render_chunk, start_row, end_row));
    }

    for (auto &future : futures)
    {
      future.get();
    }

    std::cout << "P3\n"
              << image_width << ' ' << image_height << "\n255\n";
    for (int j = 0; j < image_height; ++j)
    {
      for (int i = 0; i < image_width; ++i)
      {
        write_color(std::cout, image_data[j * image_width + i]);
      }
    }
    std::clog << "\rDone.                 \n";
  }

private:
  /* Private Camera Variables Here */
  int image_height;           // Rendered image height
  double pixel_samples_scale; // Color scale factor for a sum of pixel samples
  point3 center;              // Camera center
  point3 pixel00_loc;         // Location of pixel 0, 0
  vec3 pixel_delta_u;         // Offset to pixel to the right
  vec3 pixel_delta_v;         // Offset to pixel below
  vec3 u, v, w;               // Camera frame basis vectors

  vec3 defocus_disk_u; // Defocus disk horizontal radius
  vec3 defocus_disk_v; // Defocus disk vertical radius

  void initialize()
  {
    image_height = int(image_width / aspect_ratio);
    image_height = (image_height < 1) ? 1 : image_height;

    pixel_samples_scale = 1.0 / samples_per_pixel;

    center = lookfrom;

    // Determine viewport dimensions.
    auto theta = degrees_to_radians(vfov);
    auto h = std::tan(theta / 2);
    auto viewport_height = 2 * h * focus_dist;
    auto viewport_width = viewport_height * (double(image_width) / image_height);

    w = unit_vector(lookfrom - lookat);
    u = unit_vector(cross(vup, w));
    v = cross(w, u);

    // Calculate the vectors across the horizontal and down the vertical viewport edges.
    vec3 viewport_u = viewport_width * u;   // Vector across viewport horizontal edge
    vec3 viewport_v = viewport_height * -v; // Vector down viewport vertical edge

    // Calculate the horizontal and vertical delta vectors from pixel to pixel.
    pixel_delta_u = viewport_u / image_width;
    pixel_delta_v = viewport_v / image_height;

    // Calculate the location of the upper left pixel.
    auto viewport_upper_left = center - (focus_dist * w) - viewport_u / 2 - viewport_v / 2;
    pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);

    // Calculate the camera defocus disk basis vectors.
    auto defocus_radius = focus_dist * std::tan(degrees_to_radians(defocus_angle / 2));
    defocus_disk_u = u * defocus_radius;
    defocus_disk_v = v * defocus_radius;
  }

  ray get_ray(int i, int j) const
  {
    // Construct a camera ray originating from the defocus disk and directed at a randomly
    // sampled point around the pixel location i, j.

    auto offset = sample_square();
    auto pixel_sample = pixel00_loc + ((i + offset.x()) * pixel_delta_u) + ((j + offset.y()) * pixel_delta_v);

    auto ray_origin = (defocus_angle <= 0) ? center : defocus_disk_sample();
    auto ray_direction = pixel_sample - ray_origin;

    auto ray_time = random_double();

    return ray(ray_origin, ray_direction, ray_time);
  }

  vec3 sample_square() const
  {
    // Returns the vector to a random point in the [-.5,-.5]-[+.5,+.5] unit square.
    return vec3(random_double() - 0.5, random_double() - 0.5, 0);
  }

  point3 defocus_disk_sample() const
  {
    // Returns a random point in the camera defocus disk.
    auto p = random_in_unit_disk();
    return center + (p[0] * defocus_disk_u) + (p[1] * defocus_disk_v);
  }

  color ray_color(const ray &r, int depth, const hittable &world) const
  {
    if (depth <= 0)
      return color(0, 0, 0);

    hit_record rec;
    // If the ray hits nothing, return the background color.
    if (!world.hit(r, interval(0.001, infinity), rec))
      return background;

    ray scattered;
    color attenuation;
    color color_from_emission = rec.mat->emitted(rec.u, rec.v, rec.p);

    if (!rec.mat->scatter(r, rec, attenuation, scattered))
      return color_from_emission;

    color color_from_scatter = attenuation * ray_color(scattered, depth - 1, world);

    return color_from_emission + color_from_scatter;
  }

  void write_image(const std::vector<color> &image_data)
  {
    std::cout << "P3\n"
              << image_width << ' ' << image_height << "\n255\n";
    for (int j = 0; j < image_height; ++j)
    {
      for (int i = 0; i < image_width; ++i)
      {
        write_color(std::cout, image_data[j * image_width + i]);
      }
    }
    std::clog << "\rDone.                 \n";
  }
};

#endif
