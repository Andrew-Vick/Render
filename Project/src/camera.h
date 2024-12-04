#ifndef CAMERA_H
#define CAMERA_H

#include <atomic>
#include <future>
#include <mutex>
#include <thread>

#include "hittable.h"
#include "material.h"
#include "pdf.h"
#include "post_process.h"
#include "thread_pool.h"

/**
 * @brief A camera class for rendering images.
 * For motion blur, currently this has a simple representation of motion blur
 * To allow for sequence of images to be rendered where the camera or world moves between frames
 * then we would have to add a method to HITTABLE that would allow for all animated objects to setup
 * thier motion during the frame.
 */

struct RenderTask
{
  int start_row;
  int end_row;
  int start_col;
  int end_col;
};

class camera
{
public:
  /* Public Camera Parameters Here */
  double aspect_ratio = 1.0;  // Ratio of image width over height
  int image_width = 100;      // Rendered image width in pixel count
  int samples_per_pixel = 10; // Count for random samples for each pixel
  int max_depth = 10;         // Maximum number of ray bounces into scene

  double vfov = 90;                  // Vertical field-of-view in degrees
  point3 lookfrom = point3(0, 0, 0); // Point camera is looking from
  point3 lookat = point3(0, 0, -1);  // Point camera is looking at
  vec3 vup = vec3(0, 1, 0);          // Camera-relative "up" direction

  double defocus_angle = 0; // Variation angle of rays through each pixel
  double focus_dist = 10;   // Distance from camera lookfrom point to plane of perfect focus

  bool is_hdr = false;

  void set_background(shared_ptr<texture> bg_tex, bool hdr)
  {
    is_hdr = hdr;
    background = bg_tex;
  }

  void render(const hittable &world, const hittable &lights, ThreadPool &pool)
  {
    initialize();

    // Create the raster buffer to hold the image data
    std::vector<color> image_data(image_width * image_height);

    // Lambda function to render a chunk of rows
    auto render_chunk = [&](int j)
    {
      for (int i = 0; i < image_width; i++)
      {
        color pixel_color(0, 0, 0);
        for (int s_j = 0; s_j < sqrt_spp; s_j++)
        {
          for (int s_i = 0; s_i < sqrt_spp; s_i++)
          {
              ray r = get_ray(i, j, s_i, s_j);
              pixel_color += ray_color(r, max_depth, world, lights);
            
          }
        }
        image_data[j * image_width + i] = pixel_samples_scale * pixel_color;
      }
    };

    // Enqueue each row rendering task in the thread pool
    for (int j = 0; j < image_height; j++)
    {
      pool.enqueue([=, &world, &lights, &image_data]
                   { render_chunk(j); });
    }

    // Wait until all rows are rendered
    pool.wait_till_done();

    // After rendering, output the rasterized image to a file (or console)
    std::cout << "P3\n"
              << image_width << ' ' << image_height << "\n255\n";

    post_process pp;
    pp.apply_bloom(image_data, image_width, image_height);
    write_image(std::cout, image_data, image_width, image_height, true);
    std::clog << "\rDone.                 \n";
  }

private:
  int image_height;           // Rendered image height
  double pixel_samples_scale; // Color scale factor for a sum of pixel samples
  int sqrt_spp;
  double recip_sqrt_spp;

  point3 center;              // Camera center
  point3 pixel00_loc;         // Location of pixel 0, 0
  vec3 pixel_delta_u;         // Offset to pixel to the right
  vec3 pixel_delta_v;         // Offset to pixel below
  vec3 u, v, w;               // Camera frame basis vectors
  vec3 defocus_disk_u;        // Defocus disk horizontal radius
  vec3 defocus_disk_v;        // Defocus disk vertical radius

  shared_ptr<texture> background;

  void initialize()
  {
    image_height = int(image_width / aspect_ratio);
    image_height = (image_height < 1) ? 1 : image_height;

    sqrt_spp = int(std::sqrt(samples_per_pixel));
    pixel_samples_scale = 1.0 / (sqrt_spp * sqrt_spp);
    recip_sqrt_spp = 1.0 / sqrt_spp;

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

  ray get_ray(int i, int j, int s_i, int s_j) const
  {
    // Construct a camera ray originating from the defocus disk and directed at a randomly
    // sampled point around the pixel location i, j.
    auto offset = sample_square_stratified(s_i, s_j);
    auto pixel_sample = pixel00_loc + ((i + offset.x()) * pixel_delta_u) + ((j + offset.y()) * pixel_delta_v);

    auto ray_origin = (defocus_angle <= 0) ? center : defocus_disk_sample();
    auto ray_direction = pixel_sample - ray_origin;

    auto ray_time = random_double();

    return ray(ray_origin, ray_direction, ray_time);
  }

  vec3 sample_square_stratified(int s_i, int s_j) const
  {
    // Returns the vector to a random point in the square sub-pixel specified by grid
    // indices s_i and s_j, for an idealized unit square pixel [-.5,-.5] to [+.5,+.5].

    auto px = ((s_i + random_double()) * recip_sqrt_spp) - 0.5;
    auto py = ((s_j + random_double()) * recip_sqrt_spp) - 0.5;

    return vec3(px, py, 0);
  }

  vec3 sample_square() const
  {
    // Returns the vector to a random point in the [-.5,-.5]-[+.5,+.5] unit square.
    return vec3(random_double() - 0.5, random_double() - 0.5, 0);
  }

  vec3 sample_disk(double radius) const
  {
    // Returns a random point in the unit (radius 0.5) disk centered at the origin.
    return radius * random_in_unit_disk();
  }

  point3 defocus_disk_sample() const
  {
    // Returns a random point in the camera defocus disk.
    auto p = random_in_unit_disk();
    return center + (p[0] * defocus_disk_u) + (p[1] * defocus_disk_v);
  }

  color ray_color(const ray &r, int depth, const hittable &world, const hittable &lights) const
  {
    if (depth <= 0)
      return color(0.0, 0.0, 0.0);

    hit_record rec;
    // If the ray hits nothing, return the background color.
    if (!world.hit(r, interval(0.001, infinity), rec))
    {
      if (background)
      {
        if(is_hdr){
          return background->value(unit_vector(r.direction()));
        }
        return background->value(0, 0, r.direction());
      }
      return color(0, 0, 0);
    }

    scatter_record srec;
    color color_from_emission = rec.mat->emitted(r, rec, rec.u, rec.v, rec.p);

    if (!rec.mat->scatter(r, rec, srec))
      return color_from_emission;

    if (srec.skip_pdf)
    {
      return srec.attenuation * ray_color(srec.skip_pdf_ray, depth - 1, world, lights);
    }

    auto light_ptr = make_shared<hittable_pdf>(lights, rec.p);
    mixture_pdf p(light_ptr, srec.pdf_ptr);

    ray scattered = ray(rec.p, p.generate(), r.time());
    auto pdf_value = p.value(scattered.direction());

    double scattering_pdf = rec.mat->scattering_pdf(r, rec, scattered);

    color sample_color = ray_color(scattered, depth - 1, world, lights);

    color color_from_scatter =
        (srec.attenuation * scattering_pdf * sample_color ) / pdf_value;

    return color_from_emission + color_from_scatter;
  }
};

#endif