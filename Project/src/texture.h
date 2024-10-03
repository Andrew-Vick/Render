#ifndef TEXTURE_H
#define TEXTURE_H

#include "perlin.h"
#include "rtw_stb_image.h"

class texture
{
public:
  virtual ~texture() = default;

  virtual color value(double u, double v, const point3 &p) const = 0;
};

class solid_color : public texture
{
public:
  solid_color(const color &albedo) : albedo(albedo) {}

  solid_color(double red, double green, double blue) : solid_color(color(red, green, blue)) {}

  color value(double u, double v, const point3 &p) const override
  {
    return albedo;
  }

private:
  color albedo;
};

class checker_texture : public texture
{
public:
  checker_texture(double scale, shared_ptr<texture> even, shared_ptr<texture> odd)
      : inv_scale(1.0 / scale), even(even), odd(odd) {}

  checker_texture(double scale, const color &c1, const color &c2)
      : checker_texture(scale, make_shared<solid_color>(c1), make_shared<solid_color>(c2)) {}

  color value(double u, double v, const point3 &p) const override
  {
    auto xInteger = int(std::floor(inv_scale * p.x()));
    auto yInteger = int(std::floor(inv_scale * p.y()));
    auto zInteger = int(std::floor(inv_scale * p.z()));

    bool isEven = (xInteger + yInteger + zInteger) % 2 == 0;

    return isEven ? even->value(u, v, p) : odd->value(u, v, p);
  }

private:
  double inv_scale;
  shared_ptr<texture> even;
  shared_ptr<texture> odd;
};

class image_texture : public texture
{
public:
  image_texture(const char *filename) : image(filename) {}

  color value(double u, double v, const point3 &p) const override
  {
    // If we have no texture data, then return solid cyan as a debugging aid.
    if (image.height() <= 0)
      return color(0, 1, 1);

    // Clamp input texture coordinates to [0,1] x [1,0]
    u = interval(0, 1).clamp(u);
    v = 1.0 - interval(0, 1).clamp(v); // Flip V to image coordinates

    auto i = int(u * image.width());
    auto j = int(v * image.height());
    auto pixel = image.pixel_data_ldr(i, j);

    auto color_scale = 1.0 / 255.0;
    return color(color_scale * pixel[0], color_scale * pixel[1], color_scale * pixel[2]);
  }

private:
  rtw_image image;
};

class noise_texture : public texture
{
  // scale is the frequency of the noise
  // Higher frequency means more noise
public:
  noise_texture(double scale) : scale(scale) {}

  color value(double u, double v, const point3 &p) const override
  {
    // return color(1, 1, 1) * noise.noise(scale * p);

    // The output of the Perlin_interp is in the range [-1,1].
    // We want to map [-1,1] to [0,1] for color values. We do this by scaling by 0.5 and then shifting by 1 plus the noise value.

    // This is for using using with the Perlin_interp function
    // return color(1, 1, 1) * 0.5 * (1.0 + noise.noise(scale * p));

    // This is for using with the turb function
    // return color(1, 1, 1) * noise.turb(scale * p, 7);

    // This gives you a marble texture
    return color(.5, .5, .5) * (1 + std::sin(scale * p.z() + 10 * noise.turb(p, 7)));
  }

private:
  perlin noise;
  double scale;
};

class cubemap : public texture
{
public:
  // Constructor to handle 6 separate HDR files for cubemap faces
  explicit cubemap(const std::array<std::string, 6> &filenames)
  {
    load_cube_map_faces(filenames);
  }

  color value(double u, double v, const point3 &p) const override
  {
    // Determine which face of the cubemap to sample from based on the direction vector `p`
    int face_index = determine_face(p);

    // Sample the appropriate face of the cubemap
    return sample_face(face_index, u, v);
  }

private:
  std::array<std::shared_ptr<rtw_image>, 6> images;

  // Function to load 6 separate HDR files into the cubemap
  void load_cube_map_faces(const std::array<std::string, 6> &filenames)
  {
    for (int i = 0; i < 6; ++i)
    {
      images[i] = std::make_shared<rtw_image>(filenames[i].c_str());
      if (images[i]->width() == 0 || images[i]->height() == 0)
      {
        std::cerr << "ERROR: Failed to load cubemap face: " << filenames[i] << std::endl;
      }
    }
  }

  // Determine which cubemap face to sample based on direction vector `p`
  int determine_face(const point3 &p) const
  {
    double absX = fabs(p.x()), absY = fabs(p.y()), absZ = fabs(p.z());
    if (absX >= absY && absX >= absZ)
      return (p.x() > 0) ? 0 : 1; // +X or -X
    else if (absY >= absX && absY >= absZ)
      return (p.y() > 0) ? 2 : 3; // +Y or -Y
    else
      return (p.z() > 0) ? 4 : 5; // +Z or -Z
  }

  // Sample a specific cubemap face at the given (u, v) texture coordinates
  color sample_face(int face_index, double u, double v) const
  {
    const auto &image = images[face_index];

    if (image->height() <= 0)
      return color(0, 1, 1); // Debugging aid (cyan)

    // Clamp texture coordinates to [0, 1]
    u = interval(0, 1).clamp(u);
    v = 1.0 - interval(0, 1).clamp(v); // Flip v for image coordinates

    int i = int(u * image->width());
    int j = int(v * image->height());

    // Ensure i and j are within bounds
    if (i >= image->width())
      i = image->width() - 1;
    if (j >= image->height())
      j = image->height() - 1;

    if (image->is_hdr_image())
    {
      // Sample the pixel data from the HDR image
      const float *pixel = image->pixel_data_hdr(i, j);
      float r = pixel[0];
      float g = pixel[1];
      float b = pixel[2];

      // Apply scaling factor for HDR values (adjust as needed)
      float scale = 1.0f; // Adjust this scale factor based on your HDR range
      r *= scale;
      g *= scale;
      b *= scale;

      // Clamp the values to [0, 1] range to avoid overflow
      r = interval(0.0, 1.0).clamp(r);
      g = interval(0.0, 1.0).clamp(g);
      b = interval(0.0, 1.0).clamp(b);

      return color(r, g, b);
    }
    else
    {
      // Sample the pixel data from the LDR image
      const unsigned char *pixel = image->pixel_data_ldr(i, j);
      return color(pixel[0] / 255.0, pixel[1] / 255.0, pixel[2] / 255.0);
    }
  }
};

#endif