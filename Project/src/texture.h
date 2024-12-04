#ifndef TEXTURE_H
#define TEXTURE_H

#include "perlin.h"
#include "rtw_stb_image.h"
#include <array>

class texture
{
public:
  virtual ~texture() = default;

  virtual color value(double u, double v, const point3 &p) const = 0;

  // New overload for sampling based on direction
  virtual color value(const vec3 &direction) const
  {
    // Default implementation returns black
    return color(0, 0, 0);
  }
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
    if (image.height() <= 0)
      return color(0, 1, 1); // Debugging aid

    u = interval(0, 1).clamp(u);
    v = 1.0 - interval(0, 1).clamp(v); // Flip V to image coordinates

    int i = static_cast<int>(u * image.width());
    int j = static_cast<int>(v * image.height());

    i = std::min(i, image.width() - 1);
    j = std::min(j, image.height() - 1);

    const float *pixel = image.pixel_data(i, j);

    return color(pixel[0], pixel[1], pixel[2]);
  }

private:
  rtw_image image;
};

class noise_texture : public texture
{
public:
  noise_texture(double scale) : scale(scale) {}

  color value(double u, double v, const point3 &p) const override
  {
    return color(.5, .5, .5) * (1 + std::sin(scale * p.z() + 10 * noise.turb(p, 7)));
  }

private:
  perlin noise;
  double scale;
};

class cube_map_texture : public texture
{
public:
  explicit cube_map_texture(const std::array<std::string, 6> &filenames)
  {
    for (int i = 0; i < 6; ++i)
    {
      faces[i] = make_unique<rtw_image>(filenames[i].c_str());
    }
  }

  color value(const vec3 &direction) const override
  {
    vec3 dir = unit_vector(direction);
    int face_index;
    double u, v;
    get_cube_face_and_uv(dir, face_index, u, v);
    return sample_face(*faces[face_index], u, v);
  }

  // Implement the pure virtual method from the base class
  color value(double u, double v, const point3 &p) const override
  {
    // This method is not used for cube maps, so we can return a default color or handle it appropriately
    return color(0, 0, 0);
  }

private:
  std::array<std::unique_ptr<rtw_image>, 6> faces;

  void get_cube_face_and_uv(const vec3 &dir, int &face_index, double &u, double &v) const
  {
    double abs_x = fabs(dir.x());
    double abs_y = fabs(dir.y());
    double abs_z = fabs(dir.z());

    int is_x_positive = dir.x() > 0 ? 1 : 0;
    int is_y_positive = dir.y() > 0 ? 1 : 0;
    int is_z_positive = dir.z() > 0 ? 1 : 0;

    double max_axis, uc, vc;

    if (is_x_positive && abs_x >= abs_y && abs_x >= abs_z)
    {
      max_axis = abs_x;
      uc = -dir.z();
      vc = dir.y();
      face_index = 0; // Right
    }
    else if (!is_x_positive && abs_x >= abs_y && abs_x >= abs_z)
    {
      max_axis = abs_x;
      uc = dir.z();
      vc = dir.y();
      face_index = 1; // Left
    }
    else if (is_y_positive && abs_y >= abs_x && abs_y >= abs_z)
    {
      max_axis = abs_y;
      uc = dir.x();
      vc = -dir.z();
      face_index = 2; // Top
    }
    else if (!is_y_positive && abs_y >= abs_x && abs_y >= abs_z)
    {
      max_axis = abs_y;
      uc = dir.x();
      vc = dir.z();
      face_index = 3; // Bottom
    }
    else if (is_z_positive && abs_z >= abs_x && abs_z >= abs_y)
    {
      max_axis = abs_z;
      uc = dir.x();
      vc = dir.y();
      face_index = 4; // Front
    }
    else
    {
      max_axis = abs_z;
      uc = -dir.x();
      vc = dir.y();
      face_index = 5; // Back
    }

    u = 0.5f * (uc / max_axis + 1.0f);
    v = 0.5f * (vc / max_axis + 1.0f);
  }

  color sample_face(const rtw_image &image, double u, double v) const
  {
    u = interval(0, 1).clamp(u);
    v = interval(0, 1).clamp(v);

    int i = static_cast<int>(u * image.width());
    int j = static_cast<int>((1 - v) * image.height());

    i = std::min(i, image.width() - 1);
    j = std::min(j, image.height() - 1);

    const float *pixel = image.pixel_data(i, j);

    return color(pixel[0], pixel[1], pixel[2]);
  }
};

class grass_texture : public texture
{
public:
  grass_texture() {}
  grass_texture(double sc) : scale(sc) {}

  virtual color value(double u, double v, const point3 &p) const override
  {
    // Generate Perlin noise value
    double noise_value = 0.5 * (1 + noise.noise(scale * p));

    // Map the noise value to a grass color
    color grass_color = (1 - noise_value) * color(0.1, 0.4, 0.1) + noise_value * color(0.2, 0.5, 0.2);

    return grass_color;
  }

public:
  perlin noise;
  double scale;
};

class bump_texture : public texture
{
public:
  bump_texture() {}
  bump_texture(double sc, double amp) : scale(sc), amplitude(amp) {}

  // Returns a vector to perturb the normal
  virtual color value(double u, double v, const point3 &p) const override
  {
    double noise_value = noise.noise(scale * p);
    return color(1, 1, 1) * noise_value * amplitude;
  }

public:
  perlin noise;
  double scale;
  double amplitude;
};

class normal_map_texture : public texture
{
public:
  normal_map_texture() {}
  normal_map_texture(double sc, double amp) : scale(sc), amplitude(amp) {}

  virtual color value(double u, double v, const point3 &p) const override
  {
    // Generate noise value for perturbation
    double noise_value = noise.noise(scale * p);
    // Compute the perturbation vector in tangent space
    return amplitude * vec3(noise_value, noise_value, noise_value);
  }

private:
  perlin noise;
  double scale;
  double amplitude;
};

#endif