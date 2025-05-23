#ifndef MATERIAL_H
#define MATERIAL_H

#include "hittable.h"
#include "texture.h"
#include "pdf.h"

class scatter_record
{
public:
  color attenuation;
  shared_ptr<pdf> pdf_ptr;
  bool skip_pdf;
  ray skip_pdf_ray;
};

class material
{
public:
  virtual ~material() = default;

  virtual color emitted(
      const ray &r_in, const hit_record &rec, double u, double v, const point3 &p) const
  {
    return color(0, 0, 0);
  }

  virtual double scattering_pdf(const ray &r_in, const hit_record &rec, const ray &scattered)
      const
  {
    return 0;
  }

  virtual bool scatter(const ray &r_in, const hit_record &rec, scatter_record &srec) const
  {
    return false;
  }
};

class lambertian : public material
{
public:
  lambertian(const color &albedo) : tex(make_shared<solid_color>(albedo)), normal_map(nullptr) {}

  lambertian(shared_ptr<texture> tex, shared_ptr<texture> normal_map = nullptr)
      : tex(tex), normal_map(normal_map) {}

  bool scatter(const ray &r_in, const hit_record &rec, scatter_record &srec) const override
  {
    // Adjust the normal if a normal map (bump map) is provided
    vec3 normal = rec.normal;
    if (normal_map)
    {
      // Compute texture coordinates (u, v)
      double u = rec.u;
      double v = rec.v;

      // Get the normal perturbation from the normal map
      vec3 perturbation = normal_map->value(u, v, rec.p);

      // Transform the perturbation from tangent space to world space
      vec3 tangent, bitangent;
      compute_tangent_space(rec.normal, tangent, bitangent);
      vec3 perturbation_world = tangent * perturbation.x() + bitangent * perturbation.y() + rec.normal * perturbation.z();
      perturbation = perturbation_world;

      // Adjust and normalize the normal
      normal = unit_vector(normal + perturbation);
    }

    srec.attenuation = tex->value(rec.u, rec.v, rec.p);
    srec.pdf_ptr = make_shared<cosine_pdf>(normal); // Use the adjusted normal
    srec.skip_pdf = false;
    return true;
  }

  double scattering_pdf(const ray &r_in, const hit_record &rec, const ray &scattered) const override
  {
    auto cos_theta = dot(rec.normal, unit_vector(scattered.direction()));
    return cos_theta < 0 ? 0 : cos_theta / pi;
  }

private:
  shared_ptr<texture> tex;
  shared_ptr<texture> normal_map;

  // Function to compute tangent and bitangent vectors
  void compute_tangent_space(const vec3 &normal, vec3 &tangent, vec3 &bitangent) const
  {
    vec3 up = fabs(normal.y()) < 0.999 ? vec3(0, 1, 0) : vec3(1, 0, 0);
    tangent = unit_vector(cross(up, normal));
    bitangent = cross(normal, tangent);
  }
};

class metal : public material
{
public:
  metal(const color &albedo, double fuzz) : albedo(albedo), fuzz(fuzz < 1 ? fuzz : 1) {}

  bool scatter(const ray &r_in, const hit_record &rec, scatter_record &srec) const override
  {
    vec3 reflected = reflect(r_in.direction(), rec.normal);
    reflected = unit_vector(reflected) + (fuzz * random_unit_vector());

    srec.attenuation = albedo;
    srec.pdf_ptr = nullptr;
    srec.skip_pdf = true;
    srec.skip_pdf_ray = ray(rec.p, reflected, r_in.time());

    return true;
  }

private:
  color albedo;
  double fuzz;
};

class dielectric : public material
{
public:
  dielectric(double refraction_index) : refraction_index(refraction_index) {}

  bool scatter(const ray &r_in, const hit_record &rec, scatter_record &srec) const override
  {
    srec.pdf_ptr = nullptr;
    srec.skip_pdf = true;
    double ri = rec.front_face ? (1.0 / refraction_index) : refraction_index;

    vec3 unit_direction = unit_vector(r_in.direction());
    double cos_theta = std::fmin(dot(-unit_direction, rec.normal), 1.0);
    double sin_theta = std::sqrt(1.0 - cos_theta * cos_theta);

    bool cannot_refract = ri * sin_theta > 1.0;
    vec3 direction;

    if (cannot_refract || reflectance(cos_theta, ri) > random_double())
      direction = reflect(unit_direction, rec.normal);
    else
      direction = refract(unit_direction, rec.normal, ri);

    srec.skip_pdf_ray = ray(rec.p, direction, r_in.time());
    srec.attenuation = color(1.0, 1.0, 1.0);
    return true;
  }

private:
  // Refractive index in vacuum or air, or the ratio of the material's refractive index over
  // the refractive index of the enclosing media
  double refraction_index;

  static double reflectance(double cosine, double refraction_index)
  {
    // Use Schlick's approximation for reflectance.
    auto r0 = (1 - refraction_index) / (1 + refraction_index);
    r0 = r0 * r0;
    return r0 + (1 - r0) * std::pow((1 - cosine), 5);
  }
};

class diffuse_light : public material
{
public:
  diffuse_light(shared_ptr<texture> emit, double intensity = 1.0)
      : emit(emit), intensity(intensity) {}

  diffuse_light(const color &emit_color, double intensity = 1.0)
      : emit(make_shared<solid_color>(emit_color)), intensity(intensity) {}

  virtual color emitted(const ray &r_in, const hit_record &rec, double u, double v, const point3 &p) const override
  {
    if(!rec.front_face)
      return color(0, 0, 0);
    return emit->value(u, v, p) * intensity;
  }

  virtual bool scatter(const ray &r_in, const hit_record &rec, scatter_record &srec) const override
  {
    // Rays hitting this material do not scatter; they terminate here
    return false;
  }

private:
  shared_ptr<texture> emit;
  double intensity;
};

/**
 * Volumes:
 * A material that scatters rays in a random direction.
 *
 * One thing this material can't do it handle voids because it assumes
 * that once the ray leaves the boundary it will continue forever outside the boundary.
 */
class isotropic : public material
{
public:
  isotropic(const color &albedo) : tex(make_shared<solid_color>(albedo)) {}
  isotropic(shared_ptr<texture> tex) : tex(tex) {}

  bool scatter(const ray &r_in, const hit_record &rec, scatter_record &srec) const override
  {
    srec.attenuation = tex->value(rec.u, rec.v, rec.p);
    srec.pdf_ptr = make_shared<sphere_pdf>();
    srec.skip_pdf = false;
    return true;
  }

  double scattering_pdf(const ray &r_in, const hit_record &rec, const ray &scattered)
      const override
  {
    return 1 / (4 * pi);
  }

private:
  shared_ptr<texture> tex;
};

#endif