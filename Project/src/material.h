#ifndef MATERIAL_H
#define MATERIAL_H

#include "hittable.h"
#include "texture.h"
#include "pdf.h"
#include <cmath>

using namespace std;

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

// class lambertian : public material
// {
// public:
//   lambertian(const color &albedo) : tex(make_shared<solid_color>(albedo)) {}
//   lambertian(shared_ptr<texture> tex) : tex(tex) {}

//   bool scatter(const ray &r_in, const hit_record &rec, scatter_record &srec) const override
//   {
//     srec.attenuation = tex->value(rec.u, rec.v, rec.p);
//     srec.pdf_ptr = make_shared<cosine_pdf>(rec.normal);
//     srec.skip_pdf = false;
//     return true;
//   }

//   double scattering_pdf(const ray &r_in, const hit_record &rec, const ray &scattered)
//       const override
//   {
//     auto cos_theta = dot(rec.normal, unit_vector(scattered.direction()));
//     return cos_theta < 0 ? 0 : cos_theta / pi;
//   }

// private:
//   shared_ptr<texture> tex;
// };

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

// Very rudimentary subsurface scattering material
// Want to improve this to be more physically accurate
class subsurface_scatter : public material
{
public:
  subsurface_scatter(const color &albedo, const color &scattering_col,
                     double absorption_coef, double scattering_coef)
      : albedo(albedo), scattering_col(scattering_col),
        absorption_coef(absorption_coef), scattering_coef(scattering_coef) {}

  bool scatter(const ray &r_in, const hit_record &rec, scatter_record &srec) const override
  {
    double distance = -std::log(random_double()) / (absorption_coef + scattering_coef);
    vec3 scatter_dir = random_unit_vector();
    vec3 scatter_origin = rec.p + scatter_dir * 0.0001;
    double attenuation = std::exp(-(absorption_coef + scattering_coef) * distance);
    srec.attenuation = albedo * scattering_col * attenuation;
    srec.skip_pdf = true;
    srec.skip_pdf_ray = ray(scatter_origin, scatter_dir, r_in.time(), r_in.wavelength());
    return true;
  }

private:
  color albedo;
  color scattering_col;
  double absorption_coef;
  double scattering_coef;
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
    srec.skip_pdf_ray = ray(rec.p, reflected, r_in.time(), r_in.wavelength());

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
      srec.attenuation = color(1.0, 1.0, 1.0);
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

  class dielectric_new : public material
  {
  public:
    dielectric_new(double base_ior, const color &absorption_coeff)
        : base_ior(base_ior), absorption_coeff(absorption_coeff) {}

    bool scatter(const ray &r_in, const hit_record &rec, scatter_record &srec) const override
    {
      srec.attenuation = calculate_attenuation(r_in, rec);
      srec.skip_pdf = true;

      double wavelength = r_in.wavelength();
      double ior = refractive_index(wavelength);
      double refraction_ratio = rec.front_face ? (1.0 / ior) : ior;

      vec3 unit_direction = unit_vector(r_in.direction());
      double cos_theta = fmin(dot(-unit_direction, rec.normal), 1.0);
      double sin_theta = sqrt(1.0 - cos_theta * cos_theta);

      bool cannot_refract = refraction_ratio * sin_theta > 1.0;
      vec3 direction;

      if (cannot_refract || reflectance(cos_theta, refraction_ratio) > random_double())
        direction = reflect(unit_direction, rec.normal);
      else
        direction = refract(unit_direction, rec.normal, refraction_ratio);

      srec.skip_pdf_ray = ray(rec.p, direction, r_in.time(), wavelength);
      return true;
    }

  private:
    double base_ior;
    color absorption_coeff; // Wavelength-dependent absorption coefficients

    color calculate_attenuation(const ray &r_in, const hit_record &rec) const
    {
      // Calculate the distance traveled inside the material
      double distance = (rec.front_face ? 0.0 : -dot(r_in.direction(), rec.normal)) * rec.t;

      // Use Beer-Lambert law for attenuation: I = I0 * exp(-alpha * distance)
      // alpha is the absorption coefficient
      return color(std::exp(-absorption_coeff.x() * distance),
                   std::exp(-absorption_coeff.y() * distance),
                   std::exp(-absorption_coeff.z() * distance));
    }

    double refractive_index(double wavelength) const
    {
      // Cauchy's equation: n(λ) = A + (B / λ^2)
      double lambda = wavelength * 1e-3; // Convert nm to μm
      double A = base_ior;
      double B = 0.004; // Adjust based on material
      return A + B / (lambda * lambda);
    }

    static double reflectance(double cosine, double ref_idx)
    {
      // Schlick's approximation
      double r0 = (1 - ref_idx) / (1 + ref_idx);
      r0 *= r0;
      return r0 + (1 - r0) * pow((1 - cosine), 5);
    }
  };

// class diffuse_light : public material {
// public:
//     diffuse_light(shared_ptr<texture> emit) : emit(emit) {}
//     diffuse_light(const color &emit_color) : emit(make_shared<solid_color>(emit_color)) {}

//     virtual color emitted(const ray& r_in, const hit_record& rec, double u, double v, const point3& p) const override {
//         if (!rec.front_face)
//             return color(0, 0, 0);
//         return emit->value(u, v, p);
//     }

//     virtual bool scatter(const ray& r_in, const hit_record& rec, scatter_record& srec) const override {
//         return false;
//     }

// private:
//     shared_ptr<texture> emit;
// };

// In material.h

// In material.h

class diffuse_light : public material
{
public:
  diffuse_light(shared_ptr<texture> emit, double intensity = 1.0)
      : emit(emit), intensity(intensity) {}

  diffuse_light(const color &emit_color, double intensity = 1.0)
      : emit(make_shared<solid_color>(emit_color)), intensity(intensity) {}

  virtual color emitted(const ray &r_in, const hit_record &rec,
                        double u, double v, const point3 &p) const override
  {
    if (!rec.front_face)
      return color(0, 0, 0);

    return emit->value(u, v, p) * intensity;
  }

  virtual bool scatter(const ray &r_in, const hit_record &rec,
                       scatter_record &srec) const override
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