#ifndef RAY_H
#define RAY_H

#include "vec3.h"

class ray {
  public:
    ray() {}
    
    // Constructor for ray with origin, direction, and time
    // Time is used to determine motion blur by using Monte Carlo integration
    ray(const point3& origin, const vec3& direction, double time, double wavelength =550.0)
      : orig(origin), dir(direction), tm(time), wl(wavelength) {}

    ray(const point3& origin, const vec3& direction)
      : ray(origin, direction, 0) {}

    const point3& origin() const  { return orig; }
    const vec3& direction() const { return dir; }

    double time() const { return tm; }

    double wavelength() const { return wl; }

    point3 at(double t) const {
        return orig + t*dir;
    }

  private:
    point3 orig;
    vec3 dir;
    double tm;
    double wl;
};

#endif