#ifndef QUAD_H
#define QUAD_H

#include "rtweekend.h"

#include "hittable.h"
#include "hittable_list.h"

#include <cmath>

class quad : public hittable
{
public:
    quad(const point3 &_corner, const vec3 &_sideA, const vec3 &_sideB, shared_ptr<material> m)
        : corner(_corner), side_A(_sideA), side_B(_sideB), mat(m)
    {
        auto n = cross(side_A, side_B);
        normal = unit_vector(n);
        D = -dot(normal, corner);
        w = n / dot(n, n);

        set_bounding_box();
    }

    virtual void set_bounding_box()
    {
        bbox = aabb(corner, corner + side_A + side_B).pad();
    }

    aabb bounding_box() const override { return bbox; }

    virtual bool hit_ab(double a, double b, hit_record &rec) const
    {
        // Given the hit point in plane coordinates, return false if it is outside the
        // primitive, otherwise set the hit record UV coordinates and return true.

        if ((a < 0) || (1 < a) || (b < 0) || (1 < b))
            return false;

        rec.u = a;
        rec.v = b;
        return true;
    }

    bool hit(const ray &r, interval ray_t, hit_record &rec) const override
    {
        auto denom = dot(normal, r.direction());

        // No hit if the ray is parallel to the plane.
        if (fabs(denom) < 1e-8)
            return false;

        // Return false if the hit point parameter t is outside the ray interval.
        auto t = (-D - dot(normal, r.origin())) / denom;
        if (!ray_t.contains(t))
            return false;

        // Determine the hit point lies within the planar shape using its plane coordinates.
        auto intersection = r.at(t);
        vec3 planar_hitpt_vector = intersection - corner;
        auto a = dot(w, cross(planar_hitpt_vector, side_B));
        auto b = dot(w, cross(side_A, planar_hitpt_vector));

        if (!hit_ab(a, b, rec))
            return false;

        // Ray hits the 2D shape; set the rest of the hit record and return true.
        rec.t = t;
        rec.p = intersection;
        rec.mat = mat;
        rec.set_face_normal(r, normal);

        return true;
    }

protected:
    point3 corner;
    vec3 side_A, side_B;
    shared_ptr<material> mat;
    vec3 normal;
    double D;
    vec3 w;
    aabb bbox;
};

inline shared_ptr<hittable_list> box(const point3 &a, const point3 &b, shared_ptr<material> mat)
{
    // Returns the 3D box (six sides) that contains the two opposite vertices a & b.

    auto sides = make_shared<hittable_list>();

    // Construct the two opposite vertices with the minimum and maximum coordinates.
    auto min = point3(fmin(a.x(), b.x()), fmin(a.y(), b.y()), fmin(a.z(), b.z()));
    auto max = point3(fmax(a.x(), b.x()), fmax(a.y(), b.y()), fmax(a.z(), b.z()));

    auto dx = vec3(max.x() - min.x(), 0, 0);
    auto dy = vec3(0, max.y() - min.y(), 0);
    auto dz = vec3(0, 0, max.z() - min.z());

    sides->add(make_shared<quad>(point3(min.x(), min.y(), max.z()), dx, dy, mat));  // front
    sides->add(make_shared<quad>(point3(max.x(), min.y(), max.z()), -dz, dy, mat)); // right
    sides->add(make_shared<quad>(point3(max.x(), min.y(), min.z()), -dx, dy, mat)); // back
    sides->add(make_shared<quad>(point3(min.x(), min.y(), min.z()), dz, dy, mat));  // left
    sides->add(make_shared<quad>(point3(min.x(), max.y(), max.z()), dx, -dz, mat)); // top
    sides->add(make_shared<quad>(point3(min.x(), min.y(), min.z()), dx, dz, mat));  // bottom

    return sides;
}

class tri : public quad
{
public:
    tri(const point3 &p0, const point3 &p1, const point3 &p2, shared_ptr<material> m)
        : quad(p0, p1 - p0, p2 - p0, m)
    {
    }

    virtual bool hit_ab(double a, double b, hit_record &rec) const override
    {
        if ((a < 0) || (b < 0) || (a + b > 1))
            return false;

        rec.u = a;
        rec.v = b;
        return true;
    }
};

#endif