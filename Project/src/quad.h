#ifndef QUAD_H
#define QUAD_H

#include "hittable.h"
#include "hittable_list.h"

class quad : public hittable
{
public:
    quad(const point3 &_corner, const vec3 &_sideA, const vec3 &_sideB, shared_ptr<material> mat)
        : corner(_corner), sideA(_sideA), sideB(_sideB), mat(mat)
    {
        auto n = cross(sideA, sideB);
        normal = unit_vector(n);
        D = dot(normal, _corner);
        w = n / dot(n, n);

        area = n.length();

        set_bounding_box();
    }

    virtual void set_bounding_box()
    {
        // Compute the bounding box of all four vertices.
        auto bbox_diagonal1 = aabb(corner, corner + sideA + sideB);
        auto bbox_diagonal2 = aabb(corner + sideA, corner + sideB);
        bbox = aabb(bbox_diagonal1, bbox_diagonal2);
    }

    aabb bounding_box() const override { return bbox; }

    bool hit(const ray &r, interval ray_t, hit_record &rec) const override
    {
        auto denom = dot(normal, r.direction());

        // No hit if the ray is parallel to the plane.
        if (std::fabs(denom) < 1e-8)
            return false;

        // Return false if the hit point parameter t is outside the ray interval.
        auto t = (D - dot(normal, r.origin())) / denom;
        if (!ray_t.contains(t))
            return false;

        // Determine if the hit point lies within the planar shape using its plane coordinates.
        auto intersection = r.at(t);
        vec3 planar_hitpt_vector = intersection - corner;
        auto alpha = dot(w, cross(planar_hitpt_vector, sideB));
        auto beta = dot(w, cross(sideA, planar_hitpt_vector));

        if (!is_interior(alpha, beta, rec))
            return false;

        // Ray hits the 2D shape; set the rest of the hit record and return true.
        rec.t = t;
        rec.p = intersection;
        rec.mat = mat;
        rec.set_face_normal(r, normal);

        return true;
    }

    virtual bool is_interior(double a, double b, hit_record &rec) const
    {
        interval unit_interval = interval(0, 1);
        // Given the hit point in plane coordinates, return false if it is outside the
        // primitive, otherwise set the hit record UV coordinates and return true.

        if (!unit_interval.contains(a) || !unit_interval.contains(b))
            return false;

        rec.u = a;
        rec.v = b;
        return true;
    }

    double pdf_value(const point3 &origin, const vec3 &direction) const override
    {
        hit_record rec;
        if (!this->hit(ray(origin, direction), interval(0.001, infinity), rec))
            return 0;

        auto distance_squared = rec.t * rec.t * direction.length_squared();
        auto cosine = std::fabs(dot(direction, rec.normal) / direction.length());

        return distance_squared / (cosine * area);
    }

    vec3 random(const point3 &origin) const override
    {
        auto p = corner + (random_double() * sideA) + (random_double() * sideB);
        return p - origin;
    }

protected:
    point3 corner;
    vec3 sideA, sideB;
    vec3 w;
    shared_ptr<material> mat;
    aabb bbox;
    vec3 normal;
    double D;
    double area;
};

inline shared_ptr<hittable_list> box(const point3 &a, const point3 &b, shared_ptr<material> mat)
{
    // Returns the 3D box (six sides) that contains the two opposite vertices a & b.

    auto sides = make_shared<hittable_list>();

    // Construct the two opposite vertices with the minimum and maximum coordinates.
    auto min = point3(std::fmin(a.x(), b.x()), std::fmin(a.y(), b.y()), std::fmin(a.z(), b.z()));
    auto max = point3(std::fmax(a.x(), b.x()), std::fmax(a.y(), b.y()), std::fmax(a.z(), b.z()));

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
    tri(const point3 &o, const vec3 &aa, const vec3 &ab, shared_ptr<material> m, const vec3 &uv0, const vec3 &uv1, const vec3 &uv2)
        : quad(o, aa, ab, m), uv0(uv0), uv1(uv1), uv2(uv2)
    {
    }

    virtual bool is_interior(double a, double b, hit_record &rec) const override
    {
        if ((a < 0) || (b < 0) || (a + b > 1))
            return false;

        double c = 1.0 - a - b;

        // Interpolate UV coordinates
        rec.u = uv0.x() * c + uv1.x() * a + uv2.x() * b;
        rec.v = uv0.y() * c + uv1.y() * a + uv2.y() * b;
        return true;
    }

private:
    vec3 uv0, uv1, uv2;
};

class triangle : public quad
{
public:
    triangle(const point3 &o, const vec3 &aa, const vec3 &ab, shared_ptr<material> m)
        : quad(o, aa, ab, m)
    {
    }

    virtual bool is_interior(double a, double b, hit_record &rec) const override
    {
        if ((a < 0) || (b < 0) || (a + b > 1))
            return false;

        rec.u = a;
        rec.v = b;
        return true;
    }
};

#endif