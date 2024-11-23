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

// class tri : public hittable
// {
// public:
//     tri(const point3 &v0, const point3 &v1, const point3 &v2,
//         shared_ptr<material> m, const vec3 &uv0, const vec3 &uv1, const vec3 &uv2)
//         : v0(v0), v1(v1), v2(v2), mat_ptr(m), uv0(uv0), uv1(uv1), uv2(uv2)
//     {
//         // Precompute the normal for the triangle
//         normal = unit_vector(cross(v1 - v0, v2 - v0));
//         area = 0.5 * cross(v1 - v0, v2 - v0).length();
//         set_bounding_box();
//     }

//     virtual bool hit(const ray &r, interval ray_t, hit_record &rec) const override
//     {
//         // Edge vectors
//         vec3 edge1 = v1 - v0;
//         vec3 edge2 = v2 - v0;

//         // Compute the determinant
//         vec3 h = cross(r.direction(), edge2);
//         double a = dot(edge1, h);

//         // If the determinant is near zero, the ray is parallel to the triangle
//         if (fabs(a) < 1e-8)
//             return false;

//         double f = 1.0 / a;
//         vec3 s = r.origin() - v0;
//         double u = f * dot(s, h);

//         if (u < 0.0 || u > 1.0)
//             return false;

//         vec3 q = cross(s, edge1);
//         double v = f * dot(r.direction(), q);

//         if (v < 0.0 || u + v > 1.0)
//             return false;

//         // At this stage, we can compute t to find out where the intersection point is on the line
//         double t = f * dot(edge2, q);

//         if (t < ray_t.min || t > ray_t.max)
//             return false;

//         // Set hit record
//         rec.t = t;
//         rec.p = r.at(t);
//         rec.set_face_normal(r, normal);

//         // Interpolate UV coordinates
//         double w = 1.0 - u - v;
//         rec.u = w * uv0.x() + u * uv1.x() + v * uv2.x();
//         rec.v = w * uv0.y() + u * uv1.y() + v * uv2.y();

//         rec.mat = mat_ptr;

//         return true;
//     }

//     aabb bounding_box() const override { return bbox; }

//     virtual void set_bounding_box()
//     {
//         // Compute the bounding box of the triangle vertices.
//         point3 min_point(fmin(v0.x(), fmin(v1.x(), v2.x())),
//                          fmin(v0.y(), fmin(v1.y(), v2.y())),
//                          fmin(v0.z(), fmin(v1.z(), v2.z())));
//         point3 max_point(fmax(v0.x(), fmax(v1.x(), v2.x())),
//                          fmax(v0.y(), fmax(v1.y(), v2.y())),
//                          fmax(v0.z(), fmax(v1.z(), v2.z())));

//         bbox = aabb(min_point, max_point);
//     }

//     double pdf_value(const point3 &origin, const vec3 &direction) const override
//     {
//         hit_record rec;
//         if (!this->hit(ray(origin, direction), interval(0.001, infinity), rec))
//             return 0;

//         auto distance_squared = rec.t * rec.t * direction.length_squared();
//         auto cosine = std::fabs(dot(direction, rec.normal) / direction.length());

//         return distance_squared / (cosine * area);
//     }

//     vec3 random(const point3 &origin) const override
//     {
//         auto r1 = sqrt(random_double());
//         auto r2 = random_double();
//         auto p = (1 - r1) * v0 + r1 * (1 - r2) * v1 + r1 * r2 * v2;
//         return p - origin;
//     }

// private:
//     point3 v0, v1, v2; // Triangle vertices
//     vec3 normal;       // Precomputed normal
//     shared_ptr<material> mat_ptr;
//     vec3 uv0, uv1, uv2; // UV coordinates for each vertex
//     double area;        // Area of the triangle
//     aabb bbox;
// };

#endif