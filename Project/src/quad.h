#ifndef QUAD_H
#define QUAD_H

#include "hittable.h"
#include "hittable_list.h"

/**
 * Ray-Plane Intersection Notes:
 *
 * Ray-quad interections can be determined in these three steps:
 *
 * 1. Find the plane that contains the quad.
 * 2. Solce for the intersection of a ray and the quad-containg plane.
 * 3. Determin if the hit point lies inside the quad.
 *
 * To solve problem 2, we can start by looking at the implicit formula of a plane which is:
 *
 *     Ax + By +Cz + D = 0 => Ax + By + Cz = D
 *
 * Where A, B, and C are the components of the plane's vector(x,y,z), and D is some constant we can sovle for.
 * To solve for D, we can use the DOT product by taking our orignial equation and translating it into vectors
 * Which will look like this:
 *
 *    n * v = D
 *
 * To find the intersection with some ray we use R(t) = P + td, we can rewrite our equation for D as such:
 *
 *   n * (P + td) = D
 *
 * Solving for t, we get:
 *
 *  t = (D - n * P) / (n * d)
 *
 * A nice trick we can employ is if the denominator is zero, then the ray is parallel to the plane and will never intersect.
 * According to the last sentence of this chapter we can also use this approach to test any planar primitive.
 *
 * To find the coordinates of the intersection point, we can use the euqation below for any
 * Arbitrary point P on the plane:
 *
 *  P = Q + alpha * u +  delta * v
 *
 *  alpha = w * (p x v)  (x = cross product)
 *  delta = w * (u x p)  (* = dot product)
 * 
 * 
 */

class quad : public hittable
{
public:
    quad(const point3 &Q, const vec3 &u, const vec3 &v, shared_ptr<material> mat)
        : Q(Q), u(u), v(v), mat(mat)
    {
        auto n = cross(u, v);
        normal = unit_vector(n);
        D = dot(normal, Q);
        w = n / dot(n,n);
        set_bounding_box();
    }

    virtual void set_bounding_box()
    {
        // Compute the bounding box of all four vertices.
        auto bbox_diagonal1 = aabb(Q, Q + u + v);
        auto bbox_diagonal2 = aabb(Q + u, Q + v);
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
        vec3 planar_hitpt_vector = intersection - Q;
        auto alpha = dot(w, cross(planar_hitpt_vector, v));
        auto delta = dot(w, cross(u, planar_hitpt_vector));

        if (!is_interior(alpha, delta, rec))
            return false;

        // Ray hits the 2D shape; set the rest of the hit record and return true
        rec.t = t;
        rec.p = intersection;
        rec.mat = mat;
        rec.set_face_normal(r, normal);

        return true;
    }


    /**
     * @brief Determine if the hit point lies within the planar shape.
     * We can also use this function to check other planar primitives.
     * if we set it to be true if sqrt(alpha^2 + delta^2) < r, then we can use it to check
     * if a hit point lies within a disk primitive.
     */
    virtual bool is_interior(double alpha, double delta, hit_record& rec) const
    {
        interval unit_interval = interval(0,1);
        // Given the hit point in plane coordinates, return false if its outside the
        // primitive, otherwise set the hit record UV coordinates and return true.

        if (!unit_interval.contains(alpha) || !unit_interval.contains(delta))
            return false;
        
        rec.u = alpha;
        rec.v = delta;
        return true;
    }

private:
    point3 Q;
    vec3 u, v;
    vec3 w;
    shared_ptr<material> mat;
    aabb bbox;
    vec3 normal;
    double D;
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

#endif