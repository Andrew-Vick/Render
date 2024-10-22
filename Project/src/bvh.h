#ifndef BVH_H
#define BVH_H

#include "aabb.h"
#include "hittable.h"
#include "hittable_list.h"


#include <algorithm>


class bvh_node : public hittable
{
public:
    bvh_node(hittable_list list) : bvh_node(list.objects, 0, list.objects.size())
    {
        // There's a C++ subtlety here. This constructor (without span indices) creates an
        // implicit copy of the hittable list, which we will modify. The lifetime of the copied
        // list only extends until this constructor exits. That's OK, because we only need to
        // persist the resulting bounding volume hierarchy.
    }

    bvh_node(std::vector<shared_ptr<hittable>> &objects, size_t start, size_t end)
    {
        bbox = aabb::empty;
        for (size_t object_index = start; object_index < end; object_index++)
            bbox = aabb(bbox, objects[object_index]->bounding_box());

        int axis = bbox.longest_axis();

        auto comparator = (axis == 0)   ? box_x_compare
                          : (axis == 1) ? box_y_compare
                                        : box_z_compare;

        size_t object_span = end - start;

        if (object_span == 1)
        {
            left = right = objects[start];
        }
        else if (object_span == 2)
        {
            left = objects[start];
            right = objects[start + 1];
        }
        else
        {
            std::sort(std::begin(objects) + start, std::begin(objects) + end, comparator);

            auto mid = start + object_span / 2;
            left = make_shared<bvh_node>(objects, start, mid);
            right = make_shared<bvh_node>(objects, mid, end);
        }

    }

    // bvh_node(std::vector<shared_ptr<hittable>> &objects, size_t start, size_t end)
    // {
    //     bbox = compute_bounding_box(objects, start, end); // Precomputed bounding boxes

    //     if (end - start <= 4)
    //     {
    //         // Create leaf node
    //         left = right = objects[start];
    //         return;
    //     }

    //     // Find the best split using SAH or another splitting heuristic
    //     std::pair<size_t, int> result = find_best_split(objects, start, end);
    //     size_t split = result.first;
    //     int axis = result.second;

    //     // Sort objects along the best axis
    //     std::sort(std::begin(objects)+ start, std::begin(objects)+ end,
    //               [axis](const shared_ptr<hittable> &a, const shared_ptr<hittable> &b)
    //               {
    //                   return a->bounding_box().axis_interval(axis).min < b->bounding_box().axis_interval(axis).min;
    //               });

    //     // Recursively build left and right subtrees sequentially
    //     left = std::make_shared<bvh_node>(objects, start, split);
    //     right = std::make_shared<bvh_node>(objects, split, end);
    // }

    bool hit(const ray &r, interval ray_t, hit_record &rec) const override
    {
        if (!bbox.hit(r, ray_t))
            return false;

        bool hit_left = left->hit(r, ray_t, rec);
        bool hit_right = right->hit(r, interval(ray_t.min, hit_left ? rec.t : ray_t.max), rec);

        return hit_left || hit_right;
    }

    aabb bounding_box() const override { return bbox; }

private:
    shared_ptr<hittable> left;
    shared_ptr<hittable> right;
    aabb bbox;

    static bool box_compare(
        const shared_ptr<hittable> a, const shared_ptr<hittable> b, int axis_index)
    {
        auto a_axis_interval = a->bounding_box().axis_interval(axis_index);
        auto b_axis_interval = b->bounding_box().axis_interval(axis_index);
        return a_axis_interval.min < b_axis_interval.min;
    }

    static bool box_x_compare(const shared_ptr<hittable> a, const shared_ptr<hittable> b)
    {
        return box_compare(a, b, 0);
    }

    static bool box_y_compare(const shared_ptr<hittable> a, const shared_ptr<hittable> b)
    {
        return box_compare(a, b, 1);
    }

    static bool box_z_compare(const shared_ptr<hittable> a, const shared_ptr<hittable> b)
    {
        return box_compare(a, b, 2);
    }

    double sah_cost(int left_count, int right_count, double left_area, double right_area)
    {
        return left_count * left_area + right_count * right_area;
    }
};

#endif