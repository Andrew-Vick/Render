#ifndef BVH_H
#define BVH_H

#include "aabb.h"
#include "hittable.h"
#include "hittable_list.h"
#include "aabb.h" // Ensure this header is included for surrounding_box function


#include <algorithm>

class bvh_node : public hittable
{
public:
    bvh_node(hittable_list list) : bvh_node(list.objects, 0, list.objects.size())
    {

    }

    bvh_node(std::vector<shared_ptr<hittable>> &objects, size_t start, size_t end)
    {


        // Check for valid indices
        if (start >= end || objects.empty())
        {
            return;
        }

        size_t object_span = end - start;

        if (object_span <= 2)
        {
            // Base case: create leaf node
            left = objects[start];
            right = (object_span == 2) ? objects[start + 1] : objects[start];
        }
        else
        {
            // Compute centroid bounds
            aabb centroid_bounds = aabb::empty;
            for (size_t i = start; i < end; ++i)
            {
                point3 centroid = objects[i]->bounding_box().centroid();
                centroid_bounds = surrounding_box(centroid_bounds, aabb(centroid, centroid));
            }
            int axis = centroid_bounds.longest_axis();

            // Binning parameters
            constexpr int BIN_COUNT = 16;
            struct Bin
            {
                aabb bounds = aabb::empty;
                int count = 0;
            };
            Bin bins[BIN_COUNT];

            // Assign primitives to bins
            for (size_t i = start; i < end; ++i)
            {
                double centroid = objects[i]->bounding_box().centroid()[axis];
                double relative = (centroid - centroid_bounds.axis_interval(axis).min) / centroid_bounds.axis_interval(axis).size();
                int bin_index = std::min(static_cast<int>(relative * BIN_COUNT), BIN_COUNT - 1);
                bins[bin_index].bounds = surrounding_box(bins[bin_index].bounds, objects[i]->bounding_box());
                bins[bin_index].count++;
            }

            // Compute SAH costs
            aabb left_bounds[BIN_COUNT - 1];
            aabb right_bounds[BIN_COUNT - 1];
            int left_counts[BIN_COUNT - 1] = {0};
            int right_counts[BIN_COUNT - 1] = {0};

            aabb left_box = aabb::empty;
            int left_count = 0;
            for (int i = 0; i < BIN_COUNT - 1; ++i)
            {
                left_box = surrounding_box(left_box, bins[i].bounds);
                left_bounds[i] = left_box;
                left_count += bins[i].count;
                left_counts[i] = left_count;
            }

            aabb right_box = aabb::empty;
            int right_count = 0;
            for (int i = BIN_COUNT - 1; i > 0; --i)
            {
                right_box = surrounding_box(right_box, bins[i].bounds);
                right_counts[i - 1] = right_count;
                right_bounds[i - 1] = right_box;
                right_count += bins[i].count;
            }

            // Find the best split
            double min_cost = std::numeric_limits<double>::infinity();
            int min_index = 0;
            for (int i = 0; i < BIN_COUNT - 1; ++i)
            {
                double cost = left_counts[i] * left_bounds[i].surface_area() +
                              right_counts[i] * right_bounds[i].surface_area();
                if (cost < min_cost)
                {
                    min_cost = cost;
                    min_index = i;
                }
            }

            // Partition primitives based on the best split
            auto mid = std::partition(objects.begin() + start, objects.begin() + end,
                                      [&](const shared_ptr<hittable> &obj)
                                      {
                                          double centroid = obj->bounding_box().centroid()[axis];
                                          double relative = (centroid - centroid_bounds.axis_interval(axis).min) / centroid_bounds.axis_interval(axis).size();
                                          int bin_index = std::min(static_cast<int>(relative * BIN_COUNT), BIN_COUNT - 1);
                                          return bin_index <= min_index;
                                      });

            size_t mid_index = mid - objects.begin();

           // Fallback to median split if partitioning fails
            if (mid_index == start || mid_index == end)
            {
                auto comparator = (axis == 0)   ? box_x_compare
                                  : (axis == 1) ? box_y_compare
                                                : box_z_compare;
                std::sort(objects.begin() + start, objects.begin() + end, comparator);
                mid_index = start + object_span / 2;
            }

            left = make_shared<bvh_node>(objects, start, mid_index);
            right = make_shared<bvh_node>(objects, mid_index, end);
        }

        // Compute bounding box for this node
        bbox = surrounding_box(left->bounding_box(), right->bounding_box());
    }

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