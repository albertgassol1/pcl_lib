#pragma once

#include <vector>

namespace pcl_lib
{
    template <typename PointT>
        class PointCloud {
            public:
                PointCloud () = default;
                PointCloud (const std::size_t n_points) : points(n_points), num_points(n_points), fixed_size(true) {
                    assert(n_points >= 0);
                };
                PointCloud(const std::vector<PointT>& pts, const bool fixed) : points(pts), num_points(pts.size()), fixed_size(fixed) {};

                void add(const PointT& point) {
                    assert(!fixed_size);
                }

            private:
                std::vector<PointT> points;
                std::size_t num_points;
                bool fixed_size;
        };
} // namespace pcl_lib
