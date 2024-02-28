#pragma once

#include <vector>
#include "point.h"

namespace pcl_lib
{
    template <typename T> 
        class PointCloud {
            public:
                PointCloud () = default;
                PointCloud (const std::size_t n_points) : points(n_points), num_points(n_points), fixed_size(true) {
                    assert(n_points >= 0);
                };
                PointCloud(const std::vector<pcl_lib::Point<T>>& pts, const bool fixed) : points(pts), num_points(pts.size()), fixed_size(fixed) {};

                void add(const pcl_lib::Point<T>& point) {
                    assert(!fixed_size);
                    points.emplace_back(point);
                    num_points++;
                }

                void clear() {
                    points.clear();
                    num_points = 0;
                }

                std::size_t size() const {
                    return num_points;
                }

                const pcl_lib::Point<T>& operator[](const std::size_t i) const {
                    assert(i < num_points);
                    return points[i];
                }

                const pcl_lib::Point<T>& at(const std::size_t i) const {
                    assert(i < num_points);
                    return points[i];
                }

                const std::vector<pcl_lib::Point<T>>& get_points() const {
                    return points;
                }

                bool is_fixed_size() const {
                    return fixed_size;
                }

                void translate(const pcl_lib::Point<T>& translation);

                void translate(const std::vector<T>& translation);

                void translate(const Eigen::Matrix<T, 3, 1>& translation);

                void rotate(const Eigen::Matrix<T, 3, 3>& rotation);

                void rotate(const Eigen::Quaternion<T>& rotation);

                void rotate(const Eigen::Matrix<T, 3, 1>& rotation);

                void rotate(const std::vector<T>& rotation);

                void rotate(const T& angle_x, const T& angle_y, const T& angle_z);

                void transform(const Eigen::Matrix<T, 4, 4>& transformation);

                void transform(const Eigen::Matrix<T, 3, 3>& rotation, const Eigen::Matrix<T, 3, 1>& translation);

                void transform(const Eigen::Quaternion<T>& rotation, const Eigen::Matrix<T, 3, 1>& translation);

                void transform(const Eigen::Matrix<T, 3, 1>& rotation, const Eigen::Matrix<T, 3, 1>& translation);

                void transform(const std::vector<T>& rotation, const std::vector<T>& translation);

                void transform(const T& angle_x, const T& angle_y, const T& angle_z, const T& translation_x, const T& translation_y, const T& translation_z);

                // TODO: Implement displacement wrt to the normals of the points. First compute the normals and then displace the points.

            private:
                std::vector<pcl_lib::Point<T>> points;
                std::size_t num_points{0};
                bool fixed_size{false};
        };
} // namespace pcl_lib
