#pragma once

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <vector>
#include "point.h"

namespace pcl_lib
{
    template <typename T> 
        class PointCloudBase {
            public:
                PointCloudBase () : num_points(0), fixed_size(false) {};

                PointCloudBase(const std::size_t n_points, const bool fixed) : num_points(n_points), fixed_size(fixed) {};

                void clear() {
                    num_points = 0;
                };
 
                std::size_t size() const {
                    return num_points;
                };
 
                bool isFixedSize() const {
                    return fixed_size;
                };
 
                virtual void translate(const std::vector<T>& translation) = 0;
 
                virtual void translate(const Eigen::Matrix<T, 3, 1>& translation) = 0;

                virtual void rotate(const Eigen::Matrix<T, 3, 3>& rotation) = 0;

                virtual void rotate(const Eigen::Quaternion<T>& rotation) = 0;

                virtual void rotate(const Eigen::Matrix<T, 3, 1>& rotation) = 0;

                virtual void rotate(const std::vector<T>& rotation) = 0;

                virtual void rotate(const T& angle_x, const T& angle_y, const T& angle_z) = 0;
                
                virtual void transform(const Eigen::Matrix<T, 4, 4>& transformation) = 0;

                virtual void transform(const Eigen::Matrix<T, 3, 3>& rotation, const Eigen::Matrix<T, 3, 1>& translation) = 0;

                virtual void transform(const Eigen::Quaternion<T>& rotation, const Eigen::Matrix<T, 3, 1>& translation) = 0;

                virtual void transform(const Eigen::Matrix<T, 3, 1>& rotation, const Eigen::Matrix<T, 3, 1>& translation) = 0;

                virtual void transform(const std::vector<T>& rotation, const std::vector<T>& translation) = 0;

                virtual void transform(const T& angle_x, const T& angle_y, const T& angle_z, const T& translation_x, const T& translation_y, const T& translation_z) = 0;

                virtual void displace(const T& displacement, const float search_radius = 0.03) {};

                virtual const pcl::PointCloud<pcl::PointXYZ>::Ptr toPclXYZ() const = 0;

            protected:
                std::size_t num_points{0};
                bool fixed_size{false};

                virtual const pcl::PointCloud<pcl::Normal>::Ptr computeNormals(const float search_radius = 0.03) const {
                    return  pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
                };
    };
} // namespace pcl_lib
