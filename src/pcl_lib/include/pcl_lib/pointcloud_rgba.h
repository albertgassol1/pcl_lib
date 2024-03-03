#pragma once

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <vector>
#include "point.h"
#include "pointcloud.h"

namespace pcl_lib
{
    template <typename T> 
        class PointCloudRGBA : public PointCloudBase<T>{
            public:
                PointCloudRGBA () : PointCloudBase<T>(0, false){};

                PointCloudRGBA(const PointCloudRGBA& other) : points(other.points), PointCloudBase<T>(other.num_points, other.fixed_size) {};

                PointCloudRGBA (const std::size_t n_points) : points(n_points), PointCloudBase<T>(n_points, true) {
                    assert(n_points >= 0);
                };

                PointCloudRGBA(const std::vector<pcl_lib::PointRGBA<T>>& pts, const bool fixed)  : points(pts), PointCloudBase<T>(pts.size(), fixed) {};

                void add(const PointRGBA<T>& point) {
                    assert(!fixed_size);
                    points.emplace_back(point);
                    this->num_points++;
                };
                void clear() {
                    points.clear();
                    PointCloudBase<T>::clear();
                };
 
                const pcl_lib::PointRGBA<T>& operator[](const std::size_t i) const {
                    assert(i < this->num_points);
                    return points[i];
                };

                pcl_lib::PointCloudRGBA<T> operator+(const pcl_lib::PointCloudRGBA<T>& other) const {
                    std::vector<pcl_lib::PointRGBA<T>> new_points;
                    new_points.reserve(this->num_points + other.size());
                    new_points.insert(new_points.end(), points.begin(), points.end());
                    new_points.insert(new_points.end(), other.get_points().begin(), other.get_points().end());
                    
                    return pcl_lib::PointCloudRGBA<T>(new_points, false); 
                };
 
                const pcl_lib::PointRGBA<T>& at(const std::size_t i) const {
                    assert(i < this->num_points);
                    return points[i];
                };
 
                const std::vector<pcl_lib::PointRGBA<T>>& get_points() const {
                    return points;
                };
 
                std::vector<pcl_lib::PointRGBA<T>> get_points_copy() const {
                    return std::vector<pcl_lib::PointRGBA<T>>(points);
                };
 
                void translate(const pcl_lib::PointRGBA<T>& translation) {
                    #pragma omp parallel for
                    for (auto &point : points) {
                        point.translate(translation);
                    }
                };
 
                void translate(const std::vector<T>& translation) {
                    #pragma omp parallel for
                    for (auto &point : points) {
                        point.translate(translation);
                    }
                };
 
                void translate(const Eigen::Matrix<T, 3, 1>& translation) {
                    #pragma omp parallel for
                    for (auto &point : points) {
                        point.translate(translation);
                    }
                }

                void rotate(const Eigen::Matrix<T, 3, 3>& rotation) {
                    #pragma omp parallel for
                    for (auto &point : points) {
                        point.rotate(rotation);
                    }
                };

                void rotate(const Eigen::Quaternion<T>& rotation) {
                    #pragma omp parallel for
                    for (auto &point : points) {
                        point.rotate(rotation);
                    }
                };

                void rotate(const Eigen::Matrix<T, 3, 1>& rotation) {
                    #pragma omp parallel for
                    for (auto &point : points) {
                        point.rotate(rotation);
                    }
                };

                void rotate(const std::vector<T>& rotation) {
                    #pragma omp parallel for
                    for (auto &point : points) {
                        point.rotate(rotation);
                    }
                };

                void rotate(const T& angle_x, const T& angle_y, const T& angle_z) {
                    #pragma omp parallel for
                    for (auto &point : points) {
                        point.rotate(angle_x, angle_y, angle_z);
                    }
                };

                void transform(const Eigen::Matrix<T, 4, 4>& transformation) {
                    #pragma omp parallel for
                    for (auto &point : points) {
                        point.transform(transformation);
                    }
                };

                void transform(const Eigen::Matrix<T, 3, 3>& rotation, const Eigen::Matrix<T, 3, 1>& translation) {
                    #pragma omp parallel for
                    for (auto &point : points) {
                        point.transform(rotation, translation);
                    }
                };

                void transform(const Eigen::Quaternion<T>& rotation, const Eigen::Matrix<T, 3, 1>& translation) {
                    #pragma omp parallel for
                    for (auto &point : points) {
                        point.transform(rotation, translation);
                    }
                };

                void transform(const Eigen::Matrix<T, 3, 1>& rotation, const Eigen::Matrix<T, 3, 1>& translation) {
                    #pragma omp parallel for
                    for (auto &point : points) {
                        point.transform(rotation, translation);
                    }
                };

                void transform(const std::vector<T>& rotation, const std::vector<T>& translation) {
                    #pragma omp parallel for
                    for (auto &point : points) {
                        point.transform(rotation, translation);
                    }
                };

                void transform(const T& angle_x, const T& angle_y, const T& angle_z, const T& translation_x, const T& translation_y, const T& translation_z) {
                    #pragma omp parallel for
                    for (auto &point : points) {
                        point.transform(angle_x, angle_y, angle_z, translation_x, translation_y, translation_z);
                    }
                };

                void displace(const T& displacement, const float search_radius = 0.03) {
                    // Get normals
                    pcl::PointCloud<pcl::Normal>::Ptr normals = computeNormals(search_radius);
                    // Displace points in the direction of the normals
                    #pragma omp parallel for
                    for (std::size_t i = 0; i < this->num_points; ++i) {
                        pcl_lib::Point<T> normal = pcl_lib::Point<T>(normals->points[i].normal_x, 
                                                                         normals->points[i].normal_y, 
                                                                         normals->points[i].normal_z);
                        normal.normalize();
                        points[i].translate(normal * displacement);
                    }
                };

                const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr toPcl() const{
                    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
                    for (const auto &point : points){
                        pcl_cloud->push_back(pcl::PointXYZRGBA(point.x, point.y, point.z, point.r, point.g, point.b, point.a));
                    }
                    return pcl_cloud;
                };

                const pcl::PointCloud<pcl::PointXYZ>::Ptr toPclXYZ() const{
                    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                    for (const auto &point : points){
                        pcl_cloud->push_back(pcl::PointXYZ(point.x, point.y, point.z));
                    }
                    return pcl_cloud;
                };

                void setAChannel(const T& value) {
                    #pragma omp parallel for
                    for (auto &point : points) {
                        point.a = value;
                    }
                };

            private:
                std::vector<pcl_lib::PointRGBA<T>> points;

                const pcl::PointCloud<pcl::Normal>::Ptr computeNormals(const float search_radius = 0.03) const{
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = toPclXYZ();
                    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
                    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
                    ne.setInputCloud(cloud);
                    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
                    ne.setSearchMethod(tree);
                    ne.setRadiusSearch(search_radius);
                    ne.compute(*normals);
                    return normals;
                };
    };
} // namespace pcl_lib
