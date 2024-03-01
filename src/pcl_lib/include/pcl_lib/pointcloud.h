#pragma once

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <vector>
#include "point.h"

namespace pcl_lib
{
    template <typename T> 
        class PointCloud {
            public:
                PointCloud () : num_points(0), fixed_size(false) {};

                PointCloud(const PointCloud& other) : points(other.points), num_points(other.num_points), fixed_size(other.fixed_size) {};

                PointCloud (const std::size_t n_points) : points(n_points), num_points(n_points), fixed_size(true) {
                    assert(n_points >= 0);
                };

                PointCloud(const std::vector<pcl_lib::Point<T>>& pts, const bool fixed)  : points(pts), num_points(pts.size()), fixed_size(fixed) {};

                void add(const Point<T>& point) {
                    assert(!fixed_size);
                    points.emplace_back(point);
                    num_points++;
                };
                void clear() {
                    points.clear();
                    num_points = 0;
                };
 
                std::size_t size() const {
                    return num_points;
                };
 
                const pcl_lib::Point<T>& operator[](const std::size_t i) const {
                    assert(i < num_points);
                    return points[i];
                };

                pcl_lib::PointCloud<T> operator+(const pcl_lib::PointCloud<T>& other) const {
                    std::vector<pcl_lib::Point<T>> new_points;
                    new_points.reserve(num_points + other.size());
                    new_points.insert(new_points.end(), points.begin(), points.end());
                    new_points.insert(new_points.end(), other.get_points().begin(), other.get_points().end());
                    
                    return pcl_lib::PointCloud<T>(new_points, false); 
                };
 
                const pcl_lib::Point<T>& at(const std::size_t i) const {
                    assert(i < num_points);
                    return points[i];
                };
 
                const std::vector<pcl_lib::Point<T>>& get_points() const {
                    return points;
                };
 
                std::vector<pcl_lib::Point<T>> get_points_copy() const {
                    return std::vector<pcl_lib::Point<T>>(points);
                };
 
                bool is_fixed_size() const {
                    return fixed_size;
                };
 
                void translate(const pcl_lib::Point<T>& translation) {
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
                    pcl::PointCloud<pcl::Normal>::Ptr normals = compute_normals(search_radius);
                    // Displace points in the direction of the normals
                    #pragma omp parallel for
                    for (std::size_t i = 0; i < num_points; ++i) {
                        pcl_lib::Point<T> normal = pcl_lib::Point<T>(normals->points[i].normal_x, 
                                                                     normals->points[i].normal_y, 
                                                                     normals->points[i].normal_z);
                        normal.normalize();
                        points[i].translate(normal * displacement);
                    }
                };

                const pcl::PointCloud<pcl::PointXYZ>::Ptr to_pcl() const{
                    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                    for (const auto &point : points){
                        pcl_cloud->push_back(pcl::PointXYZ(point.x, point.y, point.z));
                    }
                    return pcl_cloud;
                };

            private:
                std::vector<pcl_lib::Point<T>> points;
                std::size_t num_points{0};
                bool fixed_size{false};

                const pcl::PointCloud<pcl::Normal>::Ptr compute_normals(const float search_radius = 0.03) const{
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = to_pcl();
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
