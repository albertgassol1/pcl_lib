#pragma once

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
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
                    for (auto &point : points) {
                        point.translate(translation);
                    }
                };
 
                void translate(const std::vector<T>& translation) {
                    for (auto &point : points) {
                        point.translate(translation);
                    }
                };
 
                void translate(const Eigen::Matrix<T, 3, 1>& translation) {
                    for (auto &point : points) {
                        point.translate(translation);
                    }
                }

                void rotate(const Eigen::Matrix<T, 3, 3>& rotation) {
                    for (auto &point : points) {
                        point.rotate(rotation);
                    }
                };

                void rotate(const Eigen::Quaternion<T>& rotation) {
                    for (auto &point : points) {
                        point.rotate(rotation);
                    }
                };

                void rotate(const Eigen::Matrix<T, 3, 1>& rotation) {
                    for (auto &point : points) {
                        point.rotate(rotation);
                    }
                };

                void rotate(const std::vector<T>& rotation) {
                    for (auto &point : points) {
                        point.rotate(rotation);
                    }
                };

                void rotate(const T& angle_x, const T& angle_y, const T& angle_z) {
                    for (auto &point : points) {
                        point.rotate(angle_x, angle_y, angle_z);
                    }
                };

                void transform(const Eigen::Matrix<T, 4, 4>& transformation) {
                    for (auto &point : points) {
                        point.transform(transformation);
                    }
                };

                void transform(const Eigen::Matrix<T, 3, 3>& rotation, const Eigen::Matrix<T, 3, 1>& translation) {
                    for (auto &point : points) {
                        point.transform(rotation, translation);
                    }
                };

                void transform(const Eigen::Quaternion<T>& rotation, const Eigen::Matrix<T, 3, 1>& translation) {
                    for (auto &point : points) {
                        point.transform(rotation, translation);
                    }
                };

                void transform(const Eigen::Matrix<T, 3, 1>& rotation, const Eigen::Matrix<T, 3, 1>& translation) {
                    for (auto &point : points) {
                        point.transform(rotation, translation);
                    }
                };

                void transform(const std::vector<T>& rotation, const std::vector<T>& translation) {
                    for (auto &point : points) {
                        point.transform(rotation, translation);
                    }
                };

                void transform(const T& angle_x, const T& angle_y, const T& angle_z, const T& translation_x, const T& translation_y, const T& translation_z) {
                    for (auto &point : points) {
                        point.transform(angle_x, angle_y, angle_z, translation_x, translation_y, translation_z);
                    }
                };

                void displace(const T& displacement, const float search_radius = 0.03) {
                    // Get normals
                    pcl::PointCloud<pcl::Normal>::Ptr normals = compute_normals(search_radius);
                    // Displace points in the direction of the normals
                    for (std::size_t i = 0; i < num_points; ++i) {
                        pcl_lib::Point<T> normal = pcl_lib::Point<T>(normals->points[i].normal_x, 
                                                                     normals->points[i].normal_y, 
                                                                     normals->points[i].normal_z);
                        normal.normalize();
                        points[i].translate(normal * displacement);
                    }
                };

            private:
                std::vector<pcl_lib::Point<T>> points;
                std::size_t num_points{0};
                bool fixed_size{false};

                const pcl::PointCloud<pcl::Normal>::Ptr compute_normals(const float search_radius = 0.03) const{
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = to_pcl();
                    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
                    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
                    ne.setInputCloud(cloud);
                    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
                    ne.setSearchMethod(tree);
                    ne.setRadiusSearch(search_radius);
                    ne.compute(*normals);
                    return normals;
                };
                
                const pcl::PointCloud<pcl::PointXYZ>::Ptr to_pcl() const{
                    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                    for (const auto &point : points){
                        pcl_cloud->push_back(pcl::PointXYZ(point.x, point.y, point.z));
                    }
                    return pcl_cloud;
                };
    };
} // namespace pcl_lib
