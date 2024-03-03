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
        class PointCloudConstVel : public PointCloudBase<T>{

            public:
                PointCloudConstVel () : PointCloudBase<T>(0, false){};

                PointCloudConstVel(const PointCloudConstVel& other) : points(other.points), PointCloudBase<T>(other.num_points, other.fixed_size) {};

                PointCloudConstVel (const std::size_t n_points) : points(n_points), PointCloudBase<T>(n_points, true) {
                    assert(n_points >= 0);
                };

                PointCloudConstVel(const std::vector<pcl_lib::PointConstVel<T>>& pts, const bool fixed)  : points(pts), PointCloudBase<T>(pts.size(), fixed) {};

                void add(const pcl_lib::PointConstVel<T>& point) {
                    assert(!fixed_size);
                    points.emplace_back(point);
                    this->num_points++;
                };
                void clear() {
                    points.clear();
                    PointCloudBase<T>::clear();
                };
 
                pcl_lib::PointConstVel<T>& operator[](const std::size_t i) {
                    assert(i < this->num_points);
                    return points[i];
                };

                pcl_lib::PointCloudConstVel<T> operator+(const pcl_lib::PointCloudConstVel<T>& other) const {
                    std::vector<pcl_lib::PointConstVel<T>> new_points;
                    new_points.reserve(this->num_points + other.size());
                    new_points.insert(new_points.end(), points.begin(), points.end());
                    new_points.insert(new_points.end(), other.getConstPoints().begin(), other.getConstPoints().end());
                    
                    return pcl_lib::PointCloudConstVel<T>(new_points, false); 
                };
 
                const pcl_lib::PointConstVel<T>& at(const std::size_t i) const {
                    assert(i < this->num_points);
                    return points[i];
                };
 
                const std::vector<pcl_lib::PointConstVel<T>>& getConstPoints() const {
                    return points;
                };

                std::vector<pcl_lib::PointConstVel<T>>& getPoints() {
                    return points;
                };
 
                std::vector<pcl_lib::PointConstVel<T>> getConstPointsCopy() const {
                    return std::vector<pcl_lib::PointConstVel<T>>(points);
                };

                 void translate(const pcl_lib::PointConstVel<T>& translation) {
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

                void update(const T& dt) {
                    #pragma omp parallel for
                    for(auto& point : points) {
                        point.update(dt);
                    }
                }

                void update(const T& dt, const std::vector<T>& min, const std::vector<T>& max) {
                    #pragma omp parallel for
                    for(auto& point : points) {
                        point.update(dt, min, max);
                    }
                }

                void setVelocity(const std::size_t index, const Point<T>& velocity) {
                    assert(index < this->num_points);
                    points[index].velocity = velocity;
                }

                const pcl::PointCloud<pcl::PointXYZ>::Ptr toPclXYZ() const{
                    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                    for (const auto &point : points){
                        pcl_cloud->push_back(pcl::PointXYZ(point.x, point.y, point.z));
                    }
                    return pcl_cloud;
                };

            private:
                std::vector<pcl_lib::PointConstVel<T>> points;
        };
} // namespace pcl_lib
