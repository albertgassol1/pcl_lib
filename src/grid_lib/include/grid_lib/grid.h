#pragma once

#include <memory>
#include <random>
#include <pcl_lib/pointcloud_constvel.h>
#include <nanoflann.hpp>

#include "kdtree_wrapper.h"
#include <pcl_lib/point.h>

namespace pcl_lib {
    namespace grid {
        template <typename T>
        struct Grid {
            T x_min;
            T x_max;
            T y_min;
            T y_max;
            T z_min;
            T z_max;

            Grid() = default;
            
            Grid(const T _x_min, const T _x_max, 
                 const T _y_min, const T _y_max, 
                 const T _z_min, const T _z_max) : x_min(_x_min), x_max(_x_max), y_min(_y_min), y_max(_y_max), z_min(_z_min), z_max(_z_max) {}

            void setVelocityOnCollision(pcl_lib::PointConstVel<T>& point) const {
                // Assume 90 degrees rebound. 
                if(point.x < x_min || point.x > x_max) {
                    point.velocity.x = -point.velocity.x;
                }
                if(point.y < y_min || point.y > y_max) {
                    point.velocity.y = -point.velocity.y;
                }
                if(point.z < z_min || point.z > z_max) {
                    point.velocity.z = -point.velocity.z;
                }
            }
        };

        template <typename T>
        class GridHandler {
            public:
                GridHandler(const std::shared_ptr<Grid<T>>& _grid,
                            const T _max_velocity) : grid(std::move(_grid)),
                                                     pointcloud(std::make_shared<pcl_lib::PointCloudConstVel<T>>(),
                                                     max_velocity(_max_velocity)) {
                    assert(max_velocity > 0);
                }

                GridHandler(const T _x_min, const T _x_max, 
                            const T _y_min, const T _y_max, 
                            const T _z_min, const T _z_max,
                            const T _max_velocity) : grid(std::make_shared<Grid<T>>(_x_min, _x_max, _y_min, _y_max, _z_min, _z_max)),
                                                     pointcloud(std::make_shared<pcl_lib::PointCloudConstVel<T>>(),
                                                     max_velocity(_max_velocity)) {
                    assert(max_velocity > 0);
                }

                GridHandler(const std::shared_ptr<Grid<T>>& _grid, 
                            const std::shared_ptr<pcl_lib::PointCloudConstVel<T>>& _pointcloud,
                            const T _max_velocity) : grid(std::move(_grid)), 
                                                     pointcloud(std::move(_pointcloud),
                                                     kdtree(std::make_unique<KDTreeWrapper<T>>(pointcloud)),
                                                     max_velocity(_max_velocity)) {
                    assert(max_velocity > 0);
                }

                GridHandler(const T _x_min, const T _x_max, 
                            const T _y_min, const T _y_max, 
                            const T _z_min, const T _z_max, 
                            const std::shared_ptr<pcl_lib::PointCloudConstVel<T>>& _pointcloud,
                            const T _max_velocity) : grid(std::make_shared<Grid<T>>(_x_min, _x_max, _y_min, _y_max, _z_min, _z_max)), 
                                                     pointcloud(std::move(_pointcloud),
                                                     kdtree(std::make_unique<KDTreeWrapper<T>>(pointcloud)), 
                                                     max_velocity(_max_velocity)) {
                    assert(max_velocity > 0);
                }

                void setPointcloud(const std::shared_ptr<pcl_lib::PointCloudConstVel<T>>& _pointcloud) {
                    pointcloud = std::move(_pointcloud);
                    if (kdtree == nullptr) {
                        kdtree = std::make_unique<KDTreeWrapper<T>>(pointcloud);
                    } else {
                        kdtree->rebuild();
                    }
                }

                const std::shared_ptr<pcl_lib::PointCloudConstVel<T>> get_pointcloud() const {
                    return pointcloud;
                }

                void initializeRandomPointcloud(const std::size_t n_points) {
                    if(pointcloud->size() > 0) {
                        pointcloud->clear();
                    }

                    std::random_device rd;
                    std::mt19937 gen(rd());
                    std::uniform_real_distribution<float> distribX(this->grid->x_min, this->grid->x_max);
                    std::uniform_real_distribution<float> distribY(this->grid->y_min, this->grid->y_max);
                    std::uniform_real_distribution<float> distribZ(this->grid->z_min, this->grid->z_max);
                    std::uniform_real_distribution<float> distribV(-max_velocity, max_velocity);

                    for(std::size_t i = 0; i < n_points; i++) {
                        pointcloud->add(pcl_lib::PointConstVel<T>(distribX(gen), distribY(gen), distribZ(gen), 
                                                                  distribV(gen), distribV(gen), distribV(gen)));
                    }
                    if (kdtree == nullptr) {
                        kdtree = std::make_unique<KDTreeWrapper<T>>(pointcloud);
                    } else {
                        kdtree->rebuild();
                    }
                }

                void setGrid(const T _x_min, const T _x_max, const T _y_min, const T _y_max, const T _z_min, const T _z_max) {
                    grid->x_min = _x_min;
                    grid->x_max = _x_max;
                    grid->y_min = _y_min;
                    grid->y_max = _y_max;
                    grid->z_min = _z_min;
                    grid->z_max = _z_max;
                }

                const std::shared_ptr<Grid<T>> getGrid() {
                    return grid;
                }

                void update(const T& dt) {
                    checkGridCollisions();
                    checkPointCollisions();
                    pointcloud->update(dt);
                }

            private:
                void checkGridCollisions() {
                    #pragma omp parallel for
                    for(auto& point : pointcloud->get_points()) {
                        grid->setVelocityOnCollision(point);
                    }
                }

                void checkPointCollisions() {
                    #pragma omp parallel for
                    for(auto& point : pointcloud->get_points()) {
                        const auto& [num_results, collision_indices] = kdtree->radiusSearch(point.x, point.y, point.z, 2 * point.radius);
                        if(num_results > 1) {
                            for(const auto& collision_index : collision_indices) {
                                if(collision_index.first != point.index) {
                                    // Assume 90 degrees rebound. This is a simplification.
                                    point.velocity.x = -point.velocity.x;
                                    point.velocity.y = -point.velocity.y;
                                    point.velocity.z = -point.velocity.z;
                                    break;
                                }
                            }
                        }
                    }
                }

                std::shared_ptr<Grid<T>> grid;
                std::shared_ptr<pcl_lib::PointCloudConstVel<T>> pointcloud;
                std::unique_ptr<KDTreeWrapper<T>> kdtree;
                T max_velocity;
        };
    }
}
