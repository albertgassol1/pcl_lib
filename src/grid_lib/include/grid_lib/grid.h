#pragma once

#include <memory>
#include <random>
#include <tuple>
#include <pcl_lib/pointcloud_constvel.h>
#include <nanoflann.hpp>

#include "kdtree_adaptor.h"
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

            const std::tuple<Point<T>, Point<T>> getGridLine(const std::size_t ind) const{
                switch(ind) {
                    case 0:
                        return std::make_tuple(Point<T>(x_min, y_min, z_min), Point<T>(x_max, y_min, z_min));
                    case 1:
                        return std::make_tuple(Point<T>(x_min, y_max, z_min), Point<T>(x_max, y_max, z_min));
                    case 2:
                        return std::make_tuple(Point<T>(x_min, y_min, z_min), Point<T>(x_min, y_max, z_min));
                    case 3:
                        return std::make_tuple(Point<T>(x_max, y_min, z_min), Point<T>(x_max, y_max, z_min));
                    case 4:
                        return std::make_tuple(Point<T>(x_min, y_min, z_max), Point<T>(x_max, y_min, z_max));
                    case 5:
                        return std::make_tuple(Point<T>(x_min, y_max, z_max), Point<T>(x_max, y_max, z_max));
                    case 6:
                        return std::make_tuple(Point<T>(x_min, y_min, z_max), Point<T>(x_min, y_max, z_max));
                    case 7:
                        return std::make_tuple(Point<T>(x_max, y_min, z_max), Point<T>(x_max, y_max, z_max));
                    case 8:
                        return std::make_tuple(Point<T>(x_min, y_min, z_min), Point<T>(x_min, y_min, z_max));
                    case 9:
                        return std::make_tuple(Point<T>(x_max, y_min, z_min), Point<T>(x_max, y_min, z_max));
                    case 10:
                        return std::make_tuple(Point<T>(x_min, y_max, z_min), Point<T>(x_min, y_max, z_max));
                    case 11:
                        return std::make_tuple(Point<T>(x_max, y_max, z_min), Point<T>(x_max, y_max, z_max));
                    default:
                        throw std::out_of_range("Index out of range");
                }
            }
            
            Grid(const T _x_min, const T _x_max, 
                 const T _y_min, const T _y_max, 
                 const T _z_min, const T _z_max) : x_min(_x_min), x_max(_x_max), y_min(_y_min), y_max(_y_max), z_min(_z_min), z_max(_z_max) {}

            bool setVelocityOnCollision(pcl_lib::PointConstVel<T>& point) const {
                // Assume 90 degrees rebound. 
                bool collision = false;
                if(point.x < x_min || point.x > x_max) {
                    point.velocity.x = -point.velocity.x;
                    collision = true;
                }
                if(point.y < y_min || point.y > y_max) {
                    point.velocity.y = -point.velocity.y;
                    collision = true;
                }
                if(point.z < z_min || point.z > z_max) {
                    point.velocity.z = -point.velocity.z;
                    collision = true;
                }
                return collision;
            }
        };

        template <typename T>
        class GridHandler {

            using KDTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<T, PointCloudAdaptor<T>>, PointCloudAdaptor<T>, 3>;
            public:

                GridHandler() : grid(std::make_shared<Grid<T>>()), 
                                pointcloud(std::make_shared<pcl_lib::PointCloudConstVel<T>>()) {}

                GridHandler(const std::shared_ptr<Grid<T>>& _grid) : grid(std::move(_grid)),
                                                                     pointcloud(std::make_shared<pcl_lib::PointCloudConstVel<T>>()) {}

                GridHandler(const T _x_min, const T _x_max, 
                            const T _y_min, const T _y_max, 
                            const T _z_min, const T _z_max) : grid(std::make_shared<Grid<T>>(_x_min, _x_max, _y_min, _y_max, _z_min, _z_max)),
                                                              pointcloud(std::make_shared<pcl_lib::PointCloudConstVel<T>>()) {}

                GridHandler(const std::shared_ptr<Grid<T>>& _grid, 
                            const std::shared_ptr<pcl_lib::PointCloudConstVel<T>>& _pointcloud) : grid(std::move(_grid)), 
                                                                                    pointcloud(std::move(_pointcloud)) {}

                GridHandler(const T _x_min, const T _x_max, 
                            const T _y_min, const T _y_max, 
                            const T _z_min, const T _z_max, 
                            const std::shared_ptr<pcl_lib::PointCloudConstVel<T>>& _pointcloud) : grid(std::make_shared<Grid<T>>(_x_min, _x_max, _y_min, _y_max, _z_min, _z_max)), 
                                                                                                  pointcloud(std::move(_pointcloud)) {}

                void setPointcloud(const std::shared_ptr<pcl_lib::PointCloudConstVel<T>>& _pointcloud) {
                    pointcloud = std::move(_pointcloud);
                }

                const std::shared_ptr<pcl_lib::PointCloudConstVel<T>> getPointCloud() const {
                    return pointcloud;
                }

                const T getConstPointsSize() const {
                    assert(pointcloud->size() > 0);
                    return 2*pointcloud->at(0).radius;
                }

                void initializeRandomPointcloud(const std::size_t n_points, const T point_radius, const T max_velocity = 0.1, const T dt = 0.01) {
                    if(pointcloud->size() > 0) {
                        pointcloud->clear();
                    }
                    assert(max_velocity > point_radius / dt);
                    std::random_device rd;
                    std::mt19937 gen(rd());
                    std::srand(std::time(nullptr));
                    std::uniform_real_distribution<T> distribX(this->grid->x_min + point_radius, this->grid->x_max - point_radius);
                    std::uniform_real_distribution<T> distribY(this->grid->y_min + point_radius, this->grid->y_max - point_radius);
                    std::uniform_real_distribution<T> distribZ(this->grid->z_min + point_radius, this->grid->z_max - point_radius);
                    std::uniform_real_distribution<T> distribV(point_radius / dt , max_velocity);
    
                    for(std::size_t i = 0; i < n_points; i++) {
                        pointcloud->add(pcl_lib::PointConstVel<T>(distribX(gen), distribY(gen), distribZ(gen), 
                                                                  distribV(gen), distribV(gen), distribV(gen) * static_cast<T>(static_cast<int>(std::rand() % 2) * 2.0 - 1.0),
                                                                  point_radius));
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

                void update(const T& dt, const bool point_collisions = true) {
                    std::vector<bool> collided_with_grid(pointcloud->size(), false);
                    checkGridCollisions(collided_with_grid);
                    if (point_collisions) {
                        checkPointCollisions(collided_with_grid);
                    }
                    pointcloud->update(dt);
                }

                const std::tuple<Point<T>, Point<T>> getGridLine(const std::size_t ind) const{
                    return grid->getGridLine(ind);
                }

                const std::shared_ptr<Grid<T>> getConstGrid() const {
                    return grid;
                }  

            private:
                void checkGridCollisions(std::vector<bool>& collided_with_grid) {
                    #pragma omp parallel for
                    for(std::size_t i = 0; i < pointcloud->size(); i++) {
                        if(grid->setVelocityOnCollision((*pointcloud)[i])) {
                            collided_with_grid[i] = true;
                        }
                    }
                }

                void checkPointCollisions(std::vector<bool>& collided_with_grid) {
                    PointCloudAdaptor<T> adaptor(*pointcloud);
                    KDTree tree(3, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
                    #pragma omp parallel for
                    for (std::size_t i = 0; i < pointcloud->size(); i++){
                        if(collided_with_grid[i]) {
                            continue;
                        }
                        const auto& point = pointcloud->at(i);
                        std::vector<std::pair<std::size_t, T>> indices_dists;
                        nanoflann::RadiusResultSet<T, std::size_t> result_set(point.radius * 2, indices_dists);
                        // Define the query point
                        T query_pt[3] = {point.x, point.y, point.z};

                        // Perform radius search
                        nanoflann::SearchParams params;
                        params.sorted = true;
                        tree.findNeighbors(result_set, query_pt, params);
                    
                        if(!result_set.m_indices_dists.empty()) {
                            for(const auto& index_dist : result_set.m_indices_dists) {
                                if(index_dist.first != i) {
                                    if (collided_with_grid[index_dist.first]) {
                                        continue;
                                    }
                                    // Assume 180 degrees rebound on all axes. This is a simplification.
                                    pointcloud->setVelocity(i, point.velocity * -1);
                                    break;
                                }
                            }
                        }
                    }
                }

                std::shared_ptr<Grid<T>> grid;
                std::shared_ptr<pcl_lib::PointCloudConstVel<T>> pointcloud;
        };
    }
}
