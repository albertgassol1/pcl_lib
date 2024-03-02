#pragma once

#include <memory>
#include <random>
#include <pcl_lib/pointcloud_constvel.h>

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
                                                     max_velocity(_max_velocity)) {
                    assert(max_velocity > 0);
                }

                GridHandler(const T _x_min, const T _x_max, 
                            const T _y_min, const T _y_max, 
                            const T _z_min, const T _z_max, 
                            const std::shared_ptr<pcl_lib::PointCloudConstVel<T>>& _pointcloud,
                            const T _max_velocity) : grid(std::make_shared<Grid<T>>(_x_min, _x_max, _y_min, _y_max, _z_min, _z_max)), 
                                                     pointcloud(std::move(_pointcloud),
                                                     max_velocity(_max_velocity)) {
                    assert(max_velocity > 0);
                }

                void set_pointcloud(const std::shared_ptr<pcl_lib::PointCloudConstVel<T>>& _pointcloud) {
                    pointcloud = std::move(_pointcloud);
                }

                const std::shared_ptr<pcl_lib::PointCloudConstVel<T>> get_pointcloud() const {
                    return pointcloud;
                }

                void initialize_random_pointcloud(const std::size_t n_points) {
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
                }

                void set_grid(const T _x_min, const T _x_max, const T _y_min, const T _y_max, const T _z_min, const T _z_max) {
                    grid->x_min = _x_min;
                    grid->x_max = _x_max;
                    grid->y_min = _y_min;
                    grid->y_max = _y_max;
                    grid->z_min = _z_min;
                    grid->z_max = _z_max;
                }

                const std::shared_ptr<Grid<T>> get_grid() {
                    return grid;
                }

            private:
                std::shared_ptr<Grid<T>> grid;
                std::shared_ptr<pcl_lib::PointCloudConstVel<T>> pointcloud;
                T max_velocity;
        };
    }

}
