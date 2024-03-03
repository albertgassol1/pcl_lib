#include <iostream>
#include <thread>

#include <pcl/common/angles.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <grid_lib/grid.h>
#include <pcl_lib/pointcloud_rgba.h>
#include <pcl_lib/pointcloud_constvel.h>


namespace pcl_lib {
    namespace visualizer {

        void pclVisualize(pcl::visualization::PCLVisualizer::Ptr viewer,
                          pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, 
                          const double point_size = 3.0,
                          const std::string& name = "pcl") {
            viewer->getRenderWindow()->GlobalWarningDisplayOff();
            viewer->setBackgroundColor(1.0, 1.0, 1.0);
            pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgba(cloud);
            viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, rgba, name);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, name);
            viewer->addCoordinateSystem(1.0);
            viewer->initCameraParameters();
        }

        void pclVisualize(pcl::visualization::PCLVisualizer::Ptr viewer, 
                          pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, 
                          const double point_size = 3.0,
                          const std::string& name = "pcl") {
            viewer->getRenderWindow()->GlobalWarningDisplayOff();
            viewer->setBackgroundColor(1.0, 1.0, 1.0);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 0, 0);
            viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, name);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size * 10, name);
            viewer->addCoordinateSystem(1.0);
            viewer->initCameraParameters();
        }

        template <typename T>
        void visualizeStaticPcl(const std::shared_ptr<pcl_lib::PointCloudRGBA<T>> &pointcloud){
            const auto& points = pointcloud->toPcl();
            pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
            pclVisualize(viewer, points);
            while (!viewer->wasStopped()) {
                viewer->spinOnce(100);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        template <typename T>
        void visualizeGrid(pcl::visualization::PCLVisualizer::Ptr viewer, 
                           std::shared_ptr<pcl_lib::grid::GridHandler<T>>& grid, 
                           const std::string& name = "pcl"){

            // Visualize 6 planes of the grid
            for (std::size_t i = 0; i < 12; ++i) {
                const auto [p1, p2] = grid->getGridLine(i);
                viewer->addLine(pcl::PointXYZ(p1.x, p1.y, p1.z), pcl::PointXYZ(p2.x, p2.y, p2.z), 255, 0, 0, "line_" + std::to_string(i));
            }
        }

        template <typename T>
        void visualizeGrid(std::shared_ptr<pcl_lib::grid::GridHandler<T>>& grid, const T& dt, 
                           const bool update = false, const std::string& name = "pcl"){
            const auto& originalPoints = grid->getPointCloud()->toPclXYZ();
            pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
            std::cout << "Points size: " << grid->getConstPointsSize() << std::endl;
            pclVisualize(viewer, originalPoints, static_cast<double>(grid->getConstPointsSize()), name);
            visualizeGrid(viewer, grid, name);
            while (!viewer->wasStopped()) {
                viewer->spinOnce(static_cast<int>(dt*1000));
                if(update) {
                    grid->update(dt);
                    const auto& updatedPoints = grid->getPointCloud()->toPclXYZ();
                    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(updatedPoints, 0, 0, 0);
                    viewer->updatePointCloud(updatedPoints, single_color, name);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt*1000)));
            }
        }
    }
}