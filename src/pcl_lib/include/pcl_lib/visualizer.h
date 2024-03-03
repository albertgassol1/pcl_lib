#include <iostream>
#include <thread>

#include <pcl/common/angles.h> 
#include <pcl/visualization/pcl_visualizer.h>


#include "pointcloud_rgba.h"


namespace pcl_lib {
    namespace visualizer {

        pcl::visualization::PCLVisualizer::Ptr pclVisualize(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud) {
            pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
            viewer->getRenderWindow()->GlobalWarningDisplayOff();
            viewer->setBackgroundColor(1.0, 1.0, 1.0);
            pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgba(cloud);
            viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, rgba, "sample cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
            viewer->addCoordinateSystem(1.0);
            viewer->initCameraParameters();
            return viewer;
        }

        template <typename T>
        void visualize(const std::shared_ptr<pcl_lib::PointCloudRGBA<T>> &pointcloud){
            const auto& points = pointcloud->toPcl();
            std::cout << "A: " << pointcloud->at(0).a << std::endl;
            const auto& viewer = pclVisualize(points);
            while (!viewer->wasStopped()) {
                viewer->spinOnce(100);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
}