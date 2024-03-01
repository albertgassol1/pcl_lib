#include <iostream>
#include <thread>

#include <pcl/common/angles.h> 
#include <pcl/visualization/pcl_visualizer.h>


#include "pointcloud.h"


namespace pcl_lib {
    namespace visualizer {

        // pcl::visualization::PCLVisualizer::Ptr rgbaVis (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud) {
        //     pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        //     viewer->setBackgroundColor(0, 0, 0);
        //     pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgba(cloud);
        //     viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, rgba, "sample cloud");
        //     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
        //     viewer->addCoordinateSystem(1.0);
        //     viewer->initCameraParameters();
        //     return viewer;
        // }

        pcl::visualization::PCLVisualizer::Ptr xzyVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
            pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
            viewer->getRenderWindow()->GlobalWarningDisplayOff();
            viewer->setBackgroundColor(0, 0, 0);
            viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
            viewer->addCoordinateSystem(1.0);
            viewer->initCameraParameters();
            return viewer;
        }

        // template <typename T>
        // void visualizeRGBA(const std::shared_ptr<pcl_lib::PointCloud<T>> &pointcloud){
        //     const auto& points = pointcloud->to_pcl();
        //     rgbaVis(points);
        // }

        template <typename T>
        void visualizeXYZ(const std::shared_ptr<pcl_lib::PointCloud<T>> &pointcloud){
            const auto& points = pointcloud->to_pcl();
            const auto& viewer = xzyVis(points);
            while (!viewer->wasStopped()) {
                viewer->spinOnce(100);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
}