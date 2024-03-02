#pragma once

#include <memory>   
#include <boost/filesystem.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include "pointcloud_rgba.h"
#include "point.h"


namespace pcl_lib {
    namespace io {

        template <typename T> 
        void read_ply(const boost::filesystem::path &path, std::shared_ptr<pcl_lib::PointCloudRGBA<T>> &pointcloud){
            // Read the ply file using pcl
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plc_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::PLYReader Reader;
            Reader.read(path.string(), *plc_cloud);

            // Convert the pcl pointcloud to pcl_lib pointcloud
            std::cout << "clear" << std::endl;
            pointcloud->clear();
            std::cout << "Read" << std::endl;
            for (const auto &point : plc_cloud->points){
                pointcloud->add(pcl_lib::PointRGBA<T>(point.x, point.y, point.z, point.r, point.g, point.b, point.a));
            }
        }

        template <typename T>
        void write_ply(const boost::filesystem::path &path, const std::shared_ptr<pcl_lib::PointCloudRGBA<T>> &pointcloud){
            // Convert the pcl_lib pointcloud to pcl pointcloud
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plc_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
            for (const auto& point : pointcloud->get_points_copy()){
                plc_cloud->push_back(pcl::PointXYZRGBA(point.x, point.y, point.z, point.r, point.g, point.b, point.a));
            }

            // Write the pcl pointcloud to ply file
            pcl::PLYWriter Writer;
            Writer.write(path.string(), *plc_cloud);
        }
    }
}
