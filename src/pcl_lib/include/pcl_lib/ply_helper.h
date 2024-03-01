#pragma once

#include <memory>   
#include <boost/filesystem.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include "pointcloud.h"
#include "point.h"


namespace pcl_lib {
    namespace io {
        template <typename T> 
        void readXYZ_ply(const boost::filesystem::path &path, std::shared_ptr<pcl_lib::PointCloud<T>> &pointcloud){
            // Read the ply file using pcl
            pcl::PointCloud<pcl::PointXYZ>::Ptr plc_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PLYReader Reader;
            Reader.read(path.string(), *plc_cloud);

            // Convert the pcl pointcloud to pcl_lib pointcloud
            pointcloud->clear();
            for (const auto &point : plc_cloud->points){
                pointcloud->add(pcl_lib::Point<T>(point.x, point.y, point.z));
            }
        }

        template <typename T> 
        void readXYZRGBA_ply(const boost::filesystem::path &path, std::shared_ptr<pcl_lib::PointCloud<T>> &pointcloud){
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
        void writeXYZ_ply(const boost::filesystem::path &path, const std::shared_ptr<pcl_lib::PointCloud<T>> &pointcloud){
            // Convert the pcl_lib pointcloud to pcl pointcloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr plc_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& point : pointcloud->get_points()){
                plc_cloud->push_back(pcl::PointXYZ(point.x, point.y, point.z));
            }

            // Write the pcl pointcloud to ply file
            pcl::PLYWriter Writer;
            Writer.write(path.string(), *plc_cloud);
        }

        template <typename T>
        void writeXYZRGBA_ply(const boost::filesystem::path &path, const std::shared_ptr<pcl_lib::PointCloud<T>> &pointcloud){
            // Convert the pcl_lib pointcloud to pcl pointcloud
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plc_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
            for (auto& point : pointcloud->get_points_copy()){
                pcl_lib::PointRGBA<T>& pointRGBA = static_cast<pcl_lib::PointRGBA<T>&>(point);
                plc_cloud->push_back(pcl::PointXYZRGBA(pointRGBA.x, pointRGBA.y, pointRGBA.z, pointRGBA.r, pointRGBA.g, pointRGBA.b, pointRGBA.a));
            }

            // Write the pcl pointcloud to ply file
            pcl::PLYWriter Writer;
            Writer.write(path.string(), *plc_cloud);
        }
    }
}
