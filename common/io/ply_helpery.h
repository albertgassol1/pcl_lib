#pragma once

#include <memory>   
#include <boost/filesystem.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl_lib/common/types/pointcloud.h>
#include <common/types/point.h>


namespace pcl_lib {
    namespace io {
        template <typename T> 
        void read_ply(const boost::filesystem::path &path, std::shared_ptr<pcl_lib::PointCloud<pcl_lib::Point<T>>> &pointcloud){
            // Read the ply file using pcl
            pcl::PointCloud<pcl::PointXYZ>::Ptr plc_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PLYReader Reader;
            Reader.read(path, *plc_cloud);

            // Convert the pcl pointcloud to pcl_lib pointcloud
            pointcloud->clear();
            for (const auto &point : plc_cloud->points){
                pointcloud->add(pcl_lib::Point<T>(point.x, point.y, point.z));
            }
        }

        template <typename T> 
        void read_ply(const boost::filesystem::path &path, std::shared_ptr<pcl_lib::PointCloud<pcl_lib::PointRGBA<T>>> &pointcloud){
            // Read the ply file using pcl
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plc_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::PLYReader Reader;
            Reader.read(path.string(), *plc_cloud);

            // Convert the pcl pointcloud to pcl_lib pointcloud
            pointcloud->clear();
            for (const auto &point : plc_cloud->points){
                pointcloud->add(pcl_lib::PointRGBA<T>(point.x, point.y, point.z, point.r, point.g, point.b, point.a));
            }
        }

        template <typename T>
        void write_ply(const boost::filesystem::path &path, const std::shared_ptr<pcl_lib::PointCloud<pcl_lib::Point<T>>> &pointcloud){
            // Convert the pcl_lib pointcloud to pcl pointcloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr plc_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            for (std::size_t i = 0; i < pointcloud->size(); i++){
                const auto &point = pointcloud->at(i);
                plc_cloud->push_back(pcl::PointXYZ(point.x, point.y, point.z));
            }

            // Write the pcl pointcloud to ply file
            pcl::PLYWriter Writer;
            Writer.write(path.string(), *plc_cloud);
        }

        template <typename T>
        void write_ply(const boost::filesystem::path &path, const std::shared_ptr<pcl_lib::PointCloud<pcl_lib::PointRGBA<T>>> &pointcloud){
            // Convert the pcl_lib pointcloud to pcl pointcloud
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plc_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
            for (std::size_t i = 0; i < pointcloud->size(); i++){
                const auto &point = pointcloud->at(i);
                plc_cloud->push_back(pcl::PointXYZRGBA(point.x, point.y, point.z, point.r, point.g, point.b, point.a));
            }

            // Write the pcl pointcloud to ply file
            pcl::PLYWriter Writer;
            Writer.write(path.string(), *plc_cloud);
        }
    }
}