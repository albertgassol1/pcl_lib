#pragma once

#include "ply_helpery.h"
#include "xyz_helper.h"
#include <common/types/pointcloud.h>
#include <common/types/point.h>


// Create common read and write functions for pointclouds and pointclouds with color that can accept .ply and .xyz files. They call the proper helper functions based on the file extension.

namespace pcl_lib {
    namespace io {
        template <typename T>
        void read(const boost::filesystem::path &path, std::shared_ptr<pcl_lib::PointCloud<pcl_lib::Point<T>>> &pointcloud){
            if (path.extension() == ".ply"){
                read_ply(path, pointcloud);
            }
            else if (path.extension() == ".xyz"){
                read_xyz(path, pointcloud);
            }
            else {
                throw std::runtime_error("Unsupported file extension");
            }
        }

        template <typename T>
        void read(const boost::filesystem::path &path, std::shared_ptr<pcl_lib::PointCloud<pcl_lib::PointRGBA<T>>> &pointcloud){
            if (path.extension() == ".ply"){
                read_ply(path, pointcloud);
            }
            else if (path.extension() == ".xyz"){
                read_xyz(path, pointcloud);
            }
            else {
                throw std::runtime_error("Unsupported file extension");
            }
        }

        template <typename T>
        void write(const boost::filesystem::path &path, const std::shared_ptr<pcl_lib::PointCloud<pcl_lib::Point<T>>> &pointcloud){
            if (path.extension() == ".ply"){
                write_ply(path, pointcloud);
            }
            else if (path.extension() == ".xyz"){
                write_xyz(path, pointcloud);
            }
            else {
                throw std::runtime_error("Unsupported file extension");
            }
        }

        template <typename T>
        void write(const boost::filesystem::path &path, const std::shared_ptr<pcl_lib::PointCloud<pcl_lib::PointRGBA<T>>> &pointcloud){
            if (path.extension() == ".ply"){
                write_ply(path, pointcloud);
            }
            else if (path.extension() == ".xyz"){
                write_xyz(path, pointcloud);
            }
            else {
                throw std::runtime_error("Unsupported file extension");
            }
        }
    }
}