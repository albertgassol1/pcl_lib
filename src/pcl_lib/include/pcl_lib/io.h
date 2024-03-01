#pragma once

#include <boost/filesystem.hpp>

#include "ply_helper.h"
#include "xyz_helper.h"
#include "pointcloud.h"
#include "point.h"

namespace pcl_lib {
    namespace io {
        template <typename T>
        inline void readXYZRGBA(const boost::filesystem::path &path, std::shared_ptr<pcl_lib::PointCloud<T>> &pointcloud) {
            if (path.extension() == ".ply"){
                pcl_lib::io::readXYZRGBA_ply(path, pointcloud);
            }
            else if (path.extension() == ".xyz"){
                pcl_lib::io::readXYZRGBA_xyz(path, pointcloud);
            }
            else {
                throw std::runtime_error("Unsupported file extension");
            }
        }

        template <typename T>
        inline void readXYZ(const boost::filesystem::path &path, std::shared_ptr<pcl_lib::PointCloud<T>> &pointcloud) {
            if (path.extension() == ".ply"){
                pcl_lib::io::readXYZ_ply(path, pointcloud);
            }
            else if (path.extension() == ".xyz"){
                pcl_lib::io::readXYZ_xyz(path, pointcloud);
            }
            else {
                throw std::runtime_error("Unsupported file extension");
            }
        }

        template <typename T>
        inline void writeXYZRGBA(const boost::filesystem::path &path, const std::shared_ptr<pcl_lib::PointCloud<T>> &pointcloud) {
            if (path.extension() == ".ply"){
                pcl_lib::io::writeXYZRGBA_ply(path, pointcloud);
            }
            else if (path.extension() == ".xyz"){
                pcl_lib::io::writeXYZRGBA_xyz(path, pointcloud);
            }
            else {
                throw std::runtime_error("Unsupported file extension");
            }
        }

        template <typename T>
        inline void writeXYZ(const boost::filesystem::path &path, const std::shared_ptr<pcl_lib::PointCloud<T>> &pointcloud) {
            if (path.extension() == ".ply"){
                pcl_lib::io::writeXYZ_ply(path, pointcloud);
            }
            else if (path.extension() == ".xyz"){
                pcl_lib::io::writeXYZ_xyz(path, pointcloud);
            }
            else {
                throw std::runtime_error("Unsupported file extension");
            }
        }
    }
}