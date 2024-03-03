#pragma once

#include <boost/filesystem.hpp>

#include "ply_helper.h"
#include "xyz_helper.h"
#include "pointcloud_rgba.h"
#include "pointcloud_constvel.h"
#include "point.h"

namespace pcl_lib {
    namespace io {
        template <typename T>
        void read(const boost::filesystem::path &path, std::shared_ptr<pcl_lib::PointCloudRGBA<T>> &pointcloud) {
            if (path.extension() == ".ply"){
                pcl_lib::io::read_ply(path, pointcloud);
            }
            else if (path.extension() == ".xyz"){
                pcl_lib::io::read_xyz(path, pointcloud);
            }
            else {
                throw std::runtime_error("Unsupported file extension");
            }
        }

        template <typename T>
        void read(const boost::filesystem::path &path, std::shared_ptr<pcl_lib::PointCloudConstVel<T>> &pointcloud) {
            if (path.extension() == ".xyz"){
                pcl_lib::io::read_xyz(path, pointcloud);
            }
            else {
                throw std::runtime_error("Unsupported file extension");
            }
        }

        template <typename T>
        void write(const boost::filesystem::path &path, const std::shared_ptr<pcl_lib::PointCloudRGBA<T>> &pointcloud) {
            if (path.extension() == ".ply"){
                pcl_lib::io::write_ply(path, pointcloud);
            }
            else if (path.extension() == ".xyz"){
                pcl_lib::io::write_xyz(path, pointcloud);
            }
            else {
                throw std::runtime_error("Unsupported file extension");
            }
        }

        template <typename T>
        void write(const boost::filesystem::path &path, const std::shared_ptr<pcl_lib::PointCloudConstVel<T>> &pointcloud) {
            if (path.extension() == ".ply"){
                pcl_lib::io::write_ply(path, pointcloud);
            }
            else if (path.extension() == ".xyz"){
                pcl_lib::io::write_xyz(path, pointcloud);
            }
            else {
                throw std::runtime_error("Unsupported file extension");
            }
        }
    }
}