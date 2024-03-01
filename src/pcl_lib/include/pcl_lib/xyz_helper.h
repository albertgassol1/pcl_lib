#pragma once

#include <memory>   
#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/filesystem.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include "pointcloud.h"
#include "point.h"

namespace pcl_lib {
    namespace io {

        template <typename T>
        void read_xyz(const boost::filesystem::path &path, std::shared_ptr<pcl_lib::PointCloudRGBA<T>> &pointcloud){
            std::ifstream file(path);
            if (!file.is_open()) {
                std::cerr << "Failed to open file: " << path.string() << std::endl;
                return;
            }

            pointcloud->clear();
            std::string line;
            while (std::getline(file, line)) {
                std::istringstream iss(line);
                pcl_lib::PointRGBA<T> point;
                if (!(iss >> point.x >> point.y >> point.z >> point.r >> point.g >> point.b >> point.a)) {
                    std::istringstream iss(line);
                    if((iss >> point.x >> point.y >> point.z >> point.r >> point.g >> point.b)){
                        point.a = 255;
                    }
                    else{
                        std::cerr << "Failed to read line: " << line << std::endl;
                        continue;
                    }
                }
                pointcloud->add(point);
            }
            file.close();
        }


        template <typename T>
        void write_xyz(const boost::filesystem::path &path, const std::shared_ptr<pcl_lib::PointCloudRGBA<T>> &pointcloud){
            std::ofstream file(path);
            if (!file.is_open()) {
                std::cerr << "Failed to open file: " << path.string() << std::endl;
                return;
            }

            for (const auto& point : pointcloud->get_points_copy()) {
                file << point.x << " " << point.y << " " << point.z << " " << point.r << " " << point.g << " " << point.b << " " << point.a << std::endl;
            }
            file.close();
        }
    }
}