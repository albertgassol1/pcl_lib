#include <iostream>
#include <vector>
#include <cmath>
#include <ctime>
#include <chrono>
#include <memory>
#include <tuple>
#include <nanoflann.hpp>

#include <pcl_lib/pointcloud_constvel.h>

namespace pcl_lib {
    namespace grid {

        // Define the kd-tree adaptor for the PointCloud type
        template <typename T>
        struct PointCloudAdaptor {
            pcl_lib::PointCloudConstVel<T> cloud;
            // Constructor
            PointCloudAdaptor(const PointCloudConstVel<T>& cloud_) : cloud(cloud_) {}
            // Must return the number of data points
            inline std::size_t kdtree_get_point_count() const { return cloud.size(); }
            // Must return the dim'th component of the idx'th point in the dataset
            inline T kdtree_get_pt(const std::size_t idx, const std::size_t dim) const {
                return cloud.at(idx).at(dim);
            }
            template <class BBOX> bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
        };
    } // namespace grid
} // namespace pcl_lib
