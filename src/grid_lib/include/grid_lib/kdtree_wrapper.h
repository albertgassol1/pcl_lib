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
            std::shared_ptr<pcl_lib::PointCloudConstVel<T>> cloud;
            // Constructor
            PointCloudAdaptor(const std::shared_ptr<pcl_lib::PointCloudConstVel<T>>& cloud_) : cloud(cloud_) {}
            // Must return the number of data points
            inline std::size_t kdtree_get_point_count() const { return cloud->size(); }
            // Must return the dim'th component of the idx'th point in the dataset
            inline float kdtree_get_pt(const std::size_t idx, const std::size_t dim) const {
                return cloud->at(idx)[dim];
            }
            // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
            inline T kdtree_distance(const T* p1, const std::size_t idx_p2, std::size_t /*size*/) const {
                const T d0 = p1[0] - cloud->at(idx_p2)[0];
                const T d1 = p1[1] - cloud->at(idx_p2)[1];
                const T d2 = p1[2] - cloud->at(idx_p2)[2];
                return d0 * d0 + d1 * d1 + d2 * d2;
            }
            // Not used for this demonstration
            inline T kdtree_get_bbox_min(const std::size_t /*idx*/, std::size_t /*dim*/) const { return 0; }
            // Not used for this demonstration
            inline T kdtree_get_bbox_max(const std::size_t /*idx*/, std::size_t /*dim*/) const { return 0; }

            template <class BBOX> bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
        };

        template <typename T>
        class KDTreeWrapper {
            using KDTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<T, PointCloudAdaptor<T>>, PointCloudAdaptor<T>, 3>;

            public:
                KDTreeWrapper(const std::shared_ptr<pcl_lib::PointCloudConstVel<T>>& cloud) : cloud(cloud), params() {
                    // Create the kd-tree index
                    tree = std::make_unique<KDTree>(3, PointCloudAdaptor<T>(cloud), nanoflann::KDTreeSingleIndexAdaptorParams(10));
                    tree->buildIndex();
                    params.sorted = true;
                }

                void rebuild() {
                    tree->buildIndex();
                }

                std::tuple<std::size_t, std::vector<std::pair<std::uint32_t, T>>> kNearestNeighbors(const T& x, const T& y, const T& z, const std::size_t k) {
                    std::vector<std::uint32_t> collision_indices(k);
                    std::vector<T> out_dists_sqr(k);
                    // Define the query point
                    std::vector<T> query_pt = {x, y, z};
                    // Perform k-nearest neighbor search
                    const std::size_t num_results = tree->knnSearch(&query_pt[0], k, &collision_indices, &out_dists_sqr);
                    return std::make_tuple<std::size_t, std::vector<std::pair<std::uint32_t, T>>>(num_results, collision_indices);
                }

                std::tuple<std::size_t, std::vector<std::pair<std::uint32_t, T>>> radiusSearch(const T& x, const T& y, const T& z, const T& r) {
                    std::vector<std::pair<std::uint32_t, T>> collision_indices;
                    // Define the query point
                    std::vector<T> query_pt = {x, y, z};
                    // Perform radius search
                    const std::size_t num_results = tree->radiusSearch(&query_pt[0], r, collision_indices, params);
                    return std::make_tuple<std::size_t, std::vector<std::pair<std::uint32_t, T>>>(num_results, collision_indices);
                }

            private:
                std::shared_ptr<pcl_lib::PointCloudConstVel<T>> cloud;
                std::unique_ptr<KDTree> tree;
                nanoflann::SearchParams params;

        };
    } // namespace grid
} // namespace pcl_lib
