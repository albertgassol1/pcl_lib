// #include <pcl_lib/pointcloud.h>

// #include <pcl/point_types.h>
// #include <pcl/features/normal_3d.h>
// #include <Eigen/Core>

// namespace pcl_lib
// {
//     template <typename T> 
//     pcl_lib::PointCloud<T>::PointCloud(const PointCloud& other) : points(other.points), num_points(other.num_points), fixed_size(other.fixed_size) {}

//     template <typename T> 
//     pcl_lib::PointCloud<T>::PointCloud (const std::size_t n_points) : points(n_points), num_points(n_points), fixed_size(true) {
//         assert(n_points >= 0);
//     }

//     template <typename T> 
//     pcl_lib::PointCloud<T>::PointCloud(const std::vector<pcl_lib::Point<T>>& pts, const bool fixed) : points(pts), num_points(pts.size()), fixed_size(fixed) {}

//     template <typename T> 
//     void pcl_lib::PointCloud<T>::add(const Point<T>& point) {
//         assert(!fixed_size);
//         points.emplace_back(point);
//         num_points++;
//     }

//     template <typename T> 
//     void pcl_lib::PointCloud<T>::clear() {
//         points.clear();
//         num_points = 0;
//     }

//     template <typename T> 
//     std::size_t pcl_lib::PointCloud<T>::size() const {
//         return num_points;
//     }

//     template <typename T> 
//     const pcl_lib::Point<T>& pcl_lib::PointCloud<T>::operator[](const std::size_t i) const {
//         assert(i < num_points);
//         return points[i];
//     }

//     template <typename T> 
//     const pcl_lib::Point<T>& pcl_lib::PointCloud<T>::at(const std::size_t i) const {
//         assert(i < num_points);
//         return points[i];
//     }

//     template <typename T> 
//     const std::vector<pcl_lib::Point<T>>& pcl_lib::PointCloud<T>::get_points() const {
//         return points;
//     }

//     template <typename T> 
//     std::vector<pcl_lib::Point<T>> pcl_lib::PointCloud<T>::get_points_copy() const {
//         return std::vector<pcl_lib::Point<T>>(points);
//     }

//     template <typename T> 
//     bool pcl_lib::PointCloud<T>::is_fixed_size() const {
//         return fixed_size;
//     }

//     template <typename T> 
//     void pcl_lib::PointCloud<T>::translate(const pcl_lib::Point<T>& translation) {
//         for (auto &point : points) {
//             point.translate(translation);
//         }
//     }

//     template <typename T> 
//     void pcl_lib::PointCloud<T>::translate(const std::vector<T>& translation) {
//         for (auto &point : points) {
//             point.translate(translation);
//         }
//     }

//     template <typename T> 
//     void pcl_lib::PointCloud<T>::translate(const Eigen::Matrix<T, 3, 1>& translation) {
//         for (auto &point : points) {
//             point.translate(translation);
//         }
//     }

//     template <typename T>
//     void pcl_lib::PointCloud<T>::rotate(const Eigen::Matrix<T, 3, 3>& rotation) {
//         for (auto &point : points) {
//             point.rotate(rotation);
//         }
//     }

//     template <typename T>
//     void pcl_lib::PointCloud<T>::rotate(const Eigen::Quaternion<T>& rotation) {
//         for (auto &point : points) {
//             point.rotate(rotation);
//         }
//     }

//     template <typename T>
//     void pcl_lib::PointCloud<T>::rotate(const Eigen::Matrix<T, 3, 1>& rotation) {
//         for (auto &point : points) {
//             point.rotate(rotation);
//         }
//     }

//     template <typename T>
//     void pcl_lib::PointCloud<T>::rotate(const std::vector<T>& rotation) {
//         for (auto &point : points) {
//             point.rotate(rotation);
//         }
//     }

//     template <typename T>
//     void pcl_lib::PointCloud<T>::rotate(const T& angle_x, const T& angle_y, const T& angle_z) {
//         for (auto &point : points) {
//             point.rotate(angle_x, angle_y, angle_z);
//         }
//     }

//     template <typename T>
//     void pcl_lib::PointCloud<T>::transform(const Eigen::Matrix<T, 4, 4>& transformation) {
//         for (auto &point : points) {
//             point.transform(transformation);
//         }
//     }

//     template <typename T>
//     void pcl_lib::PointCloud<T>::transform(const Eigen::Matrix<T, 3, 3>& rotation, const Eigen::Matrix<T, 3, 1>& translation) {
//         for (auto &point : points) {
//             point.transform(rotation, translation);
//         }
//     }

//     template <typename T>
//     void pcl_lib::PointCloud<T>::transform(const Eigen::Quaternion<T>& rotation, const Eigen::Matrix<T, 3, 1>& translation) {
//         for (auto &point : points) {
//             point.transform(rotation, translation);
//         }
//     }

//     template <typename T>
//     void pcl_lib::PointCloud<T>::transform(const Eigen::Matrix<T, 3, 1>& rotation, const Eigen::Matrix<T, 3, 1>& translation) {
//         for (auto &point : points) {
//             point.transform(rotation, translation);
//         }
//     }

//     template <typename T>
//     void pcl_lib::PointCloud<T>::transform(const std::vector<T>& rotation, const std::vector<T>& translation) {
//         for (auto &point : points) {
//             point.transform(rotation, translation);
//         }
//     }

//     template <typename T>
//     void pcl_lib::PointCloud<T>::transform(const T& angle_x, const T& angle_y, const T& angle_z, const T& translation_x, const T& translation_y, const T& translation_z) {
//         for (auto &point : points) {
//             point.transform(angle_x, angle_y, angle_z, translation_x, translation_y, translation_z);
//         }
//     }

//     template <typename T>
//     void pcl_lib::PointCloud<T>::displace(const T& displacement) {
//         // Get normals
//         pcl::PointCloud<pcl::Normal>::Ptr normals = compute_normals();
//         // Displace points in the direction of the normals
//         for (auto &point : points) {
//             point.translate(point.getNormalVector3fMap() * displacement);
//         }
//     }

//     template <typename T>
//     const pcl::PointCloud<pcl::Normal>::Ptr& pcl_lib::PointCloud<T>::compute_normals() const{
//         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = to_pcl();
//         pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//         pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//         // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//         ne.setInputCloud(cloud);
//         pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
//         ne.setSearchMethod(tree);
//         ne.setRadiusSearch(0.03);
//         ne.compute(*normals);
//         return normals;
//     }
    
//     template <typename T>
//     const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_lib::PointCloud<T>::to_pcl() const{
//         pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//         for (const auto &point : points){
//             pcl_cloud->push_back(pcl::PointXYZ(point.x, point.y, point.z));
//         }
//         return pcl_cloud;
//     }
    
// } // namespace pcl_lib
