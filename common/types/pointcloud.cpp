#include "pointcloud.h"

namespace pcl_lib
{
    template <typename T> 
    void PointCloud<T>::translate(const pcl_lib::Point<T>& translation) {
        for (auto &point : points) {
            point.translate(translation);
        }
    }

    template <typename T> 
    void PointCloud<T>::translate(const std::vector<T>& translation) {
        for (auto &point : points) {
            point.translate(translation);
        }
    }

    template <typename T> 
    void PointCloud<T>::translate(const Eigen::Matrix<T, 3, 1>& translation) {
        for (auto &point : points) {
            point.translate(translation);
        }
    }

    template <typename T>
    void PointCloud<T>::rotate(const Eigen::Matrix<T, 3, 3>& rotation) {
        for (auto &point : points) {
            point.rotate(rotation);
        }
    }

    template <typename T>
    void PointCloud<T>::rotate(const Eigen::Quaternion<T>& rotation) {
        for (auto &point : points) {
            point.rotate(rotation);
        }
    }

    template <typename T>
    void PointCloud<T>::rotate(const Eigen::Matrix<T, 3, 1>& rotation) {
        for (auto &point : points) {
            point.rotate(rotation);
        }
    }

    template <typename T>
    void PointCloud<T>::rotate(const std::vector<T>& rotation) {
        for (auto &point : points) {
            point.rotate(rotation);
        }
    }

    template <typename T>
    void PointCloud<T>::rotate(const T& angle_x, const T& angle_y, const T& angle_z) {
        for (auto &point : points) {
            point.rotate(angle_x, angle_y, angle_z);
        }
    }

    template <typename T>
    void PointCloud<T>::transform(const Eigen::Matrix<T, 4, 4>& transformation) {
        for (auto &point : points) {
            point.transform(transformation);
        }
    }

    template <typename T>
    void PointCloud<T>::transform(const Eigen::Matrix<T, 3, 3>& rotation, const Eigen::Matrix<T, 3, 1>& translation) {
        for (auto &point : points) {
            point.transform(rotation, translation);
        }
    }

    template <typename T>
    void PointCloud<T>::transform(const Eigen::Quaternion<T>& rotation, const Eigen::Matrix<T, 3, 1>& translation) {
        for (auto &point : points) {
            point.transform(rotation, translation);
        }
    }

    template <typename T>
    void PointCloud<T>::transform(const Eigen::Matrix<T, 3, 1>& rotation, const Eigen::Matrix<T, 3, 1>& translation) {
        for (auto &point : points) {
            point.transform(rotation, translation);
        }
    }

    template <typename T>
    void PointCloud<T>::transform(const std::vector<T>& rotation, const std::vector<T>& translation) {
        for (auto &point : points) {
            point.transform(rotation, translation);
        }
    }

    template <typename T>
    void PointCloud<T>::transform(const T& angle_x, const T& angle_y, const T& angle_z, const T& translation_x, const T& translation_y, const T& translation_z) {
        for (auto &point : points) {
            point.transform(angle_x, angle_y, angle_z, translation_x, translation_y, translation_z);
        }
    }
    
} // namespace pcl_lib
