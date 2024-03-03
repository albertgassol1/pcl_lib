#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Core>

namespace pcl_lib {
    template <typename T> 
        struct Point { 
            T x;
            T y;
            T z;

            Point() = default;
            
            Point(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}

            const T& operator[](const std::size_t index) const {
                switch (index) {
                    case 0:
                        return x;
                    case 1:
                        return y;
                    case 2:
                        return z;
                    default:
                        throw std::out_of_range("Index out of range");
                }
            }

            void normalize() {
                T length = std::sqrt(x * x + y * y + z * z);
                x /= length;
                y /= length;
                z /= length;
            }

            void translate(const T& translation_x, const T& translation_y, const T& translation_z) {
                x += translation_x;
                y += translation_y;
                z += translation_z;
            }

            void translate(const Point<T>& translation) {
                x += translation.x;
                y += translation.y;
                z += translation.z;
            }

            void translate(const std::vector<T>& translation) {
                assert(translation.size() == 3);
                x += translation[0];
                y += translation[1];
                z += translation[2];
            }

            void translate(const Eigen::Matrix<T, 3, 1>& translation) {
                x += translation(0);
                y += translation(1);
                z += translation(2);
            }

            void rotate(const Eigen::Matrix<T, 3, 3>& rotation) {
                Eigen::Matrix<T, 3, 1> point(x, y, z);
                point = rotation * point;
                x = point(0);
                y = point(1);
                z = point(2);
            }

            void rotate(const Eigen::Quaternion<T>& rotation) {
                Eigen::Matrix<T, 3, 1> point(x, y, z);
                point = rotation * point;
                x = point(0);
                y = point(1);
                z = point(2);
            }

            void rotate(const Eigen::Matrix<T, 3, 1>& rotation) {
                assert(rotation.size() == 3);
                Eigen::Matrix<T, 3, 3> rotation_matrix;
                rotation_matrix = Eigen::AngleAxis<T>(rotation(0), Eigen::Matrix<T, 3, 1>::UnitX())
                    * Eigen::AngleAxis<T>(rotation(1), Eigen::Matrix<T, 3, 1>::UnitY())
                    * Eigen::AngleAxis<T>(rotation(2), Eigen::Matrix<T, 3, 1>::UnitZ());
                rotate(rotation_matrix);
            }

            void rotate(const std::vector<T>& rotation) {
                assert(rotation.size() == 3);
                Eigen::Matrix<T, 3, 3> rotation_matrix;
                rotation_matrix = Eigen::AngleAxis<T>(rotation[0], Eigen::Matrix<T, 3, 1>::UnitX())
                    * Eigen::AngleAxis<T>(rotation[1], Eigen::Matrix<T, 3, 1>::UnitY())
                    * Eigen::AngleAxis<T>(rotation[2], Eigen::Matrix<T, 3, 1>::UnitZ());
                rotate(rotation_matrix);
            }

            void rotate(const T& angle_x, const T& angle_y, const T& angle_z) {
                Eigen::Matrix<T, 3, 3> rotation_matrix;
                rotation_matrix = Eigen::AngleAxis<T>(angle_x, Eigen::Matrix<T, 3, 1>::UnitX())
                    * Eigen::AngleAxis<T>(angle_y, Eigen::Matrix<T, 3, 1>::UnitY())
                    * Eigen::AngleAxis<T>(angle_z, Eigen::Matrix<T, 3, 1>::UnitZ());
                rotate(rotation_matrix);
            }

            void transform(const Eigen::Matrix<T, 4, 4>& transformation) {
                Eigen::Matrix<T, 4, 1> point(x, y, z, 1);
                point = transformation * point;
                x = point(0);
                y = point(1);
                z = point(2);
            }

            void transform(const Eigen::Matrix<T, 3, 3>& rotation, const Eigen::Matrix<T, 3, 1>& translation) {
                rotate(rotation);
                translate(translation);
            }

            void transform(const Eigen::Quaternion<T>& rotation, const Eigen::Matrix<T, 3, 1>& translation) {
                rotate(rotation);
                translate(translation);
            }

            void transform(const Eigen::Matrix<T, 3, 1>& rotation, const Eigen::Matrix<T, 3, 1>& translation) {
                rotate(rotation);
                translate(translation);
            }

            void transform(const std::vector<T>& rotation, const std::vector<T>& translation) {
                rotate(rotation);
                translate(translation);
            }

            void transform(const T& angle_x, const T& angle_y, const T& angle_z, const T& translation_x, const T& translation_y, const T& translation_z) {
                rotate(angle_x, angle_y, angle_z);
                translate(translation_x, translation_y, translation_z);
            }

            Point<T> operator+(const Point<T>& rhs) const {
                return Point<T>(x + rhs.x, y + rhs.y, z + rhs.z);
            }

            Point<T> operator-(const Point<T>& rhs) const {
                return Point<T>(x - rhs.x, y - rhs.y, z - rhs.z);
            }

            Point<T> operator*(const T& rhs) const {
                return Point<T>(x * rhs, y * rhs, z * rhs);
            }

            Point<T> operator/(const T& rhs) const {
                return Point<T>(x / rhs, y / rhs, z / rhs);
            }
    };

    template <typename T> 
        struct PointRGBA : Point<T>{ 
            T r;
            T g;
            T b;
            T a;
        
            PointRGBA() = default;
            
            PointRGBA(T _x, T _y, T _z, T _r, T _g, T _b, T _a) : Point<T>(_x, _y, _z), r(_r), g(_g), b(_b), a(_a) {}
    }; 

    template <typename T>
        struct PointConstVel : Point<T> {
            Point<T> velocity;
            T radius;

            PointConstVel() = default;

            PointConstVel(T _x, T _y, T _z, T _vx, T _vy, T _vz, T _radius) : Point<T>(_x, _y, _z), velocity(_vx, _vy, _vz), radius(_radius) {}

            PointConstVel(const Point<T>& point, const Point<T>& _velocity, T _radius) : Point<T>(point), velocity(_velocity), radius(_radius) {}

            void update(const T& dt) {
                translate(velocity * dt);
            }
        };
} // namespace pcl_lib
