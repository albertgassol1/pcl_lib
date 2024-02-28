#pragma once

namespace pcl_lib {
    template <typename T> 
        struct Point { 
            T x;
            T y;
            T z;
    };

    template <typename T> 
        struct PointRGBA : Point{ 
            T r;
            T g;
            T b;
            T a;
    }; 
} // namespace pcl_lib