# pcl_lib

This repository includes a [library](./src/pcl_lib/) for transforming point clouds (XYZRGBA), as well as point clouds (XYZ) with a constant velocity and radius. Additionally, it includes a [library](./src/grid_lib/) to generate a grid in which a pointcloud is constrained, and implements some collision handling between point-wall and poin-point. The repo includes two examples:
1. [Transform and visualize a point cloud](./src/examples/transform_points.cpp).
2. [Generate a grid and simulate points moving within it with collisions](./src/examples/grid_viewer.cpp).


## Dependencies
Before building the library and examples, install the following dependencies:
1. Eigen
2. Boost
3. VTK
4. Flann
5. Nanoflann
6. OpenMP
7. PCL (latest release)

In ubuntu 22.04, these can be installed with:

```bash
sudo apt install \
    libeigen3-dev \
    libboost-all-dev \
    libgmp3-dev \
    libflann-dev \
    libnanoflann-dev \
    openmpi-bin \
    openmpi-common \
    libvtk9-dev
```

### PCL
 PCL needs to be built from source to use the latest release (for visualization purposes). In ubuntu 22.04, it can be install with:

 ```bash
 git clone https://github.com/PointCloudLibrary/pcl.git --tag pcl-1.14.0
 cd pcl
 mkdir build
 cd build 
 cmake ..
 make -j5
 make install
 ```

 ## Building and running the examples

 This repo uses CMake to build.
 ```bash
 mkdir build
 cd build
 cmake ..
 make 
 ```

The examples executables are stored in ```build/bin```. To run the examples (and see the arguments that they suport) simply run:
``` bash
./build/bin/transform_points --input [path_to_input_pcl] --output [path_to_output_folder] --help
./build/bin/grid_viewer --help
```

A nice set of parameters for the grid viewer are:
```
./build/bin/grid_viewer --x_min -10 --y_min -10 --z_min -10 --x_max 10 --y_max 10 --z_max 10 --point_radius 0.3  --update true --max_velocity 1 --point_collisions true --viz_point_size 5 --dt 0.01 --num_points 800
```

## Docker
This repo provides a docker file to build the libraries, however it does not work well for visualization. If one wants to visualize pointclouds, this library needs to be built locally.
