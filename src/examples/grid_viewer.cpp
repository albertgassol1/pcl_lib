#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <pcl_lib/pointcloud_constvel.h>
#include <pcl_lib/io.h>
#include <grid_lib/grid.h>
#include <visualizer/visualizer.h>

namespace po = boost::program_options;

int main( int argc, char *argv[])
{
    float x_max, y_max, z_max = 1.0;
    float x_min, y_min, z_min = -x_max;
    float max_velocity = 0.1;
    std::size_t num_points = 800;
    float point_radius = 0.01;
    float dt = 0.01;
    bool update = true;
    bool point_collisions = true;
    float viz_point_size = 3.0;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("x_min", po::value<float>(&x_min)->default_value(-1.0), "x_min")
        ("x_max", po::value<float>(&x_max)->default_value(1.0), "x_max")
        ("y_min", po::value<float>(&y_min)->default_value(-1.0), "y_min")
        ("y_max", po::value<float>(&y_max)->default_value(1.0), "y_max")
        ("z_min", po::value<float>(&z_min)->default_value(-1.0), "z_min")
        ("z_max", po::value<float>(&z_max)->default_value(1.0), "z_max")
        ("max_velocity", po::value<float>(&max_velocity)->default_value(0.1), "max_velocity")
        ("num_points", po::value<std::size_t>(&num_points)->default_value(800), "num_points")
        ("point_radius", po::value<float>(&point_radius)->default_value(0.01), "point_radius")
        ("dt", po::value<float>(&dt)->default_value(0.01), "dt")
        ("update", po::value<bool>(&update)->default_value(true), "update")
        ("point_collisions", po::value<bool>(&point_collisions)->default_value(true), "point_collisions")
        ("viz_point_size", po::value<float>(&viz_point_size)->default_value(10.0), "viz_point_size")
    ;

    po::variables_map vm;
    po::store (po::command_line_parser (argc, argv).options (desc).run (), vm);
    po::notify (vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    std::shared_ptr<pcl_lib::grid::GridHandler<float>> grid_handler = std::make_shared<pcl_lib::grid::GridHandler<float>>(x_min, x_max, y_min, y_max, z_min, z_max);
    grid_handler->initializeRandomPointcloud(num_points, point_radius, max_velocity);
    
    pcl_lib::visualizer::visualizeGrid(grid_handler, dt, update, point_collisions, viz_point_size);
}
