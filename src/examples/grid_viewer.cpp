#include <boost/program_options.hpp>

#include <pcl_lib/pointcloud_constvel.h>
#include <grid_lib/grid.h>

namespace po = boost::program_options;

int main( int argc, char *argv[])
{
    float x_max, y_max, z_max = 1.0;
    float x_min, y_min, z_min = -x_max;
    float max_velocity = 0.1;
    std::size_t num_points = 800;
    float point_radius = 0.01;

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
    ;

    po::variables_map vm;
    po::store (po::command_line_parser (argc, argv).options (desc).run (), vm);
    po::notify (vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    const auto grid_handler = std::make_shared<pcl_lib::grid::GridHandler<float>>(x_min, x_max, y_min, y_max, z_min, z_max);
    grid_handler->initializeRandomPointcloud(num_points, point_radius, max_velocity);
    // grid_handler->update();
}