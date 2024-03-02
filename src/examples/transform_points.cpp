#include <boost/program_options.hpp>
#include <random>   

#include <pcl_lib/pointcloud_rgba.h>
#include <pcl_lib/io.h>
#include <pcl_lib/visualizer.h>
#include <Eigen/Core>

namespace po = boost::program_options;

int main( int argc, char *argv[])
{
    boost::filesystem::path input_path;
    boost::filesystem::path output_folder;
    float max_translation = 1.0;
    float min_translation = -1.0;
    float max_disp = 0.2;
    float min_disp = -0.2;
    float normals_search_radius = 0.05;
    std::string output_format = "ply";

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("input", po::value<boost::filesystem::path>(&input_path)->required(), "input file")
        ("output", po::value<boost::filesystem::path>(&output_folder)->required(), "output folder")
        ("min_translation", po::value<float>(&min_translation)->default_value(-1.0), "minimum translation")
        ("max_translation", po::value<float>(&max_translation)->default_value(1.0), "maximum translation")
        ("min_disp", po::value<float>(&min_disp)->default_value(-0.2), "minimum displacement")
        ("max_disp", po::value<float>(&max_disp)->default_value(0.2), "maximum displacement")
        ("output_format", po::value<std::string>(&output_format)->default_value("ply"), "output format")
        ("normals_search_radius", po::value<float>(&normals_search_radius)->default_value(0.05), "normals search radius")
    ;

    po::variables_map vm;
    po::store (po::command_line_parser (argc, argv).options (desc).run (), vm);
    po::notify (vm);

    if (vm.count("help") || !vm.count("input") || !vm.count("output")) {
        std::cout << desc << std::endl;
        return 1;
    }

    // Create output folder if it does not exist
    if (!boost::filesystem::exists(output_folder)) {
        boost::filesystem::create_directories(output_folder);
    }

    // Read pointcloud
    std::shared_ptr<pcl_lib::PointCloudRGBA<float>> pointcloud = std::make_shared<pcl_lib::PointCloudRGBA<float>>();
    pcl_lib::io::read<float>(input_path, pointcloud);
    // Make a copy of the pointcloud
    std::shared_ptr<pcl_lib::PointCloudRGBA<float>> pointcloud_original = std::make_shared<pcl_lib::PointCloudRGBA<float>>(*pointcloud);

    // Sample random translation
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> distribX(min_translation, max_translation);
    std::uniform_real_distribution<float> distribY(min_translation, max_translation);
    std::uniform_real_distribution<float> distribZ(min_translation, max_translation);
    Eigen::Vector3f translation = {distribX(gen), distribY(gen), distribZ(gen)};

    // Translate pointcloud
    pointcloud->translate(translation);
    std::cout << "Translated pointcloud with t = [" << translation[0] << ", " << 
                 translation[1] << ", " << translation[2] << "]" <<std::endl;
    pcl_lib::io::write<float>(output_folder / (std::string("translated_pcl.") + output_format), pointcloud);
    std::shared_ptr<pcl_lib::PointCloudRGBA<float>> appended_pcl = std::make_shared<pcl_lib::PointCloudRGBA<float>>();
    *appended_pcl = *pointcloud + *pointcloud_original;
    pcl_lib::visualizer::visualize(appended_pcl);

    // Translate pointcloud back
    pointcloud->translate(-translation);

    // Sample random rotation
    std::uniform_real_distribution<float> distribQuat(-1.0, 1.0);
    Eigen::Quaternion<float> quaternion(distribQuat(gen), distribQuat(gen), distribQuat(gen), distribQuat(gen));    
    quaternion.normalize();

    // Rotate pointcloud
    pointcloud->rotate(quaternion);
    std::cout << "Rotated pointcloud with q = [" << quaternion.w() << ", " << 
                 quaternion.x() << ", " << quaternion.y() << ", " << quaternion.z() << "]" <<std::endl;
    pcl_lib::io::write<float>(output_folder / (std::string("rotated_pcl.") + output_format), pointcloud);
    *appended_pcl = *pointcloud + *pointcloud_original;
    pcl_lib::visualizer::visualize(appended_pcl);

    // Rotate pointcloud back
    pointcloud->rotate(quaternion.inverse());

    // Displace pointcloud
    std::uniform_real_distribution<float> distribDisp(min_disp, max_disp);
    float displacement = distribDisp(gen);
    pointcloud->displace(displacement, normals_search_radius);
    std::cout << "Displaced pointcloud with d = " << displacement << std::endl;
    pcl_lib::io::write<float>(output_folder / (std::string("displaced_pcl.") + output_format), pointcloud);
    // Translate for visualization purposes
    pointcloud->translate(Eigen::Vector3f(1.5, 0.0, 0.0));
    *appended_pcl = *pointcloud + *pointcloud_original;
    pcl_lib::visualizer::visualize(appended_pcl);
}