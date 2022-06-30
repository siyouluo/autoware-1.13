//https://pcl.readthedocs.io/projects/tutorials/en/master/matrix_transform.html#matrix-transform
#include <string>
#include <vector>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include "mkpath.h"

// This function displays the help
void showHelp(char *program_name)
{
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " cloud_filename.pcd" << std::endl;
    std::cout << "-h:  Show this help." << std::endl;
}

int main(int argc, char **argv)
{

    // Show help
    if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
    {
        showHelp(argv[0]);
        return 0;
    }
    std::vector<int> filenames;
    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
    if (filenames.size() != 1)
    {
        showHelp(argv[0]);
        return 0;
    }

    std::string filename = argv[filenames[0]];
    std::string filename_prefix = filename.substr(0, filename.find_last_of('.')) + "-transformed";
    std::string filename_basename = filename.substr(filename.find_last_of('/')+1, filename.find_last_of('.')-filename.find_last_of('/')-1);
    std::string filename_transformed = filename_prefix + "/" + filename_basename + "-transformed.pcd";

    std::cout << "loading point cloud from file: " << filename << std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZI>());
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(filename, *cloud_raw) == -1) //* load the file
    {
        std::cout << "Error loading point cloud " << filename << std::endl << std::endl;
        showHelp (argv[0]);
        return (-1);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*cloud_raw, *cloud_map);
    std::cout << "points num: " << cloud_map->points.size() << std::endl;

    Eigen::Matrix4f transform, transform_inverse;
    transform << 0.999691545963, -0.005265118554, 0.024284398183, -3.872330665588,
0.004438136239, 0.999412477016, 0.033982899040, 5.908507347107,
-0.024449052289, -0.033864635974, 0.999127328396, -51.049522399902,
0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000;

    transform_inverse = transform.inverse();
    // Print the transformation
    printf ("Transforming cloud with a Matrix4f\n");
    std::cout << "tranform (map->ref)\n" << transform << std::endl;
    std::cout << "transform_inverse (ref->map)\n" << transform_inverse << std::endl;

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*cloud_map, *transformed_cloud, transform);  
     light::mkpath(filename_prefix);
    std::cout << "saving transformed points_map to file: " << filename_transformed << std::endl;
    pcl::io::savePCDFileASCII(filename_transformed, *transformed_cloud);  

    return 0;
}
