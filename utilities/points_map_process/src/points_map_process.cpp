//https://pcl.readthedocs.io/projects/tutorials/en/master/matrix_transform.html#matrix-transform
#include <string>
#include <vector>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/impl/common.hpp>
// #include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

static void split_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr map, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &submaps)
{
    submaps.clear();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZ>());
    // pcl::copyPointCloud(*map, *cloud_map);
    cloud_map = map;
    pcl::PointXYZ min, max;
    pcl::getMinMax3D(*cloud_map, min, max);
    std::cout << "map min: " << min.x << ", " << min.y << ", " << min.z << std::endl;
    std::cout << "map max: " << max.x << ", " << max.y << ", " << max.z << std::endl;
    int blocksize = 100;
    pcl::PassThrough<pcl::PointXYZ> pass;
    for (float x_begin = (float)blocksize * (float)((int)(min.x / blocksize)); x_begin <= max.x; x_begin += blocksize)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_x_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pass.setInputCloud(cloud_map);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(x_begin - blocksize, x_begin);
        pass.filter(*cloud_x_filtered);
        for (float y_begin = (float)blocksize * (float)((int)(min.y / blocksize)); y_begin <= max.y; y_begin += blocksize)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_y_filtered(new pcl::PointCloud<pcl::PointXYZ>());
            pass.setInputCloud(cloud_x_filtered);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(y_begin - blocksize, y_begin);
            pass.filter(*cloud_y_filtered);
            if (cloud_y_filtered->points.size() != 0)
            {
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_y_filtered_xyzi(new pcl::PointCloud<pcl::PointXYZI>());
                pcl::copyPointCloud(*cloud_y_filtered, *cloud_y_filtered_xyzi);
                submaps.push_back(cloud_y_filtered_xyzi);
            }
        }
    }
}

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
    std::string filename_prefix = filename.substr(0, filename.find_last_of('.'));
    std::string filename_transformed_prefix = filename_prefix + "-transformed";
    std::string filename_transformed = filename_transformed_prefix + ".pcd";

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
    transform << 0.94617846,  0.32332626, -0.01436857,  1260.84009488,
                -0.32338822,  0.94626392, -0.00215662, 94.83101367,
                0.01289917,  0.00668717,  0.99989444, 17.91198603,
                0, 0, 0, 1;
    transform_inverse = transform.inverse();
    // Print the transformation
    printf ("Transforming cloud with a Matrix4f\n");
    std::cout << "tranform (map->ref)\n" << transform << std::endl;
    std::cout << "transform_inverse (ref->map)\n" << transform_inverse << std::endl;

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*cloud_map, *transformed_cloud, transform_inverse);  
    std::cout << "saving transformed points_map to file: " << filename_transformed << std::endl;
    pcl::io::savePCDFileASCII(filename_transformed, *transformed_cloud);  
    
    std::cout << "splitting transformed points_map to susbmaps" << std::endl;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> submaps;
    split_pointcloud(transformed_cloud, submaps);
    for(int i=0;i<(int)submaps.size();i++)
    {
      std::string filename_submap = filename_transformed_prefix + "-" + std::to_string(i) + ".pcd";
      std::cout << "saving submap to file: " << filename_submap << std::endl;
      pcl::io::savePCDFileASCII(filename_submap, *submaps[i]);
    }
    std::cout << "Saved " << submaps.size() << " submaps to " << filename_prefix << "-transformed-*.pcd." << std::endl;

    return 0;
}
