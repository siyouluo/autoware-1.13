//https://pcl.readthedocs.io/projects/tutorials/en/master/matrix_transform.html#matrix-transform
#include <string>
#include <vector>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <fstream>
#include "mkpath.h"

// This function displays the help
void showHelp(char *program_name)
{
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " cloud_filename.pcd transform_matrix.txt [-i] [--inverse]" << std::endl;
    std::cout << "-h:  Show this help." << std::endl;
}


/* 将txt文件数据写入到矩阵中, i.e.
0.89996983 0.43582784 0.01041164 1263.21656413
-0.43594505 0.8995618  0.02721162 -59.07489965
0.00249367 -0.02902854 0.99957547 -11.48439517
0 0 0 1
*/
Eigen::Matrix4f load_mat_from_txt(const char* filename,int rows=4,int cols=4)
{
	std::fstream fsread(filename);
	Eigen::Matrix4f M;
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			fsread >> M(i, j);
		}
	}
	fsread.close();
	return M;
}


int main(int argc, char **argv)
{

    // Show help
    if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
    {
        showHelp(argv[0]);
        return 0;
    }

    bool use_inverse_matrix = false;
    if (pcl::console::find_switch (argc, argv, "-i")||pcl::console::find_switch (argc, argv, "--inverse"))
    {
        use_inverse_matrix = true;
        std::cout << "use the inverse matrix to transform pointcloud" << std::endl;
    }
    else
    {
        use_inverse_matrix = false;
        std::cout << "use the provided matrix to transform pointcloud" << std::endl;
    }


    std::vector<int> filenames, transform_matrix_filenames;
    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
    if (filenames.size() != 1)
    {
        showHelp(argv[0]);
        return 0;
    }
    transform_matrix_filenames = pcl::console::parse_file_extension_argument(argc, argv, ".txt");
    if (transform_matrix_filenames.size() != 1)
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

    Eigen::Matrix4f transform = load_mat_from_txt(argv[transform_matrix_filenames[0]], 4, 4);
    Eigen::Matrix4f  transform_inverse = transform.inverse();
    // Print the transformation
    printf ("Transforming cloud with a Matrix4f\n");
    std::cout << "transform \n" << transform << std::endl;
    std::cout << "transform^{-1} \n" << transform_inverse << std::endl;

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    if(use_inverse_matrix)
    {
        pcl::transformPointCloud (*cloud_map, *transformed_cloud, transform_inverse); 
    }
    else
    {
        pcl::transformPointCloud (*cloud_map, *transformed_cloud, transform); 
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
    pcl::copyPointCloud(*transformed_cloud, *output_cloud);
    // save transformed pointcloud into *-transformed.pcd
    light::mkpath(filename_prefix);
    std::cout << "saving transformed points_map to file: " << filename_transformed << std::endl;
    pcl::io::savePCDFileASCII(filename_transformed, *output_cloud);  

    return 0;
}
