//      ___         _            _      __   __  _        _                  ____  ___  
//     / __|  ___  | |__   ___  | |_    \ \ / / (_)  ___ (_)  ___   _ _     |__ / |   \ 
//    | (__  / _ \ | '_ \ / _ \ |  _|    \ V /  | | (_-< | | / _ \ | ' \     |_ \ | |) |
//     \___| \___/ |_.__/ \___/  \__|     \_/   |_| /__/ |_| \___/ |_||_|   |___/ |___/ 
//                                                                                                                        
// Armin Niedermüller
//
// PCL_Viewer - View pcd files
//


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
int
main (int argc, char** argv)
{

    if(argc < 1){
        std::cout << "usage: pcl_viewer <file.pcd>";
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);


    // Read Cloud from File
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1){ 

        std::cout << "could not read " << argv[1] << std::endl;
        return (-1);
    }


    // View the Cloud
    pcl::visualization::CloudViewer viewer_1 ("Simple Cloud Viewer");

    viewer_1.showCloud (cloud);

    while (!viewer_1.wasStopped ()){

    }


  return (0);
}