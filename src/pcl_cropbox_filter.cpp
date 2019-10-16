//   ___     _      ___     _              ___         _            _      __   __  _        _                  ____  ___  
//  | _ )   /_\    / __|   / |    ___     / __|  ___  | |__   ___  | |_    \ \ / / (_)  ___ (_)  ___   _ _     |__ / |   \ 
//  | _ \  / _ \  | (__    | |   |___|   | (__  / _ \ | '_ \ / _ \ |  _|    \ V /  | | (_-< | | / _ \ | ' \     |_ \ | |) |
//  |___/ /_/ \_\  \___|   |_|            \___| \___/ |_.__/ \___/  \__|     \_/   |_| /__/ |_| \___/ |_||_|   |___/ |___/ 
//                                                                                                                        
// Armin Niedermüller
//
// PCL_CROPBOX_FILTER - Filters Points outside a certain threshold
//

#include <iostream>
#include <string>
#include <algorithm>
#include	<ros/ros.h>
#include    <ros/topic.h>
#include    <ros/console.h>
#include	<pcl/point_cloud.h>	
#include    <pcl/point_types.h>
#include	<pcl_conversions/pcl_conversions.h>	
#include	<sensor_msgs/PointCloud2.h>	
#include    <pcl/io/pcd_io.h>
#include    <pcl/filters/crop_box.h>
#include    <pcl/common/transforms.h>




// THE MAIN
	
main(int	argc,	char**	argv) {	

    ros::init(argc,	argv,	"pcl_cropbox");	
    std::cout << "PCL Cropbox Filter" << std::endl;

    if (argc < 2){
        std::cout << "Usage: " << "pcl_cropbox <input_file.pcd> <minX> <minY> <minZ> <maxX> <maxY> <maxZ>" << std::endl;
        return -1;
    }

    // Variables
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>	cloud_filtered;



    // Read Cloud from File
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1){ 

        std::cout << "could not read " << argv[1] << std::endl;
        return (-1);
    }

    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(atof(argv[2]), atof(argv[3]), atof(argv[4]), 1.0));      // X Y Z    //Cloud 1: -0.5, -6, 0.5, 1.0   //Cloud 2:       2, -6, 0.5, 1.0
    boxFilter.setMax(Eigen::Vector4f(atof(argv[5]), atof(argv[6]), atof(argv[7]), 1.0));      // X Y Z    //Cloud 1:    2,  6, 2.3, 1.0   //Cloud 2:    -0.5,  6, 2.3, 1.0
    boxFilter.setInputCloud(cloud);
    boxFilter.filter(cloud_filtered);

    // Give the output file a proper name 
    std::string output_file = argv[1];
    output_file = output_file.substr (0,output_file.length()-4);
    output_file+="_cropbox_filtered";
    output_file+=".pcd";


    // Save the PointClouds into a file
    pcl::io::savePCDFileASCII(output_file, cloud_filtered);


    std::cout << "finished - Pointclouds filtered cropbox filter " << std::endl;

    return	0;	
}	