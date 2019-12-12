//   ___     _      ___     _              ___         _            _      __   __  _        _                  ____  ___  
//  | _ )   /_\    / __|   / |    ___     / __|  ___  | |__   ___  | |_    \ \ / / (_)  ___ (_)  ___   _ _     |__ / |   \ 
//  | _ \  / _ \  | (__    | |   |___|   | (__  / _ \ | '_ \ / _ \ |  _|    \ V /  | | (_-< | | / _ \ | ' \     |_ \ | |) |
//  |___/ /_/ \_\  \___|   |_|            \___| \___/ |_.__/ \___/  \__|     \_/   |_| /__/ |_| \___/ |_||_|   |___/ |___/ 
//                                                                                                                        
// Armin Niedermüller
//
// PCL_PASSTHROUGH_FILTER - Filters Points outside a certain threshold
//

#include <iostream>
#include <string>
#include <algorithm>
#include <ros/ros.h>
#include <ros/topic.h>
#include <ros/console.h>
#include <pcl/point_cloud.h>	
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>	
#include <sensor_msgs/PointCloud2.h>	
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>




// THE MAIN
	
main(int	argc,	char**	argv) {	

    ros::init(argc,	argv,	"pcl_passthrough");	
    std::cout << "PCL Passthrough Filter" << std::endl;

    if (argc < 2){
        std::cout << "Usage: " << "pcl_passthrough <pointcloud.pcd> <z-distance>" << std::endl;
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

    // Filtering
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud (cloud);
    pass_z.setFilterFieldName ("z");
    pass_z.setFilterLimits (0.5, atof(argv[2]));
    //pass.setFilterLimitsNegative (true);
    pass_z.filter (cloud_filtered);


    // Give the output file a proper name 
    std::string output_file = argv[1];
    output_file = output_file.substr (0,output_file.length()-4);
    output_file+="_passthrough_filtered_z";
    output_file+=argv[2];
    output_file+=".pcd";

    // Save the PointClouds into a file
    pcl::io::savePCDFileASCII(output_file, cloud_filtered);


    std::cout << "finished - Pointclouds filtered with passthrough filter and a value of z-max: " << argv[2] << std::endl;

    return	0;	
}	