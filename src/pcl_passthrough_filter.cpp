//   ___     _      ___     _              ___         _            _      __   __  _        _                  ____  ___  
//  | _ )   /_\    / __|   / |    ___     / __|  ___  | |__   ___  | |_    \ \ / / (_)  ___ (_)  ___   _ _     |__ / |   \ 
//  | _ \  / _ \  | (__    | |   |___|   | (__  / _ \ | '_ \ / _ \ |  _|    \ V /  | | (_-< | | / _ \ | ' \     |_ \ | |) |
//  |___/ /_/ \_\  \___|   |_|            \___| \___/ |_.__/ \___/  \__|     \_/   |_| /__/ |_| \___/ |_||_|   |___/ |___/ 
//                                                                                                                        
// Armin Niedermüller
//
// PCL_PASSTHROUGH_FILTER - Filters Points outside a certain threshold
//

#include    <iostream>
#include	<ros/ros.h>
#include    <ros/topic.h>
#include    <ros/console.h>
#include	<pcl/point_cloud.h>	
#include    <pcl/point_types.h>
#include	<pcl_conversions/pcl_conversions.h>	
#include	<sensor_msgs/PointCloud2.h>	
#include    <pcl/io/pcd_io.h>
#include    <pcl/filters/passthrough.h>


#include <pcl/common/transforms.h>


// THE MAIN
	
main(int	argc,	char**	argv) {	

    ros::init(argc,	argv,	"pcl_passthrough");	
    std::cout << "PCL Passthrough Filter" << std::endl;

    // Variables
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>	cloud_1_filtered;
    pcl::PointCloud<pcl::PointXYZ>	cloud_2_filtered;


    // Save 1 Pointcloud Message into Message object
    sensor_msgs::PointCloud2ConstPtr cloud_1_msg =   ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/cam_1/depth/color/points", ros::Duration(10));
    sensor_msgs::PointCloud2ConstPtr cloud_2_msg =   ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/cam_2/depth/color/points", ros::Duration(10));


    // Convert from ROS Topic to point cloud object
    pcl::fromROSMsg(*cloud_1_msg, *cloud_1);
    pcl::fromROSMsg(*cloud_2_msg, *cloud_2);


    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass_1;
    pass_1.setInputCloud (cloud_1);
    pass_1.setFilterFieldName ("z");
    pass_1.setFilterLimits (1.0, atof(argv[1]));
    //pass.setFilterLimitsNegative (true);
    pass_1.filter (cloud_1_filtered);

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass_2;
    pass_2.setInputCloud (cloud_2);
    pass_2.setFilterFieldName ("z");
    pass_2.setFilterLimits (1.0, atof(argv[1]));
    //pass.setFilterLimitsNegative (true);
    pass_2.filter (cloud_2_filtered);


    // Save the PointClouds into a file
    pcl::io::savePCDFileASCII("./cam_1_passthrough_filtered.pcd", cloud_1_filtered);
    pcl::io::savePCDFileASCII("./cam_2_passthrough_filtered.pcd", cloud_2_filtered);


    std::cout << "finished - Pointclouds saved to file" << std::endl;

    return	0;	
}	