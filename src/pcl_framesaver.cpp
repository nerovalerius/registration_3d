//   ___     _      ___     _              ___         _            _      __   __  _        _                  ____  ___  
//  | _ )   /_\    / __|   / |    ___     / __|  ___  | |__   ___  | |_    \ \ / / (_)  ___ (_)  ___   _ _     |__ / |   \ 
//  | _ \  / _ \  | (__    | |   |___|   | (__  / _ \ | '_ \ / _ \ |  _|    \ V /  | | (_-< | | / _ \ | ' \     |_ \ | |) |
//  |___/ /_/ \_\  \___|   |_|            \___| \___/ |_.__/ \___/  \__|     \_/   |_| /__/ |_| \___/ |_||_|   |___/ |___/ 
//                                                                                                                        
// Armin Niedermüller
//
// PCL_FRAMESAVER - Saves a Point Cloud from two Intel d435 ROS Pointcloud2 streams into two pcl.files
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


#include <pcl/common/transforms.h>


// THE MAIN
	
main(int	argc,	char**	argv) {	

    ros::init(argc,	argv,	"pcl_framesaver");	
    std::cout << "PCL Framesaver - Save PointCloud2 topics into pcd files" << std::endl;

    // Variables
    pcl::PointCloud<pcl::PointXYZ>	cloud_1;
    pcl::PointCloud<pcl::PointXYZ>	cloud_2;


    // Save 1 Pointcloud Message into Message object
    sensor_msgs::PointCloud2ConstPtr cloud_1_msg =   ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/cam_1/depth/color/points", ros::Duration(10));
    sensor_msgs::PointCloud2ConstPtr cloud_2_msg =   ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/cam_1/depth/color/points_processed", ros::Duration(10));


    // Convert from ROS Topic to point cloud object
    pcl::fromROSMsg(*cloud_1_msg, cloud_1);
    pcl::fromROSMsg(*cloud_2_msg, cloud_2);

    // Rotate second point cloud
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
// rot on z 180°:
// -1 0  0  0
// 0 -1  0  0  
// 0  0  1  0
// 0  0  0  1
    transform(0,0) = -1;
    transform(1,1) = -1;
//    transform(2,2) = 1;
//    transform(3,3) = 1;
//
    // Rotate and translate the point cloud 
    pcl::PointCloud<pcl::PointXYZ> transformedCloud;
    pcl::transformPointCloud (cloud_2, transformedCloud, transform);


    // Save Pointclouds to files
    pcl::io::savePCDFileASCII("./cam_1.pcd", cloud_1);
    pcl::io::savePCDFileASCII("./cam_1_processed.pcd", cloud_2);
    pcl::io::savePCDFileASCII("./cam_2_transformed.pcd", transformedCloud);


    std::cout << "finished - Pointclouds saved to file" << std::endl;

    return	0;	
}	