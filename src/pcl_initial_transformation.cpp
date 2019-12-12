//   ___     _      ___     _              ___         _            _      __   __  _        _                  ____  ___  
//  | _ )   /_\    / __|   / |    ___     / __|  ___  | |__   ___  | |_    \ \ / / (_)  ___ (_)  ___   _ _     |__ / |   \ 
//  | _ \  / _ \  | (__    | |   |___|   | (__  / _ \ | '_ \ / _ \ |  _|    \ V /  | | (_-< | | / _ \ | ' \     |_ \ | |) |
//  |___/ /_/ \_\  \___|   |_|            \___| \___/ |_.__/ \___/  \__|     \_/   |_| /__/ |_| \___/ |_||_|   |___/ |___/ 
//                                                                                                                        
// Armin Niedermüller
//
// PCL_INITIAL_TRANSFORM - Makes the initial transform for ICP
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
#include <pcl/filters/filter.h>

#include <pcl/common/transforms.h>



// THE MAIN
	
main(int	argc,	char**	argv) {	

  //  ros::init(argc,	argv,	"pcl_initial_transformation");	
    std::cout << "PCL Framesaver - Save PointCloud1 topics into pcd files" << std::endl;

    // Variables
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);


    // Read Cloud from File
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud_1) == -1){ 

        std::cout << "could not read " << argv[1] << std::endl;
        return (-1);
    }

    // Variables
   // pcl::PointCloud<pcl::PointXYZ>	cloud_1;


    // Save 1 Pointcloud Message into Message object
 //   sensor_msgs::PointCloud2ConstPtr cloud_1_msg =   ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/cam_1/depth/color/points", ros::Duration(10));
  

    // Convert from ROS Topic to point cloud object
 //   pcl::fromROSMsg(*cloud_1_msg, cloud_1);
 

    // Rotate second point cloud
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    // rot on z 180°:
    // -1 0  0  0
    // 0 -1  0  0  
    // 0  0  1  0
    // 0  0  0  1

int grad = -45;
    // rotation in x um 45° 
    transform(0,0) = 1;
    transform(1,1) = cos(grad*M_PI / 180);
    transform(1,2) = -sin(grad*M_PI / 180);
    transform(2,1) = sin(grad*M_PI / 180);
    transform(2,2) = cos(grad*M_PI / 180);
    transform(3,3) = 1;

    pcl::PointCloud<pcl::PointXYZ> transformedCloud;
    pcl::transformPointCloud (*cloud_1, transformedCloud, transform);

    grad = 180;
    Eigen::Matrix4f transformZ = Eigen::Matrix4f::Identity();
    // rotation in z um 180° 
    transformZ(0,0) = cos(grad*M_PI / 180);
    transformZ(0,1) = -sin(grad*M_PI / 180);
    transformZ(1,0) = sin(grad*M_PI / 180);
    transformZ(1,1) = cos(grad*M_PI / 180);
    transformZ(2,2) = 1;
    transformZ(3,3) = 1;

    pcl::PointCloud<pcl::PointXYZ> transformedCloud_z;
    pcl::transformPointCloud (transformedCloud, transformedCloud_z, transformZ);


    // Give the output file a proper name 
    std::string output_file = argv[1];
    output_file = output_file.substr (0,output_file.length()-4);
    output_file+="_transformed";
    output_file+=".pcd";

    // Save the PointClouds into a file
    pcl::io::savePCDFileASCII(output_file, transformedCloud_z);


    std::cout << "finished - Pointclouds saved to file" << std::endl;

    return	0;	
}	