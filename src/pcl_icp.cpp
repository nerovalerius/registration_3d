//   ___     _      ___     _              ___         _            _      __   __  _        _                  ____  ___  
//  | _ )   /_\    / __|   / |    ___     / __|  ___  | |__   ___  | |_    \ \ / / (_)  ___ (_)  ___   _ _     |__ / |   \ 
//  | _ \  / _ \  | (__    | |   |___|   | (__  / _ \ | '_ \ / _ \ |  _|    \ V /  | | (_-< | | / _ \ | ' \     |_ \ | |) |
//  |___/ /_/ \_\  \___|   |_|            \___| \___/ |_.__/ \___/  \__|     \_/   |_| /__/ |_| \___/ |_||_|   |___/ |___/ 
//                                                                                                                        
// Armin Niedermüller
//
// PCL_FRAMESAVER - Saves a Point Cloud from two Intel d435 ROS Pointcloud2 streams into two pcl.files
//

#include <iostream>
#include <ros/ros.h>
#include <ros/topic.h>
#include <ros/console.h>
#include <pcl/point_cloud.h>	
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>	
#include <sensor_msgs/PointCloud2.h>	
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>


// THE MAIN
	
main(int	argc,	char**	argv) {	

  ros::init(argc,	argv,	"pcl_icp");	
  std::cout << "PCL icp" << std::endl;
  
  // Variables
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
  
  
  // Save 1 Pointcloud Message into Message object
  sensor_msgs::PointCloud2ConstPtr cloud_1_msg =   ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/cam_1/depth/color/points", ros::Duration(10));
  sensor_msgs::PointCloud2ConstPtr cloud_2_msg =   ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/cam_2/depth/color/points", ros::Duration(10));
  
  
  // Convert from ROS Topic to point cloud object
  pcl::fromROSMsg(*cloud_1_msg, *cloud_1);
  pcl::fromROSMsg(*cloud_2_msg, *cloud_2);

  pcl::io::savePCDFileASCII("input_cloud_1.pcd", *cloud_1);
  pcl::io::savePCDFileASCII("input_cloud_2.pcd", *cloud_2);

  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>(cloud_2));
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_1);
  icp.setInputTarget(cloud_2);
  pcl::PointCloud<pcl::PointXYZ> resultCloud;
  icp.align(resultCloud);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

  pcl::io::savePCDFileASCII("icp_result.pcd", resultCloud);

// for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
//      cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
//      cloud_in->points[i].z << std::endl;
//  *cloud_out = *cloud_in;
//  std::cout << "size:" << cloud_out->points.size() << std::endl;
//  for (size_t i = 0; i < cloud_in->points.size (); ++i)
//    cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
//  std::cout << "Transformed " << cloud_in->points.size () << " data points:"
//      << std::endl;
//  for (size_t i = 0; i < cloud_out->points.size (); ++i)
//    std::cout << "    " << cloud_out->points[i].x << " " <<
//      cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
//  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//  icp.setInputSource(cloud_in);
//  icp.setInputTarget(cloud_out);
//  pcl::PointCloud<pcl::PointXYZ> Final;
//  icp.align(Final);
//  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
//  icp.getFitnessScore() << std::endl;
//  std::cout << icp.getFinalTransformation() << std::endl;



    //// Save the PointClouds into a file
    //pcl::io::savePCDFileASCII("./cam_1.pcd", cloud_1);
    //pcl::io::savePCDFileASCII("./cam_2.pcd", cloud_2);
//
//
    //std::cout << "finished - Pointclouds saved to file" << std::endl;

    return	0;	
}	