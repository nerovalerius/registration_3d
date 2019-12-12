/*    ___         _            _      __   __  _        _                  ____  ___  
 *   / __|  ___  | |__   ___  | |_    \ \ / / (_)  ___ (_)  ___   _ _     |__ / |   \ 
 *  | (__  / _ \ | '_ \ / _ \ |  _|    \ V /  | | (_-< | | / _ \ | ' \     |_ \ | |) |
 *   \___| \___/ |_.__/ \___/  \__|     \_/   |_| /__/ |_| \___/ |_||_|   |___/ |___/ 
 *                                                                                                                       
 *  Armin Niedermüller
 *
 */


#include <pcl/point_cloud.h>	
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>	
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <iostream>
#include <ros/ros.h>
#include <ros/topic.h>
#include <ros/console.h>	
#include <sensor_msgs/PointCloud2.h>

// ------------------------------------------------------------------------------------------------------------
//                                                  THE MAIN
// ------------------------------------------------------------------------------------------------------------

int main(int	argc,	char**	argv) {	

  // ------------------------------------------------------
  //  PROCESS ARGUMENTS
  // ------------------------------------------------------

  // We need at least two point clouds
  if (argc < 3){
    std::cout << "not enough arguments - see --help for more info" << std::endl;
    return -1;
  }

  // Argument-Vector
  std::vector<std::string> args;

  // Pointcloud-Vector
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;

  for (int i = 1; i < argc; i++){
    args.push_back(std::string(argv[i]));
  }

  // Parse Arguments
  for (const auto & arg : args){
    // User needs HELP
    if(arg.find("help") != std::string::npos){
      std::cout << "This program is designed to:"
                << "\n 1. Read two pointclouds from two ros PointCloud2 streams"
                << "\n 2. Downsample and filter both pointclouds"
                << "\n 3. Smooth Surfaces and make a coarse alignment"
                << "\n 4. Apply an iterative closest point algorithm"
                << "\n 5. Publish the Transformation Matrix to the ros /tf topic"
                << "\n ---------------------------------------------------------"
                << "\n Argumets: <cam_1_pointcloud2_topic> <cam_2_pointcloud2_topic>"
                << "\n Usually: /cam_1/depth/color/points and /cam_2/depth/color/points"
                << std::endl;
      return 0;
    }

  // ------------------------------------------------------
  //  STEP 1 - READ TWO POINTCLOUDS FROM TWO ROS TOPICS
  // ------------------------------------------------------

    // Arguments are point cloud streams? usually contain "points" in their path
    if(arg.find("points") != std::string::npos){
      // Define a new point cloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      // An object to catch the next pointcloud from a ROS PointCloud2 Topic
      sensor_msgs::PointCloud2ConstPtr cloud_msg =   ros::topic::waitForMessage<sensor_msgs::PointCloud2>(arg, ros::Duration(10));
      // Convert the catched message into a pointcloud object
      pcl::fromROSMsg(*cloud_msg, *cloud);
      // Add the point cloud to a vector
      clouds.push_back(cloud);
    }

  }


  // ------------------------------------------------------
  //  STEP 2 - DOWNSAMPLING
  // ------------------------------------------------------

  // Downsampling of the point clouds using a voxelgrid filter
  // significantly reduces computing time in the next steps

  // Configure the filter
  pcl::VoxelGrid<pcl::PointXYZ> voxelgrid_filter;
  voxelgrid_filter.setLeafSize(0.05, 0.05, 0.05);

  // Apply voxel grid filter
  for (auto & cloud : clouds){
    voxelgrid_filter.setInputCloud(cloud);
    voxelgrid_filter.filter(*cloud);
  }

  // ------------------------------------------------------
  //  STEP 3 - REMOVING OUTLIERS
  // ------------------------------------------------------

  // Configure the filter
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ>	outlier_filter;	
  outlier_filter.setMeanK(50);	
  outlier_filter.setStddevMulThresh(2.0);		

  // Apply outlier filter
  for (auto & cloud : clouds){
    outlier_filter.setInputCloud(cloud);	
    outlier_filter.filter(*cloud);
  }

  // ------------------------------------------------------
  //  STEP 4 - SMOOTHING SURFACES
  // ------------------------------------------------------

  // Pointcloud Vector with normals
  std::vector<pcl::PointCloud<pcl::PointNormal>> smoothed_clouds;

  // KD-Tree as search method for moving least squares algorithm
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Configure the filter
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls_smoothing;
  mls_smoothing.setComputeNormals (true);
  mls_smoothing.setPolynomialOrder (2);
  mls_smoothing.setSearchMethod (kd_tree);
  mls_smoothing.setSearchRadius (0.03);

  // Apply moving least squares algorithm
  for (auto & cloud : clouds){
    mls_smoothing.setInputCloud (cloud);
    // To also use the point normals, we need a PointNormal as type
    pcl::PointCloud<pcl::PointNormal> smoothed_cloud;
    mls_smoothing.process (smoothed_cloud);
    smoothed_clouds.push_back(smoothed_cloud);
  }


  // ------------------------------------------------------
  //  STEP 5 - APPLY ITERATIVE CLOSEST POINT ALGORITHM
  // ------------------------------------------------------

  // ------------------------------------------------------
  //  STEP 6 - PUBLISH TRANSFORMATION MATRIX TO TF TOPICS
  // ------------------------------------------------------

  // ROS NODE
  ros::init(argc,	argv,	"preprocess_align_publish");	
  std::cout << "PREPROCESS_ALIGN_PUBLISH" << std::endl;
  
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