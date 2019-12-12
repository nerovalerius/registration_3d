/*    ___         _            _      __   __  _        _                  ____  ___  
 *   / __|  ___  | |__   ___  | |_    \ \ / / (_)  ___ (_)  ___   _ _     |__ / |   \ 
 *  | (__  / _ \ | '_ \ / _ \ |  _|    \ V /  | | (_-< | | / _ \ | ' \     |_ \ | |) |
 *   \___| \___/ |_.__/ \___/  \__|     \_/   |_| /__/ |_| \___/ |_||_|   |___/ |___/ 
 *                                                                                                                       
 *  Armin Niedermüller
 *
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
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

// Nonlinear ICP also uses curvature - crete own Point class with x, y, z, curvature
class PointCurvature : public pcl::PointRepresentation<pcl::PointNormal>
{
  using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;

public:
  PointCurvature()
  {
    // x, y, z, curvature
    nr_dimensions_ = 4;
  }

  // Override method
  virtual void copyToFloatArray(const pcl::PointNormal &p, float *out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

// ------------------------------------------------------------------------------------------------------------
//                                                  THE MAIN
// ------------------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{

  // ------------------------------------------------------
  //  VARIABLES & CLASSES
  // ------------------------------------------------------

  // Argument-Vector
  std::vector<std::string> args;

  // Pointcloud-Vector
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;

  // ------------------------------------------------------
  //  START ROS NODE
  // ------------------------------------------------------

  // ROS NODE
  ros::init(argc, argv, "preprocess_align_publish");
  std::cout << "PREPROCESS_ALIGN_PUBLISH" << std::endl;

  // ------------------------------------------------------
  //  PROCESS ARGUMENTS
  // ------------------------------------------------------

  if (argc < 2)
  {
    std::cout << "not enough arguments - type --help for more info" << std::endl;
    return -1;
  }

  // Parse Arguments
  for (int i = 1; i < argc; i++)
  {
    args.push_back(std::string(argv[i]));
  }

  for (const auto &arg : args)
  {
    // User needs HELP
    if (arg.find("help") != std::string::npos)
    {
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
    else

        // ------------------------------------------------------
        //  STEP 1 - READ TWO POINTCLOUDS FROM TWO ROS TOPICS
        // ------------------------------------------------------

        // Arguments are point cloud streams? usually contain "points" in their path
        if (arg.find("points") != std::string::npos)
    {

      // Define a pointer to a cloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

      // An object to catch the next pointcloud from a ROS PointCloud2 Topic
      sensor_msgs::PointCloud2ConstPtr cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(arg, ros::Duration(10));
      // Convert the catched message into a pointcloud object
      pcl::fromROSMsg(*cloud_msg, *cloud_ptr);
      // Add the point cloud to a vector
      clouds.push_back(cloud_ptr);
    }
    else
    {
      std::cout << "No proper PointCloud ROS Topics given - type --help for more info " << std::endl;
      return -1;
    }
  }

  // Output
  std::cout << "Finished STEP 1 - READ TWO POINTCLOUDS FROM TWO ROS TOPICS" << std::endl;

  // ------------------------------------------------------
  //  STEP 2 - PASSTHROUGH_FILTER
  // ------------------------------------------------------

  // Configure the filter
  pcl::PassThrough<pcl::PointXYZ> passthrough_filter;
  passthrough_filter.setFilterFieldName("z");
  // Set filter from 0.5m to 2.2m
  passthrough_filter.setFilterLimits(0.5, 2.2);

  // Apply passthrough filter
  for (auto &cloud : clouds)
  {
    passthrough_filter.setInputCloud(cloud);
    passthrough_filter.filter(*cloud);
  }

  // Output
  std::cout << "Finished STEP 2 - PASSTHROUGH_FILTER" << std::endl;

  // ------------------------------------------------------
  //  STEP 3 - REMOVE OUTLIERS
  // ------------------------------------------------------

  // Configure the filter
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_filter;

  // Apply outlier filter
  for (auto &cloud : clouds)
  {
    outlier_filter.setInputCloud(cloud);
    outlier_filter.setMeanK(50);
    outlier_filter.setStddevMulThresh(2.0);
    outlier_filter.filter(*cloud);
  }

  // Output
  std::cout << "Finished STEP 3 - REMOVE OUTLIERS" << std::endl;

  // ------------------------------------------------------
  //  STEP 4 - DOWNSAMPLING
  // ------------------------------------------------------

  // Downsampling of the point clouds using a voxelgrid filter
  // significantly reduces computing time in the next steps

  // Configure the filter
  pcl::VoxelGrid<pcl::PointXYZ> voxelgrid_filter;
  voxelgrid_filter.setLeafSize(0.02, 0.02, 0.02);

  // Apply voxel grid filter
  for (auto &cloud : clouds)
  {
    voxelgrid_filter.setInputCloud(cloud);
    voxelgrid_filter.filter(*cloud);
  }

  // Output
  std::cout << "Finished STEP 4 - DOWNSAMPLING" << std::endl;

  // ------------------------------------------------------
  //  STEP 5 - SMOOTH SURFACES
  // ------------------------------------------------------

  // Pointcloud Vector with normals
  std::vector<pcl::PointCloud<pcl::PointNormal>> smoothed_clouds;

  // KD-Tree as search method for moving least squares algorithm
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>);

  // Configure the filter
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls_smoothing;
  mls_smoothing.setComputeNormals(true);
  mls_smoothing.setPolynomialOrder(2);
  mls_smoothing.setSearchMethod(kd_tree);
  mls_smoothing.setSearchRadius(0.03);

  // Apply moving least squares algorithm
  for (auto &cloud : clouds)
  {
    mls_smoothing.setInputCloud(cloud);
    // To also use the point normals, we need a PointNormal as type
    pcl::PointCloud<pcl::PointNormal> smoothed_cloud;
    mls_smoothing.process(smoothed_cloud);
    smoothed_clouds.push_back(smoothed_cloud);
  }

  // Output
  std::cout << "Finished STEP 5 - SMOOTHE SURFACES" << std::endl;

  // ------------------------------------------------------
  //  STEP 6 - COARSE MANUAL ALIGNMENT
  // ------------------------------------------------------

  // Rotation Matrices
  Eigen::Matrix4f transform_x = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f transform_z = Eigen::Matrix4f::Identity();

  // 45 Degrees
  int angle = -45;

  // Rotate fist pointcloud by 45° in X axis
  transform_x(0, 0) = 1;
  transform_x(1, 1) = cos(angle * M_PI / 180);
  transform_x(1, 2) = -sin(angle * M_PI / 180);
  transform_x(2, 1) = sin(angle * M_PI / 180);
  transform_x(2, 2) = cos(angle * M_PI / 180);
  transform_x(3, 3) = 1;

  // Apply 45° rotation - X axis
  pcl::transformPointCloud(smoothed_clouds.at(0), smoothed_clouds.at(0), transform_x);

  // 180 Degrees
  angle = 180;

  // Rotate fist pointcloud by 180 in Z axis
  transform_z(0, 0) = cos(angle * M_PI / 180);
  transform_z(0, 1) = -sin(angle * M_PI / 180);
  transform_z(1, 0) = sin(angle * M_PI / 180);
  transform_z(1, 1) = cos(angle * M_PI / 180);
  transform_z(2, 2) = 1;
  transform_z(3, 3) = 1;

  // Apply 180 rotation - Z axis
  pcl::transformPointCloud(smoothed_clouds.at(0), smoothed_clouds.at(0), transform_z);

  // Output
  std::cout << "Finished STEP 6 - COARSE MANUAL ALIGNMENT" << std::endl;

  // pcl::io::savePCDFileASCII("./cam_1.pcd", smoothed_clouds.at(0));
  // pcl::io::savePCDFileASCII("./cam_2.pcd", smoothed_clouds.at(1));

  // ------------------------------------------------------
  //  STEP 7 - ITERATIVE CLOSEST POINT ALGORITHM
  // ------------------------------------------------------

  //Points with x, y, z, curvature
  PointCurvature point_xyzc;
  // Curvature has to be weighted in x, y, z
  float weight[4] = {1.0, 1.0, 1.0, 1.0};
  point_xyzc.setRescaleValues(weight);

  // Configure the ICP algorithm
  pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> icp_nonlinear;
  // Maximum distance between two point correspondences - 0.000001 = 10cm
  icp_nonlinear.setTransformationEpsilon(1e-6);
  icp_nonlinear.setMaxCorrespondenceDistance(0.3);

  // Set PointXYZC as representation inside the registration method
  icp_nonlinear.setPointRepresentation(boost::make_shared<const PointCurvature>(point_xyzc));

  auto source = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>(smoothed_clouds.at(0));
  auto target = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>(smoothed_clouds.at(1));

  // Set both pointclouds - in order to align them
  icp_nonlinear.setInputSource(source);
  icp_nonlinear.setInputTarget(target);

  // Final Transformation
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f prev;

  auto result_cloud = source;
  icp_nonlinear.setMaximumIterations(2);
  for (int i = 0; i < 30; ++i)
  {
    PCL_INFO("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    source = result_cloud;

    // Estimate
    icp_nonlinear.setInputSource(source);
    icp_nonlinear.align(*result_cloud);

    // accumulate transformation between each Iteration
    T = icp_nonlinear.getFinalTransformation() * T;

    // if the difference between this transformation and the previous one
    // is smaller than the threshold, refine the process by reducing
    // the maximal correspondence distance
    if (std::abs((icp_nonlinear.getLastIncrementalTransformation() - prev).sum()) < icp_nonlinear.getTransformationEpsilon())
    {
      icp_nonlinear.setMaxCorrespondenceDistance(icp_nonlinear.getMaxCorrespondenceDistance() - 0.001);
    }
    prev = icp_nonlinear.getLastIncrementalTransformation();
  }

  // Fitness Score
  double fitness_score;

  fitness_score = icp_nonlinear.getFitnessScore();

  // ------------------------------------------------------
  //  STEP 8 - PUBLISH TRANSFORMATION MATRIX TO TF TOPICS
  // ------------------------------------------------------

  // Output
  std::cout << "Finished STEP 7 - ITERATIVE CLOSEST POINT ALGORITHM\n"
            << std::endl;
  std::cout << T << std::endl;
  std::cout << "\nFitness Score: " << fitness_score << std::endl;

  // ------------------------------------------------------
  //  END
  // ------------------------------------------------------

  return 0;
}