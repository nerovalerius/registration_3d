# Point Cloud Registration of two Intel D435i 3D-Cameras with the Iterative Closest Point Algorithm 
Point Cloud Library, Robot Operating System, Intel D435

![alt text](https://repository-images.githubusercontent.com/215542871/3e9e6c00-24e2-11ea-9a2c-60b583b701e3)

![alt text](https://i.ibb.co/W3w2vqp/4000-3000-max.jpg)

This is a collection of ros launch files to start two intel d435 ros camera nodes and to apply a pointcloud registration
on the two images. The resulting transformation is then applied via ros /tf topics and the clouds are vizualized inside rviz.

Simply clone this git into your ros_workspace/src folder and build it with catkin_make.

## Prerequisites
Download the following rosbag file: [Link](https://drive.google.com/file/d/1eIEW_tNSs0p7Sgny7x9dS-HAtSRvAcDm/view?usp=sharing)

This is a 1 second sequence of both 3D cameras in our laboratory.

Use `rosbag play -l 3d_cams.bag` to loop the 3D camera topics endlessly, simulating our laboratory environment.
A real camera is not needed in this case, and you can test everything.

The rosbag was recorded from both realsense-ros camera nodes with:
```sh
rosbag record -a -x "(.*)/cam_1/infra1/(.*)|(.*)/cam_2/infra1/(.*)|(.*)/cam_1/infra2/(.*)|(.*)/cam_2/infra2/(.*)|(.*)/cam_1/color/(.*)|(.*)/cam_2/color/(.*)|(.*)/cam_1/depth/image_rect_raw(.*)|(.*)/cam_2/depth/image_rect_raw/(.*)" --duration=1 -O 3d_cams.bag
```

## Fully automated start script
This script starts both 3d cam nodes, aligns their pointclouds via rostopic tf, vizualises the result in rviz
Parameters to set:
1. The serialnumbers of your 3d cams inside the start_3d_cams.launch file
2. The coarse manual transformation of both cams before fully aligning them via the ICP -> void TransformationCalculator::CoarseManualAlignment() inside preprocess_align_publish.cpp

## launch/start_3d_cams.launch
Starts both intel D435i ROS Nodes. Change serial numbers inside this launch file to fit your models.

## launch/icp_align_rviz.launch
Start your two ROS PointCloud2 topics before this program is started. Works with robags or real nodes. See launch/start_3d_cams.launch

This launch file starts preprocess_align_publish, which consists of the following steps:
Arguments can also be added into the launch file.

## PREPROCESS_ALIGN_PUBLISH 
### Usage
```sh
rosrun regisration_3d preprocess_align_publish /cam_1/depth/color/points /cam_2/depth/color/points manualalignment=true passthrough=true downsampling=true outlier=true displayresult=true mls=true algorithm=gicp publishtoros=true

This program is designed to:
- 1a. Read two pointclouds from ros PointCloud2 streams
- 1b. OR read two pointclouds from .pcd files
- 2. Downsample and filter both pointclouds
- 3. Smooth Surfaces and make a coarse alignment
- 4. Apply an alignment algorithm
- 5. Publish the Transformation Matrix to the ros /tf topic
- 6. Display the alignment result
   
 ---------------------------------------------------------
- Arguments: <cam_1_pointcloud2_topic> <cam_2_pointcloud2_topic>
- OR: <cam_1_pointcloud_file.pcd> <cam_2_pointcloud_file.pcd>
- Usually: /cam_1/depth/color/points and /cam_2/depth/color/points
- This program only reads the pointclouds and applies an ICP if you give no arguments but the topics or files

 ---------------------------------------------------------
 The following arguments activate the single steps:
   * algorithm=<alignment_alogrithm> - alignment_alogrithm has to be on of the following:
      * icp - regular icp
      * gicp - generalized icp
      * nlicp - nonlinear icp
      * fpfh - Fast Point Feature Histogram (default)
   * allsteps=true 				   - activates all preprocessing steps
   * passthrough=true				 - activates passthrough filter
   * downsampling=true			 - activates downsampling
   * outlier=true				     - activates outlier filter
   * mls=true					       - activates mls_smoothing
   * manualalignment=true 	 - activates manual pre alignment. only used with icp variants
   * publishtoros=true			 - activates quaternion transformation publishing to ros /tf topic
   * displayresult=true			 - displays alignment result
