# Point Cloud Registration of two Intel D435i 3D-Cameras with the Iterative Closest Point Algorithm 
Point Cloud Library, Robot Operating System, Intel D435

![alt text](https://repository-images.githubusercontent.com/215542871/3e9e6c00-24e2-11ea-9a2c-60b583b701e3)

![alt text](https://i.ibb.co/W3w2vqp/4000-3000-max.jpg)

This is a collection of ros launch files to start two intel d435 ros camera nodes and to apply a pointcloud registration
on the two images. The resulting transformation is then applied via ros /tf topics and the clouds are vizualized inside rviz.

Simply clone this git into your ros_workspace/src folder and build it with catkin_make.

## launch/start_3d_cams.launch
Starts both intel D435i ROS Nodes. Change serial numbers inside this launch file to fit your models.

## launch/icp_align_rviz.launch
Start your two ROS PointCloud2 topics before this program is started. Works with robags or real nodes. See launch/start_3d_cams.launch

This launch file starts preprocess_align_publish, which consists of the following steps:
Arguments can also be added into the launch file.

## PREPROCESS_ALIGN_PUBLISH 
example call: rosrun perception

This program is designed to:  
 1a. Read two pointclouds from ros PointCloud2 streams  
 1b. OR read two pointclouds from .pcd files  
 2. Downsample and filter both pointclouds  
 3. Smooth Surfaces and make a coarse alignment  
 4. Apply an iterative closest point algorithm 
 5. Publish the Transformation Matrix to the ros /tf topic
 
 ---------------------------------------------------------
 The following arguments read two point clouds:
 * FROM ROS TOPIC:\
   <cam_1_pointcloud2_topic> <cam_2_pointcloud2_topic>  
   e.g: /cam_1/depth/color/points and /cam_2/depth/color/points  
 * FROM PCD FILE:\
   <cam_1_pointcloud_file.pcd> <cam_2_pointcloud_file.pcd>  
   e.g: perception/pointcloud_samples/cam_1_optimal.pcd perception/pointcloud_samples/cam_2_optimal.pcd
 

 
 ---------------------------------------------------------
 The following arguments activate the single preprocessing / filtering steps:
   * allstepsfpfh=true OR allstepsmanual=true    - activates all steps with manual or fpfh feature pre alignment
   * passthrough=true                            - activates passthrough filter
   * downsampling=true                           - activates downsampling
   * outlier=true                                - activates outlier filter
   * mls=true                                    - activates mls_smoothing  - CURRENTLY only usable with downsampling and passthrough
   * manualalignment=true OR fpfhalignment=true  - activates manual or fpfh feature pre alignment
   * publishtoros=true                           - activates quaternion transformation publishing to ros /tf topic

 ---------------------------------------------------------
 This program only reads the pointclouds and applies an ICP if you give no arguments but the topics or files
