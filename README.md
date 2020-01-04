# Point Cloud Registration of two Intel D435i 3D-Cameras with the Iterative Closest Point Algorithm 
Point Cloud Library, Robot Operating System, Intel D435

![alt text](https://repository-images.githubusercontent.com/215542871/3e9e6c00-24e2-11ea-9a2c-60b583b701e3)

![alt text](https://i.ibb.co/W3w2vqp/4000-3000-max.jpg)


## launch/start_3d_cams.launch
Starts both intel D435i ROS Nodes. Change serial numbers to your models.

## launch/icp_align_rviz.launch
Start your two ROS PointCloud2 topics before this program is started. See launch/start_3d_cams.launch

This launch file starts preprocess_align_publish, which consists of the following steps:

### PREPROCESS_ALIGN_PUBLISH 
test


This program is designed to:
 1a. Read two pointclouds from ros PointCloud2 streams
 1b. OR read two pointclouds from .pcd files
 2. Downsample and filter both pointclouds
 3. Smooth Surfaces and make a coarse alignment
 4. Apply an iterative closest point algorithm
 5. Publish the Transformation Matrix to the ros /tf topic
 ---------------------------------------------------------
 Arguments: <cam_1_pointcloud2_topic> <cam_2_pointcloud2_topic>
 OR: <cam_1_pointcloud_file.pcd> <cam_2_pointcloud_file.pcd>
 Usually: /cam_1/depth/color/points and /cam_2/depth/color/points
 This program only reads the pointclouds and applies an ICP if you give no arguments but the topics or files
 ---------------------------------------------------------
 The following arguments activate the single steps:
   * allstepsfpfh=true OR allstepsmanual=true    - activates all steps with manual or fpfh feature pre alignment
   * passthrough=true                            - activates passthrough filter
   * downsampling=true                           - activates downsampling
   * outlier=true                                - activates outlier filter
   * mls=true                                    - activates mls_smoothing  - CURRENTLY only usable with downsampling and passthrough
   * manualalignment=true OR fpfhalignment=true  - activates manual or fpfh feature pre alignment
   * publishtoros=true                           - activates quaternion transformation publishing to ros /tf topic
