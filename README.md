# Point Cloud Registration of two Intel D435i 3D-Cameras with the Iterative Closest Point Algorithm 
Point Cloud Library, Robot Operating System, Intel D435

![alt text](https://repository-images.githubusercontent.com/215542871/3e9e6c00-24e2-11ea-9a2c-60b583b701e3)

![alt text](https://i.ibb.co/W3w2vqp/4000-3000-max.jpg)


## launch/start_3d_cams.launch
Starts both intel D435i ROS Nodes. Change serial numbers to your models.

## launch/icp_align_rviz.launch
Start your two ROS PointCloud2 topics before this program is started. See launch/start_3d_cams.launch

This launch file starts preprocess_align_publish, which consists of the following steps:

STEP 1 - READ TWO POINTCLOUDS FROM TWO ROS TOPICS  
STEP 2 - PASSTHROUGH_FILTER  
STEP 3 - REMOVE OUTLIERS  
STEP 4 - DOWNSAMPLING  
STEP 5 - SMOOTH SURFACES  
STEP 6 - COARSE MANUAL ALIGNMENT          <-- Applies a rotation in z and x. See preprocess_align_publish.cpp for angles.  
STEP 7 - ITERATIVE CLOSEST POINT ALGORITHM  
STEP 8 - PUBLISH TRANSFORMATION MATRIX TO TF TOPICS  
  
After that, rviz is loaded with a predefined config. Both cameras should be seen fully aligned inside rviz.
