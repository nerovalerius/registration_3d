## PCL-Cropbox Filter
Usage: pcl_cropbox_filter <input_file.pcd> <minX> <minY> <minZ> <maxX> <maxY> <maxZ>
rosrun perception pcl_cropbox_filter cam_2.pcd -0.8 -6 0.5 0.5 6 2.5
rosrun perception pcl_cropbox_filter cam_1.pcd -0.5 -6 -0.5 2 6 2.3

## PCL-Outlier Filter
Usage pcl_outlier_filter <cloud.pcd>
rosrun perception pcl_outlier_filter cam_1_cropbox_filtered.pcd
rosrun perception pcl_outlier_filter cam_2_cropbox_filtered.pcd 
