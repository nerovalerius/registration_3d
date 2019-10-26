## PCL-Cropbox Filter
Usage: pcl_cropbox_filter <input_file.pcd> <minX> <minY> <minZ> <maxX> <maxY> <maxZ>
rosrun perception pcl_cropbox_filter cam_1.pcd -0.5 -0.135 -0.5 2 6 2.3
rosrun perception pcl_cropbox_filter cam_2.pcd -0.8 -0.2 0.5 0.5 0.5 2.5

## PCL-Outlier Filter
Usage pcl_outlier_filter <cloud.pcd>
rosrun perception pcl_outlier_filter cam_1_cropbox_filtered.pcd
rosrun perception pcl_outlier_filter cam_2_cropbox_filtered.pcd 

## PCL-Transform
Usage pcl_initial_transform <input_file.pcd>
rosrun perception pcl_initial_transformation cam_1_cropbox_filtered_outlier_filtered.pcd

## Iterative Closest Point
Usage pcl_ICP_nonlinear <cloud_1.pcd> <cloud_2.pcd>
rosrun perception pcl_ICP_nonlinear cam_1_cropbox_filtered_outlier_filtered_transformed.pcd cam_2_cropbox_filtered_outlier_filtered.pcd



# Working Procedure

## Apply Passthrough filter with Z Distance of 2.2 on cam_1
rosrun perception pcl_passthrough_filter cam_2.pcd 2.2

## Apply Passthrough filter with Z Distance of 2 on cam_2
rosrun perception pcl_passthrough_filter cam_1.pcd 2

## Apply Outlier filter on both camera images
rosrun perception pcl_outlier_filter cam_1_passthrough_filtered_z2.pcd 
rosrun perception pcl_outlier_filter cam_2_passthrough_filtered_z2.2.pcd

## Perform initial transformation X-rotation -45° - Z-rotation 180° on cam_1
rosrun perception pcl_initial_transformation cam_1_passthrough_filtered_z2_outlier_filtered.pcd 

## Perform nonlinear ICP
rosrun perception pcl_ICP_nonlinear cam_1_passthrough_filtered_z2_outlier_filtered_transformed.pcd cam_2_passthrough_filtered_z2.2_outlier_filtered.pcd
