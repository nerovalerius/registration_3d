#!/bin/bash
# Armin Niedermueller 


echo "Starting Both 3D Cams, Alignment and RVIZ"
sleep 1

echo "Starting 3D Cams"
gnome-terminal -e "roslaunch perception start_3d_cams.launch"
sleep 3

#echo "Aligning first 3D Cam to Desk"
#gnome-terminal -e "rosrun tf static_transform_publisher 0.0 0.0 -2.5 -1.2 0.2 -0.3 4.0 world_frame cam_1_depth_optical_frame 3"

echo "Starting ICP Alignment"
gnome-terminal -e "bash -c 'rosrun perception preprocess_align_publish allsteps=true algorithm=nlicp /cam_1/depth/color/points /cam_2/depth/color/points'"
sleep 1

#echo "Aligning second 3D Cam to Desk"
#gnome-terminal -e "rosrun tf static_transform_publisher 0.0 0.0 -2.5 -1.2 0.2 -0.3 4.0 world_frame cam_2_depth_optical_frame3"

echo "Starting RVIZ"
gnome-terminal -e "bash -c 'roslaunch perception rviz_with_both_cams.launch'"
sleep 1

echo "Finished"
