#!/bin/bash

gnome-terminal -x bash -c "cd ~/catkin_make;roslaunch experiment_fast_planner experiment_fast_planner.launch;exec bash"



sleep 1s

#gnome-terminal -x bash -c "cd ~/droneMs;rosbag record /vins_estimator/image_track /camera/color/image_raw;exec bash"
gnome-terminal -x bash -c "cd ~/droneMs;rosbag record /camera/imu /waypoint_generator/waypoints /camera/depth/image_rect_raw /camera/color/image_raw /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw /vins_estimator/camera_pose /vins_estimator/camera_pose_visual /vins_estimator/image_track /vins_estimator/extrinsic /vins_estimator/margin_cloud /vins_estimator/point_cloud /vins_estimator/odometry /vins_estimator/path /vins_estimator/key_poses /vins_restart /vins_imu_switch /vins_estimator/keyframe_pose /mavros/vision_pose/pose /mavros/local_position/odom;exec bash"

sleep 2s

rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml