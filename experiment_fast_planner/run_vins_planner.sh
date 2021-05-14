#!/bin/bash

gnome-terminal -x bash -c "cd ~/catkin_make;roslaunch experiment_fast_planner experiment_fast_planner.launch;exec bash"

sleep 2s

rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml

