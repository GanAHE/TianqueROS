#!/bin/bash
echo "republish Topic ..."

rosrun topic_tools throttle messages /camera/color/image_raw 20.0 /cam0/image_raw
rosrun topic_tools throttle messages /camera/imu 200.0 /imu &
record=0
read -p "record now? >> " record

if [ ${record} -eq 1];then

    rosbag record /cam0/image_raw /imu
fi