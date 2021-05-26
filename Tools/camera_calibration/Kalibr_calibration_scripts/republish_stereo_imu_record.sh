#!/bin/bash
echo "republish Topic ..."

rosrun topic_tools throttle messages /camera/infra1/image_rect_raw 20.0 /infra_left &
rosrun topic_tools throttle messages /camera/infra2/image_rect_raw 20.0 /infra_right &
rosrun topic_tools throttle messages /camera/imu 200.0 /imu &

sleep 1s
record=0
read -p "record now? >> " record

if [ ${record} -eq 1 ];then
    rosbag record -O imu_stereo.bag /infra_left /infra_right /imu
fi