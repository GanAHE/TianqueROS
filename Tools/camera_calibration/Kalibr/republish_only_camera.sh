#!/bin/bash

echo "------ GanAHE Republish Topic -------"

rosrun topic_tools throttle messages /camera/color/image_raw 4.0 /rgb_camera &

rosrun topic_tools throttle messages /camera/infra1/image_rect_raw 4.0 /infra_left &

rosrun topic_tools throttle messages /camera/infra2/image_rect_raw 4.0 /infra_right &

sleep 1s
red="n"
read -t 5 -p "record now? y/n>> " red
echo "--- $red"
if [ "$red" == "y" ];then
    rosbag record -O rgb_stereo_640_480.bag /infra_left /infra_right /rgb_camera
fi