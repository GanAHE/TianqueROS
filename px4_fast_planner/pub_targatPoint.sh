#!/bin/bash
x=$1
y=$2
z=$3
ox=$4
oy=$5
oz=$6
w=$7

echo "--[INFO] GanAHE: Start pub target points."

if [ ! -n "$1" ];then x=0.0
fi

if [ ! -n "$2" ];then y=0.0
fi

if [ ! -n "$3" ];then z=1.0
fi

if [ ! -n "$4" ];then ox=0.0
fi

if [ ! -n "$5" ];then oy=0.0
fi

if [ ! -n "$6" ];then oz=0.0
fi

if [ ! -n "$7" ];then w=0.0
fi


echo "--[INFO] pose:position:[x: ${x},y: ${y},z: ${z}], orientation:[x: ${ox},y: ${oy},z: ${oz},w: ${w}]"

rostopic pub --once /move_base_simple/goal geometry_msgs/PoseStamped "{header:{seq: 0,stamp:{secs: 0,nsecs: 0},frame_id: ''},pose:{position:{x: ${x},y: ${y},z: ${z}},orientation:{x: ${ox},y: ${oy},z: ${oz},w: ${w} }}}"

echo "--[INFO] GanAHE: End pub target points."