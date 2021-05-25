#!/bin/bash

echo "-- Tip: input E to exit."
x="L"
#read -p "--[INFO] choose mode: L:loop; E:not loop;" mode


    while [ "$x" != "E" ]
    do
        if [ ! -n "$1" ];then 
            read -p " Please input target coor [x y z ox oy oz w] >> " x y z ox oy oz w
            echo "--[INFO] GanAHE: Start pub target points."

            if [ "${x}" == "E" ];then break
            fi

            if [ ! -n "${y}" ];then y=0.0
            fi

            if [ ! -n "${z}" ];then z=1.0
            fi

            if [ ! -n "${ox}" ];then ox=0.0
            fi

            if [ ! -n "${oy}" ];then oy=0.0
            fi

            if [ ! -n "${oz}" ];then oz=0.0
            fi

            if [ ! -n "${w}" ];then w=0.0
            fi


            echo "--[INFO] pose:position:[x: ${x},y: ${y},z: ${z}], orientation:[x: ${ox},y: ${oy},z: ${oz},w: ${w}]"

            rostopic pub --once /move_base_simple/goal geometry_msgs/PoseStamped "{header:{seq: 0,stamp:{secs: 0,nsecs: 0},frame_id: ''},pose:{position:{x: ${x},y: ${y},z: ${z}},orientation:{x: ${ox},y: ${oy},z: ${oz},w: ${w} }}}"

            echo "--"
        else echo "Input none"
        fi
        
    done
    echo  " Done "
