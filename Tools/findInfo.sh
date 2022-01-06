find /home/ganahe/catkin_ws/src/ -name '*' |xargs grep -r "\/depth\/points" >> ~/findInfoResult.txt

#find /home/ganahe/uavAutoNavigation/modules/ -name '*' |xargs grep -r "\/prometheus" >> fuc.txt

path=/home/ganahe/XTDrone/sensing/slam/vio

#sed -i "s/\/prometheus/\/uavAutoNavigation_GanAHE/g" ${path}*.rviz

#echo -e "--Successful!-- \n Path is:${path}*.cpp"
