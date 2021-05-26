#!/bin/bash
type=1
echo "------ GanAHE calibration -------"
echo "1.Monocular; 2.Stereo; 3.more_camera. 4.imu"
read -p "--[INFO] Please input camera type by code: >> " type
if [ ${type} -eq 1 ];then 
    echo "--[INFO] Monocular calibrating.."
    kalibr_calibrate_cameras --target checkboard.yaml --bag rgb_stereo_640_480.bag --bag-from-to 5 80 --models pinhole-radtan --topics /rgb_camera

elif [ ${type} -eq 2 ];then
    echo "--[INFO] Stereo calibrating.."
    kalibr_calibrate_cameras --target checkboard.yaml --bag  rgb_stereo_640_480.bag --models pinhole-equi pinhole-equi --topics /infra_left /infra_right --bag-from-to 5 60 --show-extraction

elif [ ${type} -eq 3 ];then
    echo "--[INFO] more camera calibrating.."
    kalibr_calibrate_cameras --target ./checkboard.yaml --bag  multicameras_calibration.bag --models pinhole-equi pinhole-equi pinhole-equi --topics /infra_left /infra_right /color --bag-from-to 10 100 --show-extraction

else echo "--[Error] Please input currect code!"

fi