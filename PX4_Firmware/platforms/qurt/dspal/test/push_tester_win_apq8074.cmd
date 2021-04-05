@echo on
adb wait-for-device

echo start QDSPs


set adsp_lib_dir=/usr/share/data/adsp

adb push build\dspal_tester\libdspal_tester.so   %adsp_lib_dir%/
adb push build\dspal_tester\libdspal_tester_skel.so  %adsp_lib_dir%/
adb push build\dspal_tester\dspal_tester /home/linaro/.
adb push build\dspal_tester\libdspal_tester.so   /home/linaro/.
adb shell chmod 755 /home/linaro/dspal_tester
