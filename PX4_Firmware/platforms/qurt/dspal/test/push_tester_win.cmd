@echo on
adb wait-for-device

echo start QDSPs


set adsp_lib_dir=/usr/lib/rfsa/adsp

adb push build\dspal_tester\libdspal_tester.so   %adsp_lib_dir%/
adb push build\dspal_tester\libdspal_tester_skel.so  %adsp_lib_dir%/
adb push build\dspal_tester\dspal_tester /home/root/.
adb push build\dspal_tester\libdspal_tester.so   /home/root/.
adb shell chmod 755 /home/root/dspal_tester
