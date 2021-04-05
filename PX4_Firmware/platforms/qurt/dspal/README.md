# DSPAL

The DSP Abstraction Layer (DSPAL) provides a standard interface for porting
code to the Hexagon processor.

## Change Log

The DSPAL change log is located [here](CHANGE_LOG.md).

## Setup Development Environment

See [GettingStarted](https://github.com/ATLFlight/ATLFlightDocs/blob/master/GettingStarted.md)

After installing the development tools, make sure you have your [environment variables set up] https://github.com/ATLFlight/ATLFlightDocs/blob/master/GettingStarted.md#setup-environment-variables).

## Testing DSPAL

```
git clone https://github.com/ATLFlight/dspal
cd dspal
git submodule update --init --recursive
cd test
make
```

Connect the device via ADB and make sure it can be found.
```
adb devices
```
You should see something like:
```
$ adb devices
List of devices attached 
997e5d3a	device
```
Now load the dspal_tester and version_test apps on the device.
```
make load
```

This will push dspal_tester and version_test to /home/linaro/ on the device, and it will push
libdspal_tester.so, libdspal_tester_skel.so, libversion_test_skel.so and libversion_test.so to
/usr/share/data/adsp/ on the device.

To see the program output from the code running on the DSP, you will need to run mini-dm in another terminal.
```
${HEXAGON_SDK_ROOT}/tools/mini-dm/Linux_Debug/mini-dm
```
If not found in the above location try this:

```
${HEXAGON_SDK_ROOT}/tools/debug/mini-dm/Linux_Debug/mini-dm
```

### Running dspal_tester
To run the program:
```
$ adb shell
# cd /home/linaro
# ./dspal_tester
```

You should see output on the ADB terminal similar to (if you don't, see the "Troubleshooting" section below):
```
Starting DspAL tests
testing time.h
/local/mnt/workspace/lnxbuild/project/trees_in_use/free_tree_le_manifest_LNX.LER.1.0_eagle8074_commander_17199839/checkout/oe-core/build/tmp-eglibc/work/cortexa8hf-vfp-neon-linux-gnueabi/adsprpc/1.0-r0/adsprpc-1.0/src/fastrpc_apps_user.c:136:failed to create tls key/local/mnt/workspace/lnxbuild/project/trees_in_use/free_tree_le_manifest_LNX.LER.1.0_eagle8074_commander_17199839/checkout/oe-core/build/tmp-eglibc/work/cortexa8hf-vfp-neon-linux-gnueabi/adsprpc/1.0-r0/adsprpc-1.0/src/listener_android.c:112:listener using ion heap: -1
[  PASS] clockid values exist
[  PASS] sigevent values exist
[  PASS] time returns good value
[  PASS] timer realtime
[  PASS] timer monotonic
[- SKIP] timer process cputime
[- SKIP] timer thread cputime
[  PASS] time return value
[  PASS] time parameter
[  PASS] usleep for two seconds
[  PASS] clock_getres
[  PASS] clock_gettime
[  PASS] clock_settime
[  PASS] one shot timer cb
[  PASS] periodic timer cb
[  PASS] periodic timer signal cb
[  PASS] periodic timer sigwait
testing pthread.h
[  PASS] pthread attr init
[  PASS] pthread create
[- SKIP] pthread cancel
[  PASS] pthread self
[  PASS] pthread exit
[- SKIP] pthread kill
[  PASS] pthread condition timed wait
[  PASS] pthread mutex lock
[  PASS] thread mutex lock thread
[  PASS] thread large allocation on stack
[  PASS] thread large allocation on heap
[  PASS] usleep for two seconds
testing semaphore.h
[  PASS] semaphore wait
testing C++
[  PASS] test C++ heap
[  PASS] test C++ static initialization
tests complete
testing device path access
[  PASS] spi loopback test
...
DspAL some tests skipped.
DspAL tests succeeded.
```

### Running version_test

There is a utility to check the version of the DSP SW.

Connect the Snapdragon Flight board via micro USB connector for ADB.

Then get a shell on the device:

```
adb shell
# cd /home/linaro
# ./version_test
# exit
```

Amongst the debug output you should see something like:

```
version: DSPAL_VERSION_STRING=DSPAL-1.0.1.0001
build date: BUILD_DATE_STRING=Feb  2 2016
build time: BUILD_TIME_STRING=19:10:21
```

### Troubleshooting

1. If you see output like this when trying to run mini-dm, you need to update your aDSP image to one that supports pthread_cond_timedwait. To get an updated aDSP image, please contact the vendor who sold you the board.
```
host>$ ${HEXAGON_SDK_ROOT}/tools/mini-dm/Linux_Debug/mini-dm
Device found with Product ID 0x9025. Continuing...
mini-dm is waiting for a DMSS connection...
DMSS is connected. Running mini-dm...
[08500/02]  17:13.750  HAP:12332:Verification skipped, no function specified!!  0256  map_object.c
[08500/03]  17:13.755  HAP:12332:Found text relocation in /libdspal_tester_skel.so. Support for text relocations in shared objects i  0094  reloc.c
[08500/03]  17:13.756  HAP:12332:Found text relocation in /libdspal_tester.so. Support for text relocations in shared objects is dep  0094  reloc.c
[08500/03]  17:13.756  HAP:12332:HAP_debug_v2 weak ref not found, return _rtld_sym_zero@_rtld_objmain  0294  symbol.c
[08500/03]  17:13.756  HAP:12332:HAP_debug_v2 weak ref not found, return _rtld_sym_zero@_rtld_objmain  0294  symbol.c
[08500/03]  17:13.756  HAP:12332:undefined PLT symbol pthread_cond_timedwait (452) /libdspal_tester.so  0303  symbol.c
[08500/02]  17:13.760  HAP:12332:Verification skipped, no function specified!!  0256  map_object.c
[08500/02]  17:13.765  HAP:12332:Verification skipped, no function specified!!  0256  map_object.c
[08500/03]  17:13.770  HAP:12332:Found text relocation in /libdspal_tester_skel.so. Support for text relocations in shared objects i  0094  reloc.c
[08500/03]  17:13.770  HAP:12332:Found text relocation in /libdspal_tester.so. Support for text relocations in shared objects is dep  0094  reloc.c
[08500/03]  17:13.770  HAP:12332:HAP_debug_v2 weak ref not found, return _rtld_sym_zero@_rtld_objmain  0294  symbol.c
[08500/03]  17:13.771  HAP:12332:HAP_debug_v2 weak ref not found, return _rtld_sym_zero@_rtld_objmain  0294  symbol.c
[08500/03]  17:13.771  HAP:12332:undefined PLT symbol pthread_cond_timedwait (452) /libdspal_tester.so  0303  symbol.c
[08500/02]  17:13.780  HAP:12332:Verification skipped, no function specified!!  0256  map_object.c
[08500/02]  17:13.786  HAP:12332:Verification skipped, no function specified!!  0256  map_object.c
[08500/03]  17:13.790  HAP:12332:Found text relocation in /libdspal_tester_skel.so. Support for text relocations in shared objects i  0094  reloc.c
```

