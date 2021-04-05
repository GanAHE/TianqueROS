# CMake Rules for QuRT Applications

Hexagon apps are started from an app running on the apps processor 
of the SoC. A RPC mechanism (called FastRPC) is used to load a shared library
on the DSP and the RPC stubs are generated from a IDL complier (qaic). The
RTOS on the DSP is QuRT but is often abstraced by the DSPAL APIs.

## Using cmake_hexagon

Add cmake_hexagon as a submodule to your project.

In the CMakeLists.txt file for your project, add:
```
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_hexagon")
```

Then add whichever modules you wish:
```
include(qurt_app)
```
or
```
include(linux_app)
```
or
```
include(bundle)
```

## Building simple applications

The QURT_BUNDLE function can be made to build DSP applications that have a
simple apps proc side launcher app.

QURT_BUNDLE is used to specify the files and libraries to build
into the DSP lib and into the apps application. The generated stubs are
automatically build into the appropriate target.

The CMakeLists.txt file calls QURT_BUNDLE and requires that the file <appname>.idl
exists for the IDL interface between the apps processor app and the DSP library.

QURT_BUNDLE(APP_NAME testapp
	DSP_SOURCES testapp_dsp.c
	APPS_SOURCES testapp.c
	APPS_INCS "-Iinclude"
	APPS_COMPILER arm-linux-gnueabihf-gcc
	)

For an app named testapp, the result will be:
- testapp_app        - Run on apps processor
- libtestapp.so      - copy to target at /usr/share/date/adsp/
- libtestapp_skel.so - copy to target at /usr/share/date/adsp/

The file testapp.idl is used in this example to create the stub functions that automatically
get linked in to the app and DSP lib.

QURT_BUNDLE adds a rule to load the app onto the DSP (<appname>-load).

To load the testapp application to the target you can run the 
following command from inside the build tree:

```
cd build
make testapp-load
```

## Building QuRT libs and apps proc apps and/or libs separately

### DSP Lib(s)

In the DSP lib CMakeLists.txt file:

```
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_hexagon")
include(qurt_lib)

FASTRPC_STUB_GEN(hello.idl)

QURT_LIB(
	LIB_NAME helloworld
	IDL_NAME hello
	SOURCES helloworld_dsp.c
	)
```

### Apps proc Lib(s) and app

An application that calls the IDL interface is required to load the library
that runs on the DSP.

In the apps proc app CMakeLists.txt file:

```
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_hexagon")
include(linux_app)

FASTRPC_STUB_GEN(hello.idl)

LINUX_APP(
	APP_NAME helloworld
	IDL_NAME hello
	SOURCES helloworld.c
	)
```
