#
# Copyright (C) 2015 Mark Charlebois. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#	notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#	notice, this list of conditions and the following disclaimer in
#	the documentation and/or other materials provided with the
#	distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#	used to endorse or promote products derived from this software
#	without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# defines:
#
# NM
# OBJCOPY
# LD
# CXX_COMPILER
# C_COMPILER
# CMAKE_SYSTEM_NAME
# CMAKE_SYSTEM_VERSION
# LINKER_FLAGS
# CMAKE_EXE_LINKER_FLAGS
# CMAKE_FIND_ROOT_PATH
# CMAKE_FIND_ROOT_PATH_MODE_PROGRAM
# CMAKE_FIND_ROOT_PATH_MODE_LIBRARY
# CMAKE_FIND_ROOT_PATH_MODE_INCLUDE

include(CMakeForceCompiler)

if ("$ENV{HEXAGON_SDK_ROOT}" STREQUAL "")
        message(FATAL_ERROR "HEXAGON_SDK_ROOT not set")
else()
        set(HEXAGON_SDK_ROOT $ENV{HEXAGON_SDK_ROOT})
endif()

if ("$ENV{HEXAGON_ARM_SYSROOT}" STREQUAL "")
        message(FATAL_ERROR "HEXAGON_ARM_SYSROOT not set")
else()
        set(HEXAGON_ARM_SYSROOT $ENV{HEXAGON_ARM_SYSROOT})
endif()

# GCC version from latest installsdk.sh script
set(ARM_GCC_DEFAULT "gcc-4.9-2014.11")

if ("$ENV{ARM_CROSS_GCC_ROOT}" STREQUAL "")
	if (EXISTS "${HEXAGON_SDK_ROOT}/../../ARM_Tools/${ARM_GCC_DEFAULT}/bin/")
		set(ARM_CROSS_GCC_ROOT "${HEXAGON_SDK_ROOT}/../../ARM_Tools/${ARM_GCC_DEFAULT}")
	elseif (EXISTS "${HEXAGON_SDK_ROOT}/gcc-linaro-4.9-2014.11-x86_64_arm-linux-gnueabihf_linux/bin/arm-linux-gnueabihf-gcc")
		set(ARM_CROSS_GCC_ROOT "${HEXAGON_SDK_ROOT}/gcc-linaro-4.9-2014.11-x86_64_arm-linux-gnueabihf_linux")
	elseif (EXISTS "${HEXAGON_SDK_ROOT}/gcc-linaro-4.9-2016.02-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc")
		set(ARM_CROSS_GCC_ROOT "${HEXAGON_SDK_ROOT}/gcc-linaro-4.9-2016.02-x86_64_arm-linux-gnueabihf")
	else()
		message(FATAL_ERROR "No supported version of ARMv7hf GCC cross compiler found")
	endif()
else()
	if (EXISTS "$ENV{ARM_CROSS_GCC_ROOT}/bin/arm-linux-gnueabihf-gcc")
		set(ARM_CROSS_GCC_ROOT $ENV{ARM_CROSS_GCC_ROOT})
	else()
		message(FATAL_ERROR "No supported version of ARMv7hf GCC cross compiler found in ${ARM_CROSS_GCC_ROOT}/bin")
	endif()
endif()

# this one is important
set(CMAKE_SYSTEM_NAME Generic)

# this one not so much
set(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
find_program(C_COMPILER arm-linux-gnueabihf-gcc
	PATHS
		${ARM_CROSS_GCC_ROOT}/bin
	NO_DEFAULT_PATH
	)

if(NOT C_COMPILER)
	message(FATAL_ERROR "could not find arm-linux-gnueabihf-gcc compiler")
endif()
cmake_force_c_compiler(${C_COMPILER} GNU)

find_program(CXX_COMPILER arm-linux-gnueabihf-g++
	PATHS
		${ARM_CROSS_GCC_ROOT}/bin
	NO_DEFAULT_PATH
	)

if(NOT CXX_COMPILER)
	message(FATAL_ERROR "could not find arm-linux-gnueabihf-g++ compiler")
endif()
cmake_force_cxx_compiler(${CXX_COMPILER} GNU)

# compiler tools
foreach(tool objcopy nm ld)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} arm-linux-gnueabihf-${tool}
		PATHS
			${ARM_CROSS_GCC_ROOT}/bin
		NO_DEFAULT_PATH
		)
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find arm-linux-gnueabihf-${tool}")
	endif()
endforeach()

# os tools
foreach(tool echo grep rm mkdir nm cp touch make unzip)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} ${tool})
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find ${TOOL}")
	endif()
endforeach()

set(CMAKE_SYSROOT ${HEXAGON_ARM_SYSROOT})
set(CMAKE_EXE_LINKER_FLAGS "-Wl,-gc-sections -Wl,-rpath-link,${HEXAGON_ARM_SYSROOT}/usr/lib/arm-linux-gnueabihf -Wl,-rpath-link,${HEXAGON_ARM_SYSROOT}/lib/arm-linux-gnueabihf")

# where is the target environment
set(CMAKE_FIND_ROOT_PATH  get_file_component(${C_COMPILER} PATH))

# search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
