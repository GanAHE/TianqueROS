#
# Copyright (C) 2016 Ramakrishna Kintada. All rights reserved.
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

# this one is important
set(CMAKE_SYSTEM_NAME "Linux")

#this one not so much
set(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler

#temporary variables to set up the cross compilation environment
set(ARM_CROSS_COMPILIER_PREFIX "arm-oemllib32-linux-gnueabi" )
set(ARM_COMPILER_PATH "${HEXAGON_ARM_SYSROOT}/x86_64-linux/usr/bin/${ARM_CROSS_COMPILIER_PREFIX}" )
set(ARM_C_COMPILER "${ARM_CROSS_COMPILIER_PREFIX}-gcc" )
set(ARM_CPP_COMPILER "${ARM_CROSS_COMPILIER_PREFIX}-g++" )

find_program(C_COMPILER ${ARM_C_COMPILER}
        PATHS ${ARM_COMPILER_PATH}
	NO_DEFAULT_PATH
	)

if(NOT C_COMPILER)
	message(FATAL_ERROR "could not find ${ARM_C_COMPILER} compiler")
endif()
cmake_force_c_compiler(${C_COMPILER} GNU)

find_program(CXX_COMPILER ${ARM_CPP_COMPILER} 
        PATHS ${ARM_COMPILER_PATH}
	NO_DEFAULT_PATH
	)

if(NOT CXX_COMPILER)
	message(FATAL_ERROR "could not find ${ARM_CPP_COMPILER} compiler")
endif()
cmake_force_cxx_compiler(${CXX_COMPILER} GNU)

# compiler tools
foreach(tool objcopy nm ld)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} ${ARM_CROSS_COMPILIER_PREFIX}-${tool}
                PATHS ${ARM_COMPILER_PATH}
		NO_DEFAULT_PATH
		)
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find ${ARM_CROSS_COMPILIER_PREFIX}-${tool}")
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

#override the ARM sysroot.
set(CMAKE_SYSROOT "${HEXAGON_ARM_SYSROOT}/lib32-apq8096" )
set(CMAKE_EXE_LINKER_FLAGS "-Wl,-gc-sections -Wl,-rpath-link,${CMAKE_SYSROOT}/usr/lib/${ARM_CROSS_COMPILIER_PREFIX} -Wl,-rpath-link,${CMAKE_SYSROOT}/lib/${ARM_CROSS_COMPILIER_PREFIX}" )

# where is the target environment
set(CMAKE_FIND_ROOT_PATH  get_file_component(${C_COMPILER} PATH))

# search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
