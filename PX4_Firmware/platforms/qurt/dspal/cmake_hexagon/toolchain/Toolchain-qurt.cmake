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
# 3. Neither the name ATLFlight nor the names of its contributors may be
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

include(CMakeForceCompiler)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

set(TOOLS_ERROR_MSG
	"The HexagonTools version 7.2.12 must be installed and the environment variable HEXAGON_TOOLS_ROOT must be set"
	"(e.g. export HEXAGON_TOOLS_ROOT=${HOME}/Qualcomm/HEXAGON_Tools/7.2.12/Tools)")

if ("$ENV{HEXAGON_TOOLS_ROOT}" STREQUAL "")
	message(FATAL_ERROR ${TOOLS_ERROR_MSG})
else()
	set(HEXAGON_TOOLS_ROOT $ENV{HEXAGON_TOOLS_ROOT})
endif()

macro (list2string out in)
	set(list ${ARGV})
	list(REMOVE_ITEM list ${out})
	foreach(item ${list})
		set(${out} "${${out}} ${item}")
	endforeach()
endmacro(list2string)

set(V_ARCH "v5")
set(CROSSDEV "hexagon-")

# Detect compiler version
if(${HEXAGON_TOOLS_ROOT} MATCHES "HEXAGON_Tools/6.4.")

	# Use the HexagonTools compiler (6.4.06) - This is deprecated
	set(HEXAGON_BIN	${HEXAGON_TOOLS_ROOT}/qc/bin)
	set(HEXAGON_GNU_BIN ${HEXAGON_TOOLS_ROOT}/gnu/bin)
	set(HEXAGON_ISS_DIR ${HEXAGON_TOOLS_ROOT}/lib/iss)
set(TOOLSLIB ${HEXAGON_TOOLS_ROOT}/dinkumware/lib/${V_ARCH}/G0/pic)

	set(CMAKE_C_COMPILER	${HEXAGON_BIN}/${CROSSDEV}clang)
	set(CMAKE_CXX_COMPILER  ${HEXAGON_BIN}/${CROSSDEV}clang++)

	set(CMAKE_AR	  ${HEXAGON_GNU_BIN}/${CROSSDEV}ar CACHE FILEPATH "Archiver")
	set(CMAKE_RANLIB  ${HEXAGON_GNU_BIN}/${CROSSDEV}ranlib)
	set(CMAKE_NM	  ${HEXAGON_GNU_BIN}/${CROSSDEV}nm)
	set(CMAKE_OBJDUMP ${HEXAGON_GNU_BIN}/${CROSSDEV}objdump)
	set(CMAKE_OBJCOPY ${HEXAGON_GNU_BIN}/${CROSSDEV}objcopy)
	set(HEXAGON_LINK  ${HEXAGON_GNU_BIN}/${CROSSDEV}ld)
set(HEXAGON_ARCH_FLAGS  -march=hexagonv5)

elseif(${HEXAGON_TOOLS_ROOT} MATCHES "HEXAGON_Tools/7.2.")

	# Use the HexagonTools compiler (7.2.12) from Hexagon 3.0 SDK
	set(HEXAGON_BIN	${HEXAGON_TOOLS_ROOT}/bin)
	set(HEXAGON_ISS_DIR ${HEXAGON_TOOLS_ROOT}/lib/iss)
set(TOOLSLIB ${HEXAGON_TOOLS_ROOT}/target/hexagon/lib/${V_ARCH}/G0/pic)

	set(CMAKE_C_COMPILER	${HEXAGON_BIN}/${CROSSDEV}clang)
	set(CMAKE_CXX_COMPILER  ${HEXAGON_BIN}/${CROSSDEV}clang++)

	set(CMAKE_AR	  ${HEXAGON_BIN}/${CROSSDEV}ar CACHE FILEPATH "Archiver")
	set(CMAKE_RANLIB  ${HEXAGON_BIN}/${CROSSDEV}ranlib)
	set(CMAKE_NM	  ${HEXAGON_BIN}/${CROSSDEV}nm)
	set(CMAKE_OBJDUMP ${HEXAGON_BIN}/${CROSSDEV}objdump)
	set(CMAKE_OBJCOPY ${HEXAGON_BIN}/${CROSSDEV}objcopy)
	set(HEXAGON_LINK  ${HEXAGON_BIN}/${CROSSDEV}link)
set(HEXAGON_ARCH_FLAGS
	-march=hexagon
	-mcpu=hexagonv5
	)

else()
	message(FATAL_ERROR ${TOOLS_ERROR_MSG})
endif()

set(CMAKE_SKIP_RPATH TRUE CACHE BOOL SKIP_RPATH FORCE)

# where is the target environment
set(CMAKE_FIND_ROOT_PATH  get_file_component(${C_COMPILER} PATH))

set(CMAKE_C_COMPILER_ID, "Clang")
set(CMAKE_CXX_COMPILER_ID, "Clang")

# search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# The Hexagon compiler doesn't support the -rdynamic flag and this is set
# in the base cmake scripts. We have to redefine the __linux_compiler_gnu
# macro for cmake 2.8 to work
set(__LINUX_COMPILER_GNU 1)

macro(__linux_compiler_gnu lang)
	set(CMAKE_SHARED_LIBRARY_LINK_${lang}_FLAGS "")
endmacro()

set(SDK_ERROR_MSG
	"The Hexagon_SDK version 3 must be installed and the environment variable HEXAGON_SDK_ROOT must be set"
	"(e.g. export HEXAGON_SDK_ROOT=${HOME}/Qualcomm/Hexagon_SDK/3.0)")

if ("$ENV{HEXAGON_SDK_ROOT}" STREQUAL "")
	message(FATAL_ERROR ${SDK_ERROR_MSG})
else()
	set(HEXAGON_SDK_ROOT $ENV{HEXAGON_SDK_ROOT})
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

# Find the ARM cross compiler for making a bundle
foreach(tool arm-linux-gnueabihf-gcc arm-linux-gnueabihf-g++)
        string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} ${tool}
		PATHS
			${ARM_CROSS_GCC_ROOT}/bin
		NO_DEFAULT_PATH
		)
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find ${TOOL}")
	endif()
endforeach()
