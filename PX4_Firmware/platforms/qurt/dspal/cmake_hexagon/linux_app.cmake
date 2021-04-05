############################################################################
#
# Copyright (c) 2016 Mark Charlebois. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
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
############################################################################

# Overview:
# Hexagon/QuRT apps are built in 2 parts, the part that runs on the
# application (apps) processor, and the library that is invoked on the DSP.
#
# PREREQUISITES:
#
# Environment variables:
#	HEXAGON_TOOLS_ROOT
#	HEXAGON_SDK_ROOT
#
# USAGE:
#
# For simple DSP apps that use a simple apps processor app to invoke the
# DSP lib, the QURT_BUNDLE function can be used.
#
# When the apps proc app requires its own cmake build, the RPC stub functions
# can be generated with FASTRPC_STUB_GEN. A Linux lib can be built with the
# LINUX_LIB function and the apps proc app can be built with the LINUX_APP
# function.
#
# Build targets to load the apps proc app and libs are created from the
# rules below. Look for resulting make targets ending in -load.

include(fastrpc)

include(CMakeParseArguments)

function(FASTRPC_ARM_LINUX_LOAD)
	set(oneValueArgs LOADNAME TARGET DEPNAME DEST)
	cmake_parse_arguments(FASTRPC_ARM_LINUX_LOAD "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

   if ("${QC_SOC_TARGET}" STREQUAL "APQ8096")
      # Set the location for 8x96 target
      set(APPS_TARGET_PATH "/home/root")
   else()
      # 8x74 target is assumed by default.
      set(APPS_TARGET_PATH "/home/linaro")
   endif()

	# Build lib that is run on the DSP invoked by RPC framework
	# Set default install path of apps processor executable
	if ("${FASTRPC_ARM_LINUX_LOAD_DEST}" STREQUAL "")
		set(FASTRPC_ARM_LINUX_LOAD_DEST "${APPS_TARGET_PATH}")
	endif()

	# Add a rule to load the file onto the target
	add_custom_target(${FASTRPC_ARM_LINUX_LOAD_LOADNAME}-load
		DEPENDS ${FASTRPC_ARM_LINUX_LOAD_DEPNAME}
		COMMAND adb wait-for-device
		COMMAND adb push ${FASTRPC_ARM_LINUX_LOAD_TARGET} ${FASTRPC_ARM_LINUX_LOAD_DEST}
		COMMAND echo "Pushed ${FASTRPC_ARM_LINUX_LOAD_TARGET} to ${FASTRPC_ARM_LINUX_LOAD_DEST}"
		)
endfunction()

# Process DSP files
function (LINUX_LIB)
	set(options)
	set(oneValueArgs LIB_NAME IDL_NAME LIB_DEST)
	set(multiValueArgs SOURCES LINK_LIBS INCS FLAGS)
	cmake_parse_arguments(LINUX_LIB "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

	if ("${LINUX_LIB_IDL_NAME}" STREQUAL "")
		message(FATAL_ERROR "LINUX_LIB called without IDL_NAME")
	endif()

	include_directories(
		${CMAKE_CURRENT_BINARY_DIR}
		${FASTRPC_ARM_LINUX_INCLUDES}
		)

	add_definitions( "-DDSP_TYPE_${DSP_TYPE}" )

	#message("LINUX_LIB_INCS = ${LINUX_LIB_INCS}")

	if (NOT "${LINUX_LIB_SOURCES}" STREQUAL "")

		# Build lib that is run on the DSP
		add_library(${LINUX_LIB_LIB_NAME} SHARED
			${LINUX_LIB_SOURCES}
			${LINUX_LIB_IDL_NAME}_stub.c
			${HEXAGON_SDK_ROOT}/${SDKLIB}/common/rpcmem/src/rpcmem.c
			)

		if (NOT "${LINUX_LIB_FLAGS}" STREQUAL "")
			set_target_properties(${LINUX_LIB_LIB_NAME} PROPERTIES COMPILE_FLAGS "${LINUX_LIB_FLAGS}")
		endif()

		if (NOT "${LINUX_LIB_INCS}" STREQUAL "")
			target_include_directories(${LINUX_LIB_LIB_NAME} PUBLIC ${LINUX_LIB_INCS})
		endif()

		#message("LINUX_LIB_LINK_LIBS = ${LINUX_LIB_LINK_LIBS}")

		target_link_libraries(${LINUX_LIB_LIB_NAME}
			${LINUX_LIB_LINK_LIBS}
			${FASTRPC_ARM_LIBS}
			)

		add_dependencies(${LINUX_LIB_LIB_NAME} generate_${LINUX_LIB_IDL_NAME}_stubs)

	endif()

	# Add a rule to load the files onto the target that run on apps proc
	FASTRPC_ARM_LINUX_LOAD(
		LOADNAME lib${LINUX_LIB_LIB_NAME}
		TARGET lib${LINUX_LIB_LIB_NAME}.so
		DEPNAME ${LINUX_LIB_LIB_NAME}
		DEST ${LINUX_LIB_LIB_DEST}
		)
endfunction()

# Process Apps proc app source and libs
function (LINUX_APP)
	set(oneValueArgs APP_NAME IDL_NAME APP_DEST)
	set(multiValueArgs SOURCES LINK_LIBS INCS)
	cmake_parse_arguments(LINUX_APP "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

	if ("${LINUX_APP_SOURCES}" STREQUAL "")
		message(FATAL_ERROR "LINUX_APP called without SOURCES")
	endif()

	if ("${LINUX_APP_IDL_NAME}" STREQUAL "")
		message(FATAL_ERROR "LINUX_APP called without IDL_NAME")
	endif()

	include_directories(
		${CMAKE_CURRENT_BINARY_DIR}
		${FASTRPC_ARM_LINUX_INCLUDES}
		)

	add_definitions( "-DDSP_TYPE_${DSP_TYPE}" )

	#message("LINUX_APP_INCS = ${LINUX_APP_INCS}")

	# Build lib that is run on the DSP
	add_executable(${LINUX_APP_APP_NAME}
		${LINUX_APP_SOURCES}
		${LINUX_APP_IDL_NAME}_stub.c
		${HEXAGON_SDK_ROOT}/${SDKLIB}/common/rpcmem/src/rpcmem.c
		)

	if (NOT "${LINUX_APP_INCS}" STREQUAL "")
		target_include_directories(${LINUX_APP_APP_NAME} PUBLIC ${LINUX_APP_INCS})
	endif()

	#message("LINUX_APP_LINK_LIBS = ${LINUX_APP_LINK_LIBS}")

	target_link_libraries(${LINUX_APP_APP_NAME}
		${LINUX_APP_LINK_LIBS}
		${FASTRPC_ARM_LIBS}
		)

	add_dependencies(${LINUX_APP_APP_NAME} generate_${LINUX_APP_IDL_NAME}_stubs)

	FASTRPC_ARM_LINUX_LOAD(
		LOADNAME ${LINUX_APP_APP_NAME}
		TARGET ${LINUX_APP_APP_NAME}
		DEPNAME ${LINUX_APP_APP_NAME}
		DEST ${LINUX_APP_APP_DEST}
		)
endfunction()
