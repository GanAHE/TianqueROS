############################################################################
#
# Copyright (c) 2015 Mark Charlebois. All rights reserved.
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
# can be generated with FASTRPC_STUB_GEN. The DSP lib can be built with the
# QURT_LIB function and the apps proc app can be built with the help of the
# the FASTRPC_ARM_APP_DEPS_GEN function.
#
# Build targets to load the apps proc app and DSP libs are created from the
# rules below. Look for resulting make targets ending in -load.

include(fastrpc)
include(qurt_lib)
include(linux_app)

#
# Hexagon apps are started from an app running on the apps processor
# of the SoC. An RPC mechanism is used to load the app on the DSP and
# the RPC stubs are generated from a IDL complier (qaic). The RTOS on
# the DSP is QuRT but is often abstraced by the DSPAL APIs.
#
# The default idl file is <APP_NAME>.idl
#
# QURT_BUNDLE is used to specify the files and libraries to build
# in the DSP lib and in the apps application. The generated stubs are
# automatically build into the appropriate target.
#
# For an app named testapp, the result will be:
#    testapp_app     - Run on apps processor
#    testapp.so      - copy to target at /usr/share/date/adsp/
#    testapp_skel.so - copy to target at /usr/share/date/adsp/
#

function(QURT_BUNDLE)
	set(options)
	set(oneValueArgs APP_NAME IDL_FILE APPS_COMPILER APP_DEST)
	set(multiValueArgs APPS_SOURCES APPS_LINK_LIBS APPS_INCS DSP_SOURCES DSP_LINK_LIBS DSP_INCS QAIC_INCS)
	cmake_parse_arguments(QURT_BUNDLE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

	if ("${QURT_BUNDLE_APP_NAME}" STREQUAL "")
		message(FATAL_ERROR "APP_NAME not specified in call to QURT_BUNDLE")
	endif()

	if ("${QURT_BUNDLE_IDL_FILE}" STREQUAL "")
		set(QURT_BUNDLE_IDL_FILE ${CMAKE_CURRENT_SOURCE_DIR}/${QURT_BUNDLE_APP_NAME}.idl)
	endif()
	FASTRPC_STUB_GEN(${QURT_BUNDLE_IDL_FILE} ${QURT_BUNDLE_QAIC_INCS})
	get_filename_component(QURT_BUNDLE_IDL_NAME ${QURT_BUNDLE_IDL_FILE} NAME_WE)

	#message("APP_NAME = ${QURT_BUNDLE_APP_NAME}")
	#message("IDL_FILE = ${QURT_BUNDLE_IDL_FILE}")

	# Process Apps processor files
	if (NOT "${QURT_BUNDLE_APPS_SOURCES}" STREQUAL "")

		# Make sure apps compiler is provided
		if ("${QURT_BUNDLE_APPS_COMPILER}" STREQUAL "")
			message(FATAL_ERROR "APPS_COMPILER not specified in call to QURT_BUNDLE")
		endif()

		set(${QURT_BUNDLE_APP_NAME}_INCLUDE_DIRS
			-I${CMAKE_CURRENT_BINARY_DIR}
			)

		# prepend -I in front of APPS include dirs
		foreach(inc ${QURT_BUNDLE_APPS_INCS})
			list(APPEND ${QURT_BUNDLE_APP_NAME}_INCLUDE_DIRS -I${inc})
		endforeach()

		# prepend -I in front of FastRPC include dirs
		foreach(inc ${FASTRPC_ARM_LINUX_INCLUDES})
			list(APPEND ${QURT_BUNDLE_APP_NAME}_INCLUDE_DIRS -I${inc})
		endforeach()

	# prepend -I in front of QAIC include dirs
	set(QAIC_INCLUDE_DIRS)
	foreach(inc ${QURT_BUNDLE_QAIC_INCS})
		string(SUBSTRING ${inc} 0 1 absolute_path_character)
		if (absolute_path_character STREQUAL "/")
			list(APPEND QAIC_INCLUDE_DIRS -I${inc})
		else()
			list(APPEND QAIC_INCLUDE_DIRS -I${CMAKE_CURRENT_SOURCE_DIR}/${inc})
		endif()
	endforeach()

		set(${QURT_BUNDLE_APP_NAME}_LINK_DIRS ${FASTRPC_ARM_LIBS})

		# Build the apps processor app and RPC stub using the provided ${QURT_BUNDLE_APPS_COMPILER}
		add_custom_target(${QURT_BUNDLE_APP_NAME}_app ALL
			COMMAND ${QURT_BUNDLE_APPS_COMPILER}  ${${QURT_BUNDLE_APP_NAME}_INCLUDE_DIRS} ${QAIC_INCLUDE_DIRS} -o ${CMAKE_CURRENT_BINARY_DIR}/${QURT_BUNDLE_APP_NAME} ${QURT_BUNDLE_APPS_SOURCES} ${HEXAGON_SDK_ROOT}/${SDKLIB}/common/rpcmem/src/rpcmem.c "${CMAKE_CURRENT_BINARY_DIR}/${QURT_BUNDLE_IDL_NAME}_stub.c" ${FASTRPC_ARM_LIBS}
			DEPENDS generate_${QURT_BUNDLE_IDL_NAME}_stubs
			WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
			)

		FASTRPC_ARM_LINUX_LOAD(
			LOADNAME ${QURT_BUNDLE_APP_NAME}_app
			TARGET ${QURT_BUNDLE_APP_NAME}
			DEPNAME ${QURT_BUNDLE_APP_NAME}_app
			DEST ${QURT_BUNDLE_APP_DEST}
			)
	endif()

	if (NOT "${QURT_BUNDLE_DSP_SOURCES}" STREQUAL "")
		QURT_LIB(LIB_NAME ${QURT_BUNDLE_APP_NAME}
			IDL_NAME ${QURT_BUNDLE_IDL_NAME}
			SOURCES ${QURT_BUNDLE_DSP_SOURCES}
			LINK_LIBS ${QURT_BUNDLE_DSP_LINK_LIBS}
			INCS ${QURT_BUNDLE_DSP_INCS}
		)

	endif()

	# Create a target to load both Apps and DSP code on the target
	if ((NOT "${QURT_BUNDLE_APPS_SOURCES}" STREQUAL "") AND (NOT "${QURT_BUNDLE_DSP_SOURCES}" STREQUAL ""))
		# Add a rule to load the files onto the target
		add_custom_target(${QURT_BUNDLE_APP_NAME}-load
			DEPENDS ${QURT_BUNDLE_APP_NAME}_app-load lib${QURT_BUNDLE_APP_NAME}-load
			COMMAND echo "Pushed ${QURT_BUNDLE_APP_NAME}"
			)
	endif()

endfunction()

