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

include (CMakeParseArguments)

# Process DSP files
function (QURT_LIB)
	set(options)
	set(oneValueArgs LIB_NAME IDL_NAME)
	set(multiValueArgs SOURCES LINK_LIBS INCS FLAGS)
	cmake_parse_arguments(QURT_LIB "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

	if ("${QURT_LIB_IDL_NAME}" STREQUAL "")
		message(FATAL_ERROR "QURT_LIB called without IDL_NAME")
	endif()

	include_directories(
		${CMAKE_CURRENT_BINARY_DIR}
		${FASTRPC_DSP_INCLUDES}
		)

	message("QURT_LIB_INCS = ${QURT_LIB_INCS}")

	add_library(${QURT_LIB_IDL_NAME}_skel MODULE
		${QURT_LIB_IDL_NAME}_skel.c
		)

	if ("${QURT_LIB_SOURCES}" STREQUAL "")
		message(FATAL_ERROR "QURT_LIB called without SOURCES")
	else()
		# Build lib that is run on the DSP
		add_library(${QURT_LIB_LIB_NAME} SHARED
			${QURT_LIB_SOURCES}
			)

		if (NOT "${QURT_LIB_FLAGS}" STREQUAL "")
			set_target_properties(${QURT_LIB_LIB_NAME} PROPERTIES COMPILE_FLAGS "${QURT_LIB_FLAGS}")
		endif()

		if (NOT "${QURT_LIB_INCS}" STREQUAL "")
			target_include_directories(${QURT_LIB_LIB_NAME} PUBLIC ${QURT_LIB_INCS})
		endif()

		message("QURT_LIB_LINK_LIBS = ${QURT_LIB_LINK_LIBS}")

		target_link_libraries(${QURT_LIB_LIB_NAME}
			${QURT_LIB_LINK_LIBS}
			)

		add_dependencies(${QURT_LIB_LIB_NAME} generate_${QURT_LIB_IDL_NAME}_stubs)

		# Hack to support PX4 - because it links static libs into .so targets it ends up with
		# Duplicate symbols. This can be reverted to link ${QURT_LIB_LIB_NAME} when PX4 is fixed.
		target_link_libraries(${QURT_LIB_IDL_NAME}_skel
			"${CMAKE_CURRENT_BINARY_DIR}/lib${QURT_LIB_LIB_NAME}.so"
			)

		add_dependencies(${QURT_LIB_IDL_NAME}_skel
			generate_${QURT_LIB_IDL_NAME}_stubs
			${QURT_LIB_LIB_NAME}
			)
	endif()

	#message("Making custom target build_${QURT_LIB_LIB_NAME}_dsp")
	#add_custom_target(build_${QURT_LIB_LIB_NAME}_dsp ALL
	#	DEPENDS ${QURT_LIB_IDL_NAME} ${QURT_LIB_IDL_NAME}_skel
	#	)

   if ("${QC_SOC_TARGET}" STREQUAL "APQ8096")
      # Set the location for 8x96 target
      set(DSPLIB_TARGET_PATH "/usr/lib/rfsa/adsp/")
   else()
      # 8x74 target is assumed by default.
      set(DSPLIB_TARGET_PATH "/usr/share/data/adsp/")
   endif()
   
	if ("${QURT_LIB_LIB_NAME}" STREQUAL "")
		# Add a rule to load the files onto the target that run in the DSP
		add_custom_target(lib${QURT_LIB_IDL_NAME}_skel-load
			DEPENDS ${QURT_LIB_IDL_NAME}_skel
			COMMAND adb wait-for-device
			COMMAND adb push lib${QURT_LIB_IDL_NAME}_skel.so ${DSPLIB_TARGET_PATH}
			COMMAND echo "Pushed lib${QURT_LIB_IDL_NAME}_skel.so ${DSPLIB_TARGET_PATH}"
			)
	else()
		# Add a rule to load the files onto the target that run in the DSP
		add_custom_target(lib${QURT_LIB_LIB_NAME}-load
			DEPENDS ${QURT_LIB_LIB_NAME} ${QURT_LIB_IDL_NAME}_skel
			COMMAND adb wait-for-device
			COMMAND adb push lib${QURT_LIB_IDL_NAME}_skel.so ${DSPLIB_TARGET_PATH}
			COMMAND adb push lib${QURT_LIB_LIB_NAME}.so ${DSPLIB_TARGET_PATH}
			COMMAND echo "Pushed lib${QURT_LIB_LIB_NAME}.so and dependencies to ${DSPLIB_TARGET_PATH}"
			)
	endif()
endfunction()

