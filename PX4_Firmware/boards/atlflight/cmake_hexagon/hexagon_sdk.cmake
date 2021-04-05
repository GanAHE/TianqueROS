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
# Hexagon SDK paths need to be set based on env variables
#
# PREREQUISITES:
#
# Environment variables:
#	HEXAGON_TOOLS_ROOT
#	HEXAGON_SDK_ROOT
#
# CMake Variables:
#	QC_SOC_TARGET
#
# OPTIONAL:
#	DSP_TYPE (ADSP or SLPI)

set(TOOLS_ERROR_MSG
		"HEXAGON_Tools must be installed and the environment variable HEXAGON_TOOLS_ROOT must be set"
		"(e.g. export HEXAGON_TOOLS_ROOT=$ENV{HOME}/Qualcomm/HEXAGON_Tools/7.2.12/Tools)")

if ("$ENV{HEXAGON_TOOLS_ROOT}" STREQUAL "")
	message(FATAL_ERROR ${TOOLS_ERROR_MSG})
else()
	set(HEXAGON_TOOLS_ROOT $ENV{HEXAGON_TOOLS_ROOT})
endif()

if ("$ENV{HEXAGON_SDK_ROOT}" STREQUAL "")
	message(FATAL_ERROR "HEXAGON_SDK_ROOT not set")
endif()

if ("$ENV{HEXAGON_SDK_ROOT}" MATCHES "/Hexagon_SDK/2.0")
	message(FATAL_ERROR "HEXAGON_SDK 2.0 no longer supported")
elseif ("$ENV{HEXAGON_SDK_ROOT}" MATCHES "/Hexagon_SDK/3.0")
	set(SDKINC incs)
	set(SDKLIB libs)
	set(SDKLIB libs)
	set(SDKRPCMEMINC /inc)
elseif ("$ENV{HEXAGON_SDK_ROOT}" MATCHES "/Hexagon_SDK/3.1")
	set(SDKINC incs)
	set(SDKLIB libs)
	set(SDKLIB libs)
	set(SDKRPCMEMINC /inc)
else()
        message(FATAL_ERROR "Unsupported/Unknown HEXAGON SDK version")
endif()

set(HEXAGON_SDK_ROOT $ENV{HEXAGON_SDK_ROOT})

set(HEXAGON_SDK_INCLUDES
	${HEXAGON_SDK_ROOT}/${SDKINC}
	${HEXAGON_SDK_ROOT}/${SDKINC}/stddef
	${HEXAGON_SDK_ROOT}/${SDKLIB}/common/rpcmem${SDKRPCMEMINC}
	)

if ("${QC_SOC_TARGET}" STREQUAL "APQ8074")
	set(DSP_TYPE "ADSP")
	set(V_ARCH "v55")
	set(HEXAGON_SDK_INCLUDES ${HEXAGON_SDK_INCLUDES}
		${HEXAGON_SDK_ROOT}/${SDKLIB}/common/qurt/ADSPv55MP/include
		)
elseif ("${QC_SOC_TARGET}" STREQUAL "APQ8096")
	# Set the default to SLPI
	if ("${DSP_TYPE}" STREQUAL "")
		set(DSP_TYPE "SLPI")
	endif()
	set(V_ARCH "v60")
	set(HEXAGON_SDK_INCLUDES ${HEXAGON_SDK_INCLUDES}
		${HEXAGON_SDK_ROOT}/${SDKLIB}/common/qurt/ADSPv60MP/include
		)
else()
	message(FATAL_ERROR "QC_SOC_TARGET not set")
endif()

# Validate DSP_TYPE
if (NOT ("${DSP_TYPE}" STREQUAL "ADSP" OR "${DSP_TYPE}" STREQUAL "SLPI"))
	message(FATAL_ERROR "DSP_TYPE set to invalid value")
endif()
