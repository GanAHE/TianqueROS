############################################################################
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
# 3. Neither the name ATLFlight nor the names of its contributors may be
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
############################################################################

include_directories(
	include
	)

QURT_BUNDLE(APP_NAME dspal_tester
	DSP_SOURCES
		adsp_proc/farf_test.c
		adsp_proc/gpio_test_imp.c
		adsp_proc/i2c_test_imp.c
		adsp_proc/pwm_test_imp.c
		adsp_proc/posix_file_tests.c
		adsp_proc/posix_pthread_tests.c
		adsp_proc/posix_semaphore_tests.c
		adsp_proc/posix_time_tests.c
		adsp_proc/serial_test_imp.c
		adsp_proc/termios_test_imp.c
		adsp_proc/spi_test_imp.c
		adsp_proc/cxx_tests.cpp
		adsp_proc/misc_tests.c
		adsp_proc/qurt_stubs.cpp
		common/test_utils.c
	APPS_SOURCES
		apps_proc/dspal_tester_main.c
		apps_proc/io_test_suite.c
		apps_proc/io_test_suite.h
		apps_proc/posix_test_suite.c
		apps_proc/posix_test_suite.h
		common/test_utils.c
	APPS_INCS
		include
		../include
	APPS_COMPILER ${ARM-LINUX-GNUEABIHF-GCC}
	)

# vim: set noet fenc=utf-8 ff=unix ft=cmake :
