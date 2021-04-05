/****************************************************************************
 *   Copyright (c) 2015 James Wilson. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ATLFlight nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdio.h>
#include "dspal_tester.h"
#include "test_utils.h"

#define VERSION_PREFIX "DSPAL-"

int run_io_test_suite()
{
	int test_results = TEST_PASS;

	LOG_INFO("testing device path access");
	test_results |= display_test_results( dspal_tester_spi_test(), "spi loopback test");
	test_results |= display_test_results( dspal_tester_serial_test(), "serial I/O test");
	test_results |= display_test_results( dspal_tester_i2c_test(), "i2c test");

    test_results |= display_test_results( dspal_tester_termios_test(), "termios test");

#if defined(DSP_TYPE_ADSP)
	LOG_INFO("testing PWM signaling");
	test_results |= display_test_results( dspal_tester_pwm_test(), "pwm_test");
#endif

    LOG_INFO("testing FARF");
    test_results |= display_test_results( dspal_tester_test_farf_log_info(), "farf log_info test");
    test_results |= display_test_results( dspal_tester_test_farf_log_err(), "farf log_err test");
    test_results |= display_test_results( dspal_tester_test_farf_log_debug(), "farf log_debug test");

	LOG_INFO("testing GPIO");
	test_results |= display_test_results( dspal_tester_test_gpio_open_close(), "gpio open/close test");
	test_results |= display_test_results( dspal_tester_test_gpio_ioctl_io(), "gpio ioctl I/O mode test");
	test_results |= display_test_results( dspal_tester_test_gpio_read_write(), "gpio read/write test");

#if !defined(DSP_TYPE_SLPI)	
	test_results |= display_test_results( dspal_tester_test_gpio_int(), "gpio INT test");
#endif

	LOG_INFO("testing file I/O");
	test_results |= display_test_results( dspal_tester_test_posix_file_open_close(), "file open/close");
	test_results |= display_test_results( dspal_tester_test_posix_file_read_write(), "file read/write");
	test_results |= display_test_results( dspal_tester_test_posix_file_open_trunc(), "file open_trunc");
	test_results |= display_test_results( dspal_tester_test_posix_file_open_append(), "file open_append");
	test_results |= display_test_results( dspal_tester_test_posix_file_ioctl(), "file ioctl");
	test_results |= display_test_results( dspal_tester_test_posix_file_fsync(), "file fsync");
	test_results |= display_test_results( dspal_tester_test_posix_file_remove(), "file remove");
	test_results |= display_test_results( dspal_tester_test_fopen_fclose(), "fopen/fclose test");
	test_results |= display_test_results( dspal_tester_test_fwrite_fread(), "fwrite/fread test");
	test_results |= display_test_results( dspal_tester_test_posix_file_threading(), "fwrite/fread in a different thread test");

	return test_results;
}

