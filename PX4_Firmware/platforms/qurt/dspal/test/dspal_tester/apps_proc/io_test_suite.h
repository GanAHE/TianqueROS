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

#ifndef IO_TEST_SUITE_H_
#define IO_TEST_SUITE_H_


/**
 * @brief Runs all the io tests.
 *
 * @par
 * Tests that are run (in order)
 *  1) spi loopback test (dspal_tester_spi_test)
 *  2) serial I/O test (dspal_tester_serial_test)
 *  3) i2c test (dspal_tester_i2c_test)
 *  4) pwm_test (dspal_tester_pwm_test)
 *  5) farf log_info test (dspal_tester_test_farf_log_info)
 *  6) farf log_err test (dspal_tester_test_farf_log_err)
 *  7) farf log_debug test (dspal_tester_test_farf_log_debug)
 *  8) gpio open/close test (dspal_tester_test_gpio_open_close)
 *  9) gpio ioctl I/O mode test (dspal_tester_test_gpio_ioctl_io)
 * 10) gpio read/write test (dspal_tester_test_gpio_read_write)
 * 11) gpio INT test (dspal_tester_test_gpio_int)
 * 12) file open/close (dspal_tester_test_posix_file_open_close)
 * 13) file read/write (dspal_tester_test_posix_file_read_write)
 * 14) file open_trunc (dspal_tester_test_posix_file_open_trunc)
 * 15) file open_append (dspal_tester_test_posix_file_open_append)
 * 16) file ioctl (dspal_tester_test_posix_file_ioctl)
 * 17) file fsync (dspal_tester_test_posix_file_fsync)
 * 18) file remove (dspal_tester_test_posix_file_remove)
 * 19) fopen/fclose test (dspal_tester_test_fopen_fclose)
 * 20) fwrite/fread test (dspal_tester_test_fwrite_fread)
 *
 * @return
 * TEST_PASS ------ All tests passed
 * TEST_FAIL ------ One or more tests failed
*/
int run_io_test_suite();

#endif /* IO_TEST_SUITE_H_ */
