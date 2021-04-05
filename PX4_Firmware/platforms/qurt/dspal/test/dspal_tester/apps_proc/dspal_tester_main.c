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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <getopt.h>

#include "adspmsgd.h"
#include "rpcmem.h"

#include "test_utils.h"
#include "dspal_tester.h"
#include "posix_test_suite.h"
#include "io_test_suite.h"

  
static char *main_help =
        "dspal_tester\n"
        "Usage:\n"
        "\tdspal_tester [-h|--help] [test_suite]\n\n\n"
        "\n"
        "test_suite: string containing the name of the tests to run\n"
        "List of tests:\n"
        "\t--pthread   | -p       basic pthread creation, synchronization and memory allocation tests\n"
        "\t--timers    | -t       timer functionality \n"
        "\t--core      | -c       includes pthreas and timers \n"
        "\t--devices   | -d       I/O basic tests \n"   
        "\t--all       | -a       includes all above tests \n"
        "\t--uart      | -u       uart loopback testing - not yet supported\n"
        "\t--gpio      | -g       gpio loopback testing - not yet supported\n"
        "\t--help      | -h       Prints this help \n"
        "\n";

static struct option main_long_opts[] = {
        { "pthreads", 0, 0, 'p' },
        { "timers",   0, 0, 't' },
        { "core",     0, 0, 'c' },
        { "devices",  0, 0, 'd' },
        { "all",      0, 0, 'a' },
        { "uart",     0, 0, 'u' },
        { "gpio",     0, 0, 'g' },
        { 0, 0, 0, 0 }
};

static char main_short_opts[] = "hptcdaug";

/**
 * @brief Runs all the tests requested at the command line
 *
 * @param   argc[in]    number of arguments
 * @param   argv[in]    array of parameters (each is a char array)
 *
 * @return
 * TEST_PASS ------ All tests passed
 * TEST_FAIL ------ A test has failed
*/

int main(int argc, char *argv[])
{
	int status = TEST_PASS;
    int opt;

    int uart_loopback = 0; 
    int gpio_loopback = 0; 
    int pthreads      = 0; 
    int timers        = 0;  
    int devices       = 0;

    int i = 1; 
    
    if (argc < 2)
    {
       pthreads = 1; 
       timers   = 1; 
       devices  = 1; 
    }

    while ((opt = getopt_long(argc, argv, main_short_opts, main_long_opts, NULL)) != -1) {
            switch (opt) {
            case 'd':
                devices = 1; 
                break;
            case 't':
                timers = 1; 
                break;
            case 'p':
                pthreads = 1; 
                break;

            case 'c':
                /*Core means pthreads + timers*/
                pthreads = 1; 
                timers   = 1; 
                break;

            case 'a':
                pthreads = 1; 
                timers   = 1; 
                devices  = 1; 
                break;

            case 'u':
                uart_loopback = 1; 
                break; 

            case 'g': 
                gpio_loopback = 1; 
                break; 

            case 'h':
            default:
                    LOG_INFO("%s", main_help);
                    return 1;
            }
    }

    if (argc < 2) {
         // run all
         pthreads = 1; 
         timers   = 1; 
         devices  = 1; 
    }

	LOG_INFO("Starting DSPAL tests");

	dspal_tester_test_dspal_get_version_info();

    if ( pthreads ) {
        LOG_INFO("Starting DSPAL pthread tests");
        status |= run_pthreads_test_suite();
    }

    if ( timers ) {
        LOG_INFO("Starting DSPAL timers tests");
        status |= run_timers_test_suite();
    }

    if ( devices ) {
        LOG_INFO("Starting DSPAL devices tests");
        status |= run_io_test_suite();
    }

    if ( uart_loopback ) {
        LOG_INFO("DSPAL uart loopback test not supported");
        status |= TEST_SKIP; 
    }

    if ( gpio_loopback ) {
        LOG_INFO("DSPAL gpio loopback test not supported");
        status |= TEST_SKIP; 
    }

	if ((status & TEST_FAIL) == TEST_FAIL) {
		LOG_INFO("DSPAL test failed.");

	} else {
		if ((status & TEST_SKIP) == TEST_SKIP) {
			LOG_INFO("DSPAL some tests skipped.");
		}

		LOG_INFO("DSPAL tests succeeded.");
	}

	LOG_INFO("");
	return status;
}

