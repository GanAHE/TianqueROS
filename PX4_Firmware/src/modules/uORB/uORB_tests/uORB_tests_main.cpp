/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

#include <string.h>

#include "uORBTest_UnitTest.hpp"

extern "C" { __EXPORT int uorb_tests_main(int argc, char *argv[]); }

static void usage()
{
	PX4_INFO("Usage: uorb_tests [latency_test]");
}

int
uorb_tests_main(int argc, char *argv[])
{
	/*
	 * Test the driver/device.
	 */
	if (argc == 1) {
		uORBTest::UnitTest &t = uORBTest::UnitTest::instance();
		int rc = t.test();

		if (rc == OK) {
			PX4_INFO("PASS");
			return 0;

		} else {
			PX4_ERR("FAIL");
			return -1;
		}
	}

	/*
	 * Test the latency.
	 */
	if (argc > 1 && !strcmp(argv[1], "latency_test")) {

		uORBTest::UnitTest &t = uORBTest::UnitTest::instance();

		if (argc > 2 && !strcmp(argv[2], "medium")) {
			return t.latency_test<orb_test_medium_s>(ORB_ID(orb_test_medium), true);

		} else if (argc > 2 && !strcmp(argv[2], "large")) {
			return t.latency_test<orb_test_large_s>(ORB_ID(orb_test_large), true);

		} else {
			return t.latency_test<orb_test_s>(ORB_ID(orb_test), true);
		}
	}

	usage();
	return -EINVAL;
}
