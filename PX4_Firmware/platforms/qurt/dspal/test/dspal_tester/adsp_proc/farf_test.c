/****************************************************************************
 *   Copyright (c) 2016 James Wilson. All rights reserved.
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

#include <dspal_version.h>
#include <unistd.h>
#include "test_utils.h"
#include "dspal_tester.h"

int dspal_tester_test_farf_log_info()
{
	const int NUM_MESSAGE_PERIODS = 5;
	const int NUM_MESSAGES_PER_PERIOD = 20;
	int message_periods[NUM_MESSAGE_PERIODS] = { 1e5, 1e4, 1e3, 1e2, 1e1 };

	for (int i = 0; i < NUM_MESSAGE_PERIODS; i++) {
		for (int j = 0; j < NUM_MESSAGES_PER_PERIOD; j++) {
			LOG_INFO("[%d] LOG_INFO test at %d us period", j,
					message_periods[i]);
			usleep(message_periods[i]);
		}
	}

	return 0;
}

int dspal_tester_test_farf_log_err()
{
	const int NUM_MESSAGE_PERIODS = 5;
	const int NUM_MESSAGES_PER_PERIOD = 20;
	int message_periods[NUM_MESSAGE_PERIODS] = { 1e5, 1e4, 1e3, 1e2, 1e1 };

	for (int i = 0; i < NUM_MESSAGE_PERIODS; i++) {
		for (int j = 0; j < NUM_MESSAGES_PER_PERIOD; j++) {
			LOG_ERR("[%d] LOG_ERR test at %d us period", j, message_periods[i]);
			usleep(message_periods[i]);
		}
	}

	return 0;
}

int dspal_tester_test_farf_log_debug()
{
	const int NUM_MESSAGE_PERIODS = 5;
	const int NUM_MESSAGES_PER_PERIOD = 20;
	int message_periods[NUM_MESSAGE_PERIODS] = { 1e5, 1e4, 1e3, 1e2, 1e1 };

	for (int i = 0; i < NUM_MESSAGE_PERIODS; i++) {
		for (int j = 0; j < NUM_MESSAGES_PER_PERIOD; j++) {
			LOG_DEBUG("[%d] LOG_DEBUG test at %d us period", j,
					message_periods[i]);
			usleep(message_periods[i]);
		}
	}

	return 0;
}
