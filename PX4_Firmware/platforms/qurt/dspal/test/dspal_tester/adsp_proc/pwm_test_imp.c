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

#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <dev_fs_lib_pwm.h>
#include <dspal_version.h>

#include "test_status.h"
#include "test_utils.h"

/**
* @brief
* Test to define PWM signals at a specified period and vary the pulse width.
*
* @par
* Test:
* 1) Open the PWM device (/dev/pwm-1)
* 2) Configure the
*     GPIO 5 (pin 3, J9 on the Eagle Board)
*
* 3) Close the PWM device
*
* @return
* SUCCESS ------ Test Passes
* ERROR ------ Test Failed
*/

#define PWM_TEST_PULSE_WIDTH_INCREMENTS 10
#define PWM_TEST_MINIMUM_PULSE_WIDTH 1050
#define INCREMENT_PULSE_WIDTH(x,y) ((x + PWM_TEST_PULSE_WIDTH_INCREMENTS) >= y ? PWM_TEST_MINIMUM_PULSE_WIDTH : x + PWM_TEST_PULSE_WIDTH_INCREMENTS)

int dspal_tester_pwm_test(void)
{
	int ret = SUCCESS;
	int pulse_width;
	int test_count;
	struct dspal_version_info version;
	int major_version, minor_version;

	dspal_get_version_info_ext(&version);
	strtok(version.version_string, "-.");
	major_version = atoi(strtok(NULL, "-."));
	minor_version = atoi(strtok(NULL, "-."));

	if (major_version < 1 || (major_version == 1 && minor_version < 1))
	{
		LOG_INFO("unable to test PWM signaling, current version: %d.%d, requires version 1.1 or above",
				major_version, minor_version);
		return ERROR;
	}
	LOG_INFO("testing PWM signaling, DSPAL version: %d.%d", major_version, minor_version);
	/*
	 * Open PWM device
	 */
	int fd = -1;
	fd = open("/dev/pwm-1", 0);

	if (fd > 0) {
		/*
		 * Configure PWM device
		 */
		struct dspal_pwm pwm_gpio[4];
		struct dspal_pwm_ioctl_signal_definition signal_definition;
		struct dspal_pwm_ioctl_update_buffer *update_buffer;
		struct dspal_pwm *pwm;

		// Define the initial pulse width and number of the GPIO to
		// use for this signal definition.
		// 45,46,47,48
		pwm_gpio[0].gpio_id = 45;
		pwm_gpio[0].pulse_width_in_usecs = PWM_TEST_MINIMUM_PULSE_WIDTH;
		pwm_gpio[1].gpio_id = 46;
		pwm_gpio[1].pulse_width_in_usecs = PWM_TEST_MINIMUM_PULSE_WIDTH + 10;
		pwm_gpio[2].gpio_id = 47;
		pwm_gpio[2].pulse_width_in_usecs = PWM_TEST_MINIMUM_PULSE_WIDTH + 20;
		pwm_gpio[3].gpio_id = 48;
		pwm_gpio[3].pulse_width_in_usecs = PWM_TEST_MINIMUM_PULSE_WIDTH + 21;

		// Describe the overall signal and reference the above array.
		signal_definition.num_gpios = 4;
		signal_definition.period_in_usecs = 2000;
		signal_definition.pwm_signal = &pwm_gpio[0];

		// Send the signal definition to the DSP.
		if (ioctl(fd, PWM_IOCTL_SIGNAL_DEFINITION, &signal_definition) != 0) {
			ret = ERROR;
		}

		// Retrieve the shared buffer which will be used below to update the desired
		// pulse width.
		if (ioctl(fd, PWM_IOCTL_GET_UPDATE_BUFFER, &update_buffer) != 0)
		{
			ret = ERROR;
		}
		pwm = &update_buffer->pwm_signal[0];
		update_buffer->reserved_1 = 0;

		// Wait for the ESC's to ARM:
		usleep(1000000 * 5); // wait 5 seconds

		// Change the speed of the motor, every 500 msecs.
		for (test_count = 0; test_count < 30; test_count++)
		{
			pwm[0].pulse_width_in_usecs = INCREMENT_PULSE_WIDTH(pwm[0].pulse_width_in_usecs, signal_definition.period_in_usecs);
			pwm[1].pulse_width_in_usecs = INCREMENT_PULSE_WIDTH(pwm[1].pulse_width_in_usecs, signal_definition.period_in_usecs);;
			pwm[2].pulse_width_in_usecs = INCREMENT_PULSE_WIDTH(pwm[2].pulse_width_in_usecs, signal_definition.period_in_usecs);
			pwm[3].pulse_width_in_usecs = INCREMENT_PULSE_WIDTH(pwm[3].pulse_width_in_usecs, signal_definition.period_in_usecs);
			usleep(1000 * 500);
			LOG_INFO("reserved_1 count: %d", update_buffer->reserved_1);
		}

		/*
		 * Close the device ID
		 */
		close(fd);
	} else {
		ret = ERROR;
	}

	return ret;
}
