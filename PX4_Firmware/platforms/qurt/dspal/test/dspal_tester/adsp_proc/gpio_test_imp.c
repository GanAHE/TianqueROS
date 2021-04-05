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

#include <fcntl.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <dev_fs_lib_gpio.h>

#include "test_status.h"
#include "test_utils.h"

#include "platform.h"

bool isr_called = false;


/**
 * @brief Test Open and Close functionality of the GPIO Device (gpio-56)
 *
 * @par Test:
 * 1) Opens file for GPIO device
 * 2) If devices file opens return TEST_PASS
 *
 * @return
 * TEST_PASS ------ Test Passes (file opened successfully)
 * TEST_FAIL ------ Test Failed (file failed to open)
*/
int dspal_tester_test_gpio_open_close(void)
{
	int result = TEST_PASS;
	int fd;
	fd = open(GPIO_DEVICE_PATH, 0);

	if (fd == -1) {
		LOG_ERR("open gpio device failed.");
		result = TEST_FAIL;
		goto exit;
	}

exit:
	close(fd);
	return result;
}

/**
* @brief Test IOCTL function to configure GPIO device into IO mode
*
* @par Test:
* 1) Opens file for GPIO device  (gpio-56)
* 2) Uses ioctl to set the GPIO pin to me in IO mode and checks that this succeeds
* 3) Repeat step 2 but check that it fails.  ioctl should fail the 2nd time it tries to configure the pin
*
* @return
* TEST_PASS ------ Test Passes
* TEST_FAIL ------ Test Failed
*/
int dspal_tester_test_gpio_ioctl_io(void)
{
	int result = TEST_PASS;
	int fd;
	fd = open(GPIO_DEVICE_PATH, 0);

	if (fd == -1) {
		LOG_ERR("open gpio device failed.");
		result = TEST_FAIL;
		goto exit;
	}

	struct dspal_gpio_ioctl_config_io config = {
		.direction = DSPAL_GPIO_DIRECTION_OUTPUT,
		.pull = DSPAL_GPIO_NO_PULL,
		.drive = DSPAL_GPIO_2MA,
	};

	// configure GPIO device into general purpose IO mode
	if (ioctl(fd, DSPAL_GPIO_IOCTL_CONFIG_IO, (void *)&config) != SUCCESS) {
		LOG_ERR("ioctl gpio device failed");
		result = TEST_FAIL;
		goto exit;
	}

	// Test if ioctl() is rejected if called more than once
	if (ioctl(fd, DSPAL_GPIO_IOCTL_CONFIG_IO, (void *)&config) == SUCCESS) {
		LOG_ERR("duplicate ioctl call test failed");
		result = TEST_FAIL;
		goto exit;

	} else {
		LOG_INFO("duplicate ioctl call expected to be rejected. PASSED");
	}

exit:
	close(fd);
	return result;
}


/**
* @brief This test toggles the GPIO pin at 10Hz.
*
* @par Tests:
* This test toggles the GPIO pin at 10Hz. After each write, we read the value
* from GPIO port to validate the write result.
* Test:
* 1) Opens file for GPIO device  (gpio-56)
* 2) Uses ioctl to set the GPIO pin to me in IO mode and checks that this succeeds
* 3) Write the value LOW to the GPIO pin (exit if fails to write)
* 4) Read the value of the GPIO pin back to see if the value from step 3 has been
*    written (exit if fails to read or if the value read was not the same as the value
*    written in step 3)
* 5) Loop steps 3-4 100 times at 100 milliseconds per loop, invert signal to write to GPIO Pin
*
* @return
* TEST_PASS ------ Test Passes
* TEST_FAIL ------ Test Failed
*/
int dspal_tester_test_gpio_read_write(void)
{
	int result = TEST_PASS;
	int fd;
	int bytes;
	enum DSPAL_GPIO_VALUE_TYPE value_written = DSPAL_GPIO_LOW_VALUE;
	enum DSPAL_GPIO_VALUE_TYPE value_read;
	struct dspal_gpio_ioctl_config_io config = {
		.direction = DSPAL_GPIO_DIRECTION_OUTPUT,
		.pull = DSPAL_GPIO_NO_PULL,
		.drive = DSPAL_GPIO_2MA,
	};

	// Open GPIO device
	fd = open(GPIO_DEVICE_PATH, 0);

	if (fd == -1) {
		LOG_ERR("open gpio device failed.");
		result = TEST_FAIL;
		goto exit;
	}

	// Configure GPIO device into general purpose I/O mode
	if (ioctl(fd, DSPAL_GPIO_IOCTL_CONFIG_IO, (void *)&config) != SUCCESS) {
		LOG_ERR("ioctl gpio device failed");
		result = TEST_FAIL;
		goto exit;
	}

	// Toggle GPIO device output value for a number of repetitions
	for (int i = 0; i < 100; i++) {
		value_written = value_written ^ 0x01;

		LOG_DEBUG("write GPIO %s: %d", GPIO_DEVICE_PATH, value_written);

		// set output value
		bytes = write(fd, &value_written, 1);

		if (bytes != 1) {
			LOG_ERR("error: write failed");
			result = TEST_FAIL;
			goto exit;
		}

		// verify the write result by reading the output from the same GPIO
		bytes = read(fd, &value_read, 1);

		if (bytes != 1) {
			LOG_ERR("error: read failed");
			result = TEST_FAIL;
			goto exit;
		}

		LOG_DEBUG("read from GPIO %s: %d", GPIO_DEVICE_PATH, value_read);

		if (value_read != value_written) {
			LOG_ERR("error: read inconsistent value");
			result = TEST_FAIL;
			goto exit;
		}

		// sleep between each toggle
		usleep(100000);
	}

exit:
	close(fd);
	return result;
}

/**
* @brief This test toggles one GPIO pin at read from another GPIO pin which it is connected by wire externally
*
* @par Tests:
* This tests uses 2 GPIO pins. It toggles one GPIO pin at 10Hz. After each write, we read the value
* from this GPIO port to validate the write result, and also read from the other GPIO pin connected externally
* Test:
* 1) Opens file for GPIO device  (gpio-10), and the other GPIO device (gpio-11)
* 2) Uses ioctl to set the two GPIO pin to me in IO mode and checks that this succeeds
* 3) Write the value LOW to the GPIO pin (exit if fails to write)
* 4) Read the value of these two GPIO pins back to see if the value from step 3 has been
*    written (exit if fails to read or if the value read was not the same as the value
*    written in step 3)
* 5) Loop steps 3-4 100 times at 100 milliseconds per loop, invert signal to write to GPIO Pin
*
* @return
* TEST_PASS ------ Test Passes
* TEST_FAIL ------ Test Failed
*/
int dspal_tester_test_gpio_read_write_extern_loopback(void)
{
	int result = TEST_PASS;
	int fd, fd_loopback;
	int bytes, bytes2;
	enum DSPAL_GPIO_VALUE_TYPE value_written = DSPAL_GPIO_LOW_VALUE;
	enum DSPAL_GPIO_VALUE_TYPE value_read, value_read2;
	struct dspal_gpio_ioctl_config_io config = {
		.direction = DSPAL_GPIO_DIRECTION_OUTPUT,
		.pull = DSPAL_GPIO_NO_PULL,
		.drive = DSPAL_GPIO_2MA,
	};
	struct dspal_gpio_ioctl_config_io config2 = {
		.direction = DSPAL_GPIO_DIRECTION_INPUT,
		.pull = DSPAL_GPIO_NO_PULL,
		.drive = DSPAL_GPIO_2MA,
	};

	// Open GPIO device
	fd = open(GPIO_DEVICE_PATH, 0);
	fd_loopback = open(GPIO_DEVICE_PATH_LOOPBACK, 0);
	if (fd == -1 || fd == -1) {
		LOG_ERR("open gpio device failed.");
		result = TEST_FAIL;
		goto exit;
	}

	// Configure GPIO device into general purpose I/O mode
	if (ioctl(fd, DSPAL_GPIO_IOCTL_CONFIG_IO, (void *)&config) != SUCCESS ||
		(ioctl(fd_loopback, DSPAL_GPIO_IOCTL_CONFIG_IO, (void *)&config2) != SUCCESS)) {
		LOG_ERR("ioctl gpio device failed");
		result = TEST_FAIL;
		goto exit;
	}

	// Toggle GPIO device output value for a number of repetitions
	for (int i = 0; i < 100; i++) {
		value_written = value_written ^ 0x01;

		LOG_DEBUG("write GPIO %s: %d", GPIO_DEVICE_PATH, value_written);

		// set output value
		bytes = write(fd, &value_written, 1);

		if (bytes != 1) {
			LOG_ERR("error: write failed");
			result = TEST_FAIL;
			goto exit;
		}

		// verify the write result by reading the output from the same GPIO
		bytes = read(fd, &value_read, 1);
		bytes2 = read(fd_loopback, &value_read2, 1);
		if (bytes != 1 || bytes2 != 1) {
			LOG_ERR("error: read failed");
			result = TEST_FAIL;
			goto exit;
		}

		LOG_DEBUG("read from GPIO %s: %d, %s: %d", GPIO_DEVICE_PATH, value_read, GPIO_DEVICE_PATH_LOOPBACK, value_read2);

		if (value_read != value_written || value_read != value_read2) {
			LOG_ERR("error: read inconsistent value");
			result = TEST_FAIL;
			goto exit;
		}

		// sleep between each toggle
		usleep(100000);
	}

exit:
	close(fd);
	return result;
}

/**
* @brief Interrupt service routine for the GPIO interrupt test.
*
* @return
* NULL --- Always
*/
void *gpio_int_isr(DSPAL_GPIO_INT_ISR_CTX context)
{
	bool *val = (bool *)context;

	LOG_DEBUG("gpio_int_isr called");

	*val = true;

	return NULL;
}


/**
* @brief Test to see if a GPIO hardware interrupt can be setup and used correctly
*
* @par Detailed Description:
* This tests uses 2 GPIO pins.  1 of the pins is setup as a hardware interrupt pin
* that will be triggered on a rising edge.  The other pin is set to be a GPIO IO
* pin.  It is initially set to be LOW and then the interrupt pin is configured.
* After that the IO pin state is changed from LOW to HIGH to trigger the interupt
* GPIO interrupt (the 2 pins should be wired together)

* Test:
* 1) Opens file for GPIO A device (IO Pin) (gpio-56)
* 2) Uses ioctl to set the GPIO pin A to me in IO mode and checks that this succeed
* 3) Sets the GPIO pin A to be LOW
* 4) Opens file for GPIO B device (interrupt Pin) (gpio-55)
* 5) ioctl the pin B to be a hardware interrupt pin (triggered on a rising edge)
*    using the 'gpio_int_isr' function as the interrupt service routine (ISR)
* 6) Set the GPIO pin A to be HIGH to generate a rising edge for GPIO pin B to trigger
* 7) wait 1 second of the interrupt to process
* 8) checks to see if a flag that is set by the ISR was set (AKA ISR called)
* 9) Close both GPIO devices for clean close
*
* @return
* TEST_PASS ------ Test Passes
* TEST_FAIL ------ Test Failed
* TEST_SKIP ------ Test Skipped
*/
int dspal_tester_test_gpio_int(void)
{
	int result = TEST_PASS;
#ifdef DO_JIG_TEST
	enum DSPAL_GPIO_VALUE_TYPE value_written;
	int fd;
	int int_fd = -1;
	int bytes = 0;

	// Open GPIO device at GPIO_DEVICE_PATH for general purpose I/O
	fd = open(GPIO_DEVICE_PATH, 0);

	if (fd == -1) {
		LOG_ERR("open gpio device failed.");
		result = TEST_FAIL;
		goto exit;
	}

	struct dspal_gpio_ioctl_config_io config = {
		.direction = DSPAL_GPIO_DIRECTION_OUTPUT,
		.pull = DSPAL_GPIO_NO_PULL,
		.drive = DSPAL_GPIO_2MA,
	};

	if (ioctl(fd, DSPAL_GPIO_IOCTL_CONFIG_IO, (void *)&config) != SUCCESS) {
		LOG_ERR("ioctl gpio device failed");
		result = TEST_FAIL;
		goto exit;
	}

	// set initial output value to LOW
	value_written = DSPAL_GPIO_LOW_VALUE;
	bytes = write(fd, &value_written, 1);

	if (bytes != 1) {
		LOG_ERR("error: write failed");
		result = TEST_FAIL;
		goto exit;
	}

	// Open GPIO Device at GPIO_INT_DEVICE_PATH
	int_fd = open(GPIO_INT_DEVICE_PATH, 0);

	if (int_fd == -1) {
		LOG_ERR("open gpio device failed.");
		result = TEST_FAIL;
		goto exit;
	}

	// Configure this GPIO device as interrupt source
	struct dspal_gpio_ioctl_reg_int int_config = {
		.trigger = DSPAL_GPIOINT_TRIGGER_RISING,
		.isr = (DSPAL_GPIO_INT_ISR) &gpio_int_isr,
		.isr_ctx = (DSPAL_GPIO_INT_ISR_CTX) &isr_called,
	};

	if (ioctl(int_fd, DSPAL_GPIO_IOCTL_CONFIG_REG_INT, (void *)&int_config) != SUCCESS) {
		LOG_ERR("error: ioctl DSPAL_GPIO_IOCTL_CONFIG_REG_INT failed");
		result = TEST_FAIL;
		goto exit;
	}

	// Set output to HIGH to generate RISING edge to trigger the interrupt on
	// the interrupt GPIO device
	value_written = DSPAL_GPIO_HIGH_VALUE;
	bytes = write(fd, &value_written, 1);

	if (bytes != 1) {
		LOG_ERR("error: write failed");
		result = TEST_FAIL;
		goto exit;
	}

	usleep(1000000);

	// check if isr was called
	if (!isr_called) {
		LOG_ERR("error: isr is not called");
		result = TEST_FAIL;
		goto exit;
	}

exit:
	close(int_fd);
	close(fd);
#else
	result = TEST_SKIP;
#endif
	return result;
}
