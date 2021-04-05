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

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <dev_fs_lib_serial.h>
#include <platform.h>

#include "test_utils.h"
#include "test_status.h"

#define SERIAL_TEST_CYCLES 10
#define SERIAL_SIZE_OF_DATA_BUFFER 128
#define SERIAL_WRITE_DELAY_IN_USECS (8000 * 10)

/**
 * Snapdragon Flight DSP supports up to 6 UART devices. However, the actual
 * number of available UART devices may vary depending on the board schematic
 * design. On Snapdragon Flight reference platform, 4 UART ports are available
 * for communication to external serial devices.
 * To make the serial test work for your board, please make sure the following
 * are compliant to your set up:
 * - set NUM_UART_DEVICE_ENABLED with the number of UART ports available
 * - make sure to define UART-BAM mappings in /usr/share/data/adsp/blsp.config
 *   See snapdragon flight user guide for details.
 */
#if defined(DSP_TYPE_ADSP)
#define MAX_UART_DEVICE_NUM      6

const char *serial_device_path[MAX_UART_DEVICE_NUM] = {
	"/dev/tty-1", "/dev/tty-2", "/dev/tty-3",
	"/dev/tty-4", "/dev/tty-5", "/dev/tty-6"
};

#elif defined(DSP_TYPE_SLPI)

#define MAX_UART_DEVICE_NUM      4
const char *serial_device_path[MAX_UART_DEVICE_NUM] = {
	"/dev/tty-5", "/dev/tty-7", "/dev/tty-9",
	"/dev/tty-12" 
};
#endif

int serial_fildes[MAX_UART_DEVICE_NUM] = { -1};


/**
* @brief Test serial device open and close
*
* @par Detailed Description:
* Up to 6 UART devices can be supported. Iterate over /dev/tty-1 to /dev/tty-6
* and open/close each device in O_RDWR mode one by one.
*
* @return
* SUCCESS always
*/
int dspal_tester_serial_multi_port_open(void)
{
	int i;
	int active_devices = 0;
	LOG_INFO("beginning serial device open test");
	int result = SUCCESS;

	for (i = 0; i < NUM_UART_DEVICE_ENABLED; i++) {
		serial_fildes[i] = open(serial_device_path[i], O_RDWR);
		active_devices += (serial_fildes[i] >= SUCCESS);
		LOG_INFO("open %s O_RDWR mode %s", serial_device_path[i],
			 (serial_fildes[i] < SUCCESS) ? "fail" : "succeed");
	}

	for (i = 0; i < NUM_UART_DEVICE_ENABLED; i++) {
		if (serial_fildes[i] >= SUCCESS) {
			close(serial_fildes[i]);
		}
	}

	result = (active_devices == NUM_UART_DEVICE_ENABLED) ? SUCCESS : ERROR;

	LOG_INFO("serial multi-port open test %s",
		 result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

void multi_port_read_callback(void *context, char *buffer, size_t num_bytes)
{
	int rx_dev_id = (int)context;
	char rx_buffer[SERIAL_SIZE_OF_DATA_BUFFER];

	if (num_bytes > 0) {
		memcpy(rx_buffer, buffer, num_bytes);
		rx_buffer[num_bytes] = 0;
		LOG_INFO("/dev/tty-%d read callback received bytes [%d]: %s",
			 rx_dev_id, num_bytes, rx_buffer);

	} else {
		LOG_ERR("error: read callback with no data in the buffer");
	}
}

/**
* @brief Test multiple serial device at the same time for open,write,read,close.
*
* @par Test:
* 1) Open the serial device /dev/tty-[1-6]. Note: some device open may fail
*    if it is not enabled or configured. See dev_fs_lib_serial.h for more
*    details about how to configure and enable UART devices on the board.
* 2) register read callback on the opened serial devices
* 3) write data to each ports
* 4) close all serial devices
*
* @return
* - SUCCESS if write succeeds on all serial devices through SERIAL_TEST_CYCLES cycles
* - Error otherwise
*/
int dspal_tester_serial_multi_port_write_read_callback(void)
{
	int result = SUCCESS;
	char tx_buffer[SERIAL_SIZE_OF_DATA_BUFFER];
	unsigned int num_bytes_written;
	int active_devices = 0;
	int runs, i;

	LOG_INFO("beginning serial multi-port read/write callback test");

	// try to open all uart ports
	for (i = 0; i < NUM_UART_DEVICE_ENABLED; i++) {
		serial_fildes[i] = open(serial_device_path[i], O_RDWR);
		LOG_INFO("open %s O_RDWR mode %s", serial_device_path[i],
			 (serial_fildes[i] < SUCCESS) ? "fail" : "succeed");
	}

	// set read callback on all uart ports
	struct dspal_serial_ioctl_receive_data_callback receive_callback;
	receive_callback.rx_data_callback_func_ptr = multi_port_read_callback;

	for (i = 0; i < NUM_UART_DEVICE_ENABLED; i++) {
		if (serial_fildes[i] >= SUCCESS) {
			receive_callback.context = (void *)(i + 1);

			result = ioctl(serial_fildes[i],
				       SERIAL_IOCTL_SET_RECEIVE_DATA_CALLBACK,
				       (void *)&receive_callback);
			LOG_INFO("set serial read callback on %s %s",
				 serial_device_path[i], result < SUCCESS ? "failed" : "succeeded");

			if (result < SUCCESS) {
				close(serial_fildes[i]);
				serial_fildes[i] = -1;
			}
		}
	}

	for (runs = 0; runs < SERIAL_TEST_CYCLES; runs++) {
		LOG_DEBUG("runs %d", runs);

		active_devices = 0;

		for (i = 0; i < NUM_UART_DEVICE_ENABLED; i++) {
			if (serial_fildes[i] < SUCCESS) {
				continue;
			}

			memset(tx_buffer, 0, SERIAL_SIZE_OF_DATA_BUFFER);
			sprintf(tx_buffer, "message from /dev/tty-%d\n", i + 1);

			num_bytes_written = write(serial_fildes[i],
						  (const char *)tx_buffer,
						  strlen(tx_buffer));

			if (num_bytes_written == strlen(tx_buffer)) {
				LOG_DEBUG("written %d bytes to %s", num_bytes_written,
					  serial_device_path[i]);
				active_devices++;

			} else {
				LOG_ERR("failed to write to %s", serial_device_path[i]);
				close(serial_fildes[i]);
				serial_fildes[i] = -1;
			}
		}

		if (active_devices == 0) {
			break;
		}

		usleep(SERIAL_WRITE_DELAY_IN_USECS);
	}

	// close all devices
	for (i = 0; i < NUM_UART_DEVICE_ENABLED; i++) {
		if (serial_fildes[i] >= SUCCESS) {
			close(serial_fildes[i]);
		}
	}

	if (!(runs == SERIAL_TEST_CYCLES && active_devices == NUM_UART_DEVICE_ENABLED)) {
		result = ERROR;
	}

	LOG_INFO("serial multi-port read/write callback test %s",
		 result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

/**
* @brief Test serial read and write functionality
*
* @par Detailed Description:
* The serial bus has its RX and TX wired together so it can do a loop-back of the data.
* Data is sent over the bus and read back using a read call
*
* Test:
* 1) Open the serial device /dev/tty-[1-6]
* 2) Write data to the serial device
* 3) Read data using read call, print out received data if available
* 4) Loop steps 2-3 for SERIAL_TEST_CYCLES number of loops
* 5) Close all serial devices
*
* @return
* - SUCCESS if write succeeds on all serial devices through SERIAL_TEST_CYCLES cycles
* - Error otherwise
*/
int dspal_tester_serial_multi_port_write_read(void)
{
	int result = SUCCESS;
	unsigned int num_bytes_written = 0;
	int num_bytes_read = 0;
	char tx_buffer[SERIAL_SIZE_OF_DATA_BUFFER];
	char rx_buffer[SERIAL_SIZE_OF_DATA_BUFFER];
	int active_devices;
	int runs, i;

	LOG_INFO("beginning multi-port serial read/write test");

	// try to open all uart ports
	for (i = 0; i < NUM_UART_DEVICE_ENABLED; i++) {
		serial_fildes[i] = open(serial_device_path[i], O_RDWR);
		LOG_INFO("open %s O_RDWR mode %s", serial_device_path[i],
			 (serial_fildes[i] < SUCCESS) ? "fail" : "succeed");
	}

	// repeatedly write and read from each opened serial port
	for (runs = 0; runs < SERIAL_TEST_CYCLES; runs++) {
		LOG_DEBUG("runs %d", runs);
		active_devices = 0;

		for (i = 0; i < NUM_UART_DEVICE_ENABLED; i++) {
			if (serial_fildes[i] < SUCCESS) {
				continue;
			}

			memset(tx_buffer, 0, SERIAL_SIZE_OF_DATA_BUFFER);
			sprintf(tx_buffer, "message from /dev/tty-%d\n", i + 1);

			num_bytes_written = write(serial_fildes[i],
						  (const char *)tx_buffer,
						  strlen(tx_buffer));

			if (num_bytes_written == strlen(tx_buffer)) {
				LOG_DEBUG("written %d bytes to %s", num_bytes_written,
					  serial_device_path[i]);
				active_devices++;

			} else {
				LOG_ERR("failed to write to %s", serial_device_path[i]);
				close(serial_fildes[i]);
				serial_fildes[i] = -1;
				continue;
			}

			memset(rx_buffer, 0, SERIAL_SIZE_OF_DATA_BUFFER);
			num_bytes_read = read(serial_fildes[i], rx_buffer,
					      SERIAL_SIZE_OF_DATA_BUFFER);
			LOG_DEBUG("%s read bytes [%d]: %s",
				  serial_device_path[i], num_bytes_read, rx_buffer);
		}

		if (active_devices == 0) {
			break;
		}

		usleep(SERIAL_WRITE_DELAY_IN_USECS);
	}

	// close all devices
	for (i = 0; i < NUM_UART_DEVICE_ENABLED; i++) {
		if (serial_fildes[i] >= SUCCESS) {
			close(serial_fildes[i]);
		}
	}

	if (!(runs == SERIAL_TEST_CYCLES && active_devices == NUM_UART_DEVICE_ENABLED)) {
		result = ERROR;
	}

	LOG_INFO("serial multi-port read/write test %s",
		 result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

/**
* @brief Test serial read with small buffer
*
* @par Detailed Description:
* This test case is testing the scenario in which the serial bus receives X
* bytes. User calls read() to read the data and passes in a small buffer.
* read() should return -EINVAL error code in this caes
*
* Test:
* 1) Open the serial device /dev/tty-1
* 2) Write very long bytes to the serial device
* 3) wait for 100ms to make sure the loopback data is received
* 3) read() with 10 byte buffer and check the return value
* 5) Close serial device
*
* @return
* - SUCCESS
*/
int dspal_tester_serial_read_with_small_buffer(void)
{
	int result = SUCCESS;
	int num_bytes_written = 0;
	int num_bytes_read = 0;
	char tx_buffer[SERIAL_SIZE_OF_DATA_BUFFER];
	char rx_buffer[SERIAL_SIZE_OF_DATA_BUFFER];
	int fd;
	int max_read_bytes = 20;
	int devid = 1;

	LOG_INFO("beginning serial read with small buffer test");

	fd = open(serial_device_path[devid], O_RDWR);
	LOG_INFO("open %s O_RDWR mode %s", serial_device_path[devid],
		 (fd < SUCCESS) ? "fail" : "succeed");

	if (fd < SUCCESS) {
		result = ERROR;
		goto exit;
	}

	memset(tx_buffer, 0, SERIAL_SIZE_OF_DATA_BUFFER);
	sprintf(tx_buffer, "message from /dev/tty-%d\n", devid);

	num_bytes_written = write(fd,
				  (const char *)tx_buffer,
				  SERIAL_SIZE_OF_DATA_BUFFER);

	if (num_bytes_written == SERIAL_SIZE_OF_DATA_BUFFER) {
		LOG_DEBUG("written %d bytes to %s", num_bytes_written,
			  serial_device_path[devid]);

	} else {
		LOG_ERR("failed to write to %s", serial_device_path[devid]);
		goto exit;
	}

	// wait 100ms to ensure the data is received in the loopback
	usleep(100000);
	memset(rx_buffer, 0, SERIAL_SIZE_OF_DATA_BUFFER);
	num_bytes_read = read(fd, rx_buffer, max_read_bytes);

	if (num_bytes_read == -1) {
		LOG_DEBUG("%s read() with small buffer return expected error code -1",
			  serial_device_path[devid]);

	} else {
		LOG_ERR("%s read() return: %d, expected -1",
			serial_device_path[devid], num_bytes_read);
	}

exit:

	if (fd >= SUCCESS) {
		close(fd);
	}

	LOG_INFO("serial read with small buffer test %s",
		 result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

/**
* @brief Runs all the serial tests and returns 1 aggregated result.
*
* @return
* SUCCESS ------ All tests pass
* ERROR -------- One or more tests failed
*/
int dspal_tester_serial_test(void)
{
	int result;

	// serial devices open test
	result = dspal_tester_serial_multi_port_open();

	if (result < SUCCESS) {
		return result;
	}

	// multi-port write/read test with rx callback
	result = dspal_tester_serial_multi_port_write_read_callback();

	if (result < SUCCESS) {
		return result;
	}

	// multi-port read/write test
	result = dspal_tester_serial_multi_port_write_read();

	if (result < SUCCESS) {
		return result;
	}

	result = dspal_tester_serial_read_with_small_buffer();

	if (result < SUCCESS) {
		return result;
	}

	return SUCCESS;
}
