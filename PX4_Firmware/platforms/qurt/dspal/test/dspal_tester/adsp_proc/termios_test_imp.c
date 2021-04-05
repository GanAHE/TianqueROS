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

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <dev_fs_lib_serial.h>

#include "test_utils.h"
#include "test_status.h"

#define TERMIOS_TEST_CYCLES 3
#define TERMIOS_SEND_DELAY_MSEC (400000)

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
#define NUM_UART_DEVICE_ENABLED  4

const char *serial_device_paths[MAX_UART_DEVICE_NUM] = {
	"/dev/tty-1", "/dev/tty-2", "/dev/tty-3",
	"/dev/tty-4", "/dev/tty-5", "/dev/tty-6"
};

#elif defined(DSP_TYPE_SLPI)

#define MAX_UART_DEVICE_NUM      4
#define NUM_UART_DEVICE_ENABLED  4

const char *serial_device_paths[MAX_UART_DEVICE_NUM] = {
	"/dev/tty-5", "/dev/tty-7", "/dev/tty-9",
	"/dev/tty-12" 
};
#endif
/**
* @brief Get which device paths have a loop-back connection (wire connecting RX and TX)
*
* @par Detailed Description:
* Some tests require that the RX and TX connections have a wire connecting them to allow
* for a loop-back to be performed.  This function will do a rudimentary check to see which
* device paths if any have the loop-back wire.
*
* Test:
* 1) Open the serial device /dev/tty-[1-6]
* 2) Write data to the serial device
* 3) Read data using read call
* 4) Check the data read matches the data sent, if yes then there was a loop-back.
* 5) Close all serial devices
*
* @param looped_device_paths, Pointer to array that have a loop-back connection
*
*
* @return
* - None
*/
void dspal_tester_termios_helper_get_looped_connections(int *looped_device_paths)
{
	const int BUFFER_SIZES = 50;
	int fd;
	int devices;
	int i;
	int size_send = BUFFER_SIZES - 1;
	char tx_buffer[BUFFER_SIZES];
	char rx_buffer[BUFFER_SIZES];
	int num_bytes_written;
	int num_bytes_read;
	boolean is_looped;
	boolean is_looped_tmp;

	for (devices = 0; devices < NUM_UART_DEVICE_ENABLED; devices++)
	{
		is_looped = FALSE;
		fd = open(serial_device_paths[devices], O_RDWR);

		if (fd < SUCCESS)
		{
			LOG_DEBUG("open %s O_RDWR mode failed", serial_device_paths[devices]);
			continue;
		}

		/* Clear both buffers */
		memset(tx_buffer, 0, sizeof(tx_buffer));
		memset(rx_buffer, 0, sizeof(rx_buffer));

		/* Fill the buffer with the correct number of characters which is one less
		 * than the size of the buffer.  This allows for NULL termination.
		 */
		for (i = 0; i < size_send; i++)
		{
			/* Fill in with random letters. */
			tx_buffer[i] = 'a' + (rand() % 26);
		}

		/* Transmit the data so that we can check if we can read back the same data. */
		num_bytes_written = write(fd, (const char *)tx_buffer, strlen(tx_buffer));

		/* Give enough time for the data to be transmitted. */
		usleep(TERMIOS_SEND_DELAY_MSEC);

		/* Only if the data was all correctly transmitted. */
		if (num_bytes_written == size_send)
		{
			/* Read in data, as much as we can. */
			num_bytes_read = read(fd, rx_buffer, sizeof(rx_buffer));

			is_looped_tmp = FALSE;

			/* If the number of bytes that are read equal the number of bytes sent and those
			 * those bytes were identical then we have a loop-back connection. */
			if (num_bytes_read == num_bytes_written)
			{
				is_looped_tmp = TRUE;
				for (i = 0; i < num_bytes_read; i++)
				{
					if (tx_buffer[i] != rx_buffer[i])
					{
						is_looped_tmp = FALSE;
						break;
					}
				}
			}

			is_looped = is_looped_tmp;
		}

		/* Save so we can send it back. */
		looped_device_paths[devices] = (is_looped) ? 1 : 0;

		/* Close the device. */
		close(fd);
	}
}

/**
* @brief Test termios read and write flushing functionality
*
* @par Detailed Description:
* The serial bus has its RX and TX wired together so it can do a loop-back of the data.
* Data is sent over the bus and read back using a read call.  We test flushing of the
* data on read, write and read/write.
*
* Test:
* 1) Open the serial device /dev/tty-[1-6]
* 2) Write data to the serial device
* 3) Do flushes
* 4) Read data using read call, check if data matches expected
* 5) Do steps 2-4 for other flush types
* 6) Loop 2-5 for TERMIOS_TEST_CYCLES times
* 7) Close serial device
* 8) Loop steps 1-8 for other serial devices
*
* @return
* - SUCCESS if test does expected actions.
* - Error otherwise
*/
int dspal_tester_termios_flush(int *looped_device_paths)
{
	/*
	 * Choose a large buffer size since we are going to have to test
	 * flush a buffer and we need time to do the flush after sending.
	 */
	const int BUFFER_SIZES = 250;

	int result = SUCCESS;
	int dev_path_index;
	int fd;
	int runs;
	int i;
	char tx_buffer[BUFFER_SIZES];
	char rx_buffer[BUFFER_SIZES];
	int num_bytes_written;
	int num_bytes_read;

	LOG_INFO("Beginning termios tcflush test");

	memset(tx_buffer, 0, BUFFER_SIZES);
	memset(rx_buffer, 0, BUFFER_SIZES);

	/* Fill the TX buffer with any data ('a' in this case) thats not NULL. */
	for (i = 0; i < (BUFFER_SIZES - 1); i++)
	{
		tx_buffer[i] = 'a';
	}

	/* Do each device path that we have a loop-back connection on. */
	for (dev_path_index = 0; dev_path_index < NUM_UART_DEVICE_ENABLED; dev_path_index++)
	{
		/* No way to test if there is no loop-back for that device path. */
		if (!looped_device_paths[dev_path_index])
		{
			continue;
		}

		/* Open the device path. */
		fd = open(serial_device_paths[dev_path_index], O_RDWR);

		/* Only proceed if the open succeeded. */
		if (fd < SUCCESS)
		{
			LOG_ERR("Open %s O_RDWR mode failed.", serial_device_paths[dev_path_index]);
			continue;
		}
		LOG_INFO("Open %s O_RDWR mode succeeded.", serial_device_paths[dev_path_index]);

		LOG_INFO("Doing tcflush TCOFLUSH tests.");
		for (runs = 0; runs < TERMIOS_TEST_CYCLES; runs++)
		{
			num_bytes_written = write(fd, (const char *)tx_buffer, strlen(tx_buffer));

			/* Make sure that the write went as planned. */
			if ((unsigned int) num_bytes_written != strlen(tx_buffer))
			{
				LOG_ERR("failed to write to %s", serial_device_paths[dev_path_index]);
				result = ERROR;
				break;
			}

			/*Test an Output Flush*/
			if (tcflush(fd, TCOFLUSH) != 0)
			{
				LOG_ERR("tcflush call failed.");
				result = ERROR;
				break;
			}

			//  Wait a period of time so that the data is written if it is still in the TX buffer.
			usleep(TERMIOS_SEND_DELAY_MSEC);

			/* Read the data */
			num_bytes_read = read(fd, rx_buffer, BUFFER_SIZES);

			/* If no flushing occurred then exit with an error*/
			if (num_bytes_read == num_bytes_written)
			{
				LOG_ERR("Data read does not match data sent.");
				result = ERROR;
				break;
			}
		}

		/* Only continue if we didn't fail yet. */
		if (result == SUCCESS)
		{
			LOG_INFO("Doing tcflush TCIFLUSH tests.");
			for (runs = 0; runs < TERMIOS_TEST_CYCLES; runs++)
			{

				num_bytes_written = write(fd, (const char *)tx_buffer, strlen(tx_buffer));

				/* Make sure that the write went as planned. */
				if ((unsigned int) num_bytes_written != strlen(tx_buffer))
				{
					LOG_ERR("failed to write to %s", serial_device_paths[dev_path_index]);
					result = ERROR;
					break;
				}

				//  Wait a period of time so that the data is written if it is still in the TX buffer.
				usleep(TERMIOS_SEND_DELAY_MSEC);

				/*Test an Input Flush*/
				if (tcflush(fd, TCIFLUSH) != 0)
				{
					LOG_ERR("tcflush call failed.");
					result = ERROR;
					break;
				}

				/* Read the data */
				num_bytes_read = read(fd, rx_buffer, BUFFER_SIZES);

				/* If no flushing occurred then exit with an error*/
				if (num_bytes_read != 0)
				{
					LOG_ERR("Data read does not match data sent.");
					result = ERROR;
					break;
				}
			}
		}

		if (result == SUCCESS)
		{
			LOG_INFO("Doing tcflush TCIOFLUSH tests.");
			for (runs = 0; runs < TERMIOS_TEST_CYCLES; runs++)
			{

				num_bytes_written = write(fd, (const char *)tx_buffer, strlen(tx_buffer));

				/* Make sure that the write went as planned. */
				if ((unsigned int) num_bytes_written != strlen(tx_buffer))
				{
					LOG_ERR("failed to write to %s", serial_device_paths[dev_path_index]);
					result = ERROR;
					break;
				}

				// Wait a period of time so that the data is written if it is still in the TX buffer.
				// Not all the data will be written yet after this sleep expires
				usleep(10 * 1000);

				/*Test an Input Flush*/
				if (tcflush(fd, TCOFLUSH) != 0)
				{
					LOG_ERR("tcflush call failed.");
					result = ERROR;
					break;
				}

				// Wait sometime for the TX buffer to transmit.
				usleep(TERMIOS_SEND_DELAY_MSEC);

				/*Test an Input Flush*/
				if (tcflush(fd, TCIFLUSH) != 0)
				{
					LOG_ERR("tcflush call failed.");
					result = ERROR;
					break;
				}


				/* Read the data */
				num_bytes_read = read(fd, rx_buffer, BUFFER_SIZES);

				/* If no flushing occurred then exit with an error*/
				if (num_bytes_read != 0)
				{
					LOG_ERR("Data read does not match data sent.");
					result = ERROR;
					break;
				}
			}
		}
		/* If we got here then the fd might still be active and we should close the device. */
		close(fd);

		/* No reason to continue if the test already failed. */
		if (result != SUCCESS)
		{
			break;
		}
	}

	LOG_INFO("termios tcflush test %s", result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

/**
* @brief Test termios drain
*
* @par Detailed Description:
* The serial bus has its RX and TX wired together so it can do a loop-back of the data.
* Data is sent over the bus and read back using a read call.  We test draining the TX
* buffer.
*
* Test:
* 1) Open the serial device /dev/tty-[1-6]
* 2) Get the termios struct.
* 3) Turn off OPOST.
* 4) Set the termios struct.
* 5) Write data to the serial device
* 6) Do Drain
* 7) Read data using read call, check if the amount of data read matches the amount of data written
* 8) Loop 5-7 for TERMIOS_TEST_CYCLES times
* 9) Close serial device
* 10) Loop steps 1-9 for other serial devices
*
* @return
* - SUCCESS if test does expected actions.
* - Error otherwise
*/
int dspal_tester_termios_drain(int *looped_device_paths)
{
	const int BUFFER_SIZES = 750;

	int result = SUCCESS;
	int dev_path_index;
	int fd;
	int runs;
	int i;
	char tx_buffer[BUFFER_SIZES];
	char rx_buffer[BUFFER_SIZES];
	int num_bytes_written;
	int num_bytes_read;
	struct termios t;

	LOG_INFO("Beginning termios tcdrain test");

	memset(tx_buffer, 0, sizeof(tx_buffer));
	memset(rx_buffer, 0, sizeof(rx_buffer));

	/* Fill the TX buffer with any data ('a' in this case) thats not NULL. */
	for (i = 0; i < (BUFFER_SIZES - 1); i++)
	{
		tx_buffer[i] = 'a';
	}

	for (dev_path_index = 0; dev_path_index < NUM_UART_DEVICE_ENABLED; dev_path_index++)
	{

		if (!looped_device_paths[dev_path_index])
		{
			continue;
		}

		/* Open the device path. */
		fd = open(serial_device_paths[dev_path_index], O_RDWR);

		/* Only proceed if the open succeeded. */
		if (fd < SUCCESS)
		{
			LOG_INFO("Open %s O_RDWR mode failed.", serial_device_paths[dev_path_index]);
			continue;
		}
		LOG_INFO("Open %s O_RDWR mode succeeded.", serial_device_paths[dev_path_index]);


		for (runs = 0; runs < TERMIOS_TEST_CYCLES; runs++)
		{
			num_bytes_written = write(fd, (const char *)tx_buffer, strlen(tx_buffer));

			/* Get the current configurations of the termios device. */
			if (tcgetattr(fd, &t) != 0)
			{
				LOG_INFO("tcgetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Disable Output Processing. */
			t.c_oflag &= (~OPOST);

			/* Apply the changes. */
			if (tcsetattr(fd, TCSANOW, &t) != 0)
			{
				LOG_INFO("tcsetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Make sure that the write went as planned. */
			if ((unsigned int) num_bytes_written != strlen(tx_buffer))
			{
				LOG_ERR("failed to write to %s", serial_device_paths[dev_path_index]);
				result = ERROR;
				break;
			}


			/* Test an output drain. */
			if (tcdrain(fd) != 0)
			{
				LOG_ERR("tcdrain call failed.");
				result = ERROR;
				break;
			}

			/* Read the data, the drain should have let the data transmit before getting here */
			num_bytes_read = read(fd, rx_buffer, sizeof(rx_buffer));

			/* If we have a loop-back connection and we dont see the correct amount of data, fail. */
			if (looped_device_paths[dev_path_index] && (num_bytes_read != num_bytes_written))
			{
				LOG_ERR("Data read does not match data sent.");
				result = ERROR;
				break;
			}
		}

		/* If we got here then the fd might still be active and we should close the device. */
		close(fd);

		/* No reason to continue if the test already failed. */
		if (result != SUCCESS)
		{
			break;
		}
	}

	LOG_INFO("termios tcdrain test %s", result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

/**
* @brief Test termios flow
*
* @par Detailed Description:
* The serial bus has its RX and TX wired together so it can do a loop-back of the data.
* Data is sent over the bus and read back using a read call.  We test the flow starting
* and stopping.
*
* Test:
* 1) Open the serial device /dev/tty-[1-6]
* 2) Get the termios struct.
* 3) Turn off OPOST.
* 4) Set the termios struct.
* 5) Send START and STOP characters
* 6) Read data using read call, check that data has START and STOP characters
* 7) Loop 5-6 for TERMIOS_TEST_CYCLES times
* 8) Suspend output
* 9) Write data to output
* 10) Read data using read call, check that no data was read
* 11) Restart output
* 12) Read data using read call, check that correct amount of data is read.
* 13) Loop 8-12 for TERMIOS_TEST_CYCLES times
* 14) Close serial device
* 15) Loop steps 1-14 for other serial devices
*
* @return
* - SUCCESS if test does expected actions.
* - Error otherwise
*/
int dspal_tester_termios_flow(int *looped_device_paths)
{
	const int BUFFER_SIZES = 25;

	int result = SUCCESS;
	int dev_path_index;
	int fd;
	int runs;
	int i;
	char tx_buffer[BUFFER_SIZES];
	char rx_buffer[BUFFER_SIZES];
	int num_bytes_written;
	int num_bytes_read;
	struct termios t;

	LOG_INFO("Beginning termios tcflow test");

	memset(tx_buffer, 0, sizeof(tx_buffer));
	memset(rx_buffer, 0, sizeof(rx_buffer));

	/* Fill the TX buffer with any data ('a' in this case) thats not NULL. */
	for (i = 0; i < (BUFFER_SIZES - 1); i++)
	{
		tx_buffer[i] = 'a';
	}

	/* Do each device path that we have a loop-back connection on. */
	for (dev_path_index = 0; dev_path_index < NUM_UART_DEVICE_ENABLED; dev_path_index++)
	{
		if (!looped_device_paths[dev_path_index])
		{
			/* If not looped-back then the test is kind of useless. */
			continue;
		}

		LOG_INFO("About to open %s", serial_device_paths[dev_path_index]);

		/* Open the device path. */
		fd = open(serial_device_paths[dev_path_index], O_RDWR);

		/* Only proceed if the open succeeded. */
		if (fd < SUCCESS)
		{
			LOG_INFO("Open %s O_RDWR mode failed.", serial_device_paths[dev_path_index]);
			continue;
		}
		LOG_INFO("Open %s O_RDWR mode succeeded.", serial_device_paths[dev_path_index]);


		/* Get the current configurations of the termios device. */
		if (tcgetattr(fd, &t) != 0)
		{
			LOG_INFO("tcgetattr call failed.")
			result =  ERROR;
			break;
		}

		/* Disable Output Processing. */
		t.c_oflag &= (~OPOST);

		/* Apply the changes. */
		if (tcsetattr(fd, TCSANOW, &t) != 0)
		{
			LOG_INFO("tcsetattr call failed.")
			result =  ERROR;
			break;
		}

		LOG_INFO("Testing tcflow TCIOFF and TCION arguments. ");
		for (runs = 0; runs < TERMIOS_TEST_CYCLES; runs++)
		{
			/* Send START character. */
			if (tcflow(fd, TCION) != 0)
			{
				LOG_INFO("tcflow with TCION parameter call failed. ");
				result = ERROR;
				break;
			}

			/* Send STOP character. */
			if (tcflow(fd, TCIOFF) != 0)
			{
				LOG_INFO("tcflow with TCIOFF parameter call failed. ");
				result = ERROR;
				break;
			}

			/* Wait for all the data to send. */
			usleep(TERMIOS_SEND_DELAY_MSEC);

			/* Read the data. */
			num_bytes_read = read(fd, rx_buffer, sizeof(rx_buffer));

			/* If the number of bytes read was not 2 then something went wrong, the START and STOP
			 * characters were sent incorrectly. */
			if (num_bytes_read != 2)
			{
				LOG_INFO("Bytes read did not match START and STOP characters. ");
				result = ERROR;
				break;
			}

			/* Make sure the characters are actually correct. */
			if ((rx_buffer[0] != CSTART) || (rx_buffer[1] != CSTOP))
			{
				LOG_INFO("Bytes read did not match START and STOP characters. ");
				result = ERROR;
				break;
			}
		}

		/* Only continue the test if the we haven't failed yet. */
		if (result == SUCCESS)
		{
			LOG_INFO("Testing tcflow TCOON and TCOOFF arguments. ");
			for (runs = 0; runs < TERMIOS_TEST_CYCLES; runs++)
			{
				/* Restart the output call.  The output was never suspended yet but this call
				 * should not fail even though the output is not suspended.
				 */
				if (tcflow(fd, TCOON) != 0)
				{
					LOG_INFO("tcflow with TCOON parameter call failed. ");
					result = ERROR;
					break;
				}

				/* Suspend the output. */
				if (tcflow(fd, TCOOFF) != 0)
				{
					LOG_INFO("tcflow with TCOOFF parameter call failed. ");
					result = ERROR;
					break;
				}

				/* Suspend the output again, This call should have no effect but it should not return an
				 * error code. */
				if (tcflow(fd, TCOOFF) != 0)
				{
					LOG_INFO("tcflow with TCOOFF parameter call failed. ");
					result = ERROR;
					break;
				}

				/* Write data out but it will not actually send the data since output is suspended, */
				num_bytes_written = write(fd, (const char *)tx_buffer, strlen(tx_buffer));

				/* Make sure that the write went as planned. */
				if ((unsigned int) num_bytes_written != strlen(tx_buffer))
				{
					LOG_ERR("failed to write to %s", serial_device_paths[dev_path_index]);
					result = ERROR;
					break;
				}

				/* wait for all the data to send. */
				usleep(TERMIOS_SEND_DELAY_MSEC);

				/* Read the data. */
				num_bytes_read = read(fd, rx_buffer, sizeof(rx_buffer));

				/* We should not have any data in the buffer since we suspended the output. */
				if (num_bytes_read != 0)
				{
					LOG_INFO("Data transmitted even when output was suspended. ");
					result = ERROR;
					break;
				}

				/* Restart the output. */
				if (tcflow(fd, TCOON) != 0)
				{
					LOG_INFO("tcflow with TCOON parameter call failed. ");
					result = ERROR;
					break;
				}

				/* Wait for all the data to send. */
				usleep(TERMIOS_SEND_DELAY_MSEC);

				/* Read the data. */
				num_bytes_read = read(fd, rx_buffer, sizeof(rx_buffer));

				/* We should have the data that was in the buffer. */
				if (num_bytes_read != num_bytes_written)
				{
					LOG_INFO("Data was not transmitted when the output is restarted. ");
					result = ERROR;
					break;
				}
			}
		}

		/* If we got here then the fd might still be active and we should close the device. */
		close(fd);

		/* No reason to continue if the test already failed. */
		if (result != SUCCESS)
		{
			break;
		}
	}

	LOG_INFO("termios tcflow %s", result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

/**
* @brief Test termios send break.
*
* @par Detailed Description:
* This test tests the serial break functionality.  If calling the break command fails.
*
* Test:
* 1) Open the serial device /dev/tty-[1-6]
* 2) Send BREAK.
* 3) Loop 2-3 for TERMIOS_TEST_CYCLES times
* 4) Close serial device
* 5) Loop steps 1-4 for other serial devices
*
* @return
* - SUCCESS if test does expected actions.
* - Error otherwise
*/
int dspal_tester_termios_tcsendbreak()
{
	int result = SUCCESS;
	int dev_path_index;
	int fd;
	int runs;

	LOG_INFO("Beginning termios tcsendbreak test");

	/* Do each device path. */
	for (dev_path_index = 0; dev_path_index < NUM_UART_DEVICE_ENABLED; dev_path_index++)
	{
		LOG_INFO("About to open %s", serial_device_paths[dev_path_index]);

		/* Open the device path. */
		fd = open(serial_device_paths[dev_path_index], O_RDWR);

		/* Only proceed if the open succeeded. */
		if (fd < SUCCESS)
		{
			LOG_INFO("Open %s O_RDWR mode failed.", serial_device_paths[dev_path_index]);
			continue;
		}
		LOG_INFO("Open %s O_RDWR mode succeeded.", serial_device_paths[dev_path_index]);

		for (runs = 0; runs < TERMIOS_TEST_CYCLES; runs++)
		{
			/* Send a break with a default parameter of 0msec*/
			if (tcsendbreak(fd, 0) != 0)
			{
				LOG_INFO("tcsendbreak call failed. ");
				result = ERROR;
				break;
			}

			/* Test send break with a parameter of 100 msec */
			if (tcsendbreak(fd, 100) != 0)
			{
				LOG_INFO("tcsendbreak call failed. ");
				result = ERROR;
				break;
			}
		}

		/* If we got here then the fd might still be active and we should close the device. */
		close(fd);

		/* No reason to continue if the test already failed. */
		if (result != SUCCESS)
		{
			break;
		}
	}

	LOG_INFO("termios tcsendbreak %s", result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

/**
* @brief Test termios helper functions.
*
* @par Detailed Description:
* This test tests the helper functions that termios exposes.
* This includes cfmakeraw, cfsetispeed, cfsetospeed, cfsetspeed,
* cfgetispeed, cfgetospeed, cfgetspeed.
*
* Test:
* 1) Test cfmakeraw.
* 2) Test cfsetispeed.
* 3) Test cfsetospeed.
* 4) Test cfgetispeed.
* 5) Test cfgetospeed.
* 6) Test cfsetspeed.
* 7) Test cfgetispeed.
* 8) Test cfgetospeed.
*
* @return
* - SUCCESS if test does expected actions.
* - Error otherwise
*/
int dspal_tester_termios_helper_functions()
{
	int result = SUCCESS;
	struct termios t;
	tcflag_t iflag_reference;
	tcflag_t oflag_reference;
	tcflag_t lflag_reference;
	tcflag_t cflag_reference;
	speed_t baud_rate;

	t.c_iflag = TTYDEF_IFLAG;
	t.c_oflag = TTYDEF_OFLAG;
	t.c_lflag = TTYDEF_LFLAG;
	t.c_cflag = TTYDEF_CFLAG;

	/* Setup references so we can check later*/
	iflag_reference = TTYDEF_IFLAG;
	oflag_reference = TTYDEF_OFLAG;
	lflag_reference = TTYDEF_LFLAG;
	cflag_reference = TTYDEF_CFLAG;
	iflag_reference &= ~(IMAXBEL | IXOFF | INPCK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IGNPAR);
	iflag_reference |= IGNBRK;
	oflag_reference &= ~OPOST;
	lflag_reference &= ~(ECHO | ECHOE | ECHOK | ECHONL | ICANON | ISIG | IEXTEN | NOFLSH | TOSTOP | PENDIN);
	cflag_reference &= ~(CSIZE | PARENB);
	cflag_reference |= CS8 | CREAD;

	LOG_INFO("Beginning termios helper functions test");


	/* Make the termios struct represent a raw serial connection */
	LOG_DEBUG("Testing cfmakeraw");
	cfmakeraw(&t);

	/* Wrong results from function call */
	if (t.c_iflag != iflag_reference     ||
	        t.c_oflag != oflag_reference ||
	        t.c_lflag != lflag_reference ||
	        t.c_cflag != cflag_reference ||
	        t.c_cc[VMIN] != 1            ||
	        t.c_cc[VTIME] != 0)
	{
		LOG_INFO("cfmakeraw result was not correct. ")
		result =  ERROR;
	}


	LOG_DEBUG("Testing cfsetispeed.");
	if (cfsetispeed(&t, B115200) != 0)
	{
		LOG_INFO("cfsetispeed call failed.");
		result = ERROR;
	}

	LOG_DEBUG("Testing cfsetospeed.");
	if (cfsetospeed(&t, B115200) != 0)
	{
		LOG_INFO("cfsetispeed call failed.");
		result = ERROR;
	}

	LOG_DEBUG("Testing cfgetispeed.");
	baud_rate = cfgetispeed(&t);
	if (baud_rate != B115200)
	{
		LOG_INFO("Input baud rate (speed) not as expected.");
		result = ERROR;
	}

	LOG_DEBUG("Testing cfgetospeed.");
	baud_rate = cfgetospeed(&t);
	if (baud_rate != B115200)
	{
		LOG_INFO("Output baud rate (speed) not as expected.");
		result = ERROR;
	}

	LOG_DEBUG("Testing cfsetspeed.");
	if (cfsetspeed(&t, B115200) != 0)
	{
		LOG_INFO("cfsetispeed call failed.");
		result = ERROR;
	}

	baud_rate = cfgetispeed(&t);
	if (baud_rate != B115200)
	{
		LOG_INFO("Input baud rate (speed) not as expected.");
		result = ERROR;
	}

	baud_rate = cfgetospeed(&t);
	if (baud_rate != B115200)
	{
		LOG_INFO("Output baud rate (speed) not as expected.");
		result = ERROR;
	}

	LOG_INFO("termios helper functions %s", result == SUCCESS ? "PASSED" : "FAILED");
	return result;
}

/**
* @brief Test termios tcgetattr.
*
* @par Detailed Description:
* This test tests that tcgetattr returns the correct termios struct.
* Returned struct must match what was expected.
*
* Test:
* 1) Open the serial device /dev/tty-[1-6]
* 2) Get the termios struct.
* 3) Loop 2-3 for TERMIOS_TEST_CYCLES times
* 4) Close serial device
* 5) Loop steps 1-4 for other serial devices
*
* @return
* - SUCCESS if test does expected actions.
* - Error otherwise
*/
int dspal_tester_termios_get_struct()
{
	int result = SUCCESS;
	int dev_path_index;
	int fd;
	int runs;
	struct termios t;

	LOG_INFO("Beginning termios tcgetattr test");
	for (dev_path_index = 0; dev_path_index < NUM_UART_DEVICE_ENABLED; dev_path_index++)
	{
		/* Open the device path. */
		fd = open(serial_device_paths[dev_path_index], O_RDWR);

		/* Only proceed if the open succeeded. */
		if (fd < SUCCESS)
		{
			LOG_INFO("Open %s O_RDWR mode failed.", serial_device_paths[dev_path_index]);
			continue;
		}
		LOG_INFO("Open %s O_RDWR mode succeeded.", serial_device_paths[dev_path_index]);

		for (runs = 0; runs < TERMIOS_TEST_CYCLES; runs++)
		{
			/* Make sure the struct does not have anything in it*/
			memset(&t, 0, sizeof(struct termios));

			if (tcgetattr(fd, &t) != 0)
			{
				LOG_INFO("tcgetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Wrong results from function call */
			if (t.c_iflag != TTYDEF_IFLAG     ||
			        t.c_oflag != TTYDEF_OFLAG ||
			        t.c_lflag != TTYDEF_LFLAG ||
			        t.c_cflag != TTYDEF_CFLAG)
			{
				LOG_INFO("tcgetattr result was not correct.")
				result =  ERROR;
				break;
			}
		}

		/* If we got here then the fd might still be active and we should close the device. */
		close(fd);

		/* No reason to continue if the test already failed. */
		if (result != SUCCESS)
		{
			break;
		}
	}

	LOG_INFO("termios tcgetattr %s", result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

/**
* @brief Test termios setting CREAD.
*
* @par Detailed Description:
* This test tests that the CREAD flag can be set and unset.
* Proper functionality (based on the termios specs) buts be
* observed in each case.  Needs to have loopback wire.
*
* Test:
* 1) Open the serial device /dev/tty-[1-6]
* 2) Get the termios struct.
* 3) Unset the CREAD flag.
* 4) Set the termios struct.
* 5) Write data to the serial port.
* 6) Read data back and check is read data is correct.
* 7) Get the termios struct.
* 8) Set the CREAD flag.
* 9) Set the termios struct.
* 10) Write data to the serial port.
* 11) Read data back and check is read data is correct.
* 12) Loop 2-11 for TERMIOS_TEST_CYCLES times
* 13) Close the serial device.
* 14) Loop steps 1-14 for each serial device path that has a loopback wire.
*
* @return
* - SUCCESS if test does expected actions.
* - Error otherwise
*/
int dspal_tester_termios_set_struct_cread(int *looped_device_paths)
{
	const int BUFFER_SIZES = 25;

	int result = SUCCESS;
	int dev_path_index;
	int fd;
	struct termios t;
	int i;
	int runs;
	char tx_buffer[BUFFER_SIZES];
	char rx_buffer[BUFFER_SIZES];
	int num_bytes_written;
	int num_bytes_read;

	LOG_INFO("Beginning tcsetattr CREAD test");

	memset(tx_buffer, 0, sizeof(tx_buffer));
	memset(rx_buffer, 0, sizeof(rx_buffer));

	/* Fill the TX buffer with any data ('a' in this case) thats not NULL. */
	for (i = 0; i < (BUFFER_SIZES - 1); i++)
	{
		/* Fill in with random letters. */
		tx_buffer[i] = 'a' + (rand() % 26);
	}

	/* Do each device path. */
	for (dev_path_index = 0; dev_path_index < NUM_UART_DEVICE_ENABLED; dev_path_index++)
	{
		if (!looped_device_paths[dev_path_index])
		{
			/* If is not looped-back then the test is kind of useless. */
			continue;
		}

		/* Open the device path. */
		fd = open(serial_device_paths[dev_path_index], O_RDWR);

		/* Only proceed if the open succeeded. */
		if (fd < SUCCESS)
		{
			LOG_INFO("Open %s O_RDWR mode failed.", serial_device_paths[dev_path_index]);
			continue;
		}
		LOG_INFO("Open %s O_RDWR mode succeeded.", serial_device_paths[dev_path_index]);

		for (runs = 0; runs < TERMIOS_TEST_CYCLES; runs++)
		{
			LOG_INFO("Disabling CREAD Flag Test.")

			/* Make sure the struct does not have anything in it*/
			memset(&t, 0, sizeof(struct termios));

			/* Get the current configurations of the termios device. */
			if (tcgetattr(fd, &t) != 0)
			{
				LOG_INFO("tcgetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Disable Output Processing. */
			t.c_oflag &= (~OPOST);

			/* Disable the CREAD flag. */
			t.c_cflag &= (~CREAD);

			/* Apply the changes. */
			if (tcsetattr(fd, TCSANOW, &t) != 0)
			{
				LOG_INFO("tcsetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Write */
			num_bytes_written = write(fd, (const char *)tx_buffer, strlen(tx_buffer));

			/* Make sure that the write went as planned. */
			if ((unsigned int) num_bytes_written != strlen(tx_buffer))
			{
				LOG_ERR("failed to write to %s", serial_device_paths[dev_path_index]);
				result = ERROR;
				break;
			}

			usleep(TERMIOS_SEND_DELAY_MSEC);

			/* Read the data, the drain should have let the data transmit before getting here */
			num_bytes_read = read(fd, rx_buffer, sizeof(rx_buffer));

			/* If we have a loop-back connection and we dont see the correct amount of data, fail. */
			if ((num_bytes_read != 0))
			{
				LOG_ERR("Data read length is not zero, should be zero.");
				result = ERROR;
				break;
			}

			LOG_INFO("Enabling CREAD Flag Test.")

			/* Make sure the struct does not have anything in it*/
			memset(&t, 0, sizeof(struct termios));

			/* Get the current configurations of the termios device. */
			if (tcgetattr(fd, &t) != 0)
			{
				LOG_INFO("tcgetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Disable Output Processing. */
			t.c_oflag &= (~OPOST);

			/* Enable the CREAD flag. */
			t.c_cflag |= (CREAD);

			/* Apply the changes. */
			if (tcsetattr(fd, TCSANOW, &t) != 0)
			{
				LOG_INFO("tcsetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Write */
			num_bytes_written = write(fd, (const char *)tx_buffer, strlen(tx_buffer));

			/* Make sure that the write went as planned. */
			if ((unsigned int) num_bytes_written != strlen(tx_buffer))
			{
				LOG_ERR("failed to write to %s", serial_device_paths[dev_path_index]);
				result = ERROR;
				break;
			}

			usleep(TERMIOS_SEND_DELAY_MSEC);

			/* Read the data, the drain should have let the data transmit before getting here */
			num_bytes_read = read(fd, rx_buffer, sizeof(rx_buffer));

			/* If we have a loop-back connection and we dont see the correct amount of data, fail. */
			if ((num_bytes_read != num_bytes_written))
			{
				LOG_ERR("Data read length does not match data sent length.");
				result = ERROR;
				break;
			}
		}


		/* If we got here then the fd might still be active and we should close the device. */
		close(fd);

		/* No reason to continue if the test already failed. */
		if (result != SUCCESS)
		{
			break;
		}
	}

	LOG_INFO("termios tcsetattr CREAD %s", result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

/**
* @brief Test termios setting INLCR.
*
* @par Detailed Description:
* This test tests that the INLCR flag can be set.
* Proper functionality (based on the termios specs) buts be
* observed when set.  Needs to have loopback wire.
*
* Test:
* 1) Open the serial device /dev/tty-[1-6]
* 2) Get the termios struct.
* 3) Set the INLCR flag.
* 4) Set the termios struct.
* 5) Write data to the serial port.
* 6) Read data back and check is read data is correct.
* 7) Loop 2-6 for TERMIOS_TEST_CYCLES times
* 8) Close the serial device.
* 9) Loop steps 1-8 for each serial device path that has a loopback wire.
*
* @return
* - SUCCESS if test does expected actions.
* - Error otherwise
*/
int dspal_tester_termios_set_struct_inlcr(int *looped_device_paths)
{
	const int BUFFER_SIZES = 25;

	int result = SUCCESS;
	int dev_path_index;
	int fd;
	struct termios t;
	int i;
	int runs;
	char tx_buffer[BUFFER_SIZES];
	char rx_buffer[BUFFER_SIZES];
	int num_bytes_written;
	int num_bytes_read;

	LOG_INFO("Beginning tcsetattr INLCR test");

	memset(tx_buffer, 0, sizeof(tx_buffer));
	memset(rx_buffer, 0, sizeof(rx_buffer));

	/* Fill the TX buffer with any data ('a' in this case) thats not NULL. */
	for (i = 0; i < (BUFFER_SIZES - 1); i++)
	{
		/* Fill in with New lines. */
		tx_buffer[i] = '\n';
	}

	/* Do each device path. */
	for (dev_path_index = 0; dev_path_index < NUM_UART_DEVICE_ENABLED; dev_path_index++)
	{
		if (!looped_device_paths[dev_path_index])
		{
			/* If is not looped-back then the test is kind of useless. */
			continue;
		}

		/* Open the device path. */
		fd = open(serial_device_paths[dev_path_index], O_RDWR);

		/* Only proceed if the open succeeded. */
		if (fd < SUCCESS)
		{
			LOG_INFO("Open %s O_RDWR mode failed.", serial_device_paths[dev_path_index]);
			continue;
		}
		LOG_INFO("Open %s O_RDWR mode succeeded.", serial_device_paths[dev_path_index]);

		for (runs = 0; runs < TERMIOS_TEST_CYCLES; runs++)
		{
			/* Make sure the struct does not have anything in it*/
			memset(&t, 0, sizeof(struct termios));

			/* Get the current configurations of the termios device. */
			if (tcgetattr(fd, &t) != 0)
			{
				LOG_INFO("tcgetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Disable Output Processing. */
			t.c_oflag &= (~OPOST);

			/* Set the INLCR flag. */
			t.c_iflag |= INLCR;

			/* Apply the changes. */
			if (tcsetattr(fd, TCSANOW, &t) != 0)
			{
				LOG_INFO("tcsetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Write */
			num_bytes_written = write(fd, (const char *)tx_buffer, strlen(tx_buffer));

			/* Make sure that the write went as planned. */
			if ((unsigned int) num_bytes_written != strlen(tx_buffer))
			{
				LOG_ERR("failed to write to %s", serial_device_paths[dev_path_index]);
				result = ERROR;
				break;
			}

			/* Wait for data to transmit. */
			usleep(TERMIOS_SEND_DELAY_MSEC);

			/* Read the data, the drain should have let the data transmit before getting here */
			num_bytes_read = read(fd, rx_buffer, sizeof(rx_buffer));

			/* If we have a loop-back connection and we dont see the correct amount of data, fail. */
			if ((num_bytes_read != num_bytes_written))
			{
				LOG_ERR("Data read length does not match data sent length.");
				result = ERROR;
				break;
			}
			else
			{
				/* Check that the output was correct. */
				for (i = 0; i < num_bytes_read; i++)
				{
					if (rx_buffer[i] != '\r')
					{
						LOG_ERR("NL's were not changed to CR's.");
						result = ERROR;
						break;
					}
				}

				/* No need to continue if there is a failure. */
				if (result != SUCCESS)
				{
					break;
				}
			}
		}

		/* If we got here then the fd might still be active and we should close the device. */
		close(fd);

		/* No reason to continue if the test already failed. */
		if (result != SUCCESS)
		{
			break;
		}
	}

	LOG_INFO("termios tcsetattr INLCR %s", result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

/**
* @brief Test termios setting ICRNL.
*
* @par Detailed Description:
* This test tests that the ICRNL flag can be set.
* Proper functionality (based on the termios specs) buts be
* observed when set.  Needs to have loopback wire.
*
* Test:
* 1) Open the serial device /dev/tty-[1-6]
* 2) Get the termios struct.
* 3) Set the ICRNL flag.
* 4) Set the termios struct.
* 5) Write data to the serial port.
* 6) Read data back and check is read data is correct.
* 7) Loop 2-6 for TERMIOS_TEST_CYCLES times
* 8) Close the serial device.
* 9) Loop steps 1-8 for each serial device path that has a loopback wire.
*
* @return
* - SUCCESS if test does expected actions.
* - Error otherwise
*/
int dspal_tester_termios_set_struct_icrnl(int *looped_device_paths)
{
	const int BUFFER_SIZES = 25;

	int result = SUCCESS;
	int dev_path_index;
	int fd;
	struct termios t;
	int i;
	int runs;
	char tx_buffer[BUFFER_SIZES];
	char rx_buffer[BUFFER_SIZES];
	int num_bytes_written;
	int num_bytes_read;

	LOG_INFO("Beginning tcsetattr ICRNL test");

	memset(tx_buffer, 0, sizeof(tx_buffer));
	memset(rx_buffer, 0, sizeof(rx_buffer));

	/* Fill the TX buffer with any data ('a' in this case) thats not NULL. */
	for (i = 0; i < (BUFFER_SIZES - 1); i++)
	{
		/* Fill in with carriage returns. */
		tx_buffer[i] = '\r';
	}

	/* Do each device path. */
	for (dev_path_index = 0; dev_path_index < NUM_UART_DEVICE_ENABLED; dev_path_index++)
	{
		if (!looped_device_paths[dev_path_index])
		{
			/* If is not looped-back then the test is kind of useless. */
			continue;
		}

		/* Open the device path. */
		fd = open(serial_device_paths[dev_path_index], O_RDWR);

		/* Only proceed if the open succeeded. */
		if (fd < SUCCESS)
		{
			LOG_INFO("Open %s O_RDWR mode failed.", serial_device_paths[dev_path_index]);
			continue;
		}
		LOG_INFO("Open %s O_RDWR mode succeeded.", serial_device_paths[dev_path_index]);

		for (runs = 0; runs < TERMIOS_TEST_CYCLES; runs++)
		{
			/* Make sure the struct does not have anything in it*/
			memset(&t, 0, sizeof(struct termios));

			/* Get the current configurations of the termios device. */
			if (tcgetattr(fd, &t) != 0)
			{
				LOG_INFO("tcgetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Disable Output Processing. */
			t.c_oflag &= (~OPOST);

			/* Set the ICRNL flag. */
			t.c_iflag |= ICRNL;

			/* Apply the changes. */
			if (tcsetattr(fd, TCSANOW, &t) != 0)
			{
				LOG_INFO("tcsetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Write */
			num_bytes_written = write(fd, (const char *)tx_buffer, strlen(tx_buffer));

			/* Make sure that the write went as planned. */
			if ((unsigned int) num_bytes_written != strlen(tx_buffer))
			{
				LOG_ERR("failed to write to %s", serial_device_paths[dev_path_index]);
				result = ERROR;
				break;
			}

			usleep(TERMIOS_SEND_DELAY_MSEC);

			/* Read the data, the drain should have let the data transmit before getting here */
			num_bytes_read = read(fd, rx_buffer, sizeof(rx_buffer));

			/* If we have a loop-back connection and we dont see the correct amount of data, fail. */
			if ((num_bytes_read != num_bytes_written))
			{
				LOG_ERR("Data read length does not match data sent length.");
				result = ERROR;
				break;
			}
			else
			{
				/* Check that the output is correct. */
				for (i = 0; i < num_bytes_read; i++)
				{
					if (rx_buffer[i] != '\n')
					{
						LOG_ERR("CR's were not changed to NL's.");
						result = ERROR;
						break;
					}
				}

				/* No need to continue if there is a failure. */
				if (result != SUCCESS)
				{
					break;
				}
			}
		}

		/* If we got here then the fd might still be active and we should close the device. */
		close(fd);

		/* No reason to continue if the test already failed. */
		if (result != SUCCESS)
		{
			break;
		}
	}

	LOG_INFO("termios tcsetattr ICRNL %s", result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

/**
* @brief Test termios setting IGNCR.
*
* @par Detailed Description:
* This test tests that the IGNCR flag can be set.
* Proper functionality (based on the termios specs) buts be
* observed when set.  Needs to have loopback wire.
*
* Test:
* 1) Open the serial device /dev/tty-[1-6]
* 2) Get the termios struct.
* 3) Set the IGNCR flag.
* 4) Set the termios struct.
* 5) Write data to the serial port.
* 6) Read data back and check is read data is correct.
* 7) Loop 2-6 for TERMIOS_TEST_CYCLES times
* 8) Close the serial device.
* 9) Loop steps 1-8 for each serial device path that has a loopback wire.
*
* @return
* - SUCCESS if test does expected actions.
* - Error otherwise
*/
int dspal_tester_termios_set_struct_igncr(int *looped_device_paths)
{
	const int BUFFER_SIZES = 25;

	int result = SUCCESS;
	int dev_path_index;
	int fd;
	struct termios t;
	int i;
	int runs;
	char tx_buffer[BUFFER_SIZES];
	char rx_buffer[BUFFER_SIZES];
	int num_bytes_written;
	int num_bytes_read;

	LOG_INFO("Beginning tcsetattr IGNCR test");

	memset(tx_buffer, 0, sizeof(tx_buffer));
	memset(rx_buffer, 0, sizeof(rx_buffer));

	/* Fill the TX buffer with any data ('a' in this case) thats not NULL. */
	for (i = 0; i < (BUFFER_SIZES - 1); i++)
	{
		/* Fill in with carriage returns. */
		tx_buffer[i] = '\r';
	}

	/* Do each device path. */
	for (dev_path_index = 0; dev_path_index < NUM_UART_DEVICE_ENABLED; dev_path_index++)
	{
		if (!looped_device_paths[dev_path_index])
		{
			/* If is not looped-back then the test is kind of useless. */
			continue;
		}

		/* Open the device path. */
		fd = open(serial_device_paths[dev_path_index], O_RDWR);

		/* Only proceed if the open succeeded. */
		if (fd < SUCCESS)
		{
			LOG_INFO("Open %s O_RDWR mode failed.", serial_device_paths[dev_path_index]);
			continue;
		}
		LOG_INFO("Open %s O_RDWR mode succeeded.", serial_device_paths[dev_path_index]);

		for (runs = 0; runs < TERMIOS_TEST_CYCLES; runs++)
		{
			/* Make sure the struct does not have anything in it*/
			memset(&t, 0, sizeof(struct termios));

			/* Get the current configurations of the termios device. */
			if (tcgetattr(fd, &t) != 0)
			{
				LOG_INFO("tcgetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Disable Output Processing. */
			t.c_oflag &= (~OPOST);

			/* Set the IGNCR flag. */
			t.c_iflag |= IGNCR;

			/* Apply the changes. */
			if (tcsetattr(fd, TCSANOW, &t) != 0)
			{
				LOG_INFO("tcsetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Write */
			num_bytes_written = write(fd, (const char *)tx_buffer, strlen(tx_buffer));

			/* Make sure that the write went as planned. */
			if ((unsigned int) num_bytes_written != strlen(tx_buffer))
			{
				LOG_ERR("failed to write to %s", serial_device_paths[dev_path_index]);
				result = ERROR;
				break;
			}

			usleep(TERMIOS_SEND_DELAY_MSEC);

			/* Read the data, the drain should have let the data transmit before getting here. */
			num_bytes_read = read(fd, rx_buffer, sizeof(rx_buffer));

			/* We should not get anything in this test since we are ignoring carriage returns. */
			if ((num_bytes_read != 0))
			{
				LOG_ERR("Data read length does not match data sent length.");
				result = ERROR;
				break;
			}
		}

		/* If we got here then the fd might still be active and we should close the device. */
		close(fd);

		/* No reason to continue if the test already failed. */
		if (result != SUCCESS)
		{
			break;
		}
	}

	LOG_INFO("termios tcsetattr IGNCR %s", result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

/**
* @brief Test termios setting ISTRIP.
*
* @par Detailed Description:
* This test tests that the ISTRIP flag can be set.
* Proper functionality (based on the termios specs) buts be
* observed when set.  Needs to have loopback wire.
*
* Test:
* 1) Open the serial device /dev/tty-[1-6]
* 2) Get the termios struct.
* 3) Set the ISTRIP flag.
* 4) Set the termios struct.
* 5) Write data to the serial port.
* 6) Read data back and check is read data is correct.
* 7) Loop 2-6 for TERMIOS_TEST_CYCLES times
* 8) Close the serial device.
* 9) Loop steps 1-8 for each serial device path that has a loopback wire.
*
* @return
* - SUCCESS if test does expected actions.
* - Error otherwise
*/
int dspal_tester_termios_set_struct_istrip(int *looped_device_paths)
{
	const int BUFFER_SIZES = 25;

	int result = SUCCESS;
	int dev_path_index;
	int fd;
	struct termios t;
	int i;
	int runs;
	char tx_buffer[BUFFER_SIZES];
	char rx_buffer[BUFFER_SIZES];
	int num_bytes_written;
	int num_bytes_read;

	LOG_INFO("Beginning tcsetattr ISTRIP test");

	memset(tx_buffer, 0, sizeof(tx_buffer));
	memset(rx_buffer, 0, sizeof(rx_buffer));

	/* Fill the TX buffer with any data ('a' in this case) thats not NULL. */
	for (i = 0; i < (BUFFER_SIZES - 1); i++)
	{
		/* Fill in with carriage returns. */
		tx_buffer[i] = 132;
	}

	/* Do each device path. */
	for (dev_path_index = 0; dev_path_index < NUM_UART_DEVICE_ENABLED; dev_path_index++)
	{
		if (!looped_device_paths[dev_path_index])
		{
			/* If is not looped-back then the test is kind of useless. */
			continue;
		}

		/* Open the device path. */
		fd = open(serial_device_paths[dev_path_index], O_RDWR);

		/* Only proceed if the open succeeded. */
		if (fd < SUCCESS)
		{
			LOG_INFO("Open %s O_RDWR mode failed.", serial_device_paths[dev_path_index]);
			continue;
		}
		LOG_INFO("Open %s O_RDWR mode succeeded.", serial_device_paths[dev_path_index]);

		for (runs = 0; runs < TERMIOS_TEST_CYCLES; runs++)
		{
			/* Make sure the struct does not have anything in it*/
			memset(&t, 0, sizeof(struct termios));

			/* Get the current configurations of the termios device. */
			if (tcgetattr(fd, &t) != 0)
			{
				LOG_INFO("tcgetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Disable Output Processing. */
			t.c_oflag &= (~OPOST);

			/* Set the ISTRIP flag. */
			t.c_iflag |= ISTRIP;

			/* Apply the changes. */
			if (tcsetattr(fd, TCSANOW, &t) != 0)
			{
				LOG_INFO("tcsetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Write */
			num_bytes_written = write(fd, (const char *)tx_buffer, strlen(tx_buffer));

			/* Make sure that the write went as planned. */
			if ((unsigned int) num_bytes_written != strlen(tx_buffer))
			{
				LOG_ERR("failed to write to %s", serial_device_paths[dev_path_index]);
				result = ERROR;
				break;
			}

			usleep(TERMIOS_SEND_DELAY_MSEC);

			/* Read the data, the drain should have let the data transmit before getting here. */
			num_bytes_read = read(fd, rx_buffer, sizeof(rx_buffer));

			/* We should not get anything in this test since we are ignoring carriage returns. */
			if ((num_bytes_read != num_bytes_written))
			{
				LOG_ERR("Data read length does not match data sent length.");
				result = ERROR;
				break;
			}
			else
			{
				/* Check that the output is correct. */
				for (i = 0; i < num_bytes_read; i++)
				{
					if (rx_buffer[i] != 4)
					{
						LOG_ERR("Data MSB not stripped.");
						result = ERROR;
						break;
					}
				}

				/* No need to continue if there is a failure. */
				if (result != SUCCESS)
				{
					break;
				}
			}
		}

		/* If we got here then the fd might still be active and we should close the device. */
		close(fd);

		/* No reason to continue if the test already failed. */
		if (result != SUCCESS)
		{
			break;
		}
	}

	LOG_INFO("termios tcsetattr ISTRIP %s", result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

/**
* @brief Test termios setting IXON.
*
* @par Detailed Description:
* This test tests that the IXON flag can be set.
* Proper functionality (based on the termios specs) buts be
* observed when set.  Needs to have loopback wire.
*
* Test:
* 1) Open the serial device /dev/tty-[1-6]
* 2) Get the termios struct.
* 3) Set the IXON flag.
* 4) Set the termios struct.
* 5) Write data to the serial port.
* 6) Read data back and check is read data is correct (No Change in data).
* 7) Loop 2-6 for TERMIOS_TEST_CYCLES times
* 8) Close the serial device.
* 9) Loop steps 1-8 for each serial device path that has a loopback wire.
*
* @return
* - SUCCESS if test does expected actions.
* - Error otherwise
*/
int dspal_tester_termios_set_struct_ixon(int *looped_device_paths)
{
	const int BUFFER_SIZES = 250;

	int result = SUCCESS;
	int dev_path_index;
	int fd;
	struct termios t;
	int i;
	int runs;
	char tx_buffer[BUFFER_SIZES];
	char rx_buffer[BUFFER_SIZES * 2];
	int num_bytes_written;
	int num_bytes_read;

	LOG_INFO("Beginning tcsetattr IXON test");

	memset(tx_buffer, 0, sizeof(tx_buffer));
	memset(rx_buffer, 0, sizeof(rx_buffer));

	/* Fill the TX buffer with any data thats not NULL. */
	for (i = 0; i < (BUFFER_SIZES - 1); i++)
	{
		/* Fill in with carriage returns. */
		tx_buffer[i] = 'a' + (rand() % 3);
	}

	/* Do each device path. */
	for (dev_path_index = 0; dev_path_index < NUM_UART_DEVICE_ENABLED; dev_path_index++)
	{
		if (!looped_device_paths[dev_path_index])
		{
			/* If is not looped-back then the test is kind of useless. */
			continue;
		}

		/* Open the device path. */
		fd = open(serial_device_paths[dev_path_index], O_RDWR);

		/* Only proceed if the open succeeded. */
		if (fd < SUCCESS)
		{
			LOG_INFO("Open %s O_RDWR mode failed.", serial_device_paths[dev_path_index]);
			continue;
		}
		LOG_INFO("Open %s O_RDWR mode succeeded.", serial_device_paths[dev_path_index]);

		for (runs = 0; runs < TERMIOS_TEST_CYCLES; runs++)
		{
			/* Make sure the struct does not have anything in it*/
			memset(&t, 0, sizeof(struct termios));

			/* Get the current configurations of the termios device. */
			if (tcgetattr(fd, &t) != 0)
			{
				LOG_INFO("tcgetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Disable Output Processing. */
			t.c_oflag &= (~OPOST);

			/* Set the IXON flag. */
			t.c_iflag |= IXON;

			/* Apply the changes. */
			if (tcsetattr(fd, TCSANOW, &t) != 0)
			{
				LOG_INFO("tcsetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Write */
			num_bytes_written = write(fd, (const char *)tx_buffer, strlen(tx_buffer));

			/* Make sure that the write went as planned. */
			if ((unsigned int) num_bytes_written != strlen(tx_buffer))
			{
				LOG_ERR("failed to write to %s", serial_device_paths[dev_path_index]);
				result = ERROR;
				break;
			}

			usleep(TERMIOS_SEND_DELAY_MSEC);

			/* Read the data, the drain should have let the data transmit before getting here. */
			num_bytes_read = read(fd, rx_buffer, sizeof(rx_buffer));

			/* We should not get anything in this test since we are ignoring carriage returns. */
			if ((num_bytes_read != num_bytes_written))
			{
				LOG_ERR("Data read length does not match data sent length.");
				result = ERROR;
				break;
			}
			else
			{
				/* Check that the output is correct. */
				for (i = 0; i < num_bytes_read; i++)
				{
					if (rx_buffer[i] != tx_buffer[i])
					{

						LOG_ERR("Data read did not match data sent.");
						result = ERROR;
						break;
					}
				}

				/* No need to continue if there is a failure. */
				if (result != SUCCESS)
				{
					break;
				}
			}
		}

		/* If we got here then the fd might still be active and we should close the device. */
		close(fd);

		/* No reason to continue if the test already failed. */
		if (result != SUCCESS)
		{
			break;
		}
	}

	LOG_INFO("termios tcsetattr IXON %s", result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

/**
* @brief Test termios setting IXOFF.
*
* @par Detailed Description:
* This test tests that the IXOFF flag can be set.
* Proper functionality (based on the termios specs) buts be
* observed when set.  Needs to have loopback wire.
*
* Test:
* 1) Open the serial device /dev/tty-[1-6]
* 2) Get the termios struct.
* 3) Set the IXOFF flag.
* 4) Set the termios struct.
* 5) Write data to the serial port.
* 6) Read data back and check is read data is correct (No Change in data).
* 7) Loop 2-6 for TERMIOS_TEST_CYCLES times
* 8) Close the serial device.
* 9) Loop steps 1-8 for each serial device path that has a loopback wire.
*
* @return
* - SUCCESS if test does expected actions.
* - Error otherwise
*/
int dspal_tester_termios_set_struct_ixoff(int *looped_device_paths)
{
	const int BUFFER_SIZES = 250;

	int result = SUCCESS;
	int dev_path_index;
	int fd;
	struct termios t;
	int i;
	int runs;
	char tx_buffer[BUFFER_SIZES];
	char rx_buffer[BUFFER_SIZES];
	int num_bytes_written;
	int num_bytes_read;

	LOG_INFO("Beginning tcsetattr IXOFF test");

	memset(tx_buffer, 0, sizeof(tx_buffer));
	memset(rx_buffer, 0, sizeof(rx_buffer));

	/* Fill the TX buffer with any data thats not NULL. */
	for (i = 0; i < (BUFFER_SIZES - 1); i++)
	{
		/* Fill in with carriage returns. */
		tx_buffer[i] = 'a' + (rand() % 26);
	}

	/* Do each device path. */
	for (dev_path_index = 0; dev_path_index < NUM_UART_DEVICE_ENABLED; dev_path_index++)
	{
		if (!looped_device_paths[dev_path_index])
		{
			/* If is not looped-back then the test is kind of useless. */
			continue;
		}

		/* Open the device path. */
		fd = open(serial_device_paths[dev_path_index], O_RDWR);

		/* Only proceed if the open succeeded. */
		if (fd < SUCCESS)
		{
			LOG_INFO("Open %s O_RDWR mode failed.", serial_device_paths[dev_path_index]);
			continue;
		}
		LOG_INFO("Open %s O_RDWR mode succeeded.", serial_device_paths[dev_path_index]);

		for (runs = 0; runs < TERMIOS_TEST_CYCLES; runs++)
		{
			/* Make sure the struct does not have anything in it*/
			memset(&t, 0, sizeof(struct termios));

			/* Get the current configurations of the termios device. */
			if (tcgetattr(fd, &t) != 0)
			{
				LOG_INFO("tcgetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Disable Output Processing. */
			t.c_oflag &= (~OPOST);

			/* Set the IXOFF flag. */
			t.c_iflag |= IXOFF;

			/* Apply the changes. */
			if (tcsetattr(fd, TCSANOW, &t) != 0)
			{
				LOG_INFO("tcsetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Write */
			num_bytes_written = write(fd, (const char *)tx_buffer, strlen(tx_buffer));

			/* Make sure that the write went as planned. */
			if ((unsigned int) num_bytes_written != strlen(tx_buffer))
			{
				LOG_ERR("failed to write to %s", serial_device_paths[dev_path_index]);
				result = ERROR;
				break;
			}

			usleep(TERMIOS_SEND_DELAY_MSEC);

			/* Read the data, the drain should have let the data transmit before getting here. */
			num_bytes_read = read(fd, rx_buffer, sizeof(rx_buffer));

			/* We should not get anything in this test since we are ignoring carriage returns. */
			if ((num_bytes_read != num_bytes_written))
			{
				LOG_ERR("Data read length does not match data sent length.");
				result = ERROR;
				break;
			}
			else
			{
				/* Check that the output is correct. */
				for (i = 0; i < num_bytes_read; i++)
				{
					if (rx_buffer[i] != tx_buffer[i])
					{
						LOG_ERR("Data read did not match data sent.");
						result = ERROR;
						break;
					}
				}

				/* No need to continue if there is a failure. */
				if (result != SUCCESS)
				{
					break;
				}
			}
		}
		/* If we got here then the fd might still be active and we should close the device. */
		close(fd);

		/* No reason to continue if the test already failed. */
		if (result != SUCCESS)
		{
			break;
		}
	}

	LOG_INFO("termios tcsetattr IXOFF %s", result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

/**
* @brief Test termios setting CRTSCTS.
*
* @par Detailed Description:
* This test tests that the CRTSCTS flag can be set.
* Proper functionality (based on the termios specs) buts be
* observed when set.  Needs to have loopback wire.
*
* Test:
* 1) Open the serial device /dev/tty-[1-6]
* 2) Get the termios struct.
* 3) Set the CRTSCTS flag.
* 4) Set the termios struct.
* 5) Write data to the serial port.
* 6) Read data back and check is read data is correct (No Change in data).
* 7) Loop 2-6 for TERMIOS_TEST_CYCLES times
* 8) Close the serial device.
* 9) Loop steps 1-8 for each serial device path that has a loopback wire.
*
* @return
* - SUCCESS if test does expected actions.
* - Error otherwise
*/
int dspal_tester_termios_set_struct_crtscts(int *looped_device_paths)
{
	const int BUFFER_SIZES = 250;

	int result = SUCCESS;
	int dev_path_index;
	int fd;
	struct termios t;
	int i;
	int runs;
	char tx_buffer[BUFFER_SIZES];
	char rx_buffer[BUFFER_SIZES];
	int num_bytes_written;
	int num_bytes_read;

	LOG_INFO("Beginning tcsetattr CRTSCTS test");

	memset(tx_buffer, 0, sizeof(tx_buffer));
	memset(rx_buffer, 0, sizeof(rx_buffer));

	/* Fill the TX buffer with any data thats not NULL. */
	for (i = 0; i < (BUFFER_SIZES - 1); i++)
	{
		/* Fill in with carriage returns. */
		tx_buffer[i] = 'a' + (rand() % 26);
	}

	/* Do each device path. */
	for (dev_path_index = 0; dev_path_index < NUM_UART_DEVICE_ENABLED; dev_path_index++)
	{
		if (!looped_device_paths[dev_path_index])
		{
			/* If is not looped-back then the test is kind of useless. */
			continue;
		}

		/* Open the device path. */
		fd = open(serial_device_paths[dev_path_index], O_RDWR);

		/* Only proceed if the open succeeded. */
		if (fd < SUCCESS)
		{
			LOG_INFO("Open %s O_RDWR mode failed.", serial_device_paths[dev_path_index]);
			continue;
		}
		LOG_INFO("Open %s O_RDWR mode succeeded.", serial_device_paths[dev_path_index]);

		for (runs = 0; runs < TERMIOS_TEST_CYCLES; runs++)
		{
			/* Make sure the struct does not have anything in it*/
			memset(&t, 0, sizeof(struct termios));

			/* Get the current configurations of the termios device. */
			if (tcgetattr(fd, &t) != 0)
			{
				LOG_INFO("tcgetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Disable Output Processing. */
			t.c_oflag &= (~OPOST);

			/* Set the CRTSCTS flag. */
			t.c_cflag |= CRTSCTS;

			/* Apply the changes. */
			if (tcsetattr(fd, TCSANOW, &t) != 0)
			{
				LOG_INFO("tcsetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Write */
			num_bytes_written = write(fd, (const char *)tx_buffer, strlen(tx_buffer));

			/* Make sure that the write went as planned. */
			if ((unsigned int) num_bytes_written != strlen(tx_buffer))
			{
				LOG_ERR("failed to write to %s", serial_device_paths[dev_path_index]);
				result = ERROR;
				break;
			}

			usleep(TERMIOS_SEND_DELAY_MSEC);

			/* Read the data, the drain should have let the data transmit before getting here. */
			num_bytes_read = read(fd, rx_buffer, sizeof(rx_buffer));

			/* We should not get anything in this test since we are ignoring carriage returns. */
			if ((num_bytes_read != num_bytes_written))
			{
				LOG_ERR("Data read length does not match data sent length.");
				result = ERROR;
				break;
			}
			else
			{
				/* Check that the output is correct. */
				for (i = 0; i < num_bytes_read; i++)
				{
					if (rx_buffer[i] != tx_buffer[i])
					{
						LOG_ERR("Data read did not match data sent.");
						result = ERROR;
						break;
					}
				}

				/* No need to continue if there is a failure. */
				if (result != SUCCESS)
				{
					break;
				}
			}
		}

		/* If we got here then the fd might still be active and we should close the device. */
		close(fd);

		/* No reason to continue if the test already failed. */
		if (result != SUCCESS)
		{
			break;
		}
	}

	LOG_INFO("termios tcsetattr CRTSCTS %s", result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

/**
* @brief Test termios setting ONLCR.
*
* @par Detailed Description:
* This test tests that the ONLCR flag can be set.
* Proper functionality (based on the termios specs) buts be
* observed when set.  Needs to have loopback wire.
*
* Test:
* 1) Open the serial device /dev/tty-[1-6]
* 2) Get the termios struct.
* 3) Set the OPOST and ONLCR flags.
* 4) Set the termios struct.
* 5) Write data to the serial port.
* 6) Read data back and check is read data is correct.
* 7) Loop 2-6 for TERMIOS_TEST_CYCLES times
* 8) Close the serial device.
* 9) Loop steps 1-8 for each serial device path that has a loopback wire.
*
* @return
* - SUCCESS if test does expected actions.
* - Error otherwise
*/
int dspal_tester_termios_set_struct_opost_onlcr(int *looped_device_paths)
{
	const int BUFFER_SIZES = 25;

	int result = SUCCESS;
	int dev_path_index;
	int fd;
	struct termios t;
	int i;
	int runs;
	char tx_buffer[BUFFER_SIZES];
	char rx_buffer[BUFFER_SIZES * 2];
	int num_bytes_written;
	int num_bytes_read;

	LOG_INFO("Beginning tcsetattr OPOST ONLCR test");

	memset(tx_buffer, 0, sizeof(tx_buffer));
	memset(rx_buffer, 0, sizeof(rx_buffer));

	/* Fill the TX buffer. */
	for (i = 0; i < (BUFFER_SIZES - 1); i++)
	{
		tx_buffer[i] = '\n';
	}

	/* Do each device path. */
	for (dev_path_index = 0; dev_path_index < NUM_UART_DEVICE_ENABLED; dev_path_index++)
	{
		if (!looped_device_paths[dev_path_index])
		{
			/* If is not looped-back then the test is kind of useless. */
			continue;
		}

		/* Open the device path. */
		fd = open(serial_device_paths[dev_path_index], O_RDWR);

		/* Only proceed if the open succeeded. */
		if (fd < SUCCESS)
		{
			LOG_INFO("Open %s O_RDWR mode failed.", serial_device_paths[dev_path_index]);
			continue;
		}
		LOG_INFO("Open %s O_RDWR mode succeeded.", serial_device_paths[dev_path_index]);

		for (runs = 0; runs < TERMIOS_TEST_CYCLES; runs++)
		{
			/* Make sure the struct does not have anything in it*/
			memset(&t, 0, sizeof(struct termios));

			/* Get the current configurations of the termios device. */
			if (tcgetattr(fd, &t) != 0)
			{
				LOG_INFO("tcgetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Turn off whatever was on. */
			t.c_oflag &= ~(ONLCR);

			/* Set the OPOST and ONLCR flags. */
			t.c_oflag |= OPOST | ONLCR;

			/* Apply the changes. */
			if (tcsetattr(fd, TCSANOW, &t) != 0)
			{
				LOG_INFO("tcsetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Write */
			num_bytes_written = write(fd, (const char *)tx_buffer, strlen(tx_buffer));

			/* Make sure that the write went as planned. */
			if ((unsigned int) num_bytes_written != strlen(tx_buffer))
			{
				LOG_ERR("failed to write to %s", serial_device_paths[dev_path_index]);
				result = ERROR;
				break;
			}

			usleep(TERMIOS_SEND_DELAY_MSEC);

			/* Read the data, the drain should have let the data transmit before getting here. */
			num_bytes_read = read(fd, rx_buffer, sizeof(rx_buffer));

			/* We should not get anything in this test since we are ignoring carriage returns. */
			if ((num_bytes_read != (num_bytes_written * 2)) || ((num_bytes_read % 2) == 1))
			{
				LOG_ERR("Data read length does not match data sent length * 2.");
				result = ERROR;
				break;
			}
			else
			{
				/* Check that the output is correct. */
				for (i = 0; i < num_bytes_read; i += 2)
				{
					if ((rx_buffer[i] != '\r') || (rx_buffer[i + 1] != '\n'))
					{
						LOG_ERR("Read buffer was not not \r\n, had other characters. ");
						result = ERROR;
						break;
					}
				}

				/* No need to continue if there is a failure. */
				if (result != SUCCESS)
				{
					break;
				}
			}
		}

		/* If we got here then the fd might still be active and we should close the device. */
		close(fd);

		/* No reason to continue if the test already failed. */
		if (result != SUCCESS)
		{
			break;
		}
	}

	LOG_INFO("termios tcsetattr OPOST ONLCR %s", result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

/**
* @brief Test termios setting OCRNL.
*
* @par Detailed Description:
* This test tests that the OCRNL flag can be set.
* Proper functionality (based on the termios specs) buts be
* observed when set.  Needs to have loopback wire.
*
* Test:
* 1) Open the serial device /dev/tty-[1-6]
* 2) Get the termios struct.
* 3) Set the OPOST and OCRNL flags.
* 4) Set the termios struct.
* 5) Write data to the serial port.
* 6) Read data back and check is read data is correct.
* 7) Loop 2-6 for TERMIOS_TEST_CYCLES times
* 8) Close the serial device.
* 9) Loop steps 1-8 for each serial device path that has a loopback wire.
*
* @return
* - SUCCESS if test does expected actions.
* - Error otherwise
*/
int dspal_tester_termios_set_struct_opost_ocrnl(int *looped_device_paths)
{
	const int BUFFER_SIZES = 25;

	int result = SUCCESS;
	int dev_path_index;
	int fd;
	struct termios t;
	int i;
	int runs;
	char tx_buffer[BUFFER_SIZES];
	char rx_buffer[BUFFER_SIZES];
	int num_bytes_written;
	int num_bytes_read;

	LOG_INFO("Beginning tcsetattr OCRNL test");

	memset(tx_buffer, 0, sizeof(tx_buffer));
	memset(rx_buffer, 0, sizeof(rx_buffer));

	/* Fill the TX buffer with any data. */
	for (i = 0; i < (BUFFER_SIZES - 1); i++)
	{
		/* Fill in with carriage returns. */
		tx_buffer[i] = '\r';
	}

	/* Do each device path. */
	for (dev_path_index = 0; dev_path_index < NUM_UART_DEVICE_ENABLED; dev_path_index++)
	{
		if (!looped_device_paths[dev_path_index])
		{
			/* If is not looped-back then the test is kind of useless. */
			continue;
		}

		/* Open the device path. */
		fd = open(serial_device_paths[dev_path_index], O_RDWR);

		/* Only proceed if the open succeeded. */
		if (fd < SUCCESS)
		{
			LOG_INFO("Open %s O_RDWR mode failed.", serial_device_paths[dev_path_index]);
			continue;
		}
		LOG_INFO("Open %s O_RDWR mode succeeded.", serial_device_paths[dev_path_index]);

		for (runs = 0; runs < TERMIOS_TEST_CYCLES; runs++)
		{
			/* Make sure the struct does not have anything in it*/
			memset(&t, 0, sizeof(struct termios));

			/* Get the current configurations of the termios device. */
			if (tcgetattr(fd, &t) != 0)
			{
				LOG_INFO("tcgetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Turn off whatever was on. */
			t.c_oflag &= ~(ONLCR);

			/* Set the OPOST and OCRNL flags. */
			t.c_oflag |= OPOST | OCRNL;

			/* Apply the changes. */
			if (tcsetattr(fd, TCSANOW, &t) != 0)
			{
				LOG_INFO("tcsetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Write */
			num_bytes_written = write(fd, (const char *)tx_buffer, strlen(tx_buffer));

			/* Make sure that the write went as planned. */
			if ((unsigned int) num_bytes_written != strlen(tx_buffer))
			{
				LOG_ERR("failed to write to %s", serial_device_paths[dev_path_index]);
				result = ERROR;
				break;
			}

			usleep(TERMIOS_SEND_DELAY_MSEC);

			/* Read the data, the drain should have let the data transmit before getting here */
			num_bytes_read = read(fd, rx_buffer, sizeof(rx_buffer));

			/* If we have a loop-back connection and we dont see the correct amount of data, fail. */
			if ((num_bytes_read != num_bytes_written))
			{
				LOG_ERR("Data read length does not match data sent length.");
				result = ERROR;
				break;
			}
			else
			{
				/* Check that the output is correct. */
				for (i = 0; i < num_bytes_read; i++)
				{
					if (rx_buffer[i] != '\n')
					{
						LOG_ERR("CR's were not changed to NL's.");
						result = ERROR;
						break;
					}
				}

				/* No need to continue if there is a failure. */
				if (result != SUCCESS)
				{
					break;
				}
			}
		}

		/* If we got here then the fd might still be active and we should close the device. */
		close(fd);

		/* No reason to continue if the test already failed. */
		if (result != SUCCESS)
		{
			break;
		}
	}

	LOG_INFO("termios tcsetattr OCRNL %s", result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

/**
* @brief Test termios setting ONOCR.
*
* @par Detailed Description:
* This test tests that the ONOCR flag can be set.
* Proper functionality (based on the termios specs) buts be
* observed when set.  Needs to have loopback wire.
*
* Test:
* 1) Open the serial device /dev/tty-[1-6]
* 2) Get the termios struct.
* 3) Set the OPOST and ONOCR flags.
* 4) Set the termios struct.
* 5) Write data to the serial port.
* 6) Read data back and check is read data is correct.
* 7) Loop 2-6 for TERMIOS_TEST_CYCLES times
* 8) Close the serial device.
* 9) Loop steps 1-8 for each serial device path that has a loopback wire.
*
* @return
* - SUCCESS if test does expected actions.
* - Error otherwise
*/
int dspal_tester_termios_set_struct_opost_onocr(int *looped_device_paths)
{
	const int BUFFER_SIZES = 25;

	int result = SUCCESS;
	int dev_path_index;
	int fd;
	struct termios t;
	int i;
	int runs;
	char tx_buffer[BUFFER_SIZES];
	char rx_buffer[BUFFER_SIZES];
	int num_bytes_written;
	int num_bytes_read;

	LOG_INFO("Beginning tcsetattr ONOCR test");

	memset(tx_buffer, 0, sizeof(tx_buffer));
	memset(rx_buffer, 0, sizeof(rx_buffer));

	/* Fill the buffer.  This was manually set since it is easier to test against
	 * a manual character sequence.
	 */
	sprintf(tx_buffer, "\r\r\raaa\naaa\r\r\r\n\ra\r");

	/* Do each device path. */
	for (dev_path_index = 0; dev_path_index < NUM_UART_DEVICE_ENABLED; dev_path_index++)
	{
		if (!looped_device_paths[dev_path_index])
		{
			/* If is not looped-back then the test is kind of useless. */
			continue;
		}

		/* Open the device path. */
		fd = open(serial_device_paths[dev_path_index], O_RDWR);

		/* Only proceed if the open succeeded. */
		if (fd < SUCCESS)
		{
			LOG_INFO("Open %s O_RDWR mode failed.", serial_device_paths[dev_path_index]);
			continue;
		}
		LOG_INFO("Open %s O_RDWR mode succeeded.", serial_device_paths[dev_path_index]);

		for (runs = 0; runs < TERMIOS_TEST_CYCLES; runs++)
		{
			/* Make sure the struct does not have anything in it*/
			memset(&t, 0, sizeof(struct termios));

			/* Get the current configurations of the termios device. */
			if (tcgetattr(fd, &t) != 0)
			{
				LOG_INFO("tcgetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Turn off whatever was on. */
			t.c_oflag &= ~(ONLCR);

			/* Set the OPOST and ONOCR flags. */
			t.c_oflag |= OPOST | ONOCR;

			/* Apply the changes. */
			if (tcsetattr(fd, TCSANOW, &t) != 0)
			{
				LOG_INFO("tcsetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Write */
			num_bytes_written = write(fd, (const char *)tx_buffer, strlen(tx_buffer));

			/* Make sure that the write went as planned. */
			if ((unsigned int) num_bytes_written != strlen(tx_buffer))
			{
				LOG_ERR("failed to write to %s", serial_device_paths[dev_path_index]);
				result = ERROR;
				break;
			}

			usleep(TERMIOS_SEND_DELAY_MSEC);

			/* Read the data, the drain should have let the data transmit before getting here */
			num_bytes_read = read(fd, rx_buffer, sizeof(rx_buffer));

			/* If we have a loop-back connection and we dont see the correct amount of data, fail. */
			if ((num_bytes_read != 11))
			{
				LOG_ERR("Data read length incorrect.");
				result = ERROR;
			}
			else
			{
				/* Check the data read*/
				if (strncmp(rx_buffer, "aaa\naaa\r\na\r" , 11) != 0)
				{
					LOG_ERR("Data in read buffer is incorrect.");
					result = ERROR;
					break;
				}
			}
		}

		/* If we got here then the fd might still be active and we should close the device. */
		close(fd);

		/* No reason to continue if the test already failed. */
		if (result != SUCCESS)
		{
			break;
		}
	}

	LOG_INFO("termios tcsetattr ONOCR %s", result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

/**
* @brief Test termios setting the buad rate.
*
* @par Detailed Description:
* This test tests that the baud rate setting functionality of termios
* is functions correctly.
*
* Test:
* 1) Open the serial device /dev/tty-[1-6]
* 2) Get the termios struct.
* 3) Turn off output processing (OPOST flag).
* 4) Set the baud rate.
* 5) Set the termios struct.
* 6) Write data to the serial port.
* 7) Read data back and check is read data is correct.
* 8) Loop 2-7 for TERMIOS_TEST_CYCLES times
* 9) Close the serial device.
* 10) Loop steps 1-9 for each serial device path that has a loopback wire.
*
* @return
* - SUCCESS if test does expected actions.
* - Error otherwise
*/
int dspal_tester_termios_set_struct_baud(int *looped_device_paths)
{
	const int BUFFER_SIZES = 25;
	const int NUMBER_OF_BAUD_SPEEDS = 3;

	int result = SUCCESS;
	int dev_path_index;
	int fd;
	struct termios t;
	int i;
	int runs;
	char tx_buffer[BUFFER_SIZES];
	char rx_buffer[BUFFER_SIZES];
	int num_bytes_written;
	int num_bytes_read;

	int buad_speeds[NUMBER_OF_BAUD_SPEEDS] = {B9600, B115200, B19200};

	LOG_INFO("Beginning tcsetattr set baud rate test");

	memset(tx_buffer, 0, sizeof(tx_buffer));
	memset(rx_buffer, 0, sizeof(rx_buffer));

	/* Fill the TX buffer with any data ('a' in this case) thats not NULL. */
	for (i = 0; i < (BUFFER_SIZES - 1); i++)
	{
		/* Fill in with carriage returns. */
		tx_buffer[i] = 132;
	}

	/* Do each device path. */
	for (dev_path_index = 0; dev_path_index < NUM_UART_DEVICE_ENABLED; dev_path_index++)
	{
		if (!looped_device_paths[dev_path_index])
		{
			/* If is not looped-back then the test is kind of useless. */
			continue;
		}

		/* Open the device path. */
		fd = open(serial_device_paths[dev_path_index], O_RDWR);

		/* Only proceed if the open succeeded. */
		if (fd < SUCCESS)
		{
			LOG_INFO("Open %s O_RDWR mode failed.", serial_device_paths[dev_path_index]);
			continue;
		}
		LOG_INFO("Open %s O_RDWR mode succeeded.", serial_device_paths[dev_path_index]);

		for (runs = 0; runs < TERMIOS_TEST_CYCLES; runs++)
		{
			/* Make sure the struct does not have anything in it*/
			memset(&t, 0, sizeof(struct termios));

			/* Get the current configurations of the termios device. */
			if (tcgetattr(fd, &t) != 0)
			{
				LOG_INFO("tcgetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Disable Output Processing. */
			t.c_oflag &= (~OPOST);

			/* Change the baud rate. */
			if (cfsetspeed(&t, buad_speeds[runs % NUMBER_OF_BAUD_SPEEDS]) != 0)
			{
				LOG_INFO("cfsetspeed call failed.")
				result =  ERROR;
				break;
			}

			/* Apply the changes. */
			if (tcsetattr(fd, TCSANOW, &t) != 0)
			{
				LOG_INFO("tcsetattr call failed.")
				result =  ERROR;
				break;
			}

			/* Write */
			num_bytes_written = write(fd, (const char *)tx_buffer, strlen(tx_buffer));

			/* Make sure that the write went as planned. */
			if ((unsigned int) num_bytes_written != strlen(tx_buffer))
			{
				LOG_ERR("failed to write to %s", serial_device_paths[dev_path_index]);
				result = ERROR;
				break;
			}

			usleep(TERMIOS_SEND_DELAY_MSEC * 5);

			/* Read the data, the drain should have let the data transmit before getting here. */
			num_bytes_read = read(fd, rx_buffer, sizeof(rx_buffer));

			/* We should not get anything in this test since we are ignoring carriage returns. */
			if ((num_bytes_read != num_bytes_written))
			{
				LOG_ERR("Data read length does not match data sent length.");
				result = ERROR;
			}
			else
			{
				/* Check that the output is correct. */
				for (i = 0; i < num_bytes_read; i++)
				{
					if (rx_buffer[i] != tx_buffer[i])
					{
						LOG_ERR("Read data does not match written data");
						result = ERROR;
						break;
					}
				}

				/* No need to continue if there is a failure. */
				if (result != SUCCESS)
				{
					break;
				}
			}
		}

		/* If we got here then the fd might still be active and we should close the device. */
		close(fd);

		/* No reason to continue if the test already failed. */
		if (result != SUCCESS)
		{
			break;
		}
	}

	LOG_INFO("termios tcsetattr set baud rate %s", result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

/**
* @brief Test termios setting change settings TCSADRAIN option.
*
* @par Detailed Description:
* This test tests changing the termios setting using the TCSADRAIN option.
*
* Test:
* 1) Open the serial device /dev/tty-[1-6]
* 2) Get the termios struct.
* 3) Turn on OPOST and OCRNL.
* 4) Write data to the port.
* 5) Set the termios struct.
* 6) Write data to the serial port.
* 7) Read data back and check is read data is correct (same as data sent).
* 8) Write data to the port.
* 9) Read data back and check is read data is correct (CR to NL).
* 10) Close the serial device.
* 11) Loop steps 1-10 for each serial device path that has a loopback wire.
*
* @return
* - SUCCESS if test does expected actions.
* - Error otherwise
*/
int dspal_tester_termios_set_struct_ocrnl_tcsadrain(int *looped_device_paths)
{
	const int BUFFER_SIZES = 750;

	int result = SUCCESS;
	int dev_path_index;
	int fd;
	struct termios t;
	int i;
	char tx_buffer[BUFFER_SIZES];
	char rx_buffer[BUFFER_SIZES];
	int num_bytes_written;
	int num_bytes_read;

	LOG_INFO("Beginning tcsetattr OCRNL TCSADRAIN test");

	memset(tx_buffer, 0, sizeof(tx_buffer));
	memset(rx_buffer, 0, sizeof(rx_buffer));

	/* Fill the TX buffer with any data. */
	for (i = 0; i < (BUFFER_SIZES - 1); i++)
	{
		/* Fill in with carriage returns. */
		tx_buffer[i] = '\r';
	}

	/* Do each device path. */
	for (dev_path_index = 0; dev_path_index < NUM_UART_DEVICE_ENABLED; dev_path_index++)
	{
		if (!looped_device_paths[dev_path_index])
		{
			/* If IS looped-back then the test is kind of useless. */
			continue;
		}

		/* Open the device path. */
		fd = open(serial_device_paths[dev_path_index], O_RDWR);

		/* Only proceed if the open succeeded. */
		if (fd < SUCCESS)
		{
			LOG_INFO("Open %s O_RDWR mode failed.", serial_device_paths[dev_path_index]);
			continue;
		}
		LOG_INFO("Open %s O_RDWR mode succeeded.", serial_device_paths[dev_path_index]);

		/* Make sure the struct does not have anything in it*/
		memset(&t, 0, sizeof(struct termios));

		/* Get the current configurations of the termios device. */
		if (tcgetattr(fd, &t) != 0)
		{
			LOG_INFO("tcgetattr call failed.")
			result =  ERROR;
			close(fd);
			break;
		}

		/* Turn off whatever was on. */
		t.c_oflag &= ~(ONLCR);

		/* Set the OPOST and OCRNL flags. */
		t.c_oflag |= OPOST | OCRNL;

		/* Write */
		num_bytes_written = write(fd, (const char *)tx_buffer, strlen(tx_buffer));

		/* Make sure that the write went as planned. */
		if ((unsigned int) num_bytes_written != strlen(tx_buffer))
		{
			LOG_ERR("failed to write to %s", serial_device_paths[dev_path_index]);
			result = ERROR;
			close(fd);
			break;
		}

		/* Apply the changes. */
		if (tcsetattr(fd, TCSADRAIN, &t) != 0)
		{
			LOG_INFO("tcsetattr call failed.")
			result =  ERROR;
			close(fd);
			break;
		}

		/* Read the data, the drain should have let the data transmit before getting here */
		num_bytes_read = read(fd, rx_buffer, sizeof(rx_buffer));

		/* If we have a loop-back connection and we dont see the correct amount of data, fail. */
		if ((num_bytes_read != num_bytes_written))
		{
			LOG_ERR("Data read length does not match data sent length.");
			result = ERROR;
			close(fd);
			break;
		}
		else
		{
			/* Check that the output is correct. */
			for (i = 0; i < num_bytes_read; i++)
			{
				if (tx_buffer[i] != rx_buffer[i])
				{
					/* There is a non match between the data sent and the data read. */
					LOG_ERR("Data read does not match data sent.");
					result = ERROR;
					break;
				}
			}

			/* No need to continue if there is a failure. */
			if (result != SUCCESS)
			{
				close(fd);
				break;
			}
		}

		/* Clear the RX Buffer. */
		memset(rx_buffer, 0, sizeof(rx_buffer));

		/* Write */
		num_bytes_written = write(fd, (const char *)tx_buffer, strlen(tx_buffer));

		/* Make sure that the write went as planned. */
		if ((unsigned int) num_bytes_written != strlen(tx_buffer))
		{
			LOG_ERR("failed to write to %s", serial_device_paths[dev_path_index]);
			result = ERROR;
			close(fd);
			break;
		}

		/* Wait for all the data to be transmitted. */
		usleep(TERMIOS_SEND_DELAY_MSEC * 10);

		/* Read the data. */
		num_bytes_read = read(fd, rx_buffer, sizeof(rx_buffer));

		/* If we have a loop-back connection and we dont see the correct amount of data, fail. */
		if (num_bytes_read != num_bytes_written)
		{
			LOG_ERR("Data read length does not match data sent length x2.");
			result = ERROR;
		}
		else
		{
			/* Check that the output is correct. */
			for (i = 0; i < num_bytes_read; i++)
			{
				if (rx_buffer[i] != '\n')
				{
					LOG_ERR("CR's were not changed to NL's.");
					result = ERROR;
				}
			}
		}


		/* If we got here then the fd might still be active and we should close the device. */
		close(fd);

		/* No reason to continue if the test already failed. */
		if (result != SUCCESS)
		{
			break;
		}
	}

	LOG_INFO("termios tcsetattr OCRNL TCSADRAIN %s", result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

/**
* @brief Test termios setting change settings TCSAFLUSH option.
*
* @par Detailed Description:
* This test tests changing the termios setting using the TCSAFLUSH option.
*
* Test:
* 1) Open the serial device /dev/tty-[1-6]
* 2) Get the termios struct.
* 3) Turn on OPOST and OCRNL.
* 4) Write data to the port.
* 5) Set the termios struct.
* 6) Write data to the serial port.
* 7) Read data back and check is read data is correct (same as data sent).
* 8) Write data to the port.
* 9) Read data back and check is read data is correct (CR to NL).
* 10) Close the serial device.
* 11) Loop steps 1-10 for each serial device path that has a loopback wire.
*
* @return
* - SUCCESS if test does expected actions.
* - Error otherwise
*/
int dspal_tester_termios_set_struct_ocrnl_tcsaflush(int *looped_device_paths)
{
	const int BUFFER_SIZES = 750;

	int result = SUCCESS;
	int dev_path_index;
	int fd;
	struct termios t;
	int i;
	char tx_buffer[BUFFER_SIZES];
	char rx_buffer[BUFFER_SIZES];
	int num_bytes_written;
	int num_bytes_read;

	LOG_INFO("Beginning tcsetattr OCRNL TCSADRAIN test");

	memset(tx_buffer, 0, sizeof(tx_buffer));
	memset(rx_buffer, 0, sizeof(rx_buffer));

	/* Fill the TX buffer with any data. */
	for (i = 0; i < (BUFFER_SIZES - 1); i++)
	{
		/* Fill in with carriage returns. */
		tx_buffer[i] = '\r';
	}

	/* Do each device path. */
	for (dev_path_index = 0; dev_path_index < NUM_UART_DEVICE_ENABLED; dev_path_index++)
	{
		if (!looped_device_paths[dev_path_index])
		{
			/* If IS looped-back then the test is kind of useless. */
			continue;
		}

		/* Open the device path. */
		fd = open(serial_device_paths[dev_path_index], O_RDWR);

		/* Only proceed if the open succeeded. */
		if (fd < SUCCESS)
		{
			LOG_INFO("Open %s O_RDWR mode failed.", serial_device_paths[dev_path_index]);
			continue;
		}
		LOG_INFO("Open %s O_RDWR mode succeeded.", serial_device_paths[dev_path_index]);

		/* Make sure the struct does not have anything in it*/
		memset(&t, 0, sizeof(struct termios));

		/* Get the current configurations of the termios device. */
		if (tcgetattr(fd, &t) != 0)
		{
			LOG_INFO("tcgetattr call failed.")
			result =  ERROR;
			close(fd);
			break;
		}

		/* Turn off whatever was on. */
		t.c_oflag &= ~(ONLCR);

		/* Set the OPOST and OCRNL flags. */
		t.c_oflag |= OPOST | OCRNL;

		/* Write */
		num_bytes_written = write(fd, (const char *)tx_buffer, strlen(tx_buffer));

		/* Make sure that the write went as planned. */
		if ((unsigned int) num_bytes_written != strlen(tx_buffer))
		{
			LOG_ERR("failed to write to %s", serial_device_paths[dev_path_index]);
			result = ERROR;
			close(fd);
			break;
		}

		/* Apply the changes. */
		if (tcsetattr(fd, TCSAFLUSH, &t) != 0)
		{
			LOG_INFO("tcsetattr call failed.")
			result =  ERROR;
			close(fd);
			break;
		}

		/* Read the data, the drain should have let the data transmit before getting here */
		num_bytes_read = read(fd, rx_buffer, sizeof(rx_buffer));

		/* If we have a loop-back connection and we dont see the correct amount of data, fail. */
		if ((num_bytes_read != 0))
		{
			LOG_ERR("Data read length is not 0.");
			result = ERROR;
			close(fd);
			break;
		}

		/* Clear the RX Buffer. */
		memset(rx_buffer, 0, sizeof(rx_buffer));

		/* Write */
		num_bytes_written = write(fd, (const char *)tx_buffer, strlen(tx_buffer));

		/* Make sure that the write went as planned. */
		if ((unsigned int) num_bytes_written != strlen(tx_buffer))
		{
			LOG_ERR("failed to write to %s", serial_device_paths[dev_path_index]);
			result = ERROR;
			close(fd);
			break;
		}

		/* Wait for all the data to be transmitted. */
		usleep(TERMIOS_SEND_DELAY_MSEC * 10);

		/* Read the data. */
		num_bytes_read = read(fd, rx_buffer, sizeof(rx_buffer));

		/* If we have a loop-back connection and we dont see the correct amount of data, fail. */
		if (num_bytes_read != num_bytes_written)
		{
			LOG_ERR("Data read length does not match data sent length x2.");
			result = ERROR;
		}
		else
		{
			/* Check that the output is correct. */
			for (i = 0; i < num_bytes_read; i++)
			{
				if (rx_buffer[i] != '\n')
				{
					LOG_ERR("CR's were not changed to NL's.");
					result = ERROR;
				}
			}
		}


		/* If we got here then the fd might still be active and we should close the device. */
		close(fd);

		/* No reason to continue if the test already failed. */
		if (result != SUCCESS)
		{
			break;
		}
	}

	LOG_INFO("termios tcsetattr OCRNL TCSADRAIN %s", result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

/**
* @brief Runs all the serial termios tests and returns 1 aggregated result.
*
* @return
* SUCCESS ------ All tests pass
* ERROR -------- One or more tests failed
*/
int dspal_tester_termios_test(void)
{
	int result = SUCCESS;
	int i;

	int looped_connections[NUM_UART_DEVICE_ENABLED];
	memset(looped_connections, 0, sizeof(looped_connections));

	/*
	 * Get which device paths have a loop-back wire.  We will need these device paths
	 * for some of the tests we will be running
	 */
	LOG_INFO("Checking which connections have a loop-back wire.");
	dspal_tester_termios_helper_get_looped_connections(looped_connections);

	result = dspal_tester_termios_helper_functions();
	if (result < SUCCESS)
	{
		return result;
	}

	result = dspal_tester_termios_get_struct();
	if (result < SUCCESS)
	{
		return result;
	}

	result = dspal_tester_termios_set_struct_cread(looped_connections);
	if (result < SUCCESS)
	{
		return result;
	}

	result = dspal_tester_termios_set_struct_inlcr(looped_connections);
	if (result < SUCCESS)
	{
		return result;
	}

	result = dspal_tester_termios_set_struct_icrnl(looped_connections);
	if (result < SUCCESS)
	{
		return result;
	}

	result = dspal_tester_termios_set_struct_igncr(looped_connections);
	if (result < SUCCESS)
	{
		return result;
	}

	result = dspal_tester_termios_set_struct_istrip(looped_connections);
	if (result < SUCCESS)
	{
		return result;
	}

	result = dspal_tester_termios_set_struct_ixon(looped_connections);
	if (result < SUCCESS)
	{
		return result;
	}

	result = dspal_tester_termios_set_struct_ixoff(looped_connections);
	if (result < SUCCESS)
	{
		return result;
	}

	result = dspal_tester_termios_set_struct_crtscts(looped_connections);
	if (result < SUCCESS)
	{
		return result;
	}

	result = dspal_tester_termios_drain(looped_connections);
	if (result < SUCCESS)
	{
		return result;
	}

	result = dspal_tester_termios_flush(looped_connections);
	if (result < SUCCESS)
	{
		return result;
	}

	result = dspal_tester_termios_flow(looped_connections);
	if (result < SUCCESS)
	{
		return result;
	}

	result = dspal_tester_termios_tcsendbreak();
	if (result < SUCCESS)
	{
		return result;
	}

	result = dspal_tester_termios_set_struct_opost_onlcr(looped_connections);
	if (result < SUCCESS)
	{
		return result;
	}

	result = dspal_tester_termios_set_struct_opost_ocrnl(looped_connections);
	if (result < SUCCESS)
	{
		return result;
	}

	result = dspal_tester_termios_set_struct_opost_onocr(looped_connections);
	if (result < SUCCESS)
	{
		return result;
	}

	result = dspal_tester_termios_set_struct_baud(looped_connections);
	if (result < SUCCESS)
	{
		return result;
	}

	result = dspal_tester_termios_set_struct_ocrnl_tcsadrain(looped_connections);
	if (result < SUCCESS)
	{
		return result;
	}

	result = dspal_tester_termios_set_struct_ocrnl_tcsaflush(looped_connections);
	if (result < SUCCESS)
	{
		return result;
	}

	return SUCCESS;
}
