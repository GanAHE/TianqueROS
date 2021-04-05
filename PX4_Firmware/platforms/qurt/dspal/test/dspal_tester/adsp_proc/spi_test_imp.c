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
#include <assert.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdbool.h>
#include <dev_fs_lib_spi.h>
#include "test_status.h"
#include "test_utils.h"

#include "platform.h"

#define SPI_TEST_CYCLES 10

#define MPU_SPI_BUF_LEN   512
/**
 * Supported SPI frequency to talk to MPU9x50 slave device
 * MPU9x50 SPI interface supports upto 20MHz frequency. However 20MHz is not
 * reliable in our test and corrupted data is observed.
 */
enum MPU_SPI_FREQUENCY
{
   MPU_SPI_FREQUENCY_1MHZ = 1000000UL,
   MPU_SPI_FREQUENCY_5MHZ = 5000000UL,
   MPU_SPI_FREQUENCY_10MHZ = 10000000UL,
   MPU_SPI_FREQUENCY_15MHZ = 15000000UL,
   MPU_SPI_FREQUENCY_20MHZ = 20000000UL,
};

static uint8_t spiTxBuf[MPU_SPI_BUF_LEN];
static uint8_t spiRxBuf[MPU_SPI_BUF_LEN];

/**
 * NOTE: DO NOT send more than 64 bytes in loopback test. SPI bus automatically
 * switches to DMA mode to send more than 64 bytes. However, DMA mode in
 * loopback transfer results in system crash or hang. Transfering more than 64
 * bytes to/from peripheral device using DMA mode is supported.
 */
#define SPI_LOOPBACK_TEST_TRANSMIT_BUFFER_LENGTH  20

/**
 * @brief Helper function  for 'dspal_tester_spi_test', checks if 2 data buffers are equal.
 *
 *
 * @param buffer1[in]  pointer to first buffer
 * @param buffer2[in]  pointer to second buffer
 * @param length[in]   length of each buffers
 *
 * @return
 * true  ------ data buffers match
 * false ------ data buffers do not match
*/
bool dpsal_tester_is_memory_matching(uint8_t *buffer1, uint8_t *buffer2, int length)
{
	if (memcmp(buffer1, buffer2, length) != 0) {
		LOG_ERR("error: the bytes read to not match the bytes written");
		LOG_ERR("bytes written: %c, %c, %c, %c, %c", buffer1[0], buffer1[1], buffer1[2],
			buffer1[3], buffer1[4]);
		LOG_ERR("bytes read: %c, %c, %c, %c, %c", buffer2[0], buffer2[1], buffer2[2],
			buffer2[3], buffer2[4]);
		return false;
	}

	return true;
}

void init_write_buffer(uint8_t *buffer, int length)
{
	int i;
	char c = 'a';

	for (i = 0; i < length; i++) {
		buffer[i] = c;

		if (c == 'z') {
			c = 'a';
		}

		c++;
	}
}

int mpu_spi_configure_speed(int fd, enum MPU_SPI_FREQUENCY freq)
{
   struct dspal_spi_ioctl_set_bus_frequency bus_freq;

   bus_freq.bus_frequency_in_hz = freq;

   return ioctl(fd, SPI_IOCTL_SET_BUS_FREQUENCY_IN_HZ, &bus_freq);
}


int mpu_spi_get_reg(int fd, int reg, uint8_t* val)
{
   int retVal;
   struct dspal_spi_ioctl_read_write read_write;


   retVal = mpu_spi_configure_speed(fd, MPU_SPI_FREQUENCY_1MHZ);
   if (retVal != 0)
   {
      LOG_ERR("mpu_spi_get_reg: error configuring speed %d", retVal);
      return retVal;
   }

   spiTxBuf[0] = reg | 0x80; //register high bit=1 for read

   read_write.read_buffer = spiRxBuf;
   read_write.read_buffer_length = 2;
   read_write.write_buffer = spiTxBuf;
   read_write.write_buffer_length = 2;
   retVal = ioctl(fd, SPI_IOCTL_RDWR, &read_write);
   if (retVal != 2)
   {
      FARF(ALWAYS, "mpu_spi_get_reg error read/write ioctl: %d", retVal);
      return retVal;
   }

   *val = spiRxBuf[1];

   FARF(LOW, "mpu_spi_get_reg %d=%d", reg, *val);

   return 0;
}

/**
* @brief Test read/write functionality of spi by using loopback
*
* @par Detailed Description:
* Tests the read and write functionality of the spi device by putting the device
* in loopback mode.  This is tested in 2 ways: writing the data then reading it
* back from the read buffer or by doing the read/write at the same time using ioctl
*
* Test:
* 1) Opens file for spi device ('/dev/spi-8')
* 2) Sets up the spi device in loopback mode using ioctl
* 3) Write to the spi bus
* 4) Read from the spi bus buffer
* 5) Commented Out ---- Check if data written matches data read
* 6) Loop though steps 4-5 for  SPI_TEST_CYCLES number of cycles
* 7) So ioctl read/write operation and check if data written matches data read
* 8) Close spi bus
*
* @return
* SUCCESS  ------ Test Passes
* ERROR ------ Test Failed
*/
int dspal_tester_spi_loopback_test(void)
{
	int spi_fildes = SUCCESS;
	int cycle_count;
	int result = SUCCESS;
	uint8_t write_data_buffer[SPI_LOOPBACK_TEST_TRANSMIT_BUFFER_LENGTH];
	uint8_t read_data_buffer[SPI_LOOPBACK_TEST_TRANSMIT_BUFFER_LENGTH];
	int test_data_length_in_bytes = SPI_LOOPBACK_TEST_TRANSMIT_BUFFER_LENGTH - 1;
	struct dspal_spi_ioctl_loopback loopback;
	struct dspal_spi_ioctl_read_write read_write;
	struct dspal_spi_ioctl_set_spi_mode bus_mode;

	LOG_DEBUG("testing spi open for: %s", SPI_DEVICE_PATH);
	spi_fildes = open(SPI_DEVICE_PATH, 0);

	if (spi_fildes < SUCCESS) {
		LOG_ERR("error: failed to open spi device path: %s", SPI_DEVICE_PATH);
		result = ERROR;
		goto exit;
	}

	/*
	 * Initialize the write buffers in preparation for a read/write sequence.
	 */
	write_data_buffer[SPI_LOOPBACK_TEST_TRANSMIT_BUFFER_LENGTH - 1] = 0;
	init_write_buffer(write_data_buffer, SPI_LOOPBACK_TEST_TRANSMIT_BUFFER_LENGTH - 1);

	/*
	 * Enable loopback mode to allow write/reads to be tested internally.
	 */
	LOG_DEBUG("enabling spi loopback mode");
	loopback.state = SPI_LOOPBACK_STATE_ENABLED;
	result = ioctl(spi_fildes, SPI_IOCTL_LOOPBACK_TEST, &loopback);

	if (result < SUCCESS) {
		LOG_ERR("error: unable to activate spi loopback mode");
		goto exit;
	}

	/* set bus mode, don't goto exit for downward compatible */
	bus_mode.eClockPolarity = SPI_CLOCK_IDLE_HIGH;
	bus_mode.eShiftMode = SPI_OUTPUT_FIRST;
	result = ioctl(spi_fildes, SPI_IOCTL_SET_SPI_MODE, &bus_mode);
	if (result < SUCCESS)
	{
		LOG_ERR("error: unable to set bus mode");
	}

	/*
	 * Test loopback mode using combined read/write mode.
	 */
	LOG_DEBUG("testing spi write/read for %d cycles", SPI_TEST_CYCLES);

	for (cycle_count = 0; cycle_count < SPI_TEST_CYCLES; cycle_count++) {
		memset(read_data_buffer, 0, sizeof(read_data_buffer));
		read_write.read_buffer = &read_data_buffer[0];
		read_write.read_buffer_length = test_data_length_in_bytes;
		read_write.write_buffer = &write_data_buffer[0];
		read_write.write_buffer_length = test_data_length_in_bytes;

		LOG_DEBUG("writing bytes: (%d bytes)",
			  test_data_length_in_bytes);

		result = ioctl(spi_fildes, SPI_IOCTL_RDWR, &read_write);

		if (result < SUCCESS) {
			LOG_ERR("error: unable to activate read/write ioctl");
			goto exit;
		}

		if (!dpsal_tester_is_memory_matching(write_data_buffer, read_data_buffer, test_data_length_in_bytes)) {
			LOG_ERR("error: read/write memory buffers do not match");
			goto exit;
		}

		LOG_DEBUG("written data matches read data");
	}

	result = SUCCESS;
	LOG_DEBUG("SPI lookback test passed");

exit:

	if (spi_fildes > SUCCESS) {
		close(spi_fildes);
	}

	return result;
}

int dspal_tester_spi_exceed_max_length_test(void)
{
	int spi_fildes = SUCCESS;
	int result = SUCCESS;
	uint8_t write_data_buffer[DSPAL_SPI_TRANSMIT_BUFFER_LENGTH + 1];
	uint8_t read_data_buffer[DSPAL_SPI_RECEIVE_BUFFER_LENGTH + 1];
	struct dspal_spi_ioctl_loopback loopback;
	struct dspal_spi_ioctl_read_write read_write;
	struct dspal_spi_ioctl_set_spi_mode bus_mode;

	LOG_DEBUG("testing spi open for: %s", SPI_DEVICE_PATH);
	spi_fildes = open(SPI_DEVICE_PATH, 0);

	if (spi_fildes < SUCCESS) {
		LOG_ERR("error: failed to open spi device path: %s", SPI_DEVICE_PATH);
		result = ERROR;
		goto exit;
	}

	/*
	 * Enable loopback mode to allow write/reads to be tested internally.
	 */
	LOG_DEBUG("enabling spi loopback mode");
	loopback.state = SPI_LOOPBACK_STATE_ENABLED;
	result = ioctl(spi_fildes, SPI_IOCTL_LOOPBACK_TEST, &loopback);

	if (result < SUCCESS) {
		LOG_ERR("error: unable to activate spi loopback mode");
		goto exit;
	}

	/* set bus mode, don't goto exit for downward compatible */
	bus_mode.eClockPolarity = SPI_CLOCK_IDLE_HIGH;
	bus_mode.eShiftMode = SPI_OUTPUT_FIRST;
	result = ioctl(spi_fildes, SPI_IOCTL_SET_SPI_MODE, &bus_mode);
	if (result < SUCCESS)
	{
		LOG_ERR("error: unable to set bus mode");
	}

	read_write.read_buffer = &read_data_buffer[0];
	read_write.read_buffer_length = sizeof(read_data_buffer);
	read_write.write_buffer = &write_data_buffer[0];
	read_write.write_buffer_length = sizeof(write_data_buffer);
	result = ioctl(spi_fildes, SPI_IOCTL_RDWR, &read_write);

	if (result == SUCCESS) {
		LOG_ERR("error: SPI_IOCTL_RDWR transfer overly large data should "
			"have failed but didn't. ");
		goto exit;
	}

	result = SUCCESS;
	LOG_DEBUG("SPI exceed max write length test passed");

exit:

	if (spi_fildes > SUCCESS) {
		close(spi_fildes);
	}

	return result;
}

#define MPU9250_REG_WHOAMI		 117

int dspal_tester_spi_whoami_test(void)
{
	int spi_fildes = SUCCESS;
	int result = SUCCESS;
	uint8_t write_data_buffer[DSPAL_SPI_TRANSMIT_BUFFER_LENGTH + 1];
	uint8_t read_data_buffer[DSPAL_SPI_RECEIVE_BUFFER_LENGTH + 1];
	struct dspal_spi_ioctl_loopback loopback;
	struct dspal_spi_ioctl_read_write read_write;
	struct dspal_spi_ioctl_set_spi_mode bus_mode;

	LOG_DEBUG("testing spi open for: %s", SPI_DEVICE_PATH);
	spi_fildes = open(SPI_DEVICE_PATH, 0);

	if (spi_fildes < SUCCESS) {
		LOG_ERR("error: failed to open spi device path: %s", SPI_DEVICE_PATH);
		result = ERROR;
		goto exit;
	}

	int retry = 0;
	uint8_t b = 0;
	while (retry < 10)
	{
	   // get version (expecting 0x71 for the 9250)
	   mpu_spi_get_reg(spi_fildes, MPU9250_REG_WHOAMI, &b);
	   if ((b == 0x70) || (b == 0x71)) {
		  break;
	   }
	   retry++;
	}

	if (retry >= 10)
		result = ERROR;


exit:

	if (spi_fildes > SUCCESS) {
		close(spi_fildes);
	}

	return result;
}

/**
 * Main entry point for the SPI automated test.
 * @return
 * - ERROR: Indicates that the test has failed.
 * - SUCCESS: Test has passed
 */
int dspal_tester_spi_test(void)
{
	int result;

	LOG_INFO("beginning spi loopback test");

	if ((result = dspal_tester_spi_loopback_test()) < SUCCESS) {
		LOG_ERR("error: spi loopback test failed: %d", result);
		return result;
	}

	LOG_INFO("beginning spi exceed max write length test");

	if ((result = dspal_tester_spi_exceed_max_length_test()) < SUCCESS) {
		LOG_ERR("error: spi exceed max write length test failed: %d", result);
		return result;
	}

// This test is disabled for the ADSP since it causes conflicts with the serial I/O test
// when executed multiple times in succession.  This is most likely
// related to a known issue with the serial driver which must remain
// open even after the close function is executed.
#if defined(DSP_TYPE_SLPI)
	LOG_INFO("beginning whoami test");
	if ((result = dspal_tester_spi_whoami_test()) < SUCCESS) {
		LOG_ERR("error: spi whoami test failed: %d", result);
		return result;
	}
#endif
	return SUCCESS;
}
