/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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

/**
 * @file mag.cpp
 *
 * Driver for the ak09916 magnetometer within the Invensense icm20948
 *
 * @author Robert Dickenson
 *
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

#include "ICM20948_mag.h"
#include "icm20948.h"

// If interface is non-null, then it will used for interacting with the device.
// Otherwise, it will passthrough the parent ICM20948
ICM20948_mag::ICM20948_mag(ICM20948 *parent, device::Device *interface, enum Rotation rotation) :
	_interface(interface),
	_px4_mag(parent->_interface->get_device_id(), (parent->_interface->external() ? ORB_PRIO_MAX : ORB_PRIO_HIGH),
		 rotation),
	_parent(parent),
	_mag_overruns(perf_alloc(PC_COUNT, MODULE_NAME": mag overruns")),
	_mag_overflows(perf_alloc(PC_COUNT, MODULE_NAME": mag overflows")),
	_mag_errors(perf_alloc(PC_COUNT, MODULE_NAME": mag errors"))
{
	_px4_mag.set_device_type(DRV_MAG_DEVTYPE_AK09916);
	_px4_mag.set_external(_parent->is_external());
	_px4_mag.set_scale(ICM20948_MAG_RANGE_GA);
}

ICM20948_mag::~ICM20948_mag()
{
	perf_free(_mag_overruns);
	perf_free(_mag_overflows);
	perf_free(_mag_errors);
}

void
ICM20948_mag::measure()
{
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	uint8_t st1 = 0;
	int ret = _interface->read(AK09916REG_ST1, &st1, sizeof(st1));

	if (ret != OK) {
		_px4_mag.set_error_count(perf_event_count(_mag_errors));
		return;
	}

	/* Check if data ready is set.
	 * This is not described to be set in continuous mode according to the
	 * MPU9250 datasheet. However, the datasheet of the 8963 recommends to
	 * check data ready before doing the read and before triggering the
	 * next measurement by reading ST2. */
	if (!(st1 & AK09916_ST1_DRDY)) {
		return;
	}

	/* Monitor if data overrun flag is ever set. */
	if (st1 & 0x02) {
		perf_count(_mag_overruns);
	}

	ak09916_regs data{};
	ret = _interface->read(AK09916REG_ST1, &data, sizeof(data));

	if (ret != OK) {
		_px4_mag.set_error_count(perf_event_count(_mag_errors));
		return;
	}

	/* Monitor magnetic sensor overflow flag. */
	if (data.st2 & 0x08) {
		perf_count(_mag_overflows);
	}

	_measure(timestamp_sample, data);
}

void
ICM20948_mag::_measure(hrt_abstime timestamp_sample, ak09916_regs data)
{
	/* Check if data ready is set.
	 * This is not described to be set in continuous mode according to the
	 * MPU9250 datasheet. However, the datasheet of the 8963 recommends to
	 * check data ready before doing the read and before triggering the
	 * next measurement by reading ST2.
	 *
	 * If _measure is used in passthrough mode, all the data is already
	 * fetched, however, we should still not use the data if the data ready
	 * is not set. This has lead to intermittent spikes when the data was
	 * being updated while getting read.
	 */
	if (!(data.st1 & AK09916_ST1_DRDY)) {
		return;
	}

	_px4_mag.set_external(_parent->is_external());
	_px4_mag.set_temperature(_parent->_last_temperature);

	/*
	 * Align axes - Keeping consistent with the accel and gyro axes of the ICM20948 here, just aligning the magnetometer to them.
	 */
	_px4_mag.update(timestamp_sample, data.y, data.x, -data.z);
}

void
ICM20948_mag::set_passthrough(uint8_t reg, uint8_t size, uint8_t *out)
{
	uint8_t addr;

	_parent->write_reg(ICMREG_20948_I2C_SLV0_CTRL, 0); // ensure slave r/w is disabled before changing the registers

	if (out) {
		_parent->write_reg(ICMREG_20948_I2C_SLV0_DO, *out);
		addr = AK09916_I2C_ADDR;

	} else {
		addr = AK09916_I2C_ADDR | BIT_I2C_READ_FLAG;
	}

	_parent->write_reg(ICMREG_20948_I2C_SLV0_ADDR, addr);
	_parent->write_reg(ICMREG_20948_I2C_SLV0_REG,  reg);
	_parent->write_reg(ICMREG_20948_I2C_SLV0_CTRL, size | BIT_I2C_SLV0_EN);
}

void
ICM20948_mag::read_block(uint8_t reg, uint8_t *val, uint8_t count)
{
	_parent->_interface->read(reg, val, count);
}

void
ICM20948_mag::passthrough_read(uint8_t reg, uint8_t *buf, uint8_t size)
{
	set_passthrough(reg, size);
	px4_usleep(25 + 25 * size); // wait for the value to be read from slave
	read_block(ICMREG_20948_EXT_SLV_SENS_DATA_00, buf, size);
	_parent->write_reg(ICMREG_20948_I2C_SLV0_CTRL, 0); // disable new reads
}

uint8_t
ICM20948_mag::read_reg(unsigned int reg)
{
	uint8_t buf{};

	if (_interface == nullptr) {
		passthrough_read(reg, &buf, 0x01);

	} else {
		_interface->read(reg, &buf, 1);
	}

	return buf;
}

bool
ICM20948_mag::ak09916_check_id(uint8_t &deviceid)
{
	deviceid = read_reg(AK09916REG_WIA);

	return (AK09916_DEVICE_ID == deviceid);
}

/*
 * 400kHz I2C bus speed = 2.5us per bit = 25us per byte
 */
void
ICM20948_mag::passthrough_write(uint8_t reg, uint8_t val)
{
	set_passthrough(reg, 1, &val);
	px4_usleep(50); // wait for the value to be written to slave
	_parent->write_reg(ICMREG_20948_I2C_SLV0_CTRL, 0); // disable new writes
}

void
ICM20948_mag::write_reg(unsigned reg, uint8_t value)
{
	// general register transfer at low clock speed
	if (_interface == nullptr) {
		passthrough_write(reg, value);

	} else {
		_interface->write(ICM20948_LOW_SPEED_OP(reg), &value, 1);
	}
}

int
ICM20948_mag::ak09916_reset()
{
	// First initialize it to use the bus
	int rv = ak09916_setup();

	if (rv == OK) {
		// Now reset the mag
		write_reg(AK09916REG_CNTL3, AK09916_RESET);

		// Then re-initialize the bus/mag
		rv = ak09916_setup();
	}

	return rv;
}

bool
ICM20948_mag::ak09916_read_adjustments()
{
	uint8_t response[3];
	float ak09916_ASA[3];

	write_reg(AK09916REG_CNTL1, AK09916_FUZE_MODE | AK09916_16BIT_ADC);
	px4_usleep(50);

	if (_interface != nullptr) {
		_interface->read(AK09916REG_ASAX, response, 3);

	} else {
		passthrough_read(AK09916REG_ASAX, response, 3);
	}

	write_reg(AK09916REG_CNTL1, AK09916_POWERDOWN_MODE);

	for (int i = 0; i < 3; i++) {
		if (0 != response[i] && 0xff != response[i]) {
			ak09916_ASA[i] = ((float)(response[i] - 128) / 256.0f) + 1.0f;

		} else {
			return false;
		}
	}

	_px4_mag.set_sensitivity(ak09916_ASA[0], ak09916_ASA[1], ak09916_ASA[2]);

	return true;
}

int
ICM20948_mag::ak09916_setup_master_i2c()
{
	/* When _interface is null we are using SPI and must
	 * use the parent interface to configure the device to act
	 * in master mode (SPI to I2C bridge)
	 */
	if (_interface == nullptr) {
		// ICM20948 -> AK09916
		_parent->modify_checked_reg(ICMREG_20948_USER_CTRL, 0, BIT_I2C_MST_EN);

		// WAIT_FOR_ES does not exist for ICM20948. Not sure how to replace this (or if that is needed)
		_parent->write_reg(ICMREG_20948_I2C_MST_CTRL, BIT_I2C_MST_P_NSR | ICM_BITS_I2C_MST_CLOCK_400HZ);

	} else {
		_parent->modify_checked_reg(ICMREG_20948_USER_CTRL, BIT_I2C_MST_EN, 0);
	}

	return OK;
}
int
ICM20948_mag::ak09916_setup(void)
{
	int retries = 10;

	do {

		ak09916_setup_master_i2c();
		write_reg(AK09916REG_CNTL3, AK09916_RESET);

		uint8_t id = 0;

		if (ak09916_check_id(id)) {
			break;
		}

		retries--;
		PX4_WARN("AK09916: bad id %d retries %d", id, retries);
		_parent->modify_reg(ICMREG_20948_USER_CTRL, 0, BIT_I2C_MST_RST);
		up_udelay(100);
	} while (retries > 0);

	if (retries == 0) {
		PX4_ERR("AK09916: failed to initialize, disabled!");
		_parent->modify_checked_reg(ICMREG_20948_USER_CTRL, BIT_I2C_MST_EN, 0);
		_parent->write_reg(ICMREG_20948_I2C_MST_CTRL, 0);
		return -EIO;
	}

	write_reg(AK09916REG_CNTL2, AK09916_CNTL2_CONTINOUS_MODE_100HZ);

	if (_interface == nullptr) {
		// Configure mpu' I2c Master interface to read ak09916 data into to fifo
		set_passthrough(AK09916REG_ST1, sizeof(ak09916_regs));
	}

	return OK;
}
