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

#pragma once

#include <perf/perf_counter.h>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <drivers/device/Device.hpp>

/* in 16-bit sampling mode the mag resolution is 1.5 milli Gauss per bit */
static constexpr float ICM20948_MAG_RANGE_GA{1.5e-3f};

#define ICM20948_AK09916_SAMPLE_RATE 100

#define AK09916_I2C_ADDR         0x0C
#define AK09916_DEVICE_ID        0x48

#define AK09916REG_WIA           0x00
#define AK09916REG_CNTL1         0x0A
#define AK09916REG_ASAX          0x10

#define AK09916_SINGLE_MEAS_MODE 0x01
#define AK09916_CONTINUOUS_MODE1 0x02
#define AK09916_CONTINUOUS_MODE2 0x06
#define AK09916_POWERDOWN_MODE   0x00
#define AK09916_SELFTEST_MODE    0x08
#define AK09916_FUZE_MODE        0x0F
#define AK09916_16BIT_ADC        0x10
#define AK09916_14BIT_ADC        0x00
#define AK09916_RESET            0x01
#define AK09916_HOFL             0x08

/* ak09916 deviating register addresses and bit definitions */

#define AK09916_DEVICE_ID_A		0x48	// same as AK09916
#define AK09916_DEVICE_ID_B		0x09	// additional ID byte ("INFO" on AK9063 without content specification.)

#define AK09916REG_HXL        0x11
#define AK09916REG_HXH        0x12
#define AK09916REG_HYL        0x13
#define AK09916REG_HYH        0x14
#define AK09916REG_HZL        0x15
#define AK09916REG_HZH        0x16
#define AK09916REG_ST1        0x10
#define AK09916REG_ST2        0x18
#define AK09916REG_CNTL2          0x31
#define AK09916REG_CNTL3          0x32


#define AK09916_CNTL2_POWERDOWN_MODE            0x00
#define AK09916_CNTL2_SINGLE_MODE               0x01 /* default */
#define AK09916_CNTL2_CONTINOUS_MODE_10HZ       0x02
#define AK09916_CNTL2_CONTINOUS_MODE_20HZ       0x04
#define AK09916_CNTL2_CONTINOUS_MODE_50HZ       0x06
#define AK09916_CNTL2_CONTINOUS_MODE_100HZ      0x08
#define AK09916_CNTL2_SELFTEST_MODE             0x10
#define AK09916_CNTL3_SRST                      0x01
#define AK09916_ST1_DRDY                        0x01
#define AK09916_ST1_DOR                         0x02

class ICM20948;

#pragma pack(push, 1)
struct ak09916_regs {
	uint8_t st1;
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t tmps;
	uint8_t st2;
};
#pragma pack(pop)

extern device::Device *AK09916_I2C_interface(int bus, int bus_frequency);

/**
 * Helper class implementing the magnetometer driver node.
 */
class ICM20948_mag
{
public:
	ICM20948_mag(ICM20948 *parent, device::Device *interface, enum Rotation rotation);
	~ICM20948_mag();

	void set_passthrough(uint8_t reg, uint8_t size, uint8_t *out = NULL);
	void passthrough_read(uint8_t reg, uint8_t *buf, uint8_t size);
	void passthrough_write(uint8_t reg, uint8_t val);
	void read_block(uint8_t reg, uint8_t *val, uint8_t count);

	int ak09916_reset();
	int ak09916_setup();
	int ak09916_setup_master_i2c();
	bool ak09916_check_id(uint8_t &id);
	bool ak09916_read_adjustments();

	void print_status() { _px4_mag.print_status(); }

protected:
	device::Device			*_interface;

	friend class ICM20948;

	void measure();
	void _measure(hrt_abstime timestamp_sample, ak09916_regs data);

	uint8_t read_reg(unsigned reg);
	void write_reg(unsigned reg, uint8_t value);

	bool is_passthrough() { return _interface == nullptr; }

private:
	PX4Magnetometer		_px4_mag;

	ICM20948 *_parent;

	bool _mag_reading_data{false};

	perf_counter_t _mag_overruns;
	perf_counter_t _mag_overflows;
	perf_counter_t _mag_errors;

};
