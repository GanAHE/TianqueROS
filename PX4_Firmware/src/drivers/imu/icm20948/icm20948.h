/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/ecl/geo/geo.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/systemlib/conversions.h>
#include <lib/systemlib/px4_macros.h>

#include "ICM20948_mag.h"


// ICM20948 registers
#define MPUREG_WHOAMI			0x75
#define MPUREG_SMPLRT_DIV		0x19
#define MPUREG_CONFIG			0x1A
#define MPUREG_GYRO_CONFIG		0x1B
#define MPUREG_ACCEL_CONFIG		0x1C
#define MPUREG_ACCEL_CONFIG2		0x1D
#define MPUREG_LPACCEL_ODR		0x1E
#define MPUREG_WOM_THRESH		0x1F
#define MPUREG_FIFO_EN			0x23
#define MPUREG_I2C_MST_CTRL		0x24
#define MPUREG_I2C_SLV0_ADDR		0x25
#define MPUREG_I2C_SLV0_REG		0x26
#define MPUREG_I2C_SLV0_CTRL		0x27
#define MPUREG_I2C_SLV1_ADDR		0x28
#define MPUREG_I2C_SLV1_REG		0x29
#define MPUREG_I2C_SLV1_CTRL		0x2A
#define MPUREG_I2C_SLV2_ADDR		0x2B
#define MPUREG_I2C_SLV2_REG		0x2C
#define MPUREG_I2C_SLV2_CTRL		0x2D
#define MPUREG_I2C_SLV3_ADDR		0x2E
#define MPUREG_I2C_SLV3_REG		0x2F
#define MPUREG_I2C_SLV3_CTRL		0x30
#define MPUREG_I2C_SLV4_ADDR		0x31
#define MPUREG_I2C_SLV4_REG		0x32
#define MPUREG_I2C_SLV4_DO		0x33
#define MPUREG_I2C_SLV4_CTRL		0x34
#define MPUREG_I2C_SLV4_DI		0x35
#define MPUREG_I2C_MST_STATUS		0x36
#define MPUREG_INT_PIN_CFG		0x37
#define MPUREG_INT_ENABLE		0x38
#define MPUREG_INT_STATUS		0x3A
#define MPUREG_ACCEL_XOUT_H		0x3B
#define MPUREG_ACCEL_XOUT_L		0x3C
#define MPUREG_ACCEL_YOUT_H		0x3D
#define MPUREG_ACCEL_YOUT_L		0x3E
#define MPUREG_ACCEL_ZOUT_H		0x3F
#define MPUREG_ACCEL_ZOUT_L		0x40
#define MPUREG_TEMP_OUT_H		0x41
#define MPUREG_TEMP_OUT_L		0x42
#define MPUREG_GYRO_XOUT_H		0x43
#define MPUREG_GYRO_XOUT_L		0x44
#define MPUREG_GYRO_YOUT_H		0x45
#define MPUREG_GYRO_YOUT_L		0x46
#define MPUREG_GYRO_ZOUT_H		0x47
#define MPUREG_GYRO_ZOUT_L		0x48
#define MPUREG_EXT_SENS_DATA_00		0x49
#define MPUREG_I2C_SLV0_D0		0x63
#define MPUREG_I2C_SLV1_D0		0x64
#define MPUREG_I2C_SLV2_D0		0x65
#define MPUREG_I2C_SLV3_D0		0x66
#define MPUREG_I2C_MST_DELAY_CTRL	0x67
#define MPUREG_SIGNAL_PATH_RESET	0x68
#define MPUREG_MOT_DETECT_CTRL		0x69
#define MPUREG_USER_CTRL		0x6A
#define MPUREG_PWR_MGMT_1		0x6B
#define MPUREG_PWR_MGMT_2		0x6C
#define MPUREG_FIFO_COUNTH		0x72
#define MPUREG_FIFO_COUNTL		0x73
#define MPUREG_FIFO_R_W			0x74

// Configuration bits ICM20948
#define BIT_SLEEP			0x40
#define BIT_H_RESET			0x80
#define MPU_CLK_SEL_AUTO		0x01

#define BITS_GYRO_ST_X			0x80
#define BITS_GYRO_ST_Y			0x40
#define BITS_GYRO_ST_Z			0x20
#define BITS_FS_250DPS			0x00
#define BITS_FS_500DPS			0x08
#define BITS_FS_1000DPS			0x10
#define BITS_FS_2000DPS			0x18
#define BITS_FS_MASK			0x18

#define BITS_DLPF_CFG_250HZ		0x00
#define BITS_DLPF_CFG_184HZ		0x01
#define BITS_DLPF_CFG_92HZ		0x02
#define BITS_DLPF_CFG_41HZ		0x03
#define BITS_DLPF_CFG_20HZ		0x04
#define BITS_DLPF_CFG_10HZ		0x05
#define BITS_DLPF_CFG_5HZ		0x06
#define BITS_DLPF_CFG_3600HZ		0x07
#define BITS_DLPF_CFG_MASK		0x07

#define BITS_ACCEL_CONFIG2_41HZ		0x03

#define BIT_RAW_RDY_EN			0x01
#define BIT_INT_ANYRD_2CLEAR		0x10
#define BIT_INT_BYPASS_EN		0x02

#define BIT_I2C_READ_FLAG           0x80

#define BIT_I2C_SLV0_NACK           0x01
#define BIT_I2C_FIFO_EN             0x40
#define BIT_I2C_MST_EN              0x20
#define BIT_I2C_IF_DIS              0x10
#define BIT_FIFO_RST                0x04
#define BIT_I2C_MST_RST             0x02
#define BIT_SIG_COND_RST            0x01

#define BIT_I2C_SLV0_EN             0x80
#define BIT_I2C_SLV0_BYTE_SW        0x40
#define BIT_I2C_SLV0_REG_DIS        0x20
#define BIT_I2C_SLV0_REG_GRP        0x10

#define BIT_I2C_MST_MULT_MST_EN     0x80
#define BIT_I2C_MST_WAIT_FOR_ES     0x40
#define BIT_I2C_MST_SLV_3_FIFO_EN   0x20
#define BIT_I2C_MST_P_NSR           0x10
#define BITS_I2C_MST_CLOCK_258HZ    0x08
#define BITS_I2C_MST_CLOCK_400HZ    0x0D

#define BIT_I2C_SLV0_DLY_EN         0x01
#define BIT_I2C_SLV1_DLY_EN         0x02
#define BIT_I2C_SLV2_DLY_EN         0x04
#define BIT_I2C_SLV3_DLY_EN         0x08

#define ICM_WHOAMI_20948            0xEA

#define ICM20948_ACCEL_DEFAULT_RATE	1000
#define ICM20948_ACCEL_MAX_OUTPUT_RATE			280
#define ICM20948_ACCEL_DEFAULT_DRIVER_FILTER_FREQ 30
#define ICM20948_GYRO_DEFAULT_RATE	1000
/* rates need to be the same between accel and gyro */
#define ICM20948_GYRO_MAX_OUTPUT_RATE			ICM20948_ACCEL_MAX_OUTPUT_RATE
#define ICM20948_GYRO_DEFAULT_DRIVER_FILTER_FREQ 30

#define ICM20948_DEFAULT_ONCHIP_FILTER_FREQ	92


// ICM20948 registers and data

/*
 * ICM20948 I2C address LSB can be switched by the chip's AD0 pin, thus is device dependent.
 * Noting this down for now. Here GPS uses 0x69. To support a device implementing the second
 * address, probably an additional MPU_DEVICE_TYPE is the way to go.
 */
#define PX4_I2C_EXT_ICM20948_0			0x68
#define PX4_I2C_EXT_ICM20948_1			0x69

/*
 * ICM20948 uses register banks. Register 127 (0x7F) is used to switch between 4 banks.
 * There's room in the upper address byte below the port speed setting to code in the
 * used bank. This is a bit more efficient, already in use for the speed setting and more
 * in one place than a solution with a lookup table for address/bank pairs.
 */

#define BANK0	0x0000
#define BANK1	0x0100
#define BANK2	0x0200
#define BANK3	0x0300

#define BANK_REG_MASK	0x0300
#define REG_BANK(r) 			(((r) & BANK_REG_MASK)>>4)
#define REG_ADDRESS(r)			((r) & ~BANK_REG_MASK)

#define ICMREG_20948_BANK_SEL 0x7F

#define	ICMREG_20948_WHOAMI					(0x00 | BANK0)
#define ICMREG_20948_USER_CTRL				(0x03 | BANK0)
#define ICMREG_20948_PWR_MGMT_1				(0x06 | BANK0)
#define ICMREG_20948_PWR_MGMT_2				(0x07 | BANK0)
#define ICMREG_20948_INT_PIN_CFG			(0x0F | BANK0)
#define ICMREG_20948_INT_ENABLE				(0x10 | BANK0)
#define ICMREG_20948_INT_ENABLE_1			(0x11 | BANK0)
#define ICMREG_20948_ACCEL_XOUT_H			(0x2D | BANK0)
#define ICMREG_20948_INT_ENABLE_2			(0x12 | BANK0)
#define ICMREG_20948_INT_ENABLE_3			(0x13 | BANK0)
#define ICMREG_20948_EXT_SLV_SENS_DATA_00	(0x3B | BANK0)
#define ICMREG_20948_GYRO_SMPLRT_DIV		(0x00 | BANK2)
#define ICMREG_20948_GYRO_CONFIG_1			(0x01 | BANK2)
#define ICMREG_20948_GYRO_CONFIG_2			(0x02 | BANK2)
#define ICMREG_20948_ACCEL_SMPLRT_DIV_1		(0x10 | BANK2)
#define ICMREG_20948_ACCEL_SMPLRT_DIV_2		(0x11 | BANK2)
#define ICMREG_20948_ACCEL_CONFIG			(0x14 | BANK2)
#define ICMREG_20948_ACCEL_CONFIG_2			(0x15 | BANK2)
#define ICMREG_20948_I2C_MST_CTRL			(0x01 | BANK3)
#define ICMREG_20948_I2C_SLV0_ADDR			(0x03 | BANK3)
#define ICMREG_20948_I2C_SLV0_REG			(0x04 | BANK3)
#define ICMREG_20948_I2C_SLV0_CTRL			(0x05 | BANK3)
#define ICMREG_20948_I2C_SLV0_DO			(0x06 | BANK3)



/*
* ICM20948 register bits
* Most of the regiser set values from ICM20948 have the same
* meaning on ICM20948. The exceptions and values not already
* defined for ICM20948 are defined below
*/
#define ICM_BIT_PWR_MGMT_1_ENABLE       	0x00
#define ICM_BIT_USER_CTRL_I2C_MST_DISABLE   0x00

#define ICM_BITS_GYRO_DLPF_CFG_197HZ		0x01
#define ICM_BITS_GYRO_DLPF_CFG_151HZ		0x09
#define ICM_BITS_GYRO_DLPF_CFG_119HZ		0x11
#define ICM_BITS_GYRO_DLPF_CFG_51HZ			0x19
#define ICM_BITS_GYRO_DLPF_CFG_23HZ			0x21
#define ICM_BITS_GYRO_DLPF_CFG_11HZ			0x29
#define ICM_BITS_GYRO_DLPF_CFG_5HZ			0x31
#define ICM_BITS_GYRO_DLPF_CFG_361HZ		0x39
#define ICM_BITS_GYRO_DLPF_CFG_MASK			0x39

#define ICM_BITS_GYRO_FS_SEL_250DPS			0x00
#define ICM_BITS_GYRO_FS_SEL_500DPS			0x02
#define ICM_BITS_GYRO_FS_SEL_1000DPS		0x04
#define ICM_BITS_GYRO_FS_SEL_2000DPS		0x06
#define ICM_BITS_GYRO_FS_SEL_MASK			0x06

#define ICM_BITS_ACCEL_DLPF_CFG_246HZ		0x09
#define ICM_BITS_ACCEL_DLPF_CFG_111HZ		0x11
#define ICM_BITS_ACCEL_DLPF_CFG_50HZ		0x19
#define ICM_BITS_ACCEL_DLPF_CFG_23HZ		0x21
#define ICM_BITS_ACCEL_DLPF_CFG_11HZ		0x29
#define ICM_BITS_ACCEL_DLPF_CFG_5HZ			0x31
#define ICM_BITS_ACCEL_DLPF_CFG_473HZ		0x39
#define ICM_BITS_ACCEL_DLPF_CFG_MASK		0x39

#define ICM_BITS_ACCEL_FS_SEL_250DPS		0x00
#define ICM_BITS_ACCEL_FS_SEL_500DPS		0x02
#define ICM_BITS_ACCEL_FS_SEL_1000DPS		0x04
#define ICM_BITS_ACCEL_FS_SEL_2000DPS		0x06
#define ICM_BITS_ACCEL_FS_SEL_MASK			0x06

#define ICM_BITS_DEC3_CFG_4					0x00
#define ICM_BITS_DEC3_CFG_8					0x01
#define ICM_BITS_DEC3_CFG_16				0x10
#define ICM_BITS_DEC3_CFG_32				0x11
#define ICM_BITS_DEC3_CFG_MASK				0x11

#define ICM_BITS_I2C_MST_CLOCK_370KHZ    	0x00
#define ICM_BITS_I2C_MST_CLOCK_400HZ    	0x07	// recommended by datasheet for 400kHz target clock


#define MPU_OR_ICM(m,i)					((_whoami==ICM_WHOAMI_20948) ? i : m)

#pragma pack(push, 1)
/**
 * Report conversation within the mpu, including command byte and
 * interrupt status.
 */
struct ICMReport {
	uint8_t		accel_x[2];
	uint8_t		accel_y[2];
	uint8_t		accel_z[2];
	uint8_t		gyro_x[2];
	uint8_t		gyro_y[2];
	uint8_t		gyro_z[2];
	uint8_t		temp[2];
	struct ak09916_regs mag;
};
#pragma pack(pop)


#pragma pack(push, 1)
/**
 * Report conversation within the mpu, including command byte and
 * interrupt status.
 */
struct MPUReport {
	uint8_t		cmd;
	uint8_t		status;
	uint8_t		accel_x[2];
	uint8_t		accel_y[2];
	uint8_t		accel_z[2];
	uint8_t		temp[2];
	uint8_t		gyro_x[2];
	uint8_t		gyro_y[2];
	uint8_t		gyro_z[2];
	struct ak09916_regs mag;
};
#pragma pack(pop)

/*
  The ICM20948 can only handle high bus speeds on the sensor and
  interrupt status registers. All other registers have a maximum 1MHz
  Communication with all registers of the device is performed using either
  I2C at 400kHz or SPI at 1MHz. For applications requiring faster communications,
  the sensor and interrupt registers may be read using SPI at 20MHz
 */
#define ICM20948_LOW_BUS_SPEED				0
#define ICM20948_HIGH_BUS_SPEED				0x8000
#define ICM20948_REG_MASK					0x00FF
#  define ICM20948_IS_HIGH_SPEED(r) 			((r) & ICM20948_HIGH_BUS_SPEED)
#  define ICM20948_REG(r) 					((r) & ICM20948_REG_MASK)
#  define ICM20948_SET_SPEED(r, s) 			((r)|(s))
#  define ICM20948_HIGH_SPEED_OP(r) 			ICM20948_SET_SPEED((r), ICM20948_HIGH_BUS_SPEED)
#  define ICM20948_LOW_SPEED_OP(r)			((r) &~ICM20948_HIGH_BUS_SPEED)

/* interface factories */
extern device::Device *ICM20948_SPI_interface(int bus, uint32_t devid, int bus_frequency, spi_mode_e spi_mode);
extern device::Device *ICM20948_I2C_interface(int bus, uint32_t address, int bus_frequency);

class ICM20948_mag;

class ICM20948 : public I2CSPIDriver<ICM20948>
{
public:
	ICM20948(device::Device *interface, device::Device *mag_interface, enum Rotation rotation, I2CSPIBusOption bus_option,
		 int bus);
	virtual ~ICM20948();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	int		init();

	void			print_status() override;

	void RunImpl();

protected:
	device::Device *_interface;
	uint8_t			_whoami{0};	/** whoami result */

	int		probe();

	friend class ICM20948_mag;

private:

	PX4Accelerometer	_px4_accel;
	PX4Gyroscope		_px4_gyro;

	ICM20948_mag		_mag;
	uint8_t 		_selected_bank{0};		/* Remember selected memory bank to avoid polling / setting on each read/write */

	unsigned		_call_interval{1000};

	unsigned		_dlpf_freq{0};
	unsigned		_dlpf_freq_icm_gyro{0};
	unsigned		_dlpf_freq_icm_accel{0};

	unsigned		_sample_rate{1000};

	perf_counter_t		_sample_perf;
	perf_counter_t		_interval_perf;
	perf_counter_t		_bad_transfers;
	perf_counter_t		_bad_registers;
	perf_counter_t		_good_transfers;
	perf_counter_t		_duplicates;

	uint8_t			_register_wait{0};
	uint64_t		_reset_wait{0};

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset

	static constexpr int ICM20948_NUM_CHECKED_REGISTERS{15};
	static const uint16_t	_icm20948_checked_registers[ICM20948_NUM_CHECKED_REGISTERS];

	const uint16_t			*_checked_registers{nullptr};

	uint8_t					_checked_values[ICM20948_NUM_CHECKED_REGISTERS] {};
	uint8_t					_checked_bad[ICM20948_NUM_CHECKED_REGISTERS] {};
	unsigned				_checked_next{0};
	unsigned				_num_checked_registers{0};


	// last temperature reading for print_info()
	float			_last_temperature{0.0f};

	bool check_null_data(uint16_t *data, uint8_t size);
	bool check_duplicate(uint8_t *accel_data);

	// keep last accel reading for duplicate detection
	uint8_t			_last_accel_data[6] {};
	bool			_got_duplicate{false};

	void			start();
	int			reset();

	/**
	 * Resets the main chip (excluding the magnetometer if any).
	 */
	int			reset_mpu();

	/**
	 * Select a register bank in ICM20948
	 *
	 * Only actually switches if the remembered bank is different from the
	 * requested one
	 *
	 * @param		The index of the register bank to switch to (0-3)
	 * @return		Error code
	 */
	int				select_register_bank(uint8_t bank);

	/**
	 * Read a register from the mpu
	 *
	 * @param		The register to read.
	* @param       The bus speed to read with.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg, uint32_t speed = ICM20948_LOW_BUS_SPEED);
	uint16_t		read_reg16(unsigned reg);


	/**
	 * Read a register range from the mpu
	 *
	 * @param       The start address to read from.
	 * @param       The bus speed to read with.
	 * @param       The address of the target data buffer.
	 * @param       The count of bytes to be read.
	 * @return      The value that was read.
	 */
	uint8_t read_reg_range(unsigned start_reg, uint32_t speed, uint8_t *buf, uint16_t count);

	/**
	 * Write a register in the mpu
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);

	/**
	 * Modify a register in the mpu
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Write a register in the mpu, updating _checked_values
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_checked_reg(unsigned reg, uint8_t value);

	/**
	 * Modify a checked register in the mpu
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_checked_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Set the mpu measurement range.
	 *
	 * @param max_g		The maximum G value the range must support.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			set_accel_range(unsigned max_g);

	/**
	 * Swap a 16-bit value read from the mpu to native byte order.
	 */
	uint16_t		swap16(uint16_t val) { return (val >> 8) | (val << 8);	}

	/**
	 * Get the internal / external state
	 *
	 * @return true if the sensor is not on the main MCU board
	 */
	bool			is_external() { return _interface->external(); }

	/*
	  set low pass filter frequency
	 */
	void _set_dlpf_filter(uint16_t frequency_hz);

	/*
	  set sample rate (approximate) - 1kHz to 5Hz
	*/
	void _set_sample_rate(unsigned desired_sample_rate_hz);

	/*
	  check that key registers still have the right value
	 */
	void check_registers();
};
