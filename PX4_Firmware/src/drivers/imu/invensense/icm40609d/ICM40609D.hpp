/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file ICM40609D.hpp
 *
 * Driver for the Invensense ICM40609D connected via SPI.
 *
 */

#pragma once

#include "InvenSense_ICM40609D_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/ecl/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace InvenSense_ICM40609D;

class ICM40609D : public device::SPI, public I2CSPIDriver<ICM40609D>
{
public:
	ICM40609D(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation, int bus_frequency,
		  spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio);
	~ICM40609D() override;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

	void Start();
	bool Reset();

protected:
	void custom_method(const BusCLIArguments &cli) override;
	void exit_and_cleanup() override;
private:
	// Sensor Configuration
	static constexpr float FIFO_SAMPLE_DT{125.f};
	static constexpr uint32_t SAMPLES_PER_TRANSFER{1}; // ensure at least 1 new accel sample per transfer
	static constexpr float GYRO_RATE{1000000 / FIFO_SAMPLE_DT}; // 8 kHz gyro
	static constexpr float ACCEL_RATE{GYRO_RATE};               // 8 kHz accel

	static constexpr uint32_t FIFO_MAX_SAMPLES{math::min(FIFO::SIZE / sizeof(FIFO::DATA), sizeof(PX4Gyroscope::FIFOSample::x) / sizeof(PX4Gyroscope::FIFOSample::x[0]))};

	// Transfer data
	struct FIFOTransferBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::BANK_0::INT_STATUS) | DIR_READ};
		uint8_t INT_STATUS{0};
		uint8_t FIFO_COUNTH{0};
		uint8_t FIFO_COUNTL{0};
		FIFO::DATA f[FIFO_MAX_SAMPLES] {};
	};
	// ensure no struct padding
	static_assert(sizeof(FIFOTransferBuffer) == (4 + FIFO_MAX_SAMPLES *sizeof(FIFO::DATA)));

	struct register_bank0_config_t {
		Register::BANK_0 reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	int probe() override;

	bool Configure();
	void ConfigureAccel();
	void ConfigureGyro();
	void ConfigureSampleRate(int sample_rate);
	void ConfigureFIFOWatermark(uint8_t samples);

	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();

	bool RegisterCheck(const register_bank0_config_t &reg_cfg, bool notify = false);

	uint8_t RegisterRead(Register::BANK_0 reg);
	void RegisterWrite(Register::BANK_0 reg, uint8_t value);
	void RegisterSetAndClearBits(Register::BANK_0 reg, uint8_t setbits, uint8_t clearbits);
	void RegisterSetBits(Register::BANK_0 reg, uint8_t setbits) { RegisterSetAndClearBits(reg, setbits, 0); }
	void RegisterClearBits(Register::BANK_0 reg, uint8_t clearbits) { RegisterSetAndClearBits(reg, 0, clearbits); }

	uint16_t FIFOReadCount();
	bool FIFORead(const hrt_abstime &timestamp_sample, uint16_t samples);
	void FIFOReset();

	void ProcessAccel(const hrt_abstime &timestamp_sample, const FIFOTransferBuffer &buffer, const uint8_t samples);
	void ProcessGyro(const hrt_abstime &timestamp_sample, const FIFOTransferBuffer &buffer, const uint8_t samples);
	void UpdateTemperature();

	const spi_drdy_gpio_t _drdy_gpio;

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	perf_counter_t _transfer_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": transfer")};
	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _fifo_empty_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO empty")};
	perf_counter_t _fifo_overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO overflow")};
	perf_counter_t _fifo_reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO reset")};
	perf_counter_t _drdy_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": DRDY interval")};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	hrt_abstime _fifo_watermark_interrupt_timestamp{0};
	hrt_abstime _temperature_update_timestamp{0};

	px4::atomic<uint8_t> _fifo_read_samples{0};
	bool _data_ready_interrupt_enabled{false};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		FIFO_READ,
	};

	STATE _state{STATE::RESET};

	uint16_t _fifo_empty_interval_us{1000}; // default 1000 us / 1000 Hz transfer interval
	uint8_t _fifo_gyro_samples{static_cast<uint8_t>(_fifo_empty_interval_us / (1000000 / GYRO_RATE))};
	uint8_t _fifo_accel_samples{static_cast<uint8_t>(_fifo_empty_interval_us / (1000000 / ACCEL_RATE))};

	uint8_t _checked_register_bank0{0};
	static constexpr uint8_t size_register_bank0_cfg{10};
	register_bank0_config_t _register_bank0_cfg[size_register_bank0_cfg] {
		// Register                        | Set bits, Clear bits
		{ Register::BANK_0::INT_CONFIG,    INT_CONFIG_BIT::INT1_MODE | INT_CONFIG_BIT::INT1_DRIVE_CIRCUIT, INT_CONFIG_BIT::INT1_POLARITY },
		{ Register::BANK_0::FIFO_CONFIG,   FIFO_CONFIG_BIT::FIFO_MODE_STOP_ON_FULL, 0 },
		{ Register::BANK_0::PWR_MGMT0,     PWR_MGMT0_BIT::GYRO_MODE_LOW_NOISE | PWR_MGMT0_BIT::ACCEL_MODE_LOW_NOISE, 0 },
		{ Register::BANK_0::GYRO_CONFIG0,  GYRO_CONFIG0_BIT::GYRO_ODR_8kHz, Bit7 | Bit6 | Bit5 | Bit3 | Bit2 },
		{ Register::BANK_0::ACCEL_CONFIG0, ACCEL_CONFIG0_BIT::ACCEL_ODR_8kHz, Bit7 | Bit6 | Bit5 | Bit3 | Bit2 },
		{ Register::BANK_0::FIFO_CONFIG1,  FIFO_CONFIG1_BIT::FIFO_WM_GT_TH | FIFO_CONFIG1_BIT::FIFO_GYRO_EN | FIFO_CONFIG1_BIT::FIFO_ACCEL_EN, FIFO_CONFIG1_BIT::FIFO_TEMP_EN },
		{ Register::BANK_0::FIFO_CONFIG2,  0, 0 }, // FIFO_WM[7:0] set at runtime
		{ Register::BANK_0::FIFO_CONFIG3,  0, 0 }, // FIFO_WM[11:8] set at runtime
		{ Register::BANK_0::INT_CONFIG0,   INT_CONFIG0_BIT::CLEAR_ON_FIFO_READ, 0 },
		{ Register::BANK_0::INT_SOURCE0,   INT_SOURCE0_BIT::FIFO_THS_INT1_EN, 0 },
	};
};
