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

/**
 * @file main.cpp
 *
 * Driver for the Invensense icm20948 connected via I2C or SPI.
 */

#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>

#include "icm20948.h"

void
ICM20948::print_usage()
{
	PRINT_MODULE_USAGE_NAME("icm20948", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 35, "Rotation", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *ICM20948::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					int runtime_instance)
{
	device::Device *interface = nullptr;
	device::Device *mag_interface = nullptr;

	if (iterator.busType() == BOARD_I2C_BUS) {
		interface = ICM20948_I2C_interface(iterator.bus(), PX4_I2C_EXT_ICM20948_1, cli.bus_frequency);
		// For i2c interfaces, connect to the magnetometer directly
		mag_interface = AK09916_I2C_interface(iterator.bus(), cli.bus_frequency);

	} else if (iterator.busType() == BOARD_SPI_BUS) {
		interface = ICM20948_SPI_interface(iterator.bus(), iterator.devid(), cli.bus_frequency, cli.spi_mode);
	}

	if (interface == nullptr) {
		PX4_ERR("alloc failed");
		delete mag_interface;
		return nullptr;
	}

	if (interface->init() != OK) {
		delete interface;
		delete mag_interface;
		PX4_DEBUG("no device on bus %i (devid 0x%x)", iterator.bus(), iterator.devid());
		return nullptr;
	}

	ICM20948 *dev = new ICM20948(interface, mag_interface, cli.rotation, iterator.configuredBusOption(), iterator.bus());

	if (dev == nullptr) {
		delete interface;
		delete mag_interface;
		return nullptr;
	}

	if (OK != dev->init()) {
		delete dev;
		return nullptr;
	}

	return dev;
}

extern "C" int
icm20948_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = ICM20948;
	BusCLIArguments cli{true, true};
	cli.default_spi_frequency = 1000 * 1000; // low speed freq
	cli.default_i2c_frequency = 400000;

	while ((ch = cli.getopt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			cli.rotation = (enum Rotation)atoi(cli.optarg());
			break;
		}
	}

	const char *verb = cli.optarg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_IMU_DEVTYPE_ICM20948);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
