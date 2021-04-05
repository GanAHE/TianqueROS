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

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include "voxlpm.hpp"

I2CSPIDriverBase *VOXLPM::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				      int runtime_instance)
{
	VOXLPM *instance = new VOXLPM(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency,
				      (VOXLPM_CH_TYPE)cli.type);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (OK != instance->init()) {
		delete instance;
		return nullptr;
	}

	return instance;
}

void
VOXLPM::print_usage()
{
	PRINT_MODULE_USAGE_NAME_SIMPLE("voxlpm", "driver");

	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAM_STRING('T', "VBATT", "VBATT|P5VDC", "Type", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" int
voxlpm_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = VOXLPM;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;
	cli.type = VOXLPM_CH_TYPE_VBATT;

	while ((ch = cli.getopt(argc, argv, "T:")) != EOF) {
		switch (ch) {
		case 'T':
			if (strcmp(cli.optarg(), "VBATT") == 0) {
				cli.type = VOXLPM_CH_TYPE_VBATT;

			} else if (strcmp(cli.optarg(), "P5VDC") == 0) {
				cli.type = VOXLPM_CH_TYPE_P5VDC;

			} else {
				PX4_ERR("unknown type");
				return -1;
			}

			break;
		}
	}

	const char *verb = cli.optarg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_POWER_DEVTYPE_VOXLPM);

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
