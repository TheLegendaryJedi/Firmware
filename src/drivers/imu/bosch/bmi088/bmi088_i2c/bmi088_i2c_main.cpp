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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include "BMI088.hpp"

void BMI088::print_usage()
{
	PRINT_MODULE_USAGE_NAME("bmi088_i2c", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('A', "Accel", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('G', "Gyro", true);
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x76);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 35, "Rotation", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" int bmi088_i2c_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = BMI088;
	BusCLIArguments cli{true, true};
	cli.i2c_address = 0x18;
	cli.default_i2c_frequency = 100 * 1000;
	cli.default_spi_frequency = 100 * 1000;


	while ((ch = cli.getopt(argc, argv, "AGR:")) != EOF) {
		switch (ch) {
		case 'A':
			cli.type = DRV_ACC_DEVTYPE_BMI088;
			cli.i2c_address = 0x18;
			break;

		case 'G':
			cli.type = DRV_GYR_DEVTYPE_BMI088;
			cli.i2c_address = 0x68;
			break;

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

	PX4_WARN("ID:0x%02x", cli.type);
	BusInstanceIterator iterator(MODULE_NAME, cli, cli.type);

	PX4_WARN("BusInstanceIterator devid: 0x%02x", iterator.devid());

	if (!strcmp(verb, "start")) {
		PX4_WARN("start");
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	PX4_WARN("print_usage1");
	ThisDriver::print_usage();
	return -1;
}
