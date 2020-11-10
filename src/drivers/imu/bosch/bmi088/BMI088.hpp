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

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/cdev/CDev.hpp>

static constexpr int16_t combine(uint8_t msb, uint8_t lsb) { return (msb << 8u) | lsb; }

class IBMI088
{
public:
	virtual ~IBMI088() = default;

	void print_status();

	virtual int init();

	void exit_and_cleanup();

	virtual void RunImpl();

	virtual u_int32_t get_device_id() const = 0;

	virtual u_int8_t get_dev_type() const = 0;

	virtual char * get_name();

	bool Reset();// 2500 us / 400 Hz transfer interval
private:

};

class BMI088 : public I2CSPIDriver<BMI088>
{
public:
	BMI088(uint8_t devtype, const char *name, I2CSPIBusOption bus_option, int bus, uint32_t device, enum spi_mode_e mode,
	spi_drdy_gpio_t drdy_gpio, IBMI088 *interface);

	virtual ~BMI088() = default;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	int init();
	static void print_usage();
	void print_status();
	void exit_and_cleanup();

	virtual void RunImpl();

protected:
	bool Reset();
	IBMI088	*_interface{nullptr};
};
/* interface factories */
extern IBMI088 *bmi088_acc_spi_interface(I2CSPIBusOption bus_option, int busnum, uint32_t device, enum Rotation rotation, int bus_frequency, spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio);
extern IBMI088 *bmi088_acc_i2c_interface(I2CSPIBusOption bus_option, int busnum, uint32_t device, enum Rotation rotation, int bus_frequency, spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio);
extern IBMI088 *bmi088_gyro_spi_interface(I2CSPIBusOption bus_option, int busnum, uint32_t device, enum Rotation rotation, int bus_frequency, spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio);
extern IBMI088 *bmi088_gyro_i2c_interface(I2CSPIBusOption bus_option, int busnum, uint32_t device, enum Rotation rotation, int bus_frequency, spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio);
