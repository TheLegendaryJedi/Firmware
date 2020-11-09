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

#include "BMI088_Accelerometer_I2C.hpp"

#include <ecl/geo/geo.h> // CONSTANTS_ONE_G

using namespace time_literals;

namespace Bosch::BMI088::Accelerometer
{

	IBMI088 *bmi088_acc_i2c_interface(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation,
			int bus_frequency)
	{
		return new BMI088_Accelerometer_I2C(bus_option, bus, device, rotation, bus_frequency);
	}

	BMI088_Accelerometer_I2C::BMI088_Accelerometer_I2C(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation,
			int bus_frequency) :
		I2C(DRV_ACC_DEVTYPE_BMI088, "BMI088_Accelerometer", bus, device, bus_frequency),
		_px4_accel(get_device_id(), rotation)
	{
	}

	BMI088_Accelerometer_I2C::~BMI088_Accelerometer_I2C()
	{
		perf_free(_bad_register_perf);
		perf_free(_bad_transfer_perf);
		perf_free(_fifo_empty_perf);
		perf_free(_fifo_overflow_perf);
		perf_free(_fifo_reset_perf);
		perf_free(_drdy_missed_perf);
	}

	void BMI088_Accelerometer_I2C::exit_and_cleanup()
	{
		I2CSPIDriverBase::exit_and_cleanup();
	}

	void BMI088_Accelerometer_I2C::print_status()
	{
		I2CSPIDriverBase::print_status();

		PX4_INFO("FIFO empty interval: %d us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);

		perf_print_counter(_bad_register_perf);
		perf_print_counter(_bad_transfer_perf);
		perf_print_counter(_fifo_empty_perf);
		perf_print_counter(_fifo_overflow_perf);
		perf_print_counter(_fifo_reset_perf);
		perf_print_counter(_drdy_missed_perf);
	}

	int BMI088_Accelerometer_I2C::probe()
	{
		const uint8_t ACC_CHIP_ID = RegisterRead(Register::ACC_CHIP_ID);

		if (ACC_CHIP_ID != ID) {
			DEVICE_DEBUG("unexpected ACC_CHIP_ID 0x%02x", ACC_CHIP_ID);
			return PX4_ERROR;
		}

		return PX4_OK;
	}

	uint8_t BMI088_Accelerometer_I2C::RegisterRead(Register reg)
	{
		uint8_t cmd[2] = {static_cast<uint8_t>(reg) | ACC_I2C_ADDR_PRIMARY, 0};
		transfer(&cmd[0], 1, &cmd[1], 1);
		return cmd[1];
	}

	void BMI088_Accelerometer_I2C::RegisterWrite(Register reg, uint8_t value)
	{
		uint8_t cmd[2] = { static_cast<uint8_t>(reg) | ACC_I2C_ADDR_PRIMARY, value};
		return transfer(cmd, sizeof(cmd), nullptr, 0);
	}

	void BMI088_Accelerometer_I2C::ConfigureAccel()
	{
		const uint8_t ACC_RANGE = RegisterRead(Register::ACC_RANGE) & (Bit1 | Bit0);

		switch (ACC_RANGE) {
		case acc_range_3g:
			_px4_accel.set_scale(CONSTANTS_ONE_G * (powf(2, ACC_RANGE + 1) * 1.5f) / 32768.f);
			_px4_accel.set_range(3.f);
			break;

		case acc_range_6g:
			_px4_accel.set_scale(CONSTANTS_ONE_G * (powf(2, ACC_RANGE + 1) * 1.5f) / 32768.f);
			_px4_accel.set_range(6.f);
			break;

		case acc_range_12g:
			_px4_accel.set_scale(CONSTANTS_ONE_G * (powf(2, ACC_RANGE + 1) * 1.5f) / 32768.f);
			_px4_accel.set_range(12.f);
			break;

		case acc_range_24g:
			_px4_accel.set_scale(CONSTANTS_ONE_G * (powf(2, ACC_RANGE + 1) * 1.5f) / 32768.f);
			_px4_accel.set_range(24.f);
			break;
		}
	}

	void BMI088_Accelerometer_I2C::ConfigureSampleRate(int sample_rate)
	{
		if (sample_rate == 0) {
			sample_rate = 800; // default to 800 Hz
		}

		// round down to nearest FIFO sample dt * SAMPLES_PER_TRANSFER
		const float min_interval = FIFO_SAMPLE_DT;
		_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

		_fifo_samples = math::min((float)_fifo_empty_interval_us / (1e6f / RATE), (float)FIFO_MAX_SAMPLES);

		// recompute FIFO empty interval (us) with actual sample limit
		_fifo_empty_interval_us = _fifo_samples * (1e6f / RATE);

		ConfigureFIFOWatermark(_fifo_samples);
	}

	void BMI088_Accelerometer_I2C::ConfigureFIFOWatermark(uint8_t samples)
	{
		// FIFO_WTM: 13 bit FIFO watermark level value
		// unit of the fifo watermark is one byte
		const uint16_t fifo_watermark_threshold = samples * sizeof(FIFO::DATA);

		for (auto &r : _register_cfg) {
			if (r.reg == Register::FIFO_WTM_0) {
				// fifo_water_mark[7:0]
				r.set_bits = fifo_watermark_threshold & 0x00FF;
				r.clear_bits = ~r.set_bits;

			} else if (r.reg == Register::FIFO_WTM_1) {
				// fifo_water_mark[12:8]
				r.set_bits = (fifo_watermark_threshold & 0x0700) >> 8;
				r.clear_bits = ~r.set_bits;
			}
		}
	}

	bool BMI088_Accelerometer_I2C::Configure()
	{
		// first set and clear all configured register bits
		for (const auto &reg_cfg : _register_cfg) {
			RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
		}

		// now check that all are configured
		bool success = true;

		for (const auto &reg_cfg : _register_cfg) {
			if (!RegisterCheck(reg_cfg)) {
				success = false;
			}
		}

		ConfigureAccel();

		return success;
	}

	int BMI088_Accelerometer_I2C::DataReadyInterruptCallback(int irq, void *context, void *arg)
	{
		static_cast<BMI088_Accelerometer_I2C *>(arg)->DataReady();
		return 0;
	}

	void BMI088_Accelerometer_I2C::DataReady()
	{
		uint32_t expected = 0;

		if (_drdy_fifo_read_samples.compare_exchange(&expected, _fifo_samples)) {
			ScheduleNow();
		}
	}

	bool BMI088_Accelerometer_I2C::DataReadyInterruptConfigure()
	{
		if (_drdy_gpio == 0) {
			return false;
		}

		// Setup data ready on falling edge
		return px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &DataReadyInterruptCallback, this) == 0;
	}

	bool BMI088_Accelerometer_I2C::DataReadyInterruptDisable()
	{
		if (_drdy_gpio == 0) {
			return false;
		}

		return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
	}

	bool BMI088_Accelerometer_I2C::RegisterCheck(const register_config_t &reg_cfg)
	{
		bool success = true;

		const uint8_t reg_value = RegisterRead(reg_cfg.reg);

		if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
			PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
			success = false;
		}

		if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
			PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
			success = false;
		}

		return success;
	}

	void BMI088_Accelerometer_I2C::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
	{
		const uint8_t orig_val = RegisterRead(reg);

		uint8_t val = (orig_val & ~clearbits) | setbits;

		if (orig_val != val) {
			RegisterWrite(reg, val);
		}
	}

	uint16_t BMI088_Accelerometer_I2C::FIFOReadCount()
	{
		// FIFO length registers FIFO_LENGTH_1 and FIFO_LENGTH_0 contain the 14 bit FIFO byte
		uint8_t fifo_len_buf[4] {};
		fifo_len_buf[0] = static_cast<uint8_t>(Register::FIFO_LENGTH_0) | ACC_I2C_ADDR_PRIMARY;
		// fifo_len_buf[1] dummy byte

		if (transfer(&fifo_len_buf[0], &fifo_len_buf[0], &fifo_len_buf[2], 2) == PX4_OK) {
			perf_count(_bad_transfer_perf);
			return 0;
		}

		const uint8_t FIFO_LENGTH_0 = fifo_len_buf[2];        // fifo_byte_counter[7:0]
		const uint8_t FIFO_LENGTH_1 = fifo_len_buf[3] & 0x3F; // fifo_byte_counter[13:8]

		return combine(FIFO_LENGTH_1, FIFO_LENGTH_0);
	}

	bool BMI088_Accelerometer_I2C::FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples)
	{
		FIFOTransferBuffer buffer{};
		const size_t transfer_size = math::min(samples * sizeof(FIFO::DATA) + 4, FIFO::SIZE);

		if (transfer((uint8_t *)&buffer, 1, (uint8_t *)&buffer ,transfer_size) != PX4_OK) {
			perf_count(_bad_transfer_perf);
			return false;
		}

		const size_t fifo_byte_counter = combine(buffer.FIFO_LENGTH_1 & 0x3F, buffer.FIFO_LENGTH_0);

		// An empty FIFO corresponds to 0x8000
		if (fifo_byte_counter == 0x8000) {
			perf_count(_fifo_empty_perf);
			return false;

		} else if (fifo_byte_counter >= FIFO::SIZE) {
			perf_count(_fifo_overflow_perf);
			return false;
		}

		sensor_accel_fifo_s accel{};
		accel.timestamp_sample = timestamp_sample;
		accel.samples = 0;
		accel.dt = FIFO_SAMPLE_DT;

		// first find all sensor data frames in the buffer
		uint8_t *data_buffer = (uint8_t *)&buffer.f[0];
		unsigned fifo_buffer_index = 0; // start of buffer

		while (fifo_buffer_index < math::min(fifo_byte_counter, transfer_size - 4)) {
			// look for header signature (first 6 bits) followed by two bits indicating the status of INT1 and INT2
			switch (data_buffer[fifo_buffer_index] & 0xFC) {
			case FIFO::header::sensor_data_frame: {
					// Acceleration sensor data frame
					// Frame length: 7 bytes (1 byte header + 6 bytes payload)

					FIFO::DATA *fifo_sample = (FIFO::DATA *)&data_buffer[fifo_buffer_index];
					const int16_t accel_x = combine(fifo_sample->ACC_X_MSB, fifo_sample->ACC_X_LSB);
					const int16_t accel_y = combine(fifo_sample->ACC_Y_MSB, fifo_sample->ACC_Y_LSB);
					const int16_t accel_z = combine(fifo_sample->ACC_Z_MSB, fifo_sample->ACC_Z_LSB);

					// sensor's frame is +x forward, +y left, +z up
					//  flip y & z to publish right handed with z down (x forward, y right, z down)
					accel.x[accel.samples] = accel_x;
					accel.y[accel.samples] = (accel_y == INT16_MIN) ? INT16_MAX : -accel_y;
					accel.z[accel.samples] = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;
					accel.samples++;

					fifo_buffer_index += 7; // move forward to next record
				}
				break;

			case FIFO::header::skip_frame:
				// Skip Frame
				// Frame length: 2 bytes (1 byte header + 1 byte payload)
				PX4_DEBUG("Skip Frame");
				fifo_buffer_index += 2;
				break;

			case FIFO::header::sensor_time_frame:
				// Sensortime Frame
				// Frame length: 4 bytes (1 byte header + 3 bytes payload)
				PX4_DEBUG("Sensortime Frame");
				fifo_buffer_index += 4;
				break;

			case FIFO::header::FIFO_input_config_frame:
				// FIFO input config Frame
				// Frame length: 2 bytes (1 byte header + 1 byte payload)
				PX4_DEBUG("FIFO input config Frame");
				fifo_buffer_index += 2;
				break;

			case FIFO::header::sample_drop_frame:
				// Sample drop Frame
				// Frame length: 2 bytes (1 byte header + 1 byte payload)
				PX4_DEBUG("Sample drop Frame");
				fifo_buffer_index += 2;
				break;

			default:
				fifo_buffer_index++;
				break;
			}
		}

		_px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
					perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

		if (accel.samples > 0) {
			_px4_accel.updateFIFO(accel);
			return true;
		}

		return false;
	}

	void BMI088_Accelerometer_I2C::FIFOReset()
	{
		perf_count(_fifo_reset_perf);

		// ACC_SOFTRESET: trigger a FIFO reset by writing 0xB0 to ACC_SOFTRESET (register 0x7E).
		RegisterWrite(Register::ACC_SOFTRESET, 0xB0);

		// reset while FIFO is disabled
		_drdy_fifo_read_samples.store(0);
	}

	void BMI088_Accelerometer_I2C::UpdateTemperature()
	{
		// stored in an 11-bit value in 2’s complement format
		uint8_t temperature_buf[4] {};
		temperature_buf[0] = static_cast<uint8_t>(Register::TEMP_MSB) | ACC_I2C_ADDR_PRIMARY;
		// temperature_buf[1] dummy byte

		if (transfer(&temperature_buf[0], 1, &temperature_buf[0], sizeof(temperature_buf)) != PX4_OK) {
			perf_count(_bad_transfer_perf);
			return;
		}

		const uint8_t TEMP_MSB = temperature_buf[2];
		const uint8_t TEMP_LSB = temperature_buf[3];

		// Datasheet 5.3.7: Register 0x22 – 0x23: Temperature sensor data
		uint16_t Temp_uint11 = (TEMP_MSB * 8) + (TEMP_LSB / 32);
		int16_t Temp_int11 = 0;

		if (Temp_uint11 > 1023) {
			Temp_int11 = Temp_uint11 - 2048;

		} else {
			Temp_int11 = Temp_uint11;
		}

		float temperature = (Temp_int11 * 0.125f) + 23.f; // Temp_int11 * 0.125°C/LSB + 23°C

		if (PX4_ISFINITE(temperature)) {
			_px4_accel.set_temperature(temperature);

		} else {
			perf_count(_bad_transfer_perf);
		}
	}

	int BMI088_Accelerometer_I2C::init()
	{
		int ret = I2C::init();

		if (ret != PX4_OK) {
			DEVICE_DEBUG("I2C::init failed (%i)", ret);
			return ret;
		}

		return Reset() ? 0 : -1;
	}
} // namespace Bosch::BMI088::Accelerometer
