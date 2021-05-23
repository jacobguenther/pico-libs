// File: main.cpp
// Author: Jacob Guenther
// Date Created: 15 May 2021
// License: AGPLv3

#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "mpu6050.hpp"
#include "mpu6050_config.hpp"

#include "median_filter.hpp"
#include "complementary_filter.hpp"

int main() {
	stdio_init_all();

	printf("starting up\n");

	bi_decl(bi_program_name("pico-mpu-6050-driver-cpp"));
	bi_decl(bi_program_description("A utility for communicating with the MPU-6050."));
	bi_decl(bi_program_version_string("0.0.1"));
	bi_decl(bi_program_url("https://github.com/jacobguenther/pico-mpu-6050-driver-cpp"));
	bi_decl(bi_program_feature("License AGPLv3"));

	const uint8_t power_pin = 11;
	gpio_init(power_pin);
	gpio_set_dir(power_pin, GPIO_OUT);
	gpio_put(power_pin, 1);

	const uint8_t sda_pin = 12;
	const uint8_t scl_pin = 13;
	const uint32_t i2c_baudrate = 400000;

	i2c_init(i2c0, i2c_baudrate);
	gpio_set_function(sda_pin, GPIO_FUNC_I2C);
	gpio_set_function(scl_pin, GPIO_FUNC_I2C);
	gpio_pull_up(sda_pin);
	gpio_pull_up(scl_pin);

	const uint8_t mpu_interrupt_pin = 10;



	// tuning settings
	const CONFIG dlpf = CONFIG::DLPF_CFG_BANDWIDTH_94_Hz;
	const ACCEL_CONFIG accel_fs = ACCEL_CONFIG::FS_SELECT_16_G_BIT;
	const GYRO_CONFIG gyro_fs = GYRO_CONFIG::FS_SELECT_250_DEG_PER_SEC_BIT;

	const uint32_t mpu_sample_rate = 100; // hz
	const float dt = 1.0f / static_cast<float>(mpu_sample_rate);
	const float tau_ms = 500.0f;
	const float tau_s = tau_ms / 1000.0f;
	const float gyro_bias = tau_s / (tau_s + dt);

	const uint32_t mean_filter_size = 9;



	MPU6050 mpu0 = MPU6050(
		i2c0,
		MPU6050Address::DEFAULT,
		mpu_interrupt_pin,
		mpu_sample_rate,
		dlpf,
		accel_fs,
		gyro_fs
	);

	MedianFilter<mean_filter_size> accel_x_filter;
	MedianFilter<mean_filter_size> accel_y_filter;
	MedianFilter<mean_filter_size> accel_z_filter;
	ComplementaryFilter comp_filter = ComplementaryFilter(dt, gyro_bias);

	// auto last_mpu_update = get_absolute_time();
	while (true) {
		// auto now = get_absolute_time();
		if (mpu0.available()) {
			mpu0.read_data_from_device();

			// auto [accel_raw, gyro_raw, temp_raw] = mpu0.get_raw_values();
			// printf("%d %d %d\n", accel_raw[0], accel_raw[1], accel_raw[2]);
			// printf("%d %d %d\n", gyro_raw[0], gyro_raw[1], gyro_raw[2]);

			// uint32_t dt_micro = absolute_time_diff_us(last_mpu_update, now);
			// uint32_t dt_milli = dt_micro / 1000;
			// float dt = static_cast<float>(dt_micro) / 1'000'000.0f;
			// last_mpu_update = now;

			auto [accel, gyro] = mpu0.get_offset_accel_and_scaled_gyros();
			accel_x_filter.update(accel[0]);
			accel_y_filter.update(accel[1]);
			accel_z_filter.update(accel[2]);
			std::array<int16_t, 3> filtered_accel = {
				accel_x_filter.get_median(),
				accel_y_filter.get_median(),
				accel_z_filter.get_median()
			};
			comp_filter.update(filtered_accel, gyro);
			auto [pitch, roll] = comp_filter.get_filtered_angle();
			printf("pitch: %.2f roll: %.2f\n", pitch, roll);
		}
	}
}
