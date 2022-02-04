// File: main.cpp
// Author: Jacob Guenther
// Date Created: 15 May 2021
// License: AGPLv3

#include <memory>

#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "mpu6050.hpp"
#include "mpu6050_config.hpp"
#include "median_filter.hpp"
#include "complementary_filter.hpp"

extern "C" {
	#include "pico_servo.h"
}

int main() {
	stdio_init_all();
	printf("Starting up MPU-6050 driver example.\n");

	bi_decl(bi_program_name("mpu_6050_example"));
	bi_decl(bi_program_description("Demonstrates using the InvenSense MPU-6050 on the Pi Pico."));
	bi_decl(bi_program_version_string("1.0.0"));
	bi_decl(bi_program_url("https://github.com/jacobguenther/pico-libs"));
	bi_decl(bi_program_feature("License AGPLv3"));

	const uint8_t pitch_servo_pin{2};
	const uint8_t roll_servo_pin{3};
	servo_init();
	servo_clock_auto();
	servo_attach(pitch_servo_pin);
	servo_attach(roll_servo_pin);

	bi_decl(bi_1pin_with_func(pitch_servo_pin, GPIO_FUNC_PWM));
	bi_decl(bi_1pin_with_func(roll_servo_pin, GPIO_FUNC_PWM));

	const uint8_t mpu_power_pin{9};
	gpio_init(mpu_power_pin);
	gpio_set_dir(mpu_power_pin, GPIO_OUT);
	gpio_put(mpu_power_pin, 1);

	bi_decl(bi_1pin_with_name(mpu_power_pin, "MPU-6050 POWER"));

	const uint8_t sda_pin{16};
	const uint8_t scl_pin{17};
	const uint32_t i2c_baudrate{400000}; // Hz

	bi_decl(bi_2pins_with_func(sda_pin, scl_pin, GPIO_FUNC_I2C));

	i2c_init(i2c0, i2c_baudrate);
	gpio_set_function(sda_pin, GPIO_FUNC_I2C);
	gpio_set_function(scl_pin, GPIO_FUNC_I2C);
	gpio_pull_up(sda_pin);
	gpio_pull_up(scl_pin);

	const uint8_t mpu_interrupt_pin{8};

	bi_decl(bi_1pin_with_name(mpu_interrupt_pin, "MPU-6050 IRQ"));

	// tuning settings
	const auto dlpf = DLPF_CONFIG::DLPF_CFG_BANDWIDTH_94_Hz;
	const auto accel_fs = ACCEL_CONFIG::FS_SELECT_16_G_BIT;
	const auto gyro_fs = GYRO_CONFIG::FS_SELECT_250_DEG_PER_SEC_BIT;

	const uint32_t mpu_sample_rate{100}; // Hz
	const float dt{1.0F / static_cast<float>(mpu_sample_rate)}; // s
	const float tau_ms{500.0F};
	const float tau_s{tau_ms / 1000.0F};
	const float gyro_bias{tau_s / (tau_s + dt)};

	const uint32_t mean_filter_size{9};

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

	float pitch{0.0F};
	float roll{0.0F};

	printf("Entering main loop.\n");
	sleep_ms(1000);

	while (true) {
		if (mpu0.available()) {
			mpu0.read_data_from_device();

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
			std::tie(pitch, roll) = comp_filter.get_filtered_angles();

			printf("pitch: %.2f roll: %.2f\n", pitch, roll);
		}

		servo_move_to(pitch_servo_pin, pitch + 90.0F);
		servo_move_to(roll_servo_pin, roll + 90.0F);
	}
}
