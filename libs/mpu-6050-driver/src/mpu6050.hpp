// File: mpu6050.hpp
// Author: Jacob Guenther
// Date Created: 16 May 2021
// License: AGPLv3
// 
// Resources:
//   datasheet - https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
//   register map - https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
//   offset registers - https://www.digikey.com/en/pdf/i/invensense/mpu-hardware-offset-registers
//   3rd party register map - https://www.i2cdevlib.com/devices/mpu6050#registers

#ifndef MPU6050_HPP
#define MPU6050_HPP

#include <array>
#include <optional>
#include <tuple>

#include "hardware/i2c.h"

#include "mpu6050_config.hpp"

std::optional<float> accelerometer_scale_factor(ACCEL_CONFIG fs_select_bit);
std::optional<float> gyroscope_scale_factor(GYRO_CONFIG fs_select_bit);

class MPU6050 {
public:
	using Values = std::tuple<std::array<int16_t, 3>, std::array<int16_t, 3>, int16_t>;
	using ScaledValues = std::tuple<std::array<float, 3>, std::array<float, 3>, float>;

	friend void mpu6050_callback0(uint gpio, uint32_t event);
	friend void mpu6050_callback1(uint gpio, uint32_t event);

	static MPU6050* instances[];
	static void (*callbacks[])(uint, uint32_t);

	MPU6050(
		i2c_inst_t* i2c=i2c0,
		MPU6050Address address=MPU6050Address::DEFAULT,
		uint8_t interrupt_pin_number=0,
		uint32_t sample_rate=100,
		CONFIG dlpf=CONFIG::DLPF_CFG_BANDWIDTH_184_Hz,
		ACCEL_CONFIG accel_fs=ACCEL_CONFIG::FS_SELECT_4_G_BIT,
		GYRO_CONFIG gyro_fs=GYRO_CONFIG::FS_SELECT_500_DEG_PER_SEC_BIT
	);

	~MPU6050();
	MPU6050(const MPU6050&)=delete;
	MPU6050(const MPU6050&&)=delete;
	MPU6050& operator=(const MPU6050&)=delete;
	MPU6050& operator=(const MPU6050&&)=delete;

	void reset_device();
	void callibrate();
	void fast_calibrate();

	void start();

	void init_pin_interrupt();
	void deinit_pin_interrupt();

	uint8_t get_byte(Register reg) const;
	void write_byte(Register reg, uint8_t value) const;

	bool available() const;
	bool read_data_from_device();

	Values get_raw_values();
	Values get_offset_values();
	std::tuple<std::array<int16_t, 3>, std::array<float, 3> > get_offset_accel_and_scaled_gyros();
	ScaledValues get_scaled_values();
private:
	std::array<int16_t, 3> read_accel_factory_trim();
	void update_accel_offset_registers(std::array<int16_t, 3> &accel_offsets);
	void update_gyro_offset_registers(std::array<int16_t, 3> &gyro_offsets);

	MPU6050Address _address;
	i2c_inst_t* _i2c;

	bool _data_available;

	PWR_MGMT_1 _clock_source;
	uint32_t _sample_rate_hz;
	CONFIG _dlpf_bandwidth;

	ACCEL_CONFIG _accel_full_scale_select;
	float _accel_scale_factor;
	GYRO_CONFIG _gyro_full_scale_select;
	float _gyro_scale_factor;

	std::array<uint8_t, 14> _buffer;

	int16_t _accelerometer_deadzone;
	int16_t _gyroscope_deadzone;
	std::array<int16_t, 3> _accel_offsets;
	std::array<int16_t, 3> _gyro_offsets;

	uint32_t _instance_id;
	uint8_t _interrupt_pin_number;
};

#endif