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

constexpr i2c_inst_t* DEFAULT_I2C_INSTANCE{i2c0};
constexpr size_t RAW_DATA_SIZE_BYTES{14};
constexpr uint32_t DEFAULT_SAMPLE_RATE_HZ{100};
constexpr uint32_t MAX_SAMPLE_RATE{8000};
constexpr PWR_MGMT_1 DEFAULT_CLOCK_SOURCE{PWR_MGMT_1::CLOCK_SELECT_PLL_WITH_X_AXIS_GYRO_REF_BIT};
constexpr DLPF_CONFIG DEFAULT_DLPF_BANDWIDTH{DLPF_CONFIG::DLPF_CFG_BANDWIDTH_184_Hz};
constexpr ACCEL_CONFIG DEFAULT_ACCEL_FULL_SCALE_SELECT{ACCEL_CONFIG::FS_SELECT_4_G_BIT};
constexpr GYRO_CONFIG DEFAULT_GYRO_FULL_SCALE_SELECT{GYRO_CONFIG::FS_SELECT_500_DEG_PER_SEC_BIT};
constexpr int16_t DEFAULT_ACCEL_DEADZONE{4};
constexpr int16_t DEFAULT_GYRO_DEADZONE{1};

struct MPU6050Config {
	MPU6050Address _address{MPU6050Address::DEFAULT};

	PWR_MGMT_1 _clock_source{DEFAULT_CLOCK_SOURCE};
	uint32_t _sample_rate_hz{DEFAULT_SAMPLE_RATE_HZ};
	DLPF_CONFIG _dlpf_bandwidth{DEFAULT_DLPF_BANDWIDTH};

	ACCEL_CONFIG _accel_full_scale_select{DEFAULT_ACCEL_FULL_SCALE_SELECT};
	float _accel_scale_factor{accelerometer_scale_factor(_accel_full_scale_select)};
	GYRO_CONFIG _gyro_full_scale_select{DEFAULT_GYRO_FULL_SCALE_SELECT};
	float _gyro_scale_factor{gyroscope_scale_factor(_gyro_full_scale_select)};

	int16_t _accelerometer_deadzone{DEFAULT_ACCEL_DEADZONE};
	int16_t _gyroscope_deadzone{DEFAULT_GYRO_DEADZONE};

	uint8_t _interrupt_pin_number{0};
};

class MPU6050 {
public:
	using Values = std::tuple<std::array<int16_t, 3>, std::array<int16_t, 3>, int16_t>;
	using OffsetAccelScaledGyros = std::tuple<std::array<int16_t, 3>, std::array<float, 3> >;
	using ScaledValues = std::tuple<std::array<float, 3>, std::array<float, 3>, float>;

	friend void mpu6050_callback0(uint gpio, uint32_t event);
	friend void mpu6050_callback1(uint gpio, uint32_t event);

	static std::array<MPU6050*, 2> instances;
	static std::array<void (*)(uint gpio, uint32_t event), 2> callbacks;

	MPU6050(
		i2c_inst_t* i2c,
		MPU6050Address address,
		uint8_t interrupt_pin_number,
		uint32_t sample_rate,
		DLPF_CONFIG dlpf,
		ACCEL_CONFIG accel_fs,
		GYRO_CONFIG gyro_fs
	);

	~MPU6050();
	MPU6050(const MPU6050&)=delete;
	MPU6050(const MPU6050&&)=delete;
	MPU6050& operator=(const MPU6050&)=delete;
	MPU6050& operator=(const MPU6050&&)=delete;

	void reset_device() const;
	void callibrate();
	void fast_calibrate();

	void start();

	void init_pin_interrupt() const;
	void deinit_pin_interrupt() const;

	uint8_t read_byte(Register reg) const;
	void write_byte(Register reg, uint8_t value) const;

	bool available() const;
	bool read_data_from_device();

	Values get_raw_values();
	Values get_offset_values();
	OffsetAccelScaledGyros get_offset_accel_and_scaled_gyros();
	ScaledValues get_scaled_values();
private:
	std::array<int16_t, 3> read_accel_factory_trim() const;

	std::array<uint8_t, 6> calc_accel_offset_register_values(const std::array<int16_t, 3> &accel_offsets) const;
	std::array<uint8_t, 6> calc_gyro_offset_register_values(const std::array<int16_t, 3> &gyro_offsets) const;

	void update_accel_offset_registers(const std::array<uint8_t, 6> &offset_register_values) const;
	void update_gyro_offset_registers(const std::array<uint8_t, 6> &offset_register_values) const;

	MPU6050Address _address{MPU6050Address::DEFAULT};
	i2c_inst_t* _i2c{DEFAULT_I2C_INSTANCE};

	PWR_MGMT_1 _clock_source{DEFAULT_CLOCK_SOURCE};
	uint32_t _sample_rate_hz{DEFAULT_SAMPLE_RATE_HZ};
	DLPF_CONFIG _dlpf_bandwidth{DEFAULT_DLPF_BANDWIDTH};

	ACCEL_CONFIG _accel_full_scale_select{DEFAULT_ACCEL_FULL_SCALE_SELECT};
	float _accel_scale_factor;
	GYRO_CONFIG _gyro_full_scale_select{DEFAULT_GYRO_FULL_SCALE_SELECT};
	float _gyro_scale_factor;

	std::array<uint8_t, RAW_DATA_SIZE_BYTES> _buffer{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	int16_t _accelerometer_deadzone{DEFAULT_ACCEL_DEADZONE};
	int16_t _gyroscope_deadzone{DEFAULT_GYRO_DEADZONE};

	uint32_t _instance_id;
	uint8_t _interrupt_pin_number{0};
	volatile bool _data_available{false};
};

#endif