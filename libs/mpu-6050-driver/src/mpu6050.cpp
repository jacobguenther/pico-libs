// File: mpu6050.cpp
// Author: Jacob Guenther
// Date Created: 16 May 2021
// License: AGPLv3

#include "mpu6050.hpp"

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"

using std::array;
using std::optional;
using std::tuple;

using Values = tuple<array<int16_t, 3>, array<int16_t, 3>, int16_t>;
using ScaledValues = tuple<array<float, 3>, array<float, 3>, float>;

array<MPU6050*, 2> MPU6050::instances = {nullptr, nullptr};

void mpu6050_callback0(uint _gpio, uint32_t _event) { 
	MPU6050::instances[0]->_data_available = true;
}
void mpu6050_callback1(uint _gpio, uint32_t _event) {
	MPU6050::instances[1]->_data_available = true;
}

std::array<void (*)(uint gpio, uint32_t event), 2> MPU6050::callbacks = {
	mpu6050_callback0,
	mpu6050_callback1
};

MPU6050::MPU6050(
	i2c_inst_t* i2c,
	MPU6050Address address,
	uint8_t interrupt_pin_number,
	uint32_t sample_rate,
	DLPF_CONFIG dlpf,
	ACCEL_CONFIG accel_fs,
	GYRO_CONFIG gyro_fs)
	: _address{address}
	, _i2c{i2c}

	, _sample_rate_hz{sample_rate}
	, _dlpf_bandwidth{dlpf}

	, _accel_full_scale_select{accel_fs}
	, _accel_scale_factor{accelerometer_scale_factor(accel_fs)}

	, _gyro_full_scale_select{gyro_fs}
	, _gyro_scale_factor{gyroscope_scale_factor(gyro_fs)}

	, _interrupt_pin_number{interrupt_pin_number}
{
	for (uint32_t i = 0; i < 2; i++) {
		if (instances[i] == nullptr) {
			_instance_id = i;
			instances[i] = this;
			break;
		}
	}

	start();
	init_pin_interrupt();
	fast_calibrate();
}
MPU6050::~MPU6050() {
	deinit_pin_interrupt();
	MPU6050::instances[_instance_id] = nullptr;
}

void MPU6050::reset_device() const {
	write_byte(Register::PWR_MGMT_1,
		static_cast<uint8_t>(PWR_MGMT_1::RESET_BIT));
	sleep_ms(100);
	write_byte(Register::SIGNAL_PATH_RESET,
		static_cast<uint8_t>(SIGNAL_PATH_RESET::RESET_ALL));
}
void MPU6050::callibrate() {

}
void MPU6050::fast_calibrate() {
	while (true) {
		read_data_from_device();
		const auto [accel, gyro, _] = get_raw_values();

		uint8_t zero_count = 0;
		for (uint8_t i = 0; i < 3; i++) {
			if (accel[i] == 0) {
				zero_count += 1;
			}
			if (gyro[i] == 0) {
				zero_count += 1;
			}
		}
		if (zero_count < 2) {
			break;
		}
	}

	sleep_ms(10);
	read_data_from_device();
	auto [accel, gyro, _] = get_raw_values();

	accel[2] -= _accel_scale_factor;
	const auto accel_offset = calc_accel_offset_register_values(accel);
	update_accel_offset_registers(accel_offset);
	const auto gyro_offset = calc_gyro_offset_register_values(gyro);
	update_gyro_offset_registers(gyro_offset);
}

void MPU6050::start() {
	write_byte(Register::PWR_MGMT_2, 0);
	write_byte(Register::PWR_MGMT_1, static_cast<uint8_t>(_clock_source));

	write_byte(Register::ACCEL_CONFIG, static_cast<uint8_t>(_accel_full_scale_select));
	write_byte(Register::GYRO_CONFIG, static_cast<uint8_t>(_gyro_full_scale_select));


	const auto dlpf_value{static_cast<uint8_t>(_dlpf_bandwidth)};
	write_byte(Register::CONFIG, dlpf_value);

	// smplrt_div
	//   sample rate = gyroscope_output_rate / (1 + SMPLRT_DIV)
	//   gyroscope_output_rate = 8kHz when DLPF is disabled (DLPF_CFG = 0 or 7)
	//   gyroscope_output_rate = 1KHz when DLPF is enabled
	const uint16_t gyro_sample_rate = (dlpf_value == 0 || dlpf_value == 7) ? 8000 : 1000;
	const uint8_t smplrt = gyro_sample_rate / _sample_rate_hz - 1;
	write_byte(Register::SMPLRT_DIV, smplrt);
}
void MPU6050::init_pin_interrupt() const {
	gpio_set_irq_enabled_with_callback(_interrupt_pin_number, GPIO_IRQ_EDGE_RISE, true, MPU6050::callbacks[_instance_id]);
	write_byte(Register::INT_ENABLE,
		static_cast<uint8_t>(INTERRUPT_ENABLE::DATA_READY_ENABLE_BIT));
}
void MPU6050::deinit_pin_interrupt() const {
	irq_set_enabled(_interrupt_pin_number, false);
}

uint8_t MPU6050::read_byte(Register reg) const {
	const auto address{static_cast<uint8_t>(_address)};
	const auto write_register{static_cast<uint8_t>(reg)};
    i2c_write_blocking(_i2c, address, &write_register, 1, true);
	uint8_t byte{0};
	i2c_read_blocking(_i2c, address, &byte, 1, false);
	return byte;
}
void MPU6050::write_byte(Register reg, uint8_t value) const {
	const auto address{static_cast<uint8_t>(_address)};
	const array<uint8_t, 2> buffer = {static_cast<uint8_t>(reg), value};
    i2c_write_blocking(_i2c, address, &buffer[0], 2, true);
	sleep_ms(10);
}

bool MPU6050::available() const {
	return _data_available;
}
bool MPU6050::read_data_from_device() {
	uint8_t addr = static_cast<uint8_t>(_address);
	uint8_t reg = static_cast<uint8_t>(Register::ACCEL_XOUT_H);
    i2c_write_blocking(_i2c, addr, &reg, 1, true);
	int32_t read_count = i2c_read_blocking(_i2c, addr, &_buffer[0], 14, false);
	_data_available = false;
	return read_count == 14;
}
Values MPU6050::get_raw_values() {
	auto values = Values {
		{
			static_cast<int16_t>(_buffer[0] << 8 | _buffer[1]),
			static_cast<int16_t>(_buffer[2] << 8 | _buffer[3]),
			static_cast<int16_t>(_buffer[4] << 8 | _buffer[5])
		},
		{
			static_cast<int16_t>(_buffer[8]  << 8 | _buffer[9]),
			static_cast<int16_t>(_buffer[10] << 8 | _buffer[11]),
			static_cast<int16_t>(_buffer[12] << 8 | _buffer[13])
		},
		static_cast<int16_t>(_buffer[6] << 8 | _buffer[7])
	};
	_data_available = false;
	return values;
}
tuple<array<int16_t, 3>, array<float, 3> > MPU6050::get_offset_accel_and_scaled_gyros() {
	auto [accel, gyro, temp] = get_raw_values();

	auto gyro_scaled = array<float, 3>{0, 0, 0};
	for (uint32_t i = 0; i < 3; i++) {
		gyro_scaled[i] = static_cast<float>(gyro[i]) / _gyro_scale_factor;
	}
	return {
		accel,
		gyro_scaled
	};
}
ScaledValues MPU6050::get_scaled_values() {
	const auto [accel, gyro, temp] = get_raw_values();

	auto accel_scaled = array<float, 3>{0, 0, 0};
	auto gyro_scaled = array<float, 3>{0, 0, 0};
	for (uint32_t i = 0; i < 3; i++) {
		accel_scaled[i] = static_cast<float>(accel[i]) / _accel_scale_factor;
		gyro_scaled[i] = static_cast<float>(gyro[i]) / _gyro_scale_factor;
	}

	const auto temp_scaled{static_cast<float>(temp) / 340.0f + 36.53f};

	return {accel_scaled, gyro_scaled, temp_scaled};
}


array<int16_t, 3> MPU6050::read_accel_factory_trim() const {
	const auto address{static_cast<uint8_t>(_address)};
	const auto reg{static_cast<uint8_t>(Register::XA_OFFS_USRH)};
	i2c_write_blocking(_i2c, address, &reg, 1, true);
	auto buffer = array<uint8_t, 6>{0, 0, 0, 0, 0, 0};
	i2c_read_blocking(_i2c, address, &buffer[0], 6, false);
	return {
		static_cast<int16_t>((buffer[0] << 8) | buffer[1]),
		static_cast<int16_t>((buffer[2] << 8) | buffer[3]),
		static_cast<int16_t>((buffer[4] << 8) | buffer[5])
	};
}
array<uint8_t, 6> MPU6050::calc_accel_offset_register_values(const array<int16_t, 3> &accel_offsets) const {
	auto accel_factory_trim = read_accel_factory_trim();
	const int16_t mask{0x01};
	auto mask_bit = array<uint8_t, 3>{0, 0, 0};

	for (uint8_t i = 0; i < 3; i++) {
		if (accel_factory_trim[i] & mask) {
			mask_bit[i] = mask;
		}
		accel_factory_trim[i] -= accel_offsets[i] / 8;
	}

	return {
		static_cast<uint8_t>((accel_factory_trim[0] >> 8) & 0xff),
		static_cast<uint8_t>((accel_factory_trim[0]       & 0xff) | mask_bit[0]),
		static_cast<uint8_t>((accel_factory_trim[1] >> 8) & 0xff),
		static_cast<uint8_t>((accel_factory_trim[1]       & 0xff) | mask_bit[1]),
		static_cast<uint8_t>((accel_factory_trim[2] >> 8) & 0xff),
		static_cast<uint8_t>((accel_factory_trim[2]       & 0xff) | mask_bit[2])
	};
}
array<uint8_t, 6> MPU6050::calc_gyro_offset_register_values(const array<int16_t, 3> &gyro_offsets) const {
		auto gyro_divisor = [](GYRO_CONFIG gyro_scale) {
		switch (gyro_scale) {
			case GYRO_CONFIG::FS_SELECT_250_DEG_PER_SEC_BIT:
				return 4.0F;
			case GYRO_CONFIG::FS_SELECT_500_DEG_PER_SEC_BIT:
				return 2.0F;
			case GYRO_CONFIG::FS_SELECT_1000_DEG_PER_SEC_BIT:
				return 1.0F;
			case GYRO_CONFIG::FS_SELECT_2000_DEG_PER_SEC_BIT:
			default:
				return 0.5F;
		}
	};
	const auto divisor{gyro_divisor(_gyro_full_scale_select)};

	auto gyro_bias = array<int16_t, 3>{0, 0, 0};
	for (uint32_t i = 0; i < 3; i++) {
		gyro_bias[i] = -static_cast<int16_t>(
			static_cast<float>(gyro_offsets[i]) / divisor
		);
	}

	return {
		static_cast<uint8_t>((gyro_bias[0] >> 8) & 0xff),
		static_cast<uint8_t>( gyro_bias[0]       & 0xff),
		static_cast<uint8_t>((gyro_bias[1] >> 8) & 0xff),
		static_cast<uint8_t>( gyro_bias[1]       & 0xff),
		static_cast<uint8_t>((gyro_bias[2] >> 8) & 0xff),
		static_cast<uint8_t>( gyro_bias[2]       & 0xff)
	};
}

void MPU6050::update_accel_offset_registers(const array<uint8_t, 6> &offset_register_values) const {
	write_byte(Register::XA_OFFS_USRH, offset_register_values[0]);
	write_byte(Register::XA_OFFS_USRL, offset_register_values[1]);
	write_byte(Register::YA_OFFS_USRH, offset_register_values[2]);
	write_byte(Register::YA_OFFS_USRL, offset_register_values[3]);
	write_byte(Register::ZA_OFFS_USRH, offset_register_values[4]);
	write_byte(Register::ZA_OFFS_USRL, offset_register_values[5]);
}
void MPU6050::update_gyro_offset_registers(const array<uint8_t, 6> &offset_register_values) const {
	write_byte(Register::XG_OFFS_USRH, offset_register_values[0]);
	write_byte(Register::XG_OFFS_USRL, offset_register_values[1]);
	write_byte(Register::YG_OFFS_USRH, offset_register_values[2]);
	write_byte(Register::YG_OFFS_USRL, offset_register_values[3]);
	write_byte(Register::ZG_OFFS_USRH, offset_register_values[4]);
	write_byte(Register::ZG_OFFS_USRL, offset_register_values[5]);
}