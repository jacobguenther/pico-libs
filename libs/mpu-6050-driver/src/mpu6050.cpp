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

using Values = std::tuple<std::array<int16_t, 3>, std::array<int16_t, 3>, int16_t>;
using ScaledValues = std::tuple<std::array<float, 3>, std::array<float, 3>, float>;

optional<float> accelerometer_scale_factor(ACCEL_CONFIG fs_select_bit) {
	switch (fs_select_bit) {
		case ACCEL_CONFIG::FS_SELECT_2_G_BIT:
			return 16384.0f;
		case ACCEL_CONFIG::FS_SELECT_4_G_BIT:
			return 8192.0f;
		case ACCEL_CONFIG::FS_SELECT_8_G_BIT:
			return 4096.0f;
		case ACCEL_CONFIG::FS_SELECT_16_G_BIT:
			return 2048.0f;
		default:
			return {};
	}
}
optional<float> gyroscope_scale_factor(GYRO_CONFIG fs_select_bit) {
	switch (fs_select_bit) {
		case GYRO_CONFIG::FS_SELECT_250_DEG_PER_SEC_BIT:
			return 131.0f;
		case GYRO_CONFIG::FS_SELECT_500_DEG_PER_SEC_BIT:
			return 65.6f;
		case GYRO_CONFIG::FS_SELECT_1000_DEG_PER_SEC_BIT:
			return 32.8f;
		case GYRO_CONFIG::FS_SELECT_2000_DEG_PER_SEC_BIT:
			return 16.4f;
		default:
			return {};
	}
}

MPU6050* MPU6050::instances[2] = {nullptr, nullptr};
void mpu6050_callback0(uint, uint32_t) { 
	MPU6050::instances[0]->_data_available = true;
}
void mpu6050_callback1(uint, uint32_t) {
	MPU6050::instances[1]->_data_available = true;
}
void (*MPU6050::callbacks[2])(uint, uint32_t) = {mpu6050_callback0, mpu6050_callback1};

MPU6050::MPU6050(
	i2c_inst_t* i2c,
	MPU6050Address address,
	uint8_t interrupt_pin_number,
	uint32_t sample_rate,
	CONFIG dlpf,
	ACCEL_CONFIG accel_fs,
	GYRO_CONFIG gyro_fs)
	: _address{address}
	, _i2c{i2c}
	
	, _data_available{false}

	, _clock_source{PWR_MGMT_1::CLOCK_SELECT_PLL_WITH_X_AXIS_GYRO_REF}

	, _sample_rate_hz{sample_rate}
	, _dlpf_bandwidth{dlpf}

	, _accel_full_scale_select{accel_fs}
	, _accel_scale_factor{accelerometer_scale_factor(accel_fs).value()}
	, _gyro_full_scale_select{gyro_fs}
	, _gyro_scale_factor{gyroscope_scale_factor(gyro_fs).value()}

	, _buffer{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

	, _accelerometer_deadzone{4}
	, _gyroscope_deadzone{1}
	
	, _instance_id{0}
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

void MPU6050::reset_device() {
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
		auto [accel, gyro, _] = get_raw_values();

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

	sleep_ms(100);
	read_data_from_device();
	auto [accel, gyro, _] = get_raw_values();

	accel[2] -= _accel_scale_factor;
	update_accel_offset_registers(accel);
	update_gyro_offset_registers(gyro);
}

void MPU6050::start() {
	write_byte(Register::PWR_MGMT_2, 0);
	write_byte(Register::PWR_MGMT_1, static_cast<uint8_t>(_clock_source));

	write_byte(Register::ACCEL_CONFIG,
		static_cast<uint8_t>(_accel_full_scale_select) << static_cast<uint8_t>(ACCEL_CONFIG::FS_SELECT_POSITION));
	write_byte(Register::GYRO_CONFIG,
		static_cast<uint8_t>(_gyro_full_scale_select) << static_cast<uint8_t>(GYRO_CONFIG::FS_SELECT_POSITION));


	uint8_t dlpf_val = static_cast<uint8_t>(_dlpf_bandwidth);
	write_byte(Register::CONFIG, dlpf_val);

	// smplrt_div
	//   sample rate = gyroscope_output_rate / (1 + SMPLRT_DIV)
	//   gyroscope_output_rate = 8kHz when DLPF is disabled (DLPF_CFG = 0 or 7)
	//   gyroscope_output_rate = 1KHz when DLPF is enabled
	uint16_t gyro_sample_rate = (dlpf_val == 0 || dlpf_val == 7) ? 8000 : 1000;
	uint8_t smplrt = gyro_sample_rate / _sample_rate_hz - 1;
	write_byte(Register::SMPLRT_DIV, smplrt);
}
void MPU6050::init_pin_interrupt() {
	gpio_set_irq_enabled_with_callback(_interrupt_pin_number, GPIO_IRQ_EDGE_RISE, true, MPU6050::callbacks[_instance_id]);
	write_byte(Register::INT_ENABLE,
		static_cast<uint8_t>(INTERRUPT_ENABLE::DATA_READY_ENABLE_BIT));
}
void MPU6050::deinit_pin_interrupt() {
	irq_set_enabled(_interrupt_pin_number, false);
}

uint8_t MPU6050::get_byte(Register reg) const {
	uint8_t byte;
	uint8_t address = static_cast<uint8_t>(_address);
	uint8_t* reg_ptr = reinterpret_cast<uint8_t*>(&reg);
    i2c_write_blocking(_i2c, address, reg_ptr, 1, true);
	i2c_read_blocking(_i2c, address, &byte, 1, false);
	return byte;
}
void MPU6050::write_byte(Register reg, uint8_t value) const {
	uint8_t address = static_cast<uint8_t>(_address);
	uint8_t buffer[] = {static_cast<uint8_t>(reg), value};
    i2c_write_blocking(_i2c, address, buffer, 2, true);
	sleep_ms(100);
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
std::tuple<std::array<int16_t, 3>, std::array<float, 3> > MPU6050::get_offset_accel_and_scaled_gyros() {
	auto [accel, gyro, temp] = get_raw_values();

	array<float, 3> gyro_scaled;
	for (uint32_t i = 0; i < 3; i++) {
		gyro_scaled[i] = static_cast<float>(gyro[i]) / _gyro_scale_factor;
	}
	return {
		accel,
		gyro_scaled
	};
}
ScaledValues MPU6050::get_scaled_values() {
	auto [accel, gyro, temp] = get_raw_values();

	array<float, 3> accel_scaled;
	array<float, 3> gyro_scaled;
	for (uint32_t i = 0; i < 3; i++) {
		accel_scaled[i] = static_cast<float>(accel[i]) / _accel_scale_factor;
		gyro_scaled[i] = static_cast<float>(gyro[i]) / _gyro_scale_factor;
	}

	float temp_scaled = static_cast<float>(temp) / 340.0f + 36.53f;

	return {accel_scaled, gyro_scaled, temp_scaled};
}



array<int16_t, 3> MPU6050::read_accel_factory_trim() {
	uint8_t address = static_cast<uint8_t>(_address);
	uint8_t reg = static_cast<uint8_t>(Register::XA_OFFS_USRH);
	array<uint8_t, 6> buffer;
	i2c_write_blocking(_i2c, address, &reg, 1, true);
	i2c_read_blocking(_i2c, address, &buffer[0], 6, false);
	return {
		static_cast<int16_t>((buffer[0] << 8) | buffer[1]),
		static_cast<int16_t>((buffer[2] << 8) | buffer[3]),
		static_cast<int16_t>((buffer[4] << 8) | buffer[5])
	};
}
void MPU6050::update_accel_offset_registers(array<int16_t, 3> &accel_offsets) {
	auto accel_factory_trim = read_accel_factory_trim();
	int16_t mask = 0x0001;
	uint8_t mask_bit[3] = {0, 0, 0};

	for (uint8_t i = 0; i < 3; i++) {
		if (accel_factory_trim[i] & mask) {
			mask_bit[i] = 0x01;
		}
		accel_factory_trim[i] -= accel_offsets[i] / 8;
	}

	uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	data[0] = (accel_factory_trim[0] >> 8) & 0xff;
	data[1] = (accel_factory_trim[0])      & 0xff;
	data[1] = data[1] | mask_bit[0];
	data[2] = (accel_factory_trim[1] >> 8) & 0xff;
	data[3] = (accel_factory_trim[1])      & 0xff;
	data[3] = data[3] | mask_bit[1];
	data[4] = (accel_factory_trim[2] >> 8) & 0xff;
	data[5] = (accel_factory_trim[2])      & 0xff;
	data[5] = data[5] | mask_bit[2];

	write_byte(Register::XA_OFFS_USRH, data[0]);
	write_byte(Register::XA_OFFS_USRL, data[1]);
	write_byte(Register::YA_OFFS_USRH, data[2]);
	write_byte(Register::YA_OFFS_USRL, data[3]);
	write_byte(Register::ZA_OFFS_USRH, data[4]);
	write_byte(Register::ZA_OFFS_USRL, data[5]);
}
void MPU6050::update_gyro_offset_registers(array<int16_t, 3> &gyro_offsets) {
	float divisor;
	switch (_gyro_full_scale_select) {
		case GYRO_CONFIG::FS_SELECT_250_DEG_PER_SEC_BIT:
			divisor = 4.0f;
			break;
		case GYRO_CONFIG::FS_SELECT_500_DEG_PER_SEC_BIT:
			divisor = 2.0f;
			break;
		case GYRO_CONFIG::FS_SELECT_1000_DEG_PER_SEC_BIT:
			divisor = 1.0f;
			break;
		case GYRO_CONFIG::FS_SELECT_2000_DEG_PER_SEC_BIT:
		default:
			divisor = 0.5;
	}
	array<int16_t, 3> gyro_bias;
	for (uint32_t i = 0; i < 3; i++) {
		gyro_bias[i] = -static_cast<int16_t>(
			static_cast<float>(gyro_offsets[i]) / divisor
		);
	}

	uint8_t data[6] = {
		static_cast<uint8_t>((gyro_bias[0] >> 8) & 0xff),
		static_cast<uint8_t>((gyro_bias[0])      & 0xff),
		static_cast<uint8_t>((gyro_bias[1] >> 8) & 0xff),
		static_cast<uint8_t>((gyro_bias[1])      & 0xff),
		static_cast<uint8_t>((gyro_bias[2] >> 8) & 0xff),
		static_cast<uint8_t>((gyro_bias[2])      & 0xff)
	};

	write_byte(Register::XG_OFFS_USRH, data[0]);
	write_byte(Register::XG_OFFS_USRL, data[1]);
	write_byte(Register::YG_OFFS_USRH, data[2]);
	write_byte(Register::YG_OFFS_USRL, data[3]);
	write_byte(Register::ZG_OFFS_USRH, data[4]);
	write_byte(Register::ZG_OFFS_USRL, data[5]);
}