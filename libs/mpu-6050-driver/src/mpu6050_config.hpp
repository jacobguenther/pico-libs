// File: main.cpp
// Author: Jacob Guenther
// Date Created: 15 May 2021
// License: AGPLv3
// 
// Resources:
//   datasheet - https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
//   register map - https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
//   offset registers - https://www.digikey.com/en/pdf/i/invensense/mpu-hardware-offset-registers
//   3rd party register map - https://www.i2cdevlib.com/devices/mpu6050#registers

#ifndef MPU6050_CONFIG_HPP
#define MPU6050_CONFIG_HPP

enum class MPU6050Address {
	DEFAULT  = 0x68,
	AD0_HIGH = 0x69
};

enum class PWR_MGMT_1: uint8_t {
	RESET_BIT        = 0x80,
	SLEEP_BIT        = 0x40,
	CYCLE_BIT        = 0x20,
	TEMP_DISABLE_BIT = 0x08,

	CLOCK_SELECT_POSITION = 0x00,
	CLOCK_SELECT_LENGTH = 0x03,

	CLOCK_SELECT_INTERNAL = 0x00,
	CLOCK_SELECT_PLL_WITH_X_AXIS_GYRO_REF = 0x01,
	CLOCK_SELECT_PLL_WITH_Y_AXIS_GYRO_REF = 0x02,
	CLOCK_SELECT_PLL_WITH_Z_AXIS_GYRO_REF = 0x03,
	CLOCK_SELECT_PLL_WITH_EXTERNAL_32_768kHz_REF = 0x04,
	CLOCK_SELECT_PLL_WITH_EXTERNAL_19_2MHz_REF   = 0x05,
	CLOCK_SELECT_STOP = 0x07
};
enum class PWR_MGMT_2: uint8_t {
	LP_WAKE_POSITION = 0x06,
	LP_WAKE_LENGTH = 0x02,

	LP_WAKE_CTRL_1_25_Hz = 0x00,
	LP_WAKE_CTRL_5_Hz    = 0x01,
	LP_WAKE_CTRL_20_Hz   = 0x02,
	LP_WAKE_CTRL_40_Hz   = 0x03,

	STBY_XA_BIT = 0x20,
	STBY_YA_BIT = 0x10,
	STBY_ZA_BIT = 0x08,
	STBY_XG_BIT = 0x04,
	STBY_YG_BIT = 0x02,
	STBY_ZG_BIT = 0x01
};
enum class SIGNAL_PATH_RESET: uint8_t {
	RESET_ALL = 0b00000111,
	GYRO_RESET = 0x04,
	ACCEL_RESET = 0x02,
	TEMP_RESET = 0x01,
};
enum class INTERRUPT_ENABLE: uint8_t {
	DATA_READY_ENABLE_BIT           = 0x01,
	I2C_MASTER_INTERRUPT_ENABLE_BIT = 0x08,
	FIFO_OVERFLOW_ENABLE_BIT        = 0x10
};
enum class ACCEL_CONFIG: uint8_t {
	FS_SELECT_POSITION = 0x03,
	FS_SELECT_LENGTH = 0x02,

	FS_SELECT_2_G_BIT  = 0x00,
	FS_SELECT_4_G_BIT  = 0x01,
	FS_SELECT_8_G_BIT  = 0x02,
	FS_SELECT_16_G_BIT = 0x03
};
enum class GYRO_CONFIG: uint8_t {
	FS_SELECT_POSITION = 0x03,
	FS_SELECT_LENGTH = 0x02,

	FS_SELECT_250_DEG_PER_SEC_BIT  = 0x00,
	FS_SELECT_500_DEG_PER_SEC_BIT  = 0x01,
	FS_SELECT_1000_DEG_PER_SEC_BIT = 0x02,
	FS_SELECT_2000_DEG_PER_SEC_BIT = 0x03
};
enum class CONFIG: uint8_t {
	DLPF_CFG_BANDWIDTH_260_Hz = 0x00,
	DLPF_CFG_BANDWIDTH_184_Hz = 0x01,
	DLPF_CFG_BANDWIDTH_94_Hz  = 0x02,
	DLPF_CFG_BANDWIDTH_44_Hz  = 0x03,
	DLPF_CFG_BANDWIDTH_21_Hz  = 0x04,
	DLPF_CFG_BANDWIDTH_10_Hz  = 0x05,
	DLPF_CFG_BANDWIDTH_5_Hz   = 0x06
};

enum class Register: uint8_t {
	X_FINE_GAIN        = 0x03,
	Y_FINE_GAIN        = 0x04,
	Z_FINE_GAIN        = 0x05,

	XA_OFFS_USRH       = 0x06,
	XA_OFFS_USRL       = 0x07,
	YA_OFFS_USRH       = 0x08,
	YA_OFFS_USRL       = 0x09,
	ZA_OFFS_USRH       = 0x0A,
	ZA_OFFS_USRL       = 0x0B,

	SELF_TEXT_X        = 0x0D,
	SELF_TEXT_Y        = 0x0E,
	SELF_TEXT_Z        = 0x0F,
	SELF_TEXT_A        = 0x10,

	XG_OFFS_USRH       = 0x13,
	XG_OFFS_USRL       = 0x14,
	YG_OFFS_USRH       = 0x15,
	YG_OFFS_USRL       = 0x16,
	ZG_OFFS_USRH       = 0x17,
	ZG_OFFS_USRL       = 0x18,

	SMPLRT_DIV         = 0x19,
	CONFIG             = 0x1A,
	GYRO_CONFIG        = 0x1B,
	ACCEL_CONFIG       = 0x1C,

	FIFO_EN            = 0x23,
	I2C_MST_CTRL       = 0x24,
	I2C_SLV0_ADDR      = 0x25,
	I2C_SLV0_REG       = 0x26,
	I2C_SLV0_CTRL      = 0x27,
	I2C_SLV1_ADDR      = 0x28,
	I2C_SLV1_REG       = 0x29,
	I2C_SLV1_CTRL      = 0x2A,
	I2C_SLV2_ADDR      = 0x2B,
	I2C_SLV2_REG       = 0x2C,
	I2C_SLV2_CTRL      = 0x2D,
	I2C_SLV3_ADDR      = 0x2E,
	I2C_SLV3_REG       = 0x2F,
	I2C_SLV3_CTRL      = 0x30,
	I2C_SLV4_ADDR      = 0x31,
	I2C_SLV4_REG       = 0x32,
	I2C_SLV4_DO        = 0x33,
	I2C_SLV4_CTRL      = 0x34,
	I2C_SLV4_DI        = 0x35,
	I2C_MST_STATUS     = 0x36,
	INT_PIN_CFG        = 0x37,
	INT_ENABLE         = 0x38,
	INT_STATUS         = 0x3A,
	ACCEL_XOUT_H       = 0x3B,
	ACCEL_XOUT_L       = 0x3C,
	ACCEL_YOUT_H       = 0x3D,
	ACCEL_YOUT_L       = 0x3E,
	ACCEL_ZOUT_H       = 0x4F,
	ACCEL_ZOUT_L       = 0x40,
	TEMP_OUT_H         = 0x41,
	TEMP_OUT_L         = 0x42,
	GYRO_XOUT_H        = 0x43,
	GYRO_XOUT_L        = 0x44,
	GYRO_YOUT_H        = 0x45,
	GYRO_YOUT_L        = 0x46,
	GYRO_ZOUT_H        = 0x47,
	GYRO_ZOUT_L        = 0x48,
	EXT_SENS_DATA_00   = 0x49,
	EXT_SENS_DATA_01   = 0x4A,
	EXT_SENS_DATA_02   = 0x4B,
	EXT_SENS_DATA_03   = 0x4C,
	EXT_SENS_DATA_04   = 0x4D,
	EXT_SENS_DATA_05   = 0x4E,
	EXT_SENS_DATA_06   = 0x4F,
	EXT_SENS_DATA_07   = 0x50,
	EXT_SENS_DATA_08   = 0x51,
	EXT_SENS_DATA_09   = 0x52,
	EXT_SENS_DATA_10   = 0x53,
	EXT_SENS_DATA_11   = 0x54,
	EXT_SENS_DATA_12   = 0x55,
	EXT_SENS_DATA_13   = 0x56,
	EXT_SENS_DATA_14   = 0x57,
	EXT_SENS_DATA_15   = 0x58,
	EXT_SENS_DATA_16   = 0x59,
	EXT_SENS_DATA_17   = 0x5A,
	EXT_SENS_DATA_18   = 0x5B,
	EXT_SENS_DATA_19   = 0x5C,
	EXT_SENS_DATA_20   = 0x5D,
	EXT_SENS_DATA_21   = 0x5E,
	EXT_SENS_DATA_22   = 0x5F,
	EXT_SENS_DATA_23   = 0x60,
	I2C_SLV0_DO        = 0x63,
	I2C_SLV1_D0        = 0x64,
	I2C_SLV2_D0        = 0x65,
	I2C_SLV3_D0        = 0x66,
	I2C_MST_DELAY_CTRL = 0x67,
	SIGNAL_PATH_RESET  = 0x68,
	USER_CTRL          = 0x6A,
	PWR_MGMT_1         = 0x6B,
	PWR_MGMT_2         = 0x6C,
	FIFO_COUNTH        = 0x72,
	FIFO_COUNTL        = 0x73,
	FIFO_R_W           = 0x74,
	WHO_AM_I           = 0x75
};

#endif
