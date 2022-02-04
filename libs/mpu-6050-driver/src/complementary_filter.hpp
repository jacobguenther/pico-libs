// File: complementary_filter.hpp
// Author: Jacob Guenther
// Date Created: 15 May 2021
// License: AGPLv3

#ifndef COMPLEMENTARY_FILTER_HPP
#define COMPLEMENTARY_FILTER_HPP

constexpr float RAD_2_DEG{57.295779513F};
constexpr float DEFULAT_DT{1.0F/60.0F};
constexpr float DEFAULT_GYRO_BIAS{0.975F};

#include "pico/stdlib.h"
#include "pico/types.h"

#include <algorithm> // nth_element
#include <array>     // array
#include <cmath>     // atan2

class ComplementaryFilter {
public:
	ComplementaryFilter()=default;
	ComplementaryFilter(float dt, float gyro_bias)
		: _dt{dt}
		, _gyro_bias{gyro_bias}
		, _accel_bias{1.0F - gyro_bias}
	{}
	~ComplementaryFilter()=default;

	ComplementaryFilter(const ComplementaryFilter&)=delete;
	ComplementaryFilter(const ComplementaryFilter&&)=delete;
	ComplementaryFilter& operator=(const ComplementaryFilter&)=delete;
	ComplementaryFilter& operator=(const ComplementaryFilter&&)=delete;

	void update(const std::array<int16_t, 3> &accel, const std::array<float, 3> &gyro) {
		const int16_t accel_x{accel[0]};
		const int16_t accel_y{accel[1]};
		const int16_t accel_z{accel[2]};
		const int32_t accel_x_squared{accel_x*accel_x};
		const int32_t accel_y_squared{accel_y*accel_y};
		const int32_t accel_z_squared{accel_z*accel_z};

		const float accel_angle_pitch = std::atan2(
				static_cast<float>(accel_x),
				std::sqrt(static_cast<float>(accel_y_squared + accel_z_squared))
			) * RAD_2_DEG;
		const float accel_angle_roll = std::atan2(
				static_cast<float>(accel_y),
				std::sqrt(static_cast<float>(accel_x_squared + accel_z_squared))
			) * RAD_2_DEG;

		_pitch = _gyro_bias * (_pitch - gyro[1]*_dt)
			+ _accel_bias * accel_angle_pitch;
		_roll = _gyro_bias * (_roll + gyro[0]*_dt)
			+ _accel_bias * accel_angle_roll;
	}
	std::tuple<float, float> get_filtered_angles() {
		return {_pitch, _roll};
	}
private:
	float _dt{DEFULAT_DT};

	float _gyro_bias{DEFAULT_GYRO_BIAS};
	float _accel_bias{1.0F - _gyro_bias};

	float _pitch{0.0F};
	float _roll{0.0F};
};

#endif
