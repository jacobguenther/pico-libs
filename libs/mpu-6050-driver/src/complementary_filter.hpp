// File: complementary_filter.hpp
// Author: Jacob Guenther
// Date Created: 15 May 2021
// License: AGPLv3

#ifndef COMPLEMENTARY_FILTER_HPP
#define COMPLEMENTARY_FILTER_HPP

#define RAD_2_DEG 57.295779513

#include "pico/stdlib.h"
#include "pico/types.h"

#include <array>
#include <algorithm>
#include <optional>
#include <cmath>

class ComplementaryFilter {
public:
	ComplementaryFilter(float dt=1.0f/60.0f, float gyro_bias=0.975f)
		: _dt{dt}
		, _gyro_bias{gyro_bias}
		, _accel_bias{1.0f - gyro_bias}
		, _pitch{0.0f}
		, _roll{0.0f}
	{}
	~ComplementaryFilter()=default;

	ComplementaryFilter(const ComplementaryFilter&)=delete;
	ComplementaryFilter(const ComplementaryFilter&&)=delete;
	ComplementaryFilter& operator=(const ComplementaryFilter&)=delete;
	ComplementaryFilter& operator=(const ComplementaryFilter&&)=delete;

	void update(std::array<int16_t, 3> &accel, std::array<float, 3> &gyro) {
		int16_t ax = accel[0];
		int16_t ay = accel[1];
		int32_t ax2 = ax*ax;
		int32_t ay2 = ay*ay;
		int32_t az2 = accel[2]*accel[2];

		float accel_angle_pitch = std::atan2(
				static_cast<float>(ax),
				std::sqrt(static_cast<float>(ay2 + az2))
			) * RAD_2_DEG;
		float accel_angle_roll = std::atan2(
				static_cast<float>(ay),
				std::sqrt(static_cast<float>(ax2 + az2))
			) * RAD_2_DEG;

		_pitch = _gyro_bias * (_pitch - gyro[1]*_dt)
			+ _accel_bias * accel_angle_pitch;
		_roll = _gyro_bias * (_roll + gyro[0]*_dt)
			+ _accel_bias * accel_angle_roll;
	}
	std::tuple<float, float> get_filtered_angle() {
		return {_pitch, _roll};
	}
private:
	float _dt;

	float _gyro_bias;
	float _accel_bias;

	float _pitch;
	float _roll;
};

#endif
