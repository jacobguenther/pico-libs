// File: main.cpp
// Author: Jacob Guenther
// Date Created: 31 July 2021
// License: AGPLv3


#include <cstdio>

#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include <array> 
#include <cwchar> // wchar.h
#include <cstring> // memcpy
#include <memory> // tie

extern "C" {
	#include <hagl_hal.h>
	#include <hagl.h>
	#include <font6x9.h>
	#include <fps.h>
	#include <aps.h>
}

#include "pio_keyboard.hpp"

#include "mpu6050.hpp"
#include "mpu6050_config.hpp"
#include "median_filter.hpp"
#include "complementary_filter.hpp"

constexpr uint8_t ROW_COUNT{3};
constexpr uint8_t COL_COUNT{3};
constexpr uint8_t FIRAT_ROW_PIN{19};
constexpr uint8_t FIRST_COL_PIN{10};



const color_t red = hagl_color(255, 0, 0);
const color_t green = hagl_color(0, 255, 0);
const color_t blue = hagl_color(0, 0, 255);
const color_t black = hagl_color(0, 0, 0);
const color_t white = hagl_color(255, 255, 255);

constexpr int16_t header_offset_x{2};
constexpr int16_t header_height{12};
constexpr int16_t header_following_space{2};
constexpr int16_t header_width{DISPLAY_WIDTH - 1 - header_offset_x};

constexpr int16_t menu_item_width{40};
constexpr int16_t menu_item_x_spacing{2};

constexpr int16_t header_text_offset_x{2};
constexpr int16_t header_text_offset_y{2};

constexpr int16_t line_height{9};
constexpr int16_t line_indent{4};

// struct AppState {
// 	key_events;
// }
struct ScreenText {
	std::array<char[32], 3> top_menue = {"Events", "Config", "Other"};
	char events_header[32] = "Key Events";
	size_t most_recent_event_line{0};
	std::array<char[32], 5> scrolling_event_lines;
	char mpu_header[32] = "MPU-6050";
	float pitch{0.0F};
	float roll{0.0F};
};

void draw_screen_text(ScreenText* screen_text) {
		int16_t y_pos{header_following_space};
		int16_t x_pos{header_offset_x};
		wchar_t line[32];

		// Draw key event header
		auto first{true};
		color_t color = green;
		for (const auto& menu_text: screen_text->top_menue) {
			swprintf(line, sizeof(line), L"%s", menu_text);
			hagl_draw_rounded_rectangle(
				x_pos,
				y_pos,
				x_pos + menu_item_width,
				y_pos + header_height,
				2, color);
			hagl_put_text(line,
				x_pos + header_text_offset_x,
				y_pos + header_text_offset_y,
				color, font6x9);
			x_pos += menu_item_width + menu_item_x_spacing;
			if (first) {
				first = false;
				color = red;
			}
		}

		y_pos += header_height + header_following_space;
		
		swprintf(line, sizeof(line), L"%s", screen_text->events_header);
		hagl_draw_rounded_rectangle(
			header_offset_x,
			y_pos,
			header_width,
			y_pos + header_height,
			2, red);
		hagl_put_text(line,
			header_offset_x + header_text_offset_x,
			y_pos + header_text_offset_y,
			red, font6x9);
		y_pos += header_height + header_following_space;

		// Draw key events
		for (size_t i = screen_text->most_recent_event_line; i < 5; i++) {
			swprintf(line, sizeof(line), L"%s", screen_text->scrolling_event_lines[i]);
			hagl_put_text(line, line_indent, y_pos, blue, font6x9);
			y_pos += line_height;
		}
		for (size_t i = 0; i < screen_text->most_recent_event_line; i++) {
			swprintf(line, sizeof(line), L"%s", screen_text->scrolling_event_lines[i]);
			hagl_put_text(line, line_indent, y_pos, blue, font6x9);
			y_pos += line_height;
		}

		// draw mpu header
		swprintf(line, sizeof(line), L"%s", screen_text->mpu_header);
		hagl_put_text(line,
			header_offset_x + header_text_offset_x,
			y_pos + header_text_offset_y,
			red, font6x9);
		hagl_draw_rounded_rectangle(
			header_offset_x,
			y_pos,
			header_width,
			y_pos + header_height,
			2, red);
		y_pos += header_height + header_following_space;

		// draw mpu values
		swprintf(line, sizeof(line), L"Pitch: %.2f", screen_text->pitch);
		hagl_put_text(line, line_indent, y_pos, blue, font6x9);
		y_pos += line_height;

		swprintf(line, sizeof(line), L"Roll:  %.2f", screen_text->roll);
		hagl_put_text(line, line_indent, y_pos, blue, font6x9);
}

int main() {
	stdio_init_all();
	printf("Starting up combined example.\n");

	bi_decl(bi_program_name("combined_example"));
	bi_decl(bi_program_description("Demonstrates using the all of the libraries in this repository to create a controller."));
	bi_decl(bi_program_version_string("1.0.0"));
	bi_decl(bi_program_url("https://github.com/jacobguenther/pico-libs"));
	bi_decl(bi_program_feature("License AGPLv3"));

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

	auto pio_keyboard = PIOKeyboard<ROW_COUNT, COL_COUNT>(FIRAT_ROW_PIN, FIRST_COL_PIN, pio0);

	ScreenText screen_text;
	for (auto& line: screen_text.scrolling_event_lines) {
		sprintf(line, "0");
	}

	hagl_init();
	hagl_clear_screen();
	hagl_set_clip_window(0, 0, DISPLAY_WIDTH -1, DISPLAY_HEIGHT - 1);
	size_t bytes = hagl_flush();

	printf("Entering main loop.\n");
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
			std::tie(screen_text.pitch, screen_text.roll) = comp_filter.get_filtered_angles();

			// printf("pitch: %.2f roll: %.2f\n", pitch, roll);
		}

		if (pio_keyboard.available()) {
			pio_keyboard.poll_buttons();
			pio_keyboard.print_key_events();

			size_t event_count{0};
			auto events_ptr = pio_keyboard.get_event_ptr(&event_count);
			for (size_t i = 0; i < event_count; i++) {
				if (screen_text.most_recent_event_line == 0) {
					screen_text.most_recent_event_line = 4;
				} else {
					screen_text.most_recent_event_line--;
				}

				const auto event = events_ptr[i];
				switch (event.event_type) {
					case KeyEventE::KEY_UP:
						sprintf(screen_text.scrolling_event_lines[screen_text.most_recent_event_line], "UP   %i", event.key_index);
						break;
					case KeyEventE::KEY_DOWN:
						sprintf(screen_text.scrolling_event_lines[screen_text.most_recent_event_line], "DOWN %i", event.key_index);
				}
			}
			pio_keyboard.clear_events();
		}

		hagl_clear_screen();
		draw_screen_text(&screen_text);
		bytes = hagl_flush();
	}

	hagl_close();

	return 0;
}