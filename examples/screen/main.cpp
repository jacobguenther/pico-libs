// File: main.cpp
// Author: Jacob Guenther
// Date Created: 31 July 2021
// License: AGPLv3


#include <cstdio>

#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include <cwchar> // wchar.h

extern "C" {
	#include <hagl_hal.h>
	#include <hagl.h>
	#include <font6x9.h>
	#include <fps.h>
	#include <aps.h>
}

const color_t red = hagl_color(255, 0, 0);
const color_t green = hagl_color(0, 255, 0);
const color_t blue = hagl_color(0, 0, 255);
const color_t black = hagl_color(0, 0, 0);
const color_t white = hagl_color(255, 255, 255);

constexpr int16_t header_offset{2};
constexpr int16_t header_height{14};

int main() {
	stdio_init_all();
	printf("Starting up screen example.\n");

	bi_decl(bi_program_name("screen_example"));
	bi_decl(bi_program_description("Demonstrates using the hagl library with an ST7735 screen driver."));
	bi_decl(bi_program_version_string("1.0.0"));
	bi_decl(bi_program_url("https://github.com/jacobguenther/pico-libs"));
	bi_decl(bi_program_feature("License AGPLv3"));

	hagl_init();
	hagl_clear_screen();
	hagl_set_clip_window(0, 0, DISPLAY_WIDTH -1, DISPLAY_HEIGHT - 1);
	size_t bytes = hagl_flush();

	printf("Entering main loop.\n");

	while (true) {
		sleep_ms(10);
		hagl_clear_screen();

		wchar_t message[32] = L"Screen Header";
		hagl_put_text(message, 6, 4, red, font6x9);
		hagl_draw_rounded_rectangle(header_offset, header_offset, DISPLAY_WIDTH - header_offset - 1, header_height, 5, red);

		bytes = hagl_flush();
	}

	hagl_close();

	return 0;
}