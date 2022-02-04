// File: main.cpp
// Author: Jacob Guenther
// Date Created: 29 July 2021
// License: AGPLv3

#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "keyboard.hpp"

constexpr uint8_t ROW_COUNT{3};
constexpr uint8_t COL_COUNT{3};
constexpr uint8_t FIRAT_ROW_PIN{19};
constexpr uint8_t FIRST_COL_PIN{10};

int main() {
	stdio_init_all();
	printf("Starting up keyboard example.\n");

	bi_decl(bi_program_name("keyboard_example"));
	bi_decl(bi_program_description("Demonstrates polling a button matrix using the Pi Pico."));
	bi_decl(bi_program_version_string("1.0.0"));
	bi_decl(bi_program_url("https://github.com/jacobguenther/pico-libs"));
	bi_decl(bi_program_feature("License AGPLv3"));

	auto keyboard = Keyboard<ROW_COUNT, COL_COUNT>(FIRAT_ROW_PIN, FIRST_COL_PIN);

	printf("Entering main loop.\n");

	while (true) {
		if (keyboard.available()) {
			keyboard.poll_buttons();
			keyboard.print_key_events();
			keyboard.clear_events();
		}
	}

	return 0;
}