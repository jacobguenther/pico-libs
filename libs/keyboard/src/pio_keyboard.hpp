// File: keyboard.hpp
// Author: Jacob Guenther
// Date Created: 13 July 2021
// License: AGPLv3

#ifndef PIO_KEYBOARD_HPP
#define PIO_KEYBOARD_HPP

#include <cstdio>

#include <vector>

#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "key_event.hpp"
#include "../build/keyboard_program.pio.h"

template<uint8_t row_count, uint8_t col_count>
class PIOKeyboard {
public:
	PIOKeyboard(
		uint8_t first_row_pin,
		uint8_t first_col_pin,
		PIO pio = pio0
	)
		: _first_row_pin{first_row_pin}
		, _first_col_pin{first_col_pin}
		, _key_events{std::vector<KeyEvent>()}
		, _pio{pio}
		, _state_machine{0}
		, _previous_state{0}
	{
		_key_events.reserve(row_count * col_count);

		uint32_t offset{0};
		if (pio_can_add_program(_pio, &keyboard_program)) {
			offset = pio_add_program(_pio, &keyboard_program);
		}
		keyboard_program_init(_pio, _state_machine, offset, _first_col_pin, col_count, _first_row_pin, row_count);
		pio_sm_set_enabled(_pio, _state_machine, true);
	}

	bool available() {
		return !pio_sm_is_rx_fifo_empty(_pio, _state_machine);
	}
	void poll_buttons() {
		while (available()) {
			const uint32_t current_state = ~pio_sm_get(_pio, _state_machine);
			if (_previous_state != current_state) {
				const uint8_t key_count = row_count * col_count;
				for (uint8_t i = 0; i < key_count; i++) {
					const uint32_t mask = 1U << i;
					const bool is_pressed = (current_state & mask) != 0U;
					const bool was_pressed = (_previous_state & mask) != 0U;
					if (is_pressed != was_pressed) {
						const auto event_type = static_cast<KeyEventE>(KeyEventE::KEY_UP - static_cast<int>(is_pressed));
						_key_events.push_back(KeyEvent { event_type, i });
					}
				}
				_previous_state = current_state;
			}
		}
	}
	void print_key_events() const {
		for (const auto event : _key_events) {
			switch (event.event_type) {
				case KeyEventE::KEY_DOWN:
					printf("down");
					break;
				case KeyEventE::KEY_UP:
					printf("up  ");
					break;
			}
			printf(" %i\n", event.id);
		}
		printf("\n");
	}
	void clear_events() {
		_key_events.clear();
	}
private:
	uint8_t _first_row_pin;
	uint8_t _first_col_pin;

	std::vector<KeyEvent> _key_events;

	PIO _pio;
	uint32_t _state_machine;
	uint32_t _previous_state;
};

#endif
