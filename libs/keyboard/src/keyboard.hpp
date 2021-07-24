// File: keyboard.hpp
// Author: Jacob Guenther
// Date Created: 13 July 2021
// License: AGPLv3

#ifndef KEYBOARD_HPP
#define KEYBOARD_HPP

#include <cstdio>

#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "key_event.hpp"
#include "../build/keyboard_program.pio.h"

bool keyboard_callback(repeating_timer *keyboard_timer) {
	bool keyboard_available = true;
	keyboard_timer->user_data = static_cast<void*>(&keyboard_available);
	// what does the return value do?
	return true;
}

template<uint8_t row_count, uint8_t col_count>
class Keyboard {
public:
	Keyboard(
		uint8_t first_row_pin,
		uint8_t first_col_pin
	)
		: _first_row_pin{first_row_pin}
		, _first_col_pin{first_col_pin}
		, _key_event_count{0}
		, _keys_pressed{}
		, _key_events{}
	{
		const uint8_t key_count = row_count * col_count;
		for (uint8_t key = 0; key < key_count; key++) {
			_keys_pressed[key] = false;
		}

		for (uint8_t row = 0; row < row_count; row++) {
			const uint8_t row_pin = _first_row_pin + row;
			gpio_init(row_pin);
			gpio_set_dir(row_pin, GPIO_IN);
			gpio_pull_up(row_pin);
		}

		for (uint8_t col = 0; col < col_count; col++) {
			const uint8_t col_pin = _first_col_pin + col;
			gpio_init(col_pin);
			gpio_set_dir(col_pin, GPIO_OUT);
			gpio_put(col_pin, 1);
		}
	}

	void poll_buttons() {
		uint8_t key_index = 0;
		for (uint8_t col = 0, col_pin = _first_col_pin; col < col_count; col++, col_pin++) {
			gpio_put(col_pin, 0);

			for (uint8_t row = 0; row < row_count; row++) {
				const uint8_t row_pin = _first_row_pin + row;

				const bool previous_state = _keys_pressed[key_index] == 0;
				const bool is_down = gpio_get(row_pin) != 0;
				_keys_pressed[key_index] = is_down;
				
				if (is_down != previous_state) {
					auto event_type = static_cast<KeyEventE>(KeyEventE::KEY_UP - static_cast<int>(is_down));
					_key_events[_key_event_count] = KeyEvent {event_type, key_index};
					_key_event_count++;
				}

				key_index++;
			}

			gpio_put(col_pin, 1);
		}
	}
	void print_key_events() const {
		for (uint8_t i = 0; i < _key_event_count; i++) {
			const KeyEvent event = _key_events[i];
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
	}
	void clear_events() {
		_key_event_count = 0;
	}
private:
	uint8_t _first_row_pin;
	uint8_t _first_col_pin;

	std::array<bool, row_count * col_count> _keys_pressed;

	std::array<KeyEvent, row_count * col_count> _key_events;
	uint8_t _key_event_count;
};

#endif
