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
#include "keyboard_program.pio.h"

template<uint8_t row_count, uint8_t col_count>
class PIOKeyboard {
public:
	PIOKeyboard(
		uint8_t first_row_pin,
		uint8_t first_col_pin,
		PIO pio
	)
		: _first_row_pin{first_row_pin}
		, _first_col_pin{first_col_pin}
		, _pio{pio}
	{
		assert(row_count <= 30 - 5);
		assert(col_count <= 5);
		_key_events.reserve(KEY_COUNT);

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
		// std::vec<uint32_t> states;
		// while available() {
		// 	states = ~pio_sm_get(_pio, _state_machine);
		// }
		const uint32_t current_state{~pio_sm_get(_pio, _state_machine)};
		if (_previous_state != current_state) {
			for (uint8_t key_num = 0; key_num < KEY_COUNT; key_num++) {
				const uint32_t mask{1U << key_num};
				const bool is_pressed{(current_state & mask) != 0U};
				const bool was_pressed{(_previous_state & mask) != 0U};
				if (is_pressed != was_pressed) {
					const auto event_type = static_cast<KeyEventE>(KeyEventE::KEY_UP - static_cast<int>(is_pressed));
					const uint8_t key_index{static_cast<uint8_t>(KEY_COUNT - key_num)};
					_key_events.push_back(KeyEvent { event_type, key_index });
				}
			}
			_previous_state = current_state;
		}
	}
	void print_key_events() const {
		for (const auto& event : _key_events) {
			switch (event.event_type) {
				case KeyEventE::KEY_DOWN:
					printf("down");
					break;
				case KeyEventE::KEY_UP:
					printf("up  ");
					break;
			}
			printf(" %i\n", event.key_index);
		}
		printf("\n");
	}
	KeyEvent const* get_event_ptr(size_t* event_count) {
		*event_count = _key_events.size();
		return &_key_events[0];
	} 
	void clear_events() {
		_key_events.clear();
	}
private:
	uint8_t _first_row_pin;
	uint8_t _first_col_pin;

	const static uint8_t KEY_COUNT{row_count * col_count};

	std::vector<KeyEvent> _key_events{std::vector<KeyEvent>()};

	PIO _pio{pio0};
	uint32_t _state_machine{0};
	uint32_t _previous_state{0};
};

#endif
