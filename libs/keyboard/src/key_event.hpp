// File: keyboard.hpp
// Author: Jacob Guenther
// Date Created: 13 July 2021
// License: AGPLv3

#ifndef KEY_EVENT_HPP
#define KEY_EVENT_HPP

#include  <cstdint>

enum KeyEventE {
	KEY_DOWN,
	KEY_UP,
};

struct KeyEvent {
	KeyEventE event_type;
	uint8_t id;
};

#endif