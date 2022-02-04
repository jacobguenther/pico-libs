add_executable(keyboard_example)

include_directories(${KEYBOARD_SRC_DIR})

target_sources(keyboard_example PRIVATE main.cpp
	${KEYBOARD_SRC_DIR}/keyboard.hpp)

target_link_libraries(keyboard_example PRIVATE
	pico_stdlib)

pico_enable_stdio_usb(keyboard_example 0)
pico_enable_stdio_uart(keyboard_example 1)

pico_add_extra_outputs(keyboard_example)
