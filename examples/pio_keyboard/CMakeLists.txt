add_executable(pio_keyboard_example)

pico_generate_pio_header(pio_keyboard_example ${KEYBOARD_SRC_DIR}/keyboard_program.pio)

target_sources(pio_keyboard_example PRIVATE main.cpp)

include_directories(${KEYBOARD_SRC_DIR})

target_link_libraries(pio_keyboard_example PRIVATE
	pico_stdlib
	hardware_pio)

pico_enable_stdio_usb(pio_keyboard_example 0)
pico_enable_stdio_uart(pio_keyboard_example 1)

pico_add_extra_outputs(pio_keyboard_example)
