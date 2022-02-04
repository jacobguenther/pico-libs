add_executable(combined_example)

pico_generate_pio_header(combined_example ${KEYBOARD_SRC_DIR}/keyboard_program.pio)

include_directories(${KEYBOARD_SRC_DIR})
include_directories(${MPU_6050_SRC_DIR})

target_sources(combined_example PRIVATE main.cpp
	${MPU_6050_SRC_DIR}/mpu6050.cpp
	${MPU_6050_SRC_DIR}/mpu6050.hpp
	${MPU_6050_SRC_DIR}/mpu6050_config.hpp
	${MPU_6050_SRC_DIR}/median_filter.hpp
	${MPU_6050_SRC_DIR}/complementary_filter.hpp)

target_link_libraries(combined_example PRIVATE
	pico_stdlib
	hardware_pio
	hardware_i2c
	hagl
	hagl_hal

)

target_compile_definitions(combined_example PRIVATE
	HAGL_HAL_USE_DOUBLE_BUFFER

	MIPI_DISPLAY_SPI_CLOCK_SPEED_HZ=62000000
	MIPI_DISPLAY_PIN_CLK=2
	MIPI_DISPLAY_PIN_MOSI=3
	MIPI_DISPLAY_PIN_RST=4

	MIPI_DISPLAY_PIN_BL=-1

	MIPI_DISPLAY_PIN_CS=5
	MIPI_DISPLAY_PIN_DC=6

	MIPI_DISPLAY_PIN_MISO=-1

	MIPI_DISPLAY_PIXEL_FORMAT=MIPI_DCS_PIXEL_FORMAT_16BIT
	MIPI_DISPLAY_ADDRESS_MODE=MIPI_DCS_ADDRESS_MODE_BGR|MIPI_DCS_ADDRESS_MODE_FLIP_X|MIPI_DCS_ADDRESS_MODE_MIRROR_X|MIPI_DCS_ADDRESS_MODE_FLIP_Y|MIPI_DCS_ADDRESS_MODE_MIRROR_Y

	MIPI_DISPLAY_WIDTH=128
	MIPI_DISPLAY_HEIGHT=128
	MIPI_DISPLAY_OFFSET_X=2
	MIPI_DISPLAY_OFFSET_Y=3
	MIPI_DISPLAY_INVERT=0
)

pico_enable_stdio_usb(combined_example 0)
pico_enable_stdio_uart(combined_example 1)

pico_add_extra_outputs(combined_example)