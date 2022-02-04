add_executable(mpu_6050_example)

include_directories(${MPU_6050_SRC_DIR})

target_sources(mpu_6050_example PRIVATE main.cpp

	${MPU_6050_SRC_DIR}/mpu6050.cpp
	${MPU_6050_SRC_DIR}/mpu6050.hpp
	${MPU_6050_SRC_DIR}/mpu6050_config.hpp
	${MPU_6050_SRC_DIR}/median_filter.hpp
	${MPU_6050_SRC_DIR}/complementary_filter.hpp)

target_link_libraries(mpu_6050_example PRIVATE
	pico_stdlib
	hardware_i2c
	pico-servo)

pico_enable_stdio_usb(mpu_6050_example 0)
pico_enable_stdio_uart(mpu_6050_example 1)

pico_add_extra_outputs(mpu_6050_example)