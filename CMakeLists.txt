# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico-mpu6050-driver-cpp VERSION 0.0.1
	DESCRIPTION "A collection of libraries and examples for the Pi Pico."
	LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

pico_enable_stdio_usb(${PROJECT_NAME} 0)
pico_enable_stdio_uart(${PROJECT_NAME} 1)

set(EXAMPLES_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/examples/mpu-6050/src)
set(MPU_DRIVER_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/mpu-6050-driver/src)

include_directories(${EXAMPLES_SRC_DIR})
include_directories(${MPU_DRIVER_SRC_DIR})

set(SOURCES
	${EXAMPLES_SRC_DIR}/main.cpp

	${MPU_DRIVER_SRC_DIR}/mpu6050.hpp
	${MPU_DRIVER_SRC_DIR}/mpu6050.cpp
	${MPU_DRIVER_SRC_DIR}/mpu6050_config.hpp
	${MPU_DRIVER_SRC_DIR}/circular_buffer.hpp
	${MPU_DRIVER_SRC_DIR}/complementary_filter.hpp
	${MPU_DRIVER_SRC_DIR}/median_filter.hpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

pico_add_extra_outputs(${PROJECT_NAME})

set(PICO_SERVO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/submodules/pico-servo)
include_directories(${PICO_SERVO_DIR})
include_directories(${PICO_SERVO_DIR}/src)
include_directories(${PICO_SERVO_DIR}/include)
add_subdirectory(${PICO_SERVO_DIR})

target_link_libraries(${PROJECT_NAME}
	pico-servo
)
target_link_libraries(${PROJECT_NAME}
	pico_stdlib
	hardware_i2c
	hardware_pwm
	hardware_clocks
)


add_compile_options(
	-Wall -Wextra -Wpedantic
)
