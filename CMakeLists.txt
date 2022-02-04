# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(examples VERSION 0.0.1
	DESCRIPTION "A collection of libraries and examples for the Pi Pico."
	LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

set(EXAMPLES_DIR ${PROJECT_SOURCE_DIR}/examples)

set(LIBS_DIR ${PROJECT_SOURCE_DIR}/libs)
set(MPU_6050_SRC_DIR ${LIBS_DIR}/mpu-6050-driver/src)
set(KEYBOARD_SRC_DIR ${LIBS_DIR}/keyboard/src)

set(PICO_SERVO_DIR ${PROJECT_SOURCE_DIR}/submodules/pico-servo)
include_directories(${PICO_SERVO_DIR})
include_directories(${PICO_SERVO_DIR}/src)
include_directories(${PICO_SERVO_DIR}/include)
include_directories($ENV{PICO_SDK_PATH}/src/common/pico_stdlib/include)

add_subdirectory(${PICO_SERVO_DIR})
add_subdirectory(submodules/hagl)
add_subdirectory(submodules/hagl_pico_mipi)

add_subdirectory(examples)

# set(FAMILY rp2040)
# set(BOARD pico_sdk)
# set(TINYUSB_FAMILY_PROJECT_NAME_PREFIX "tinyusb_dev_")

# pico_generate_pio_header(${PROJECT_NAME} ${KEYBOARD_SRC_DIR}/keyboard_program.pio)

#target_link_libraries(${PROJECT_NAME}
#	pico_stdlib
	# hardware_i2c
	# hardware_pwm
	# hardware_clocks
	# hardware_pio
# )

# target_compile_options(
# 	${PROJECT_NAME} PRIVATE -Wall #-Wextra -pedantic -Weffc++
# )
