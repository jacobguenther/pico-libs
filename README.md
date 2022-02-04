# Pico Libs

A small collection of libraries. Currently they are "built for me" so are incomplete and lacking some functionality. Please feel free to open issues and pull requests.

## Building

git clone https://github.com/jacobguenther/pico-libs.git

git submodules init

git submodules update


mkdir build

cd build

cmake .. && make -j


The cmake files will be improved in the future to build specific examples and include specific libraries.


## Libs

**mpu-6050-driver** - A simple driver for the mpu-6050 accelerometer and gyroscope. It comes with median and complimentary filters to help process the sensor data.

**keyboard** - A class for polling a keyboard or button matrix.

**pio-keyboard** - A PIO program, and helper class for polling a keyboard or button matrix. It is very fast and light on the processor.

## License

These libraries and examples are licensed under the AGPLv3.

Submodules might use different licenses. Refer to them for more information.
