# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(touchmatrix-pico C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_compile_definitions(TM_4)

# load library
add_subdirectory(library/i2c_slave)

# include directories
include_directories(config)
include_directories(include)

# add executable
set(src_dir ${PROJECT_SOURCE_DIR}/src)
add_executable(touchmatrix-pico
        ${src_dir}/ADS8866.cpp
        ${src_dir}/peripheral.cpp
        ${src_dir}/touchmatrix-pico.cpp
        ${src_dir}/transfer.cpp)

# generate pio header
pico_generate_pio_header(touchmatrix-pico ${PROJECT_SOURCE_DIR}/pio/shift_register.pio)
pico_generate_pio_header(touchmatrix-pico ${PROJECT_SOURCE_DIR}/pio/ws2812.pio)

# setup pico program
pico_set_program_name(touchmatrix-pico "touchmatrix-pico")
pico_set_program_version(touchmatrix-pico "0.1")

# setup pico stdout
pico_enable_stdio_uart(touchmatrix-pico 0)
pico_enable_stdio_usb(touchmatrix-pico 1)

# Add the standard library to the build
target_link_libraries(touchmatrix-pico pico_stdlib)

# Add any user requested libraries
target_link_libraries(touchmatrix-pico
        hardware_spi
        hardware_i2c
        hardware_pio
        hardware_dma
        hardware_flash
        pico_multicore
        i2c_slave
        )

pico_add_extra_outputs(touchmatrix-pico)

#
#   touchmatrix BRIDGE
#

add_executable(touchmatrix-pico-bridge
        ${src_dir}/ADS8866.cpp
        ${src_dir}/peripheral.cpp
        ${src_dir}/touchmatrix-pico-bridge.cpp
        ${src_dir}/transfer.cpp)

# generate pio header
pico_generate_pio_header(touchmatrix-pico-bridge ${PROJECT_SOURCE_DIR}/pio/shift_register.pio)
pico_generate_pio_header(touchmatrix-pico-bridge ${PROJECT_SOURCE_DIR}/pio/ws2812.pio)

# setup pico program
pico_set_program_name(touchmatrix-pico-bridge "tm-bridge")
pico_set_program_version(touchmatrix-pico-bridge "0.1")

# setup pico stdout
pico_enable_stdio_uart(touchmatrix-pico-bridge 0)
pico_enable_stdio_usb(touchmatrix-pico-bridge 1)

# Add the standard library to the build
target_link_libraries(touchmatrix-pico-bridge pico_stdlib)

# Add any user requested libraries
target_link_libraries(touchmatrix-pico-bridge
        hardware_spi
        hardware_i2c
        hardware_pio
        hardware_dma
        hardware_timer
        pico_multicore
        i2c_slave
        )

pico_add_extra_outputs(touchmatrix-pico-bridge)

add_subdirectory(pcb_test)

