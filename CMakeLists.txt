# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/ura/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(touchmatrix-pico C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(touchmatrix-pico touchmatrix-pico.cpp)

pico_generate_pio_header(touchmatrix-pico ${CMAKE_CURRENT_LIST_DIR}/shift_register.pio)

pico_set_program_name(touchmatrix-pico "touchmatrix-pico")
pico_set_program_version(touchmatrix-pico "0.1")

pico_enable_stdio_uart(touchmatrix-pico 0)
pico_enable_stdio_usb(touchmatrix-pico 1)

# Add the standard library to the build
target_link_libraries(touchmatrix-pico pico_stdlib hardware_pio)

# Add any user requested libraries
target_link_libraries(touchmatrix-pico
        hardware_spi
        hardware_pio
        pico_multicore
        )

pico_add_extra_outputs(touchmatrix-pico)

