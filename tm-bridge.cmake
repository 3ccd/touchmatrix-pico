#
#   touchmatrix BRIDGE
#

add_definitions(-D_WIZCHIP_=W5500)

if(NOT DEFINED WIZNET_DIR)
    set(WIZNET_DIR ${CMAKE_SOURCE_DIR}/library/ioLibrary_Driver)
    message(STATUS "WIZNET_DIR = ${WIZNET_DIR}")
endif()

if(NOT DEFINED PORT_DIR)
    set(PORT_DIR ${CMAKE_SOURCE_DIR}/port)
    message(STATUS "PORT_DIR = ${PORT_DIR}")
endif()

add_subdirectory(${CMAKE_SOURCE_DIR}/library)
add_subdirectory(${PORT_DIR})

add_executable(touchmatrix-pico-bridge
        ${src_dir}/touchmatrix-pico-bridge.cpp)

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
        ETHERNET_FILES
        IOLIBRARY_FILES
        )

pico_add_extra_outputs(touchmatrix-pico-bridge)