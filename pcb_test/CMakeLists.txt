add_executable(ir-test ir_led.cpp)

pico_generate_pio_header(ir-test ${CMAKE_CURRENT_LIST_DIR}/../shift_register.pio)

pico_set_program_name(ir-test "ir-test")
pico_set_program_version(ir-test "0.1")

pico_enable_stdio_uart(ir-test 0)
pico_enable_stdio_usb(ir-test 1)

# Add the standard library to the build
target_link_libraries(ir-test pico_stdlib hardware_pio)

# Add any user requested libraries
target_link_libraries(ir-test
        hardware_spi
        hardware_pio
        pico_multicore
        )

pico_add_extra_outputs(ir-test)