add_executable(ir-test ir_led.cpp)

pico_generate_pio_header(ir-test ${PROJECT_SOURCE_DIR}/pio/shift_register.pio)

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

add_executable(i2c-adc ads1113.cpp)

pico_generate_pio_header(i2c-adc ${PROJECT_SOURCE_DIR}/pio/shift_register.pio)

pico_set_program_name(i2c-adc "i2c-adc")
pico_set_program_version(i2c-adc "0.1")

pico_enable_stdio_uart(i2c-adc 0)
pico_enable_stdio_usb(i2c-adc 1)

# Add the standard library to the build
target_link_libraries(i2c-adc pico_stdlib hardware_pio hardware_i2c)

# Add any user requested libraries
target_link_libraries(i2c-adc
        hardware_i2c
        hardware_pio
        pico_multicore
        )

pico_add_extra_outputs(i2c-adc)



add_executable(eval1_board eval1_board.cpp ${PROJECT_SOURCE_DIR}/src/ADS8866.cpp)

pico_set_program_name(eval1_board "eval1_board")
pico_set_program_version(eval1_board "0.1")

pico_enable_stdio_uart(eval1_board 0)
pico_enable_stdio_usb(eval1_board 1)

# Add the standard library to the build
target_link_libraries(eval1_board pico_stdlib hardware_pio hardware_spi)

# Add any user requested libraries
target_link_libraries(eval1_board
        pico_multicore
        )

target_include_directories(eval1_board PUBLIC ${PROJECT_BINARY_DIR} ${PROJECT_SOURCE_DIR}/include)

pico_add_extra_outputs(eval1_board)