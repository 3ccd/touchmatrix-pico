//
// Created by ura on 1/7/23.
//

#include "common.h"
#include "../include/ADS8866.h"

namespace ex_adc {

    void init_adc() {

        // SPI initialisation. This example will use SPI at 1MHz.
        spi_init(SPI_INSTR, 4 * 1000000);
        gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
        gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
        gpio_set_function(14, GPIO_FUNC_SPI);
        gpio_init(PIN_MOSI);
        gpio_set_dir(PIN_MOSI, GPIO_OUT);
        gpio_put(PIN_MOSI, true);

        spi_set_format(SPI_INSTR, 16, SPI_CPOL_1, SPI_CPHA_0, SPI_MSB_FIRST);

        // Chip select is active-low, so we'll initialise it to a driven-high state
        gpio_init(PIN_CS);
        gpio_set_dir(PIN_CS, GPIO_OUT);
        gpio_put(PIN_CS, true);
    }

    void read_adc(uint16_t *val) {
        uint16_t buffer = 0x00;

        // Conversion Result of (sensor_ch, led on)
        gpio_put(PIN_CS, false);
        asm volatile("nop \n nop \n nop");
        spi_read16_blocking(SPI_INSTR, 0, &buffer, 1);
        asm volatile("nop \n nop \n nop");
        gpio_put(PIN_CS, true);

        // Conversion Sample (sensor_ch, led on)
        spi_read16_blocking(SPI_INSTR, 0, &buffer, 1);

        gpio_put(PIN_CS, false);
        asm volatile("nop \n nop \n nop");
        spi_read16_blocking(SPI_INSTR, 0, val, 1);
        asm volatile("nop \n nop \n nop");
        gpio_put(PIN_CS, true);
    }

} // ex_adc