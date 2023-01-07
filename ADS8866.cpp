//
// Created by ura on 1/7/23.
//

#include "ADS8866.h"

namespace ex_adc {

    ADS8866::ADS8866(spi_inst *spiInstr, uint8_t pin_miso, uint8_t pin_cs, uint8_t pin_clk, uint8_t pin_mosi) {
        cs = pin_cs;
        spiInst = spiInstr;

        // SPI initialisation. This example will use SPI at 1MHz.
        spi_init(spiInst, 3 * 1000000);
        gpio_set_function(pin_miso, GPIO_FUNC_SPI);
        gpio_set_function(pin_clk,  GPIO_FUNC_SPI);
        gpio_set_function(14, GPIO_FUNC_SPI);
        gpio_init(pin_mosi);
        gpio_set_dir(pin_mosi, GPIO_OUT);
        gpio_put(pin_mosi, true);

        spi_set_format(spiInst, 16, SPI_CPOL_1, SPI_CPHA_0, SPI_MSB_FIRST);

        // Chip select is active-low, so we'll initialise it to a driven-high state
        gpio_init(pin_cs);
        gpio_set_dir(pin_cs, GPIO_OUT);
        gpio_put(pin_cs, 1);
    }

    uint16_t ADS8866::read() const {
        uint16_t buffer = 0x00;

        // Conversion Result of (sensor_ch, led on)
        gpio_put(cs, 0);
        asm volatile("nop \n nop \n nop");
        spi_read16_blocking(spiInst, 0, &buffer, 1);
        asm volatile("nop \n nop \n nop");
        gpio_put(cs, 1);

        // Conversion Sample (sensor_ch, led on)
        spi_read16_blocking(spiInst, 0, &buffer, 1);

        gpio_put(cs, 0);
        asm volatile("nop \n nop \n nop");
        spi_read16_blocking(spiInst, 0, &buffer, 1);
        asm volatile("nop \n nop \n nop");
        gpio_put(cs, 1);

        return buffer;
    }

} // ex_adc