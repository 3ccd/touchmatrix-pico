//
// Created by ura on 1/7/23.
//

#include "hardware/spi.h"
#include "hardware/gpio.h"

#ifndef TOUCHMATRIX_PICO_ADS8866_H
#define TOUCHMATRIX_PICO_ADS8866_H

namespace ex_adc {

    class ADS8866 {
    private:
        uint8_t cs;
        spi_inst *spiInst;
    public:
        ADS8866(spi_inst *spiInstr, uint8_t pin_miso, uint8_t pin_cs, uint8_t pin_clk, uint8_t pin_mosi);
        [[nodiscard]] uint16_t read() const;
    };
    };

} // ex_adc

#endif //TOUCHMATRIX_PICO_ADS8866_H
