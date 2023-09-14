//
// Created by ura on 1/7/23.
//

#include "hardware/spi.h"
#include "hardware/gpio.h"

#ifndef TOUCHMATRIX_PICO_ADS8866_H
#define TOUCHMATRIX_PICO_ADS8866_H

namespace ex_adc {

    void init_adc();
    void read_adc(uint16_t *val);

} // ex_adc

#endif //TOUCHMATRIX_PICO_ADS8866_H
