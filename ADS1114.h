//
// Created by ura on 1/7/23.
//

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#ifndef TOUCHMATRIX_PICO_ADS1114_H
#define TOUCHMATRIX_PICO_ADS1114_H

namespace ex_adc {

    static const uint8_t DATA_RATE_860 = 0b11100000;
    static const uint8_t DATA_RATE_475 = 0b11000000;
    static const uint8_t DATA_RATE_250 = 0b10100000;
    static const uint8_t DATA_RATE_128 = 0b10000000;

    static const uint8_t PGA_256    = 0b00001010;
    static const uint8_t PGA_512    = 0b00001000;
    static const uint8_t PGA_1024   = 0b00000110;
    static const uint8_t PGA_2048   = 0b00000100;

    static const uint8_t CONF_REG   = 0b00000001;
    static const uint8_t CONV_REG  = 0b00000000;

    class ADS1114 {
    private:
        i2c_inst *i2CInst;
        uint8_t dev_addr = 0x00;

        uint8_t buffer[2] = {};
    public:
        ADS1114(i2c_inst *i2CInstr, uint8_t pin_sda, uint8_t pin_scl);
        void init(uint8_t addr, uint8_t rate, uint8_t pga);
        void read(uint16_t *val);
    };

} // ex_adc

#endif //TOUCHMATRIX_PICO_ADS1114_H
