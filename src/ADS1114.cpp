//
// Created by ura on 1/7/23.
//

#include "../include/ADS1114.h"

namespace ex_adc {

    ADS1114::ADS1114(i2c_inst *i2CInstr, uint8_t pin_sda, uint8_t pin_scl) {
        // initialize i2c interface
        i2CInst = i2CInstr;
        i2c_init(i2CInst, 400000);
        gpio_set_function(pin_sda, GPIO_FUNC_I2C);
        gpio_set_function(pin_scl, GPIO_FUNC_I2C);
        gpio_pull_up(pin_sda);
        gpio_pull_up(pin_scl);
    }

    void ADS1114::init(uint8_t addr, uint8_t rate, uint8_t pga) {
        dev_addr = addr;
        // configuration adc
        uint8_t adc_conf[3] = {};
        adc_conf[0] = CONF_REG;
        adc_conf[1] = pga;
        adc_conf[2] = rate;
        i2c_write_blocking(i2CInst, addr, adc_conf, 3, true);
        i2c_write_blocking(i2CInst, dev_addr, &CONV_REG, 1, true);
    }

    void ADS1114::read(uint16_t *val){
        i2c_read_blocking(i2CInst, dev_addr, buffer, 2, true);
        *val = ((buffer[0] << 8) | buffer[1]) ^ 0x8000;
    }

} // ex_adc