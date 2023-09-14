//
// Created by ura on 9/14/23.
//

#ifndef TOUCHMATRIX_PICO_PERIPHERAL_H
#define TOUCHMATRIX_PICO_PERIPHERAL_H


#include <cstdint>
#include "common.h"

void init_adc();
void acquisition(uint16_t *dst);

void init_mux();
void set_mux(uint8_t num);

void init_ir();
void set_ir(uint8_t num);
void set_ir_from_map(uint8_t num, uint8_t mode);
void clear_ir();
void put_ir();
void ir_led_enable(bool enable);

void init_internal_i2c();
void set_ir_brightness(uint8_t b);

#endif //TOUCHMATRIX_PICO_PERIPHERAL_H
