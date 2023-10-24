//
// Created by ura on 6/9/23.
//

#ifndef TOUCHMATRIX_PICO_TRANSFER_H
#define TOUCHMATRIX_PICO_TRANSFER_H

#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/multicore.h"
#include "i2c_fifo.h"
#include "i2c_slave.h"
#include "common.h"

#define D_BUF_LEN (SENSOR_COUNT * (OP_MODES + 1))

namespace transfer{

    // multicore defines
    const uint8_t    CORE_STARTED    =   123;
    const uint8_t    DATA_END        =   0xC0;    // slip compatible
    const uint8_t    DATA_ESC        =   0xDB;
    const uint8_t    DATA_ESC_END    =   0xDC;
    const uint8_t    DATA_ESC_ESC    =   0xDD;
    const uint32_t   INFO_TIMING     =   2000;

    struct brd_info{
        uint8_t version;
        uint8_t chain;
        uint8_t sensors;
    };

    void data_transfer_via_usb();
    void read_via_usb();
    void start_core(uint8_t addr);
    void data_transfer_via_i2c();
    static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event);
    static void init_i2c_slave();

}



#endif //TOUCHMATRIX_PICO_TRANSFER_H
