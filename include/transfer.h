//
// Created by ura on 6/9/23.
//

#include "pico/stdlib.h"
#include "pico/multicore.h"

#ifndef TOUCHMATRIX_PICO_TRANSFER_H
#define TOUCHMATRIX_PICO_TRANSFER_H

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
    void start_core(brd_info bi);
    void data_transfer_via_i2c(brd_info *info);

}



#endif //TOUCHMATRIX_PICO_TRANSFER_H
