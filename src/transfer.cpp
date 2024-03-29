//
// Created by ura on 6/9/23.
//

#include "../include/transfer.h"


// board meta-data for sharing for other core
static transfer::brd_info b_info = {};


void transfer::data_transfer_via_usb(){

    uint32_t timing_cnt = 0;
    uint32_t board_info = 0xff << 24 | b_info.version << 16 | b_info.chain << 8 | b_info.sensors;

    multicore_fifo_push_blocking(CORE_STARTED);

    while(true){
        uint32_t data;
        if(timing_cnt == INFO_TIMING){
            data = board_info;
            timing_cnt = 0;
        }else{
            data = multicore_fifo_pop_blocking();
        }
        timing_cnt++;

        uint8_t buf[4] = {};
        buf[0] = (data >> 24) & 0xFF;
        buf[1] = (data >> 16) & 0xFF;
        buf[2] = (data >> 8) & 0xFF;
        buf[3] = data & 0xFF;

        int cnt = 0;
        bool end_flg = true;
        while(end_flg){
            switch(buf[cnt]) {
                case DATA_END:
                    putchar_raw(DATA_ESC);
                    putchar_raw(DATA_ESC_END);
                    break;
                case DATA_ESC:
                    putchar_raw(DATA_ESC);
                    putchar_raw(DATA_ESC_ESC);
                    break;
                default:
                    putchar_raw(buf[cnt]);
                    break;
            }
            cnt++;
            if(cnt == 4){
                putchar_raw(DATA_END);
                end_flg = false;
            }
        }

    }
}

void transfer::start_core(brd_info bi){
    b_info = bi;

    // core1 init
    multicore_launch_core1(transfer::data_transfer_via_usb);
    uint32_t ret = multicore_fifo_pop_blocking();
    if(ret == CORE_STARTED){

    }
}