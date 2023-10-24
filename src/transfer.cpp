//
// Created by ura on 6/9/23.
//

#include "../include/transfer.h"
#include "common.h"


// board meta-data for sharing for other core
static transfer::brd_info b_info = {};
uint16_t data_buf[D_BUF_LEN] = {};
int sens_ch = 0;
uint8_t i2c_addr;

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

void transfer::read_via_usb() {
    int ret = getchar_timeout_us(100);
    if(ret == PICO_ERROR_TIMEOUT) return;

}

void transfer::data_transfer_via_i2c() {
    init_i2c_slave();

    multicore_fifo_push_blocking(CORE_STARTED);

    while(true){
        uint32_t data = multicore_fifo_pop_blocking();

        uint8_t sensor = (data >> 24) & 0xFF;
        uint8_t mode = (data >> 16) & 0xFF;
        uint16_t val = data & 0xFFFF;

        int offset = SENSOR_COUNT * mode;
        data_buf[offset + sensor] = val;
    }
}

void transfer::i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
        case I2C_SLAVE_RECEIVE:
            break;
        case I2C_SLAVE_REQUEST: // master is requesting data
            if(sens_ch >= D_BUF_LEN * 2){
                i2c_write_byte(i2c, 0x00);
            }

            /*for(uint16_t i : data_buf){
                i2c_write_byte(i2c, (i >> 8) & 0xFF);
                i2c_write_byte(i2c, i & 0xFF);
            }*/
            i2c_write_byte(i2c, ((uint8_t *)data_buf)[sens_ch] & 0xFF);
            sens_ch++;

            break;
        case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
            sens_ch = 0;
            break;
        default:
            break;
    }
}

void transfer::init_i2c_slave() {
    gpio_init(PIN_EX_SDA);
    gpio_set_function(PIN_EX_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_EX_SDA);

    gpio_init(PIN_EX_SCL);
    gpio_set_function(PIN_EX_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_EX_SCL);

    i2c_init(i2c0, 400000);
    // configure I2C0 for slave mode
    i2c_slave_init(i2c0, i2c_addr, &i2c_slave_handler);
}

void transfer::start_core(uint8_t addr){
    i2c_addr = addr;

    // core1 init
    multicore_launch_core1(transfer::data_transfer_via_i2c);
    uint32_t ret = multicore_fifo_pop_blocking();
    if(ret == CORE_STARTED){

    }
}