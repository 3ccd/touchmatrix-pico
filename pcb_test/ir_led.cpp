//
// Created by ura on 2022/10/31.
//

#include <stdio.h>
#include <pico/binary_info.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "pico/multicore.h"
#include "shift_register.pio.h"


// LED Driver defines
#define PIN_LD_SIN      13
#define PIN_LD_SCLK     12
#define PIN_LD_LAT      11
#define PIN_LD_BLANK    10

// LED Driver Settings
#define DC_COUNT        4

uint32_t dc_mask;
uint32_t mux_mask;
uint32_t ir_buffer[DC_COUNT];


void set_ir(uint8_t num){
    //num = 127 - num;
    uint8_t ld_num = (num >> 5) & 0b00000011;
    ir_buffer[ld_num] |= 0x80000000 >> (num & 0b00011111);
}

void clear_ir(){
    for (int i = 0; i < DC_COUNT; i++){
        ir_buffer[i] = 0;
    }
}

void put_ir(PIO pio){
    pio_sm_put_blocking(pio, 0, ir_buffer[3]);
    pio_sm_put_blocking(pio, 0, ir_buffer[2]);
    pio_sm_put_blocking(pio, 0, ir_buffer[1]);
    pio_sm_put_blocking(pio, 0, ir_buffer[0]);
}

int main()
{
    stdio_init_all();

    // initialize pio
    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &shift_register_program);
    pio_shift_register_init(pio, sm, offset, PIN_LD_SIN, 1, 0, 0, PIN_LD_LAT, 2);
    // led driver enable
    gpio_init(PIN_LD_BLANK);
    gpio_set_dir(PIN_LD_BLANK, GPIO_OUT);
    gpio_put(PIN_LD_BLANK, 0);

    // initialize variable
    uint16_t sensor_ch = 0;

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, true);

    clear_ir();

    while(true){

        // set led driver
        clear_ir();
        set_ir(sensor_ch);
        put_ir(pio);
        gpio_put(PIN_LD_BLANK, 0);

        sleep_ms(300);

        // increment
        sensor_ch++;
        if(sensor_ch > 62)sensor_ch = 0;

    }

}
