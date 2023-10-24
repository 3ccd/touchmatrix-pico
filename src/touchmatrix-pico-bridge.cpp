//
// Created by ura on 9/19/23.
//

#include <cstdio>
#include <pico/binary_info.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "pico/multicore.h"

#include "common.h"
#include "ws2812.pio.h"
#include "transfer.h"

#define DATA_BUF_LEN (SENSOR_COUNT * (OP_MODES + 1))

#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define I2C_INST    i2c0

#define BRIDGE_RGB_PIN      17
#define BRIDGE_HSYNC_PIN    15
#define BRIDGE_VSYNC_PIN    14

#define BRIDGE_SYNC_INTERVAL    100

#define CORE_STARTED    0xDEADBEEF

// multicore defines
#define DATA_END        0xC0    // slip compatible
#define DATA_ESC        0xDB
#define DATA_ESC_END    0xDC
#define DATA_ESC_ESC    0xDD


uint32_t rgb_buffer[RGB_LEDS];
int rgb_led_cnt = 0;
int rgb_offset = 24;
bool end_flg = false;
bool esc_flg = false;

uint16_t data_buffer[DATA_BUF_LEN] = {};
int rgb_dma_ch;

void pushback(uint8_t c){
    rgb_buffer[rgb_led_cnt] &= !(0xff << rgb_offset);
    rgb_buffer[rgb_led_cnt] |= c << rgb_offset;

    rgb_offset -= 8;
    if(rgb_offset == 0){
        rgb_offset = 24;
        rgb_led_cnt++;
        if(rgb_led_cnt == RGB_LEDS) rgb_led_cnt = 0;
    }
}

void decodeSlip(uint8_t c) {

        switch (c) {
            case DATA_END:  // detect end packet
                end_flg = true;
                rgb_led_cnt = 0;
                break;

            case DATA_ESC:  // detect esc packet
                // if escape end-packet or esc-packet
                if (end_flg | esc_flg) esc_flg = true;
                break;

            case DATA_ESC_END:  // detect end-escape end
                if (end_flg & esc_flg) {
                    pushback(DATA_END);
                    esc_flg = false;
                    break;
                }
                pushback(DATA_ESC_END);
                break;

            case DATA_ESC_ESC:  // detect esc-escape end
                if (esc_flg) {
                    pushback(DATA_ESC);
                    esc_flg = false;
                    break;
                }
                pushback(DATA_ESC_ESC);
                break;

            default:
                if (end_flg) pushback(c);
                break;
        }
}

void read() {
    int c = getchar();
    decodeSlip((uint8_t)c);
}


void communicate(){
    multicore_fifo_push_blocking(CORE_STARTED);

    while(true){
        read();
        transfer::data_transfer_via_usb();
    }
}


void start_core(){

    // core1 init
    multicore_launch_core1(communicate);
    uint32_t ret = multicore_fifo_pop_blocking();
    if(ret == CORE_STARTED){

    }
}

int main()
{
    stdio_init_all();

    start_core();

    // initialize pio no.2
    PIO rgb_pio pio1;
    uint sm = pio_claim_unused_sm(rgb_pio, true);
    uint offset = pio_add_program(rgb_pio, &ws2812_program);
    ws2812_program_init(rgb_pio, sm, offset, BRIDGE_RGB_PIN, RGB_CLOCK, false);

    for(unsigned long & i : rgb_buffer){
        i = 0x10000000;
    }

    rgb_dma_ch = dma_claim_unused_channel(true);
    dma_channel_config dma_ch_config = dma_channel_get_default_config(rgb_dma_ch);
    channel_config_set_transfer_data_size(&dma_ch_config, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_ch_config, true);
    channel_config_set_write_increment(&dma_ch_config, false);
    channel_config_set_dreq(&dma_ch_config, pio_get_dreq(rgb_pio, sm, true));

    dma_channel_configure(
            rgb_dma_ch,
            &dma_ch_config,
            &rgb_pio->txf[sm],
            rgb_buffer,
            RGB_LEDS,
            false
    );
    /* dma_channel_set_irq0_enabled(rgb_dma_ch, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    dma_handler();*/

    i2c_init(i2c0, 400 * KHZ);

    gpio_init(I2C_SDA_PIN);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_init(I2C_SCL_PIN);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);

    gpio_init(BRIDGE_HSYNC_PIN);
    gpio_set_dir(BRIDGE_HSYNC_PIN, GPIO_IN);
    gpio_pull_up(BRIDGE_HSYNC_PIN);

    gpio_init(BRIDGE_VSYNC_PIN);
    gpio_set_dir(BRIDGE_VSYNC_PIN, GPIO_IN);
    gpio_pull_up(BRIDGE_VSYNC_PIN);

    // pilot lamp
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, true);

    int cnt = 0;
    bool pol = false;

    while(true){
        int c = i2c_read_blocking(i2c0, 0x01,(uint8_t *)data_buffer, DATA_BUF_LEN * 2, false);
        printf("%d %d\n", c, data_buffer[0]);
        sleep_ms(1);

        /*if(pol){
            rgb_buffer[cnt] = 0x10101000;
        }else{
            rgb_buffer[cnt] = 0x00000000;
        }
        cnt++;
        if(cnt >= RGB_LEDS){
            cnt = 0;
            pol = !pol;
        }*/

        dma_channel_set_read_addr(rgb_dma_ch, rgb_buffer, true); // trig dma
    }

}
