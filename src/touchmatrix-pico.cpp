
#define CHAIN     1

#include <cstdio>
#include <pico/binary_info.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "pico/multicore.h"

#include "common.h"

#include "ADS8866.h"
#include "transfer.h"
#include "peripheral.h"
#include "ws2812.pio.h"


uint32_t rgb_buffer[RGB_LEDS];

uint16_t buffer = 0;
uint16_t ext_light = 0;

int rgb_dma_ch;

void core1_init(){
    transfer::brd_info b_info = {};
    {
        b_info.sensors = SENSOR_COUNT;
        b_info.version = BOARD_VER;
        b_info.chain = CHAIN;
    }

    transfer::start_core(b_info);
}


int main()
{
    stdio_init_all();

    init_adc();
    init_mux();
    init_internal_i2c();
    init_ir();

    set_ir_brightness(2);

    // initialize pio no.2
    PIO rgb_pio pio1;
    uint sm = pio_claim_unused_sm(rgb_pio, true);
    uint offset = pio_add_program(rgb_pio, &ws2812_program);
    ws2812_program_init(rgb_pio, sm, offset, PIN_RGB, RGB_CLOCK, false);

    for(unsigned long & i : rgb_buffer){
        i = 0x00000000;
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

    // initialize variable
    uint16_t sensor_ch = 0;
    uint8_t mode = 0;
    const uint8_t mc = OP_MODES;

    clear_ir();

    // core1 init
    //core1_init();

    // pilot lamp
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, true);

    while(true){

        // set decoder and multiplexer
        set_mux(sensor_ch);

        // set led driver
        clear_ir();
        switch (mode) {
            case 0:
                set_ir_from_map(sensor_ch, LED_NORTH + LED_WEST + LED_EAST + LED_SOUTH);
                break;
            case 1:
                set_ir_from_map(sensor_ch, LED_NORTH);
                break;
            case 2:
                set_ir_from_map(sensor_ch, LED_WEST);
                break;
            case 3:
                set_ir_from_map(sensor_ch, LED_EAST);
                break;
            case 4:
                set_ir_from_map(sensor_ch, LED_SOUTH);
                break;
            default:
                break;
        }
        ir_led_enable(false);
        put_ir();

        acquisition(&ext_light);

        ir_led_enable(true);
        acquisition(&buffer);

        if(buffer > ext_light){
            buffer -= ext_light;
        }else{
            buffer = 0x0000;
        }
        //uint32_t mg = (sensor_ch << 24)| (mode << 16) | buffer;
        //multicore_fifo_push_blocking(mg);
        printf("%d : %d\n", sensor_ch, (int16_t)buffer);
        sleep_ms(10);

        // increment
        mode++;
        if(mode > mc){
            mode = 0;
            sensor_ch++;
            if(sensor_ch == SENSOR_COUNT){
                sensor_ch = 0;
                dma_channel_set_read_addr(rgb_dma_ch, rgb_buffer, true); // trig dma
            }
        }

    }

}
