
#define CHAIN     1

#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/multicore.h"

#include "common.h"

#include "ADS8866.h"
#include "transfer.h"
#include "peripheral.h"

uint16_t buffer = 0;
uint16_t ext_light = 0;

void save_i2c_address(uint8_t addr){
    const uint32_t FLASH_TARGET_OFFSET = 0x1F0000;
    uint8_t write_data[FLASH_PAGE_SIZE] = {};

    write_data[0] = addr;

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, write_data, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
}

uint8_t load_i2c_address(){
    const uint32_t FLASH_TARGET_OFFSET = 0x1F0000;
    const auto *flash_target_contents = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);

    uint8_t addr = flash_target_contents[0];
    return addr;
}

void wait_for_hsync(){
    gpio_set_dir(PIN_HSYNC, GPIO_IN);

    while(!gpio_get(PIN_HSYNC));

    sleep_us(10);

    gpio_set_dir(PIN_HSYNC, GPIO_OUT);
    gpio_put(PIN_HSYNC, false);
}

void wait_for_vsync(){
    gpio_set_dir(PIN_HSYNC, GPIO_IN);
    gpio_set_dir(PIN_VSYNC, GPIO_IN);

    while(!gpio_get(PIN_VSYNC));

    sleep_us(10);

    gpio_set_dir(PIN_HSYNC, GPIO_OUT);
    gpio_set_dir(PIN_VSYNC, GPIO_OUT);
    gpio_put(PIN_HSYNC, false);
    gpio_put(PIN_VSYNC, false);
}


int main()
{
    stdio_init_all();

    //uint8_t i2c_addr= load_i2c_address();
    uint8_t i2c_addr = 0x01;
    /*if(i2c_addr == 0x00 || i2c_addr == 0xff){
        sleep_ms(5000);
        printf("i2c Address has not been set yet.\n"
               "Enter i2c address > ");
        char line[256] = {};
        gets(line);
        printf("ok.\n");
        sscanf(line, "%d", &i2c_addr);
        printf("The Address is set to %d\n", i2c_addr);
        save_i2c_address(i2c_addr);
    }*/

    init_adc();
    init_mux();
    init_internal_i2c();
    init_ir();

    set_ir_brightness(2);

    // initialize variable
    uint16_t sensor_ch = 0;
    uint8_t mode = 0;
    const uint8_t mc = OP_MODES;

    clear_ir();

    // core1 init
    transfer::start_core(i2c_addr);

    // init hsync_pin

    // pilot lamp
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, true);

    // h sync pin
    gpio_init(PIN_HSYNC);
    gpio_set_dir(PIN_HSYNC, GPIO_IN);

    // v sync pin
    gpio_init(PIN_VSYNC);
    gpio_set_dir(PIN_VSYNC, GPIO_IN);

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

        //acquisition(&ext_light);

        ir_led_enable(true);
        acquisition(&buffer);

        /*if(buffer > ext_light){
            buffer -= ext_light;
        }else{
            buffer = 0x0000;
        }*/
        uint32_t mg = (sensor_ch << 24)| (mode << 16) | buffer;
        multicore_fifo_push_blocking(mg);
        //printf("%d : %d\n", sensor_ch, (int16_t)buffer);
        sleep_ms(100);
        printf("%d\n", i2c_addr);

        // increment
        mode++;
        if(mode > mc){
            mode = 0;
            sensor_ch++;

            if(sensor_ch == SENSOR_COUNT){
                wait_for_vsync();

                sensor_ch = 0;
            }else{

                wait_for_hsync();
            }

        }

    }

}
