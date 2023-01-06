//
// Created by ura on 12/19/22.
//

// TM board version (load led map)
//#define TM_2
#define TM_3_DISCOVERY

#include <pico/binary_info.h>
#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "pico/multicore.h"
#include "shift_register.pio.h"

#ifdef TM_2

#include "resources/led_map.h"
#define SENSOR_COUNT 121

#elif defined(TM_3_DISCOVERY)

#include "../resources/led_map_tm3dis.h"
#define SENSOR_COUNT 61

#endif

// operation mode (1LED or 4LED)
//#define OP_4LED
#define OP_1LED

// util
#define MHZ 1000000
#define KHZ 1000

// I2C ADC defines
#define PIN_SCL         17
#define PIN_SDA         16
#define I2C_ADDR        0b1001000
#define REG_CONV        0b00000000
#define REG_CONF        0b00000001


// LED Driver defines
#define PIN_LD_SIN      13
#define PIN_LD_SCLK     12
#define PIN_LD_LAT      11
#define PIN_LD_BLANK    10

// Multiplexer defines
#define PIN_MUX_S0      6
#define PIN_MUX_S1      7
#define PIN_MUX_S2      8
#define PIN_MUX_S3      9

// Decoder defines
#define PIN_DC_A        18
#define PIN_DC_B        19
#define PIN_DC_C        20
#define PIN_DC_ENABLE   21

// LED Driver Settings
#define DC_COUNT        4

// IR LED Mode
#define LED_NORTH       1
#define LED_WEST        2
#define LED_EAST        4
#define LED_SOUTH       8

uint32_t dc_mask;
uint32_t mux_mask;
uint32_t ir_buffer[DC_COUNT];


uint8_t buffer[2] = {};


inline void set_dc(uint8_t num){
    num |= 0b00001000;      // ENABLE (HIGH)
    uint32_t value = num << PIN_DC_A;
    gpio_put_masked(dc_mask, value);
}

inline void set_mux(uint8_t num){
    uint32_t value = num << PIN_MUX_S0;
    gpio_put_masked(mux_mask, value);
}

void set_ch(uint16_t num){

    uint8_t dc_sel = (num >> 4) & 0b00000111;
    uint8_t mux_sel = num & 0b00001111;

    set_dc(dc_sel);
    set_mux(mux_sel);

}

void set_ir(uint8_t num){
    uint8_t ld_num = (num >> 5) & 0b00000011;
    ir_buffer[ld_num] |= 0x80000000 >> (num & 0b00011111);
}

void set_ir_from_map(uint8_t num, uint8_t mode){
#ifdef TM_2
    uint32_t map = led_map[num];
#elif defined(TM_3_DISCOVERY)
    uint32_t map = led_map_tm3dis[num];
#endif
    if(mode | 0b1000) set_ir((map >> 24) & 0xFF);
    if(mode | 0b0100) set_ir((map >> 16) & 0xFF);
    if(mode | 0b0010) set_ir((map >> 8) & 0xFF);
    if(mode | 0b0001) set_ir(map & 0xFF);
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

void put_rgb(PIO pio, uint8_t r, uint8_t b, uint8_t g){
    uint32_t tmp_color = ((uint32_t) (r) << 8) |
                         ((uint32_t) (g) << 16) |
                         (uint32_t) (b);
    pio_sm_put_blocking(pio, 0, tmp_color << 8u);
}

inline void ir_led_enable(bool enable){
    gpio_put(PIN_LD_BLANK, !enable);
}

int main()
{
    stdio_init_all();

    // initialize i2c interface
    i2c_init(i2c_default, 400 * KHZ);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);
    // configuration adc
    uint8_t adc_conf[3] = {};
    adc_conf[0] = REG_CONF;
    adc_conf[1] = 0b00001010;
    adc_conf[2] = 0b11100000;
    i2c_write_blocking(i2c_default, I2C_ADDR, adc_conf, 3, true);

    // initialize decoder gpio
    dc_mask = 0x00;
    dc_mask |= 0x01 << PIN_DC_A;
    dc_mask |= 0x01 << PIN_DC_B;
    dc_mask |= 0x01 << PIN_DC_C;
    dc_mask |= 0x01 << PIN_DC_ENABLE;
    gpio_init_mask(dc_mask);        // initialize
    gpio_set_dir_out_masked(dc_mask);   // set direction (out)

    // initialize multiplexer gpio
    mux_mask = 0x00;
    mux_mask |= 0x01 << PIN_MUX_S0;
    mux_mask |= 0x01 << PIN_MUX_S1;
    mux_mask |= 0x01 << PIN_MUX_S2;
    mux_mask |= 0x01 << PIN_MUX_S3;
    gpio_init_mask(mux_mask);       // initialize
    gpio_set_dir_out_masked(mux_mask);  // set direction (out)

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
    uint16_t sensor_ch = 18;
    uint8_t mode = 0;
#ifdef OP_1LED
    const uint8_t mc = 0;
#elif defined(OP_4LED)
    const uint8_t mc = 4;
#endif

    clear_ir();

    // pilot lamp
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, true);

    bool rgb_on = false;

    while(true){

        // set decoder and multiplexer
        set_ch(sensor_ch);

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
        ir_led_enable(true);
        put_ir(pio);

        sleep_ms(60);

        i2c_write_blocking(i2c_default, I2C_ADDR, REG_CONV, 1, true);
        i2c_read_blocking(i2c_default, I2C_ADDR, buffer, 2, false);
        int16_t val = (buffer[0] << 8) | buffer[1];

        printf("%d: %d\n", sensor_ch, val);

        // increment
        /*mode++;
        if(mode > mc){
            mode = 0;
            sensor_ch++;
            if(sensor_ch == SENSOR_COUNT){
                sensor_ch = 0;
            }
        }*/

    }

}
