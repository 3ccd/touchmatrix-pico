
// TM board version (load led map)
//#define TM_2
#define TM_3_DISCOVERY
#define BOARD_VER 3
#define CHAIN     1

#include <cstdio>
#include <pico/binary_info.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "pico/multicore.h"

#include "../include/ADS8866.h"
#include "../include/transfer.h"
#include "shift_register.pio.h"
#include "ws2812.pio.h"

#ifdef TM_2

#include "led_map.h"
#define SENSOR_COUNT 121

#elif defined(TM_3_DISCOVERY)

#include "led_map_tm3dis.h"
#define SENSOR_COUNT 61

#endif

// operation mode (1LED or 4LED)
#define OP_MODES    5
#define OP_4LED
//#define OP_1LED

// util
#define MHZ 1000000

// I2C ADC defines
#define PIN_SCL         17
#define PIN_SDA         16
#define I2C_ADDR        0b1001000

// SPI Defines
#define SPI_PORT spi0
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  2
#define PIN_MOSI 3

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

// RGB LED defines
#define PIN_RGB         15
#define RGB_CLOCK       800000
#define RGB_IS_RGBW     false


uint32_t dc_mask;
uint32_t mux_mask;
uint32_t ir_buffer[DC_COUNT];

uint16_t buffer = 0;
uint16_t ext_light = 0;

void core1_init(){
    transfer::brd_info b_info = {};
    {
        b_info.sensors = SENSOR_COUNT;
        b_info.version = BOARD_VER;
        b_info.chain = CHAIN;
    }

    transfer::start_core(b_info);
}

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
    if(mode & 0b1000) set_ir((map >> 24) & 0xFF);
    if(mode & 0b0100) set_ir((map >> 16) & 0xFF);
    if(mode & 0b0010) set_ir((map >> 8) & 0xFF);
    if(mode & 0b0001) set_ir(map & 0xFF);
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

void acquisition(ex_adc::ADS8866 *adc, uint16_t *dst){
    uint32_t tmp = 0;
    uint16_t ret = 0;
    sleep_us(50);
    for (int i = 0; i < 16; i++){
        sleep_us(1);
        adc->read(&ret);
        tmp += ret;
    }
    *dst = tmp >> 4;
}

void rgb_led(PIO rgb_pio, int t){
    // rgb_led
    for (uint i = 0; i < 144; ++i) {
        uint x = (i + (t >> 1)) % 64;
        if (x < 10)
            put_rgb(rgb_pio, 0xff, 0, 0);
        else if (x >= 15 && x < 25)
            put_rgb(rgb_pio, 0, 0xff, 0);
        else if (x >= 30 && x < 40)
            put_rgb(rgb_pio, 0, 0, 0xff);
        else
            put_rgb(rgb_pio, 0, 0, 0);
    }
}

int main()
{
    stdio_init_all();

    // ADC init
    ex_adc::ADS8866 adc = ex_adc::ADS8866(spi0, PIN_MISO, PIN_CS, PIN_SCK, PIN_MOSI);
    // ex_adc::ADS1114 adc = ex_adc::ADS1144(i2c_default, PIN_SDA, PIN_SCL);
    // adc.init(I2C_ADDR, ex_adc::DATA_RATE_860, ex_adc::PGA_512);

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

    // initialize pio no.2
    PIO rgb_pio pio1;
    sm = pio_claim_unused_sm(rgb_pio, true);
    offset = pio_add_program(rgb_pio, &ws2812_program);
    ws2812_program_init(rgb_pio, sm, offset, PIN_RGB, RGB_CLOCK, RGB_IS_RGBW);

    // initialize variable
    uint16_t sensor_ch = 0;
    uint8_t mode = 0;
#ifdef OP_1LED
    const uint8_t mc = 0;
#elif defined(OP_4LED)
    const uint8_t mc = 4;
#endif

    clear_ir();

    // core1 init
    core1_init();

    // pilot lamp
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, true);

    int t = 0;

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
        ir_led_enable(false);
        put_ir(pio);

        acquisition(&adc, &ext_light);

        ir_led_enable(true);
        acquisition(&adc, &buffer);

        if(buffer > ext_light){
            buffer -= ext_light;
        }else{
            buffer = 0x0000;
        }
        uint32_t mg = (sensor_ch << 24)| (mode << 16) | buffer;
        multicore_fifo_push_blocking(mg);

        // increment
        mode++;
        if(mode > mc){
            mode = 0;
            sensor_ch++;
            if(sensor_ch == SENSOR_COUNT){
                sensor_ch = 0;
            }

            rgb_led(rgb_pio, t);
            t++;
        }

    }

}
