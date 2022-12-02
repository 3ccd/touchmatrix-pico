
// TM board version (load led map)
//#define TM_2
#define TM_3_DISCOVERY

#include <pico/binary_info.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "pico/multicore.h"
#include "shift_register.pio.h"
#include "ws2812.pio.h"

#ifdef TM_2

#include "resources/led_map.h"
#define SENSOR_COUNT 121

#elif defined(TM_3_DISCOVERY)

#include "resources/led_map_tm3dis.h"
#define SENSOR_COUNT 61

#endif

// operation mode (1LED or 4LED)
//#define OP_4LED
#define OP_1LED

// util
#define MHZ 1000000

// multicore defines
#define CORE_STARTED    123
#define DATA_END        0xC0    // slip compatible
#define DATA_ESC        0xDB
#define DATA_ESC_END    0xDC
#define DATA_ESC_ESC    0xDD

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


void core1_data_transfer(){
    multicore_fifo_push_blocking(CORE_STARTED);

    while(true){
        uint32_t data = multicore_fifo_pop_blocking();

        uint8_t buf[4] = {};
        buf[0] = (data >> 24) & 0xFF;
        buf[1] = (data >> 16) & 0xFF;
        buf[2] = (data >> 8) & 0xFF;
        buf[3] = data & 0xFF;
/*
        switch(buf[0]){
            case 0:
                buf[0] = 119;
                break;
            case 1:
                buf[0] = 120;
                break;
            default:
                buf[0] -= 2;
        }*/

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

int main()
{
    stdio_init_all();

    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 3 * MHZ);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(14, GPIO_FUNC_SPI);
    gpio_init(PIN_MOSI);
    gpio_set_dir(PIN_MOSI, GPIO_OUT);
    gpio_put(PIN_MOSI, true);

    spi_set_format(SPI_PORT, 16, SPI_CPOL_1, SPI_CPHA_0, SPI_MSB_FIRST);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

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
    uint16_t buffer = 0xffff;
    uint16_t dummy = 0;
    uint16_t sensor_ch = 0;
    uint8_t mode = 0;
#ifdef OP_1LED
    const uint8_t mc = 0;
#elif defined(OP_4LED)
    const uint8_t mc = 4;
#endif

    clear_ir();

    // core1 init
    multicore_launch_core1(core1_data_transfer);
    uint32_t ret = multicore_fifo_pop_blocking();
    if(ret == CORE_STARTED){

    }

    // pilot lamp
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, true);

    for(int i = 0; i < 6; i++){
        put_rgb(rgb_pio, 255, 0, 0);
    }

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
        put_ir(pio);
        gpio_put(PIN_LD_BLANK, 0);


        sleep_us(50);


        // Conversion Result of (sensor_ch, led on)
        gpio_put(PIN_CS, 0);
        asm volatile("nop \n nop \n nop");
        spi_read16_blocking(SPI_PORT, 0, &buffer, 1);
        asm volatile("nop \n nop \n nop");
        gpio_put(PIN_CS, 1);

        // Conversion Sample (sensor_ch, led on)
        spi_read16_blocking(SPI_PORT, 0, &dummy, 1);

        gpio_put(PIN_CS, 0);
        asm volatile("nop \n nop \n nop");
        spi_read16_blocking(SPI_PORT, 0, &buffer, 1);
        asm volatile("nop \n nop \n nop");
        gpio_put(PIN_CS, 1);


        uint16_t tmp = buffer;
        uint32_t mg = (sensor_ch << 24)| (mode << 16) | tmp;
        multicore_fifo_push_blocking(mg);

        // increment
        mode++;
        if(mode > mc){
            mode = 0;
            sensor_ch++;
            if(sensor_ch == SENSOR_COUNT) sensor_ch = 0;
        }

    }

}
