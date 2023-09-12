//
// Created by ura on 9/12/23.
//

#ifndef TOUCHMATRIX_PICO_TM4_CONFIG_H
#define TOUCHMATRIX_PICO_TM4_CONFIG_H

#define BOARD_VER 4

// I2C ADC defines
#define PIN_SCL         17
#define PIN_SDA         16
#define I2C_ADDR        0b1001000

// SPI Defines
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  2
#define PIN_MOSI 3

// LED Driver defines
#define PIN_LD_SIN      11
#define PIN_LD_SCLK     10
#define PIN_LD_LAT      13
#define PIN_LD_BLANK    12

// Multiplexer defines
#define PIN_MUX_S0      20
#define PIN_MUX_S1      21
#define PIN_MUX_S2      22
#define PIN_MUX_S3      26

// Decoder defines
#define PIN_DC_A        18
#define PIN_DC_B        19
#define PIN_DC_C        20
#define PIN_DC_ENABLE   21

// LED Driver Settings
#define DC_COUNT        2

// RGB LED defines
#define PIN_RGB         9
#define RGB_CLOCK       800000
#define RGB_IS_RGBW     false

#define SENSOR_COUNT 18

static uint32_t led_map[] = {
        0xff000106,
        0xff010207,
        0xff020308,
        0xff030409,
        0xff04050a,
        0x0005060b,
        0x0106070c,
        0x0207080d,
        0x0308090e,
        0x04090a0f,
        0x050a0b10,
        0x060b0c11,
        0x070c0d12,
        0x080d0e13,
        0x090e0f14,
        0x0a0f1015,
        0x0b101116,
        0x0c111217
};

#endif //TOUCHMATRIX_PICO_TM4_CONFIG_H
