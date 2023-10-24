//
// Created by ura on 9/12/23.
//

#ifndef TOUCHMATRIX_PICO_TM4_CONFIG_H
#define TOUCHMATRIX_PICO_TM4_CONFIG_H

#include <cstdint>

#define BOARD_VER 4

// I2C ADC defines
#define PIN_SCL         15
#define PIN_SDA         14
#define TMP0_ADDR       0b1001000
#define TMP1_ADDR       0b0000000
#define TMP2_ADDR       0b0000000
#define TMP3_ADDR       0b0000000
#define POT_ADDR        0b0101111

// External Connector
#define SLAVE_ADDR      0b0000001
#define PIN_EX_SCL      5
#define PIN_EX_SDA      4
#define PIN_VSYNC       1
#define PIN_HSYNC       0

// SPI Defines
#define SPI_INSTR spi0
#define PIN_MISO 16
#define PIN_CS  17
#define PIN_SCK  18
#define PIN_MOSI 19

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
#define PIN_MUX_SEL0    7
#define PIN_MUX_SEL1    6

// LED Driver Settings
#define DC_COUNT        2

// RGB LED defines
#define PIN_RGB         9
#define RGB_CLOCK       800000
#define RGB_IS_RGBW     false
#define RGB_LEDS        288

#define SENSOR_COUNT 18

static uint16_t mux_map[] = {
        0x0102,
        0x0103,
        0x0101,
        0x0107,
        0x0106,
        0x0105,
        0x0108,
        0x0109,
        0x0100,
        0x0207,
        0x0201,
        0x0200,
        0x0208,
        0x0209,
        0x020f,
        0x020c,
        0x020d,
        0x020e
};

static uint32_t led_map[] = {
        0xff000103,
        0xff010204,
        0xff02ff05,
        0x00ff030f,
        0x0103040e,
        0x0204050d,
        0x030f0e10,
        0x040e0d11,
        0x050dff12,
        0x0fff101f,
        0x0e101119,
        0x0d111218,
        0x101f191e,
        0x1119181d,
        0x1218ff1c,
        0x1fff1eff,
        0x191e1dff,
        0x181d1cff
};

#endif //TOUCHMATRIX_PICO_TM4_CONFIG_H
