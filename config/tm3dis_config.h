//
// Created by ura on 11/16/22.
//

#ifndef TOUCHMATRIX_PICO_TM3DIS_CONFIG_H
#define TOUCHMATRIX_PICO_TM3DIS_CONFIG_H


#define BOARD_VER 3

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

// RGB LED defines
#define PIN_RGB         15
#define RGB_CLOCK       800000
#define RGB_IS_RGBW     false

#define SENSOR_COUNT 61

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
        0x0c111217,
        0x0d121318,
        0x0e131419,
        0x0f14151a,
        0x1015161b,
        0x1116171c,
        0x1217181d,
        0x1318191e,
        0x14191a1f,
        0x151a1b20,
        0x161b1c21,
        0x171c1d22,
        0x181d1e23,
        0x191e1f24,
        0x1a1f2025,
        0x1b202126,
        0x1c212227,
        0x1d222328,
        0x1e232429,
        0x1f24252a,
        0x2025262b,
        0x2126272c,
        0x2227282d,
        0x2328292e,
        0x24292a2f,
        0x252a2b30,
        0x262b2c31,
        0x272c2d32,
        0x282d2e33,
        0x292e2f34,
        0x2a2f3035,
        0x2b303136,
        0x2c313237,
        0x2d323338,
        0x2e333439,
        0x2f34353a,
        0x3035363b,
        0x3136373c,
        0x3237383d,
        0x3338393e,
        0x34393a3f,
        0x353a3b40,
        0x363b3c41,
        0x373c3d42
};

#endif //TOUCHMATRIX_PICO_TM3DIS_CONFIG_H
