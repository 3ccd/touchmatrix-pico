//
// Created by ura on 2022/08/02.
//

#ifndef TOUCHMATRIX_PICO_LED_MAP_H
#define TOUCHMATRIX_PICO_LED_MAP_H


#define BOARD_VER 2

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

#define SENSOR_COUNT 121

static uint32_t led_map[] = {
        0xff00010b,
        0xff01020c,
        0xff02030d,
        0xff03040e,
        0xff04050f,
        0xff050610,
        0xff060711,
        0xff070812,
        0xff080913,
        0xff090a14,
        0xff0a0b15,
        0x000a0b16,
        0x010b0c17,
        0x020c0d18,
        0x030d0e19,
        0x040e0f1a,
        0x050f101b,
        0x0610111c,
        0x0711121d,
        0x0812131e,
        0x0913141f,
        0x0a141520,
        0x0b161721,
        0x0c171822,
        0x0d181923,
        0x0e191a24,
        0x0f1a1b25,
        0x101b1c26,
        0x111c1d27,
        0x121d1e28,
        0x131e1f29,
        0x141f202a,
        0x1520212b,
        0x1620212c,
        0x1721222d,
        0x1822232e,
        0x1923242f,
        0x1a242530,
        0x1b252631,
        0x1c262732,
        0x1d272833,
        0x1e282934,
        0x1f292a35,
        0x202a2b36,
        0x212c2d37,
        0x222d2e38,
        0x232e2f39,
        0x242f303a,
        0x2530313b,
        0x2631323c,
        0x2732333d,
        0x2833343e,
        0x2934353f,
        0x2a353640,
        0x2b363741,
        0x2c363742,
        0x2d373843,
        0x2e383944,
        0x2f393a45,
        0x303a3b46,
        0x313b3c47,
        0x323c3d48,
        0x333d3e49,
        0x343e3f4a,
        0x353f404b,
        0x3640414c,
        0x3742434d,
        0x3843444e,
        0x3944454f,
        0x3a454650,
        0x3b464751,
        0x3c474852,
        0x3d484953,
        0x3e494a54,
        0x3f4a4b55,
        0x404b4c56,
        0x414c4d57,
        0x424c4d58,
        0x434d4e59,
        0x444e4f5a,
        0x454f505b,
        0x4650515c,
        0x4751525d,
        0x4852535e,
        0x4953545f,
        0x4a545560,
        0x4b555661,
        0x4c565762,
        0x4d585963,
        0x4e595a64,
        0x4f5a5b65,
        0x505b5c66,
        0x515c5d67,
        0x525d5e68,
        0x535e5f69,
        0x545f606a,
        0x5560616b,
        0x5661626c,
        0x5762636d,
        0x5862636e,
        0x5963646f,
        0x5a646570,
        0x5b656671,
        0x5c666772,
        0x5d676873,
        0x5e686974,
        0x5f696a75,
        0x606a6b76,
        0x616b6c77,
        0x626c6d78,
        0x636e6f79,
        0x646f707a,
        0x6570717b,
        0x6671727c,
        0x6772737d,
        0x6873747e,
        0x6974757f,
        0x6a757680,
        0x6b767781,
        0x6c777882,
        0x6d787983
};

#endif //TOUCHMATRIX_PICO_LED_MAP_H
