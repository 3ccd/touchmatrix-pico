#include <stdio.h>
#include <pico/binary_info.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "pico/multicore.h"
#include "shift_register.pio.h"


// multicore defines
#define CORE_STARTED 123

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


uint32_t dc_mask;
uint32_t mux_mask;
uint32_t ir_buffer[DC_COUNT];


void core1_data_transfer(){
    multicore_fifo_push_blocking(CORE_STARTED);

    while(true){
        uint32_t data = multicore_fifo_pop_blocking();

        uint8_t sensor_ch = (data >> 24) & 0xFF;
        uint8_t mode = (data >> 16) & 0xFF;
        uint16_t raw = data & 0xFFFF;

        printf("%d, %d, %d\n", sensor_ch, mode, raw);
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
    //num = 127 - num;
    uint8_t ld_num = (num >> 5) & 0b00000011;
    ir_buffer[ld_num] |= 0x80000000 >> (num & 0b00011111);
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

int main()
{
    stdio_init_all();

    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 1000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

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
    pio_shift_register_init(pio, sm, offset, PIN_LD_SIN, 1, 0, 0, PIN_LD_BLANK, 3);

    // initialize variable
    uint16_t buffer = 0xffff;
    uint16_t sensor_ch = 0;
    clear_ir();

    // core1 init
    multicore_launch_core1(core1_data_transfer);
    uint32_t ret = multicore_fifo_pop_blocking();
    if(ret == CORE_STARTED){

    }

    while(true){

        set_ch(sensor_ch);

        clear_ir();
        set_ir(sensor_ch);
        put_ir(pio);

        sleep_ms(200);

        // CONVERSION
        spi_read16_blocking(SPI_PORT, 0, &buffer, 1);

        asm volatile("nop \n nop \n nop");
        gpio_put(PIN_CS, 0);
        asm volatile("nop \n nop \n nop");

        // READ
        spi_read16_blocking(SPI_PORT, 0, &buffer, 1);

        asm volatile("nop \n nop \n nop");
        gpio_put(PIN_CS, 1);

        //printf("ADC : %d (ch %d)\n", buffer, sensor_ch);
        uint32_t mg = (sensor_ch << 24) | buffer;
        multicore_fifo_push_blocking(mg);

        sensor_ch++;
        if(sensor_ch == 121) sensor_ch = 0;

    }

}
