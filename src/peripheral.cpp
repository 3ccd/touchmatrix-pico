//
// Created by ura on 9/14/23.
//

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "peripheral.h"
#include "ADS8866.h"

uint32_t mux_mask;
uint16_t ir_buffer[DC_COUNT];

void init_adc(){
    // ADC init
    ex_adc::init_adc();
}

void acquisition(uint16_t *dst){
    uint32_t tmp = 0;
    uint16_t ret = 0;
    sleep_us(15);

    //ex_adc::read_adc(&ret);
    //if(ret >> 15) ret = 0;

    for (int i = 0; i < 8; i++){
        sleep_us(1);
        ex_adc::read_adc(&ret);
        //if(ret >> 15) ret = 0;
        tmp += ret;
    }
    *dst = tmp >> 3;
    //*dst = ret;
}

void init_mux(){
    // initialize multiplexer gpio
    mux_mask = 0x00;
    mux_mask |= 0x01 << PIN_MUX_S0;
    mux_mask |= 0x01 << PIN_MUX_S1;
    mux_mask |= 0x01 << PIN_MUX_S2;
    mux_mask |= 0x01 << PIN_MUX_S3;
    mux_mask |= 0x01 << PIN_MUX_SEL0;
    mux_mask |= 0x01 << PIN_MUX_SEL1;
    gpio_init_mask(mux_mask);       // initialize
    gpio_set_dir_out_masked(mux_mask);  // set direction (out)

    gpio_put(PIN_MUX_SEL0, true); // select pin is normally high
    gpio_put(PIN_MUX_SEL1, true);
}

void set_mux(uint8_t num){
    if(num >= SENSOR_COUNT)return;
    uint32_t value = 0;
    value |= (((mux_map[num] >> 0) & 1) << PIN_MUX_S0);
    value |= (((mux_map[num] >> 1) & 1) << PIN_MUX_S1);
    value |= (((mux_map[num] >> 2) & 1) << PIN_MUX_S2);
    value |= (((mux_map[num] >> 3) & 1) << PIN_MUX_S3);
    value |= ((~(mux_map[num] >> 8) & 1) << PIN_MUX_SEL0);
    value |= ((~(mux_map[num] >> 9) & 1) << PIN_MUX_SEL1);
    gpio_put_masked(mux_mask, value);
}

void init_ir(){
    // initialize spi for ir led driver
    spi_init(spi1, 1000000);
    gpio_set_function(PIN_LD_SIN, GPIO_FUNC_SPI);
    gpio_set_function(PIN_LD_SCLK,  GPIO_FUNC_SPI);
    gpio_init(PIN_LD_LAT);
    gpio_init(PIN_LD_BLANK);
    gpio_set_dir(PIN_LD_LAT, GPIO_OUT);
    gpio_set_dir(PIN_LD_BLANK, GPIO_OUT);
    spi_set_format(spi1, 16, SPI_CPOL_1, SPI_CPHA_0, SPI_MSB_FIRST);
}

void set_ir(uint8_t num){
    if((num >> 4) == 0xFF) return;
    uint8_t ld_num = 1 ^ ((num >> 4) & 1);
    ir_buffer[ld_num] |= 0x0001 << (num & 0b00001111);
}

void set_ir_from_map(uint8_t num, uint8_t mode){
    uint32_t map = led_map[num];

    if(mode & 0b1000) set_ir((map >> 24) & 0xFF);
    if(mode & 0b0100) set_ir((map >> 16) & 0xFF);
    if(mode & 0b0010) set_ir((map >> 8) & 0xFF);
    if(mode & 0b0001) set_ir(map & 0xFF);
}

void clear_ir(){
    for (unsigned short & i : ir_buffer){
        i = 0;
    }
}

void put_ir(){
    spi_write16_blocking(spi1, ir_buffer, 2);
    gpio_put(PIN_LD_LAT, true);
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_LD_LAT, false);
}


void ir_led_enable(bool enable){
    gpio_put(PIN_LD_BLANK, !enable);
}

void init_internal_i2c(){
    // initialize internal i2c
    i2c_init(i2c1, 100000);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    //gpio_pull_up(PIN_SCL);    // The value of internal pull-up resistor is too high :(
    //gpio_pull_up(PIN_SDA);    // Require 1k-ohm external pull-up resistor.
}

void set_ir_brightness(uint8_t b){
    i2c_write_blocking(i2c1, POT_ADDR, &b, 1, false);
}

