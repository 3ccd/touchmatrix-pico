//
// Created by ura on 9/19/23.
//

#include <cstdio>
#include <pico/binary_info.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "pico/multicore.h"

#include "common.h"
#include "ws2812.pio.h"

extern "C" {
    #include "port_common.h"

    #include "wizchip_conf.h"
    #include "w5x00_spi.h"
    #include "socket.h"
}


#define PLL_SYS_KHZ (133 * 1000)

#define CHAIN_MAX   2
#define DATA_BUF_LEN (SENSOR_COUNT * (OP_MODES + 1) * CHAIN_MAX)
#define DATA_LEN_PER_BRD    SENSOR_COUNT * (OP_MODES + 1)

#define RGB_BUF_LEN (RGB_LEDS * CHAIN_MAX)

#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define I2C_INST    i2c0

#define BRIDGE_RGB_PIN      13
#define BRIDGE_HSYNC_PIN    15
#define BRIDGE_VSYNC_PIN    14

#define CORE_STARTED    0xDEADBEEF

// multicore defines
#define DATA_END        0xC0    // slip compatible
#define DATA_ESC        0xDB
#define DATA_ESC_END    0xDC
#define DATA_ESC_ESC    0xDD

#define UDP_PORT        8002
#define UDP_SOCK        0


uint32_t rgb_buffer[RGB_BUF_LEN];
int rgb_led_cnt = 0;
int rgb_offset = 24;
bool esc_flg = false;

uint32_t timing_cnt = 0;
uint32_t board_info = 0xff << 24 | BOARD_VER << 16 | CHAIN_MAX << 8 | SENSOR_COUNT;

uint16_t data_buffer[DATA_BUF_LEN] = {};
int rgb_dma_ch;

static wiz_NetInfo g_net_info =
        {
                .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56}, // MAC address
                .ip = {192, 168, 0, 101},                     // IP address
                .sn = {255, 255, 255, 0},                    // Subnet Mask
                .gw = {192, 168, 0, 1},                     // Gateway
                .dns = {8, 8, 8, 8},                         // DNS server
                .dhcp = NETINFO_STATIC                       // DHCP enable/disable
        };


static void set_clock_khz()
{
    // set a system clock frequency in khz
    set_sys_clock_khz(PLL_SYS_KHZ, true);

    // configure the specified clock
    clock_configure(
            clk_peri,
            0,                                                // No glitchless mux
            CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
            PLL_SYS_KHZ * 1000,                               // Input frequency
            PLL_SYS_KHZ * 1000                                // Output (must be same as no divider)
    );
}


void eth_init(){
    /* Initialize */

    wizchip_spi_initialize();
    wizchip_cris_initialize();

    wizchip_reset();
    wizchip_initialize();
    wizchip_check();

    network_initialize(g_net_info);

    /* Get network information */
    print_network_information(g_net_info);
}

int32_t tcp_transfer(uint8_t sn, uint16_t port) {
    int32_t ret;
    uint16_t size = 0, sentsize = 0;

    uint8_t destip[4];
    uint16_t destport;

    switch (getSn_SR(sn)) {
        case SOCK_ESTABLISHED :
            if (getSn_IR(sn) & Sn_IR_CON) {
                getSn_DIPR(sn, destip);
                destport = getSn_DPORT(sn);

                printf("%d:Connected - %d.%d.%d.%d : %d\r\n", sn, destip[0], destip[1], destip[2], destip[3], destport);
                setSn_IR(sn, Sn_IR_CON);
            }
            if ((size = getSn_RX_RSR(sn)) > 0) // Don't need to check SOCKERR_BUSY because it doesn't not occur.
            {
                printf("%d:Received : %d bytes\r\n", sn, size);
                if(size > RGB_BUF_LEN * 4) size = RGB_BUF_LEN * 4;
                ret = recv(sn, (uint8_t *)rgb_buffer, size);

                if (ret <= 0)
                    return ret;      // check SOCKERR_BUSY & SOCKERR_XXX. For showing the occurrence of SOCKERR_BUSY.

                sentsize = 0;
                while(sentsize != DATA_BUF_LEN * 2){
                    ret = send(sn, (uint8_t *)data_buffer+sentsize, (DATA_BUF_LEN * 2)-sentsize);
                    if(ret < 0)
                    {
                        close(sn);
                        return ret;
                    }
                    sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
                }
            }
            break;
        case SOCK_CLOSE_WAIT :
            printf("%d:CloseWait\r\n", sn);
            if ((ret = disconnect(sn)) != SOCK_OK) return ret;
            printf("%d:Socket Closed\r\n", sn);
            break;
        case SOCK_INIT :
            printf("%d:Listen, TCP server loopback, port [%d]\r\n", sn, port);
            if ((ret = listen(sn)) != SOCK_OK) return ret;
            break;
        case SOCK_CLOSED:
            printf("%d:TCP server loopback start\r\n", sn);
            if ((ret = socket(sn, Sn_MR_TCP, port, 0x00)) != sn) return ret;
            printf("%d:Socket opened\r\n", sn);
            break;
        default:
            break;
    }
    return 1;
}

int32_t tcp_send(uint8_t sn, uint16_t port){
    int32_t ret;
    uint16_t sentsize = 0;

    uint8_t destip[4];
    uint16_t destport;

    switch (getSn_SR(sn)) {
        case SOCK_ESTABLISHED :
            if (getSn_IR(sn) & Sn_IR_CON) {
                getSn_DIPR(sn, destip);
                destport = getSn_DPORT(sn);

                printf("%d:Connected - %d.%d.%d.%d : %d\r\n", sn, destip[0], destip[1], destip[2], destip[3], destport);
                setSn_IR(sn, Sn_IR_CON);
            }

            sentsize = 0;
            while(sentsize != DATA_BUF_LEN * 2){
                ret = send(sn, (uint8_t *)data_buffer+sentsize, (DATA_BUF_LEN * 2)-sentsize);
                if(ret < 0)
                {
                    close(sn);
                    return ret;
                }
                sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
            }
            break;

        case SOCK_CLOSE_WAIT :
            printf("%d:CloseWait\r\n", sn);
            if ((ret = disconnect(sn)) != SOCK_OK) return ret;
            printf("%d:Socket Closed\r\n", sn);
            break;
        case SOCK_INIT :
            printf("%d:Listen, TCP server loopback, port [%d]\r\n", sn, port);
            if ((ret = listen(sn)) != SOCK_OK) return ret;
            break;
        case SOCK_CLOSED:
            printf("%d:TCP server loopback start\r\n", sn);
            if ((ret = socket(sn, Sn_MR_TCP, port, 0x00)) != sn) return ret;
            printf("%d:Socket opened\r\n", sn);
            break;
        default:
            break;
    }
    return 1;
}

int32_t udp_transfer(uint8_t sn, uint16_t port)
{
    int32_t  ret;
    uint16_t size;
    uint8_t  destip[4] = {};
    uint16_t destport = 0;

    switch(getSn_SR(sn))
    {
        case SOCK_UDP :
            size = getSn_RX_RSR(sn);
            if(size > 0){
                if(size > RGB_LEDS * 4) size = RGB_LEDS * 4;

                int offset = (sn * RGB_LEDS) * 4;
                ret = recvfrom(sn, (uint8_t *)rgb_buffer + offset, size, destip, (uint16_t*)&destport);


                /*printf("received[%d] : %hu [%d.%d.%d.%d:%d] %lu %d\n",
                       sn,
                       size,
                       destip[0],
                       destip[1],
                       destip[2],
                       destip[3],
                       destport,
                       rgb_buffer[570],
                       offset
                       );*/
                if(ret <= 0)
                {
                    return ret;
                }
            }

        case SOCK_CLOSED:
            ret = socket(sn, Sn_MR_UDP, port, 0x00);
            if(ret != sn) return ret;
            setSn_RXBUF_SIZE(sn, 8); // set size of rx buffer to 8kByte
            break;

        default :
            break;
    }
    return 1;
}


void transfer_via_usb(uint32_t data) {
    bool end_flg_trans = true;

    uint8_t buf[4] = {};
    buf[0] = (data >> 24) & 0xFF;
    buf[1] = (data >> 16) & 0xFF;
    buf[2] = (data >> 8) & 0xFF;
    buf[3] = data & 0xFF;

    int cnt = 0;
    while (end_flg_trans) {
        switch (buf[cnt]) {
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
        if (cnt == 4) {
            putchar_raw(DATA_END);
            end_flg_trans = false;
        }
    }
}


void pushback(uint8_t c) {
    rgb_buffer[rgb_led_cnt] &= ~(0xff << rgb_offset);
    rgb_buffer[rgb_led_cnt] |= c << rgb_offset;

    rgb_offset -= 8;
    if (rgb_offset == 0) {
        rgb_offset = 24;
        rgb_led_cnt++;
        if (rgb_led_cnt == RGB_BUF_LEN) rgb_led_cnt = 0;
    }
}

void decodeSlip(uint8_t c) {
    static bool end_flg = false;

    switch (c) {
        case DATA_END:  // detect end packet
            end_flg = true;
            rgb_led_cnt = 0;
            break;

        case DATA_ESC:  // detect esc packet
            // if escape end-packet or esc-packet
            if (end_flg | esc_flg) esc_flg = true;
            break;

        case DATA_ESC_END:  // detect end-escape end
            if (end_flg & esc_flg) {
                pushback(DATA_END);
                esc_flg = false;
                break;
            }
            pushback(DATA_ESC_END);
            break;

        case DATA_ESC_ESC:  // detect esc-escape end
            if (esc_flg) {
                pushback(DATA_ESC);
                esc_flg = false;
                break;
            }
            pushback(DATA_ESC_ESC);
            break;

        default:
            if (end_flg) pushback(c);
            break;
    }
}

void read() {
    int c = getchar_timeout_us(0);
    if(c == PICO_ERROR_TIMEOUT) return;
    decodeSlip((uint8_t) c);
}


void communicate() {
    eth_init();
    multicore_fifo_push_blocking(CORE_STARTED);

    while (true) {
        for(int i = 0; i < CHAIN_MAX; i++){
            udp_transfer(UDP_SOCK + i, UDP_PORT + i);
        }
        tcp_send(UDP_SOCK + CHAIN_MAX, UDP_PORT + CHAIN_MAX);

        dma_channel_set_read_addr(rgb_dma_ch, rgb_buffer, true); // trig dma
        sleep_ms(20);
    }
}


void start_core() {

    // core1 init
    multicore_launch_core1(communicate);
    uint32_t ret = multicore_fifo_pop_blocking();
    if (ret == CORE_STARTED) {

    }
}

void test_rgb_leds(){

    for(unsigned long & i : rgb_buffer){
        i = 0x00001000;
    }
    dma_channel_set_read_addr(rgb_dma_ch, rgb_buffer, true); // trig dma
    sleep_ms(1500);

    for(unsigned long & i : rgb_buffer){
        i = 0x00000000;
    }
    dma_channel_set_read_addr(rgb_dma_ch, rgb_buffer, true); // trig dma

}


int main() {
    set_clock_khz();

    stdio_init_all();

    sleep_ms(5000);

    start_core();

    // initialize pio no.2
    PIO rgb_pio pio1;
    uint sm = pio_claim_unused_sm(rgb_pio, true);
    uint offset = pio_add_program(rgb_pio, &ws2812_program);
    ws2812_program_init(rgb_pio, sm, offset, BRIDGE_RGB_PIN, RGB_CLOCK, false);

    for (unsigned long &i: rgb_buffer) {
        i = 0x00000000;
    }

    rgb_dma_ch = dma_claim_unused_channel(true);
    dma_channel_config dma_ch_config = dma_channel_get_default_config(rgb_dma_ch);
    channel_config_set_transfer_data_size(&dma_ch_config, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_ch_config, true);
    channel_config_set_write_increment(&dma_ch_config, false);
    channel_config_set_dreq(&dma_ch_config, pio_get_dreq(rgb_pio, sm, true));

    dma_channel_configure(
            rgb_dma_ch,
            &dma_ch_config,
            &rgb_pio->txf[sm],
            rgb_buffer,
            RGB_BUF_LEN,
            false
    );
    /* dma_channel_set_irq0_enabled(rgb_dma_ch, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    dma_handler();*/

    i2c_init(i2c0, 400 * KHZ);

    gpio_init(I2C_SDA_PIN);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_init(I2C_SCL_PIN);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);

    gpio_init(BRIDGE_HSYNC_PIN);
    gpio_set_dir(BRIDGE_HSYNC_PIN, GPIO_IN);
    gpio_pull_up(BRIDGE_HSYNC_PIN);

    gpio_init(BRIDGE_VSYNC_PIN);
    gpio_set_dir(BRIDGE_VSYNC_PIN, GPIO_IN);
    gpio_pull_up(BRIDGE_VSYNC_PIN);

    // pilot lamp
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, true);

    test_rgb_leds();

    while (true) {
        for(int i = 0; i < CHAIN_MAX; i++) {
            int buf_os = SENSOR_COUNT * (OP_MODES + 1) * i;
            i2c_read_blocking(
                    i2c0,
                    i + 1,
                    (uint8_t *) (data_buffer + buf_os),
                    DATA_LEN_PER_BRD * 2,
                    false
            );
        }
        sleep_ms(10);
    }

}
