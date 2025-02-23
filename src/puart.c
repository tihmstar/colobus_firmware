#include "puart.h"
#include "uart.pio.h"

#include <hardware/pio.h>
#include <hardware/clocks.h>


#define PUART_PIO pio1
#define PUART_SM 0

#define BAUD 115200

#pragma mark static vars
static int gPuart_rx_pio_pc = -1;
static int gPuart_tx_pio_pc = -1;


#pragma mark code
void puart_init(int uart_rx, int uart_tx){
    if (gPuart_rx_pio_pc == -1){
        pio_sm_set_enabled(PUART_PIO, PUART_SM, false);

        gPuart_rx_pio_pc = pio_add_program(PUART_PIO, &uart_rx_program);
        pio_sm_config c = uart_rx_program_get_default_config(gPuart_rx_pio_pc);

        {
            float div = (float)clock_get_hz(clk_sys) / (8 * BAUD);
            sm_config_set_clkdiv(&c, div);
        }

        sm_config_set_in_pins(&c, uart_rx);
        sm_config_set_jmp_pin(&c, uart_rx);
        sm_config_set_in_shift(&c, true, false, 32);
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);

        pio_sm_set_consecutive_pindirs(PUART_PIO, PUART_SM, uart_rx, 1, false);

        pio_sm_init(PUART_PIO, PUART_SM, gPuart_rx_pio_pc, &c);
        pio_sm_set_enabled(PUART_PIO, PUART_SM, true);
    }

}

void puart_deinit(void){
    if (gPuart_rx_pio_pc != -1){
        pio_sm_set_enabled(PUART_PIO, PUART_SM, false);
        pio_remove_program(PUART_PIO, &uart_rx_program, gPuart_rx_pio_pc);
        gPuart_rx_pio_pc = -1;
    }
}

char puart_getc(){
    uint32_t w = pio_sm_get_blocking(PUART_PIO, PUART_SM);
    return (char)(w>>24);
}

bool puart_is_readable(){
    return !pio_sm_is_rx_fifo_empty(PUART_PIO, PUART_SM);
}
