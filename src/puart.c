#include "puart.h"
#include "uart.pio.h"

#include <hardware/pio.h>
#include <hardware/clocks.h>


#define BAUD 115200

#pragma mark static vars
static int gPuart_sm = -1;
static int gPuart_rx_pio_pc = -1;
static int gPuart_tx_pio_pc = -1;
static PIO gPuartPIO = NULL;//pio1;

#pragma mark code
void puart_init(PIO pio, int uart_rx, int uart_tx){
  gPuartPIO = pio;
  if (gPuart_sm == -1){
    gPuart_sm = pio_claim_unused_sm(gPuartPIO, true);
  }
  if (gPuart_rx_pio_pc == -1){
        pio_sm_set_enabled(gPuartPIO, gPuart_sm, false);

        gPuart_rx_pio_pc = pio_add_program(gPuartPIO, &uart_rx_program);
        pio_sm_config c = uart_rx_program_get_default_config(gPuart_rx_pio_pc);

        {
            float div = (float)clock_get_hz(clk_sys) / (8 * BAUD);
            sm_config_set_clkdiv(&c, div);
        }

        sm_config_set_in_pins(&c, uart_rx);
        sm_config_set_jmp_pin(&c, uart_rx);
        sm_config_set_in_shift(&c, true, false, 32);
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);

        gpio_set_function(uart_rx, GPIO_FUNC_PIO0 + pio_get_index(gPuartPIO));
        pio_sm_set_consecutive_pindirs(gPuartPIO, gPuart_sm, uart_rx, 1, false);

        pio_sm_init(gPuartPIO, gPuart_sm, gPuart_rx_pio_pc, &c);
        pio_sm_set_enabled(gPuartPIO, gPuart_sm, true);
    }

}

void puart_deinit(void){
    if (gPuart_rx_pio_pc != -1){
        pio_sm_set_enabled(gPuartPIO, gPuart_sm, false);
        pio_remove_program(gPuartPIO, &uart_rx_program, gPuart_rx_pio_pc);
        gPuart_rx_pio_pc = -1;
    }
    if (gPuart_sm != -1){
      pio_sm_unclaim(gPuartPIO, gPuart_sm); gPuart_sm = -1;
    }
}

char puart_getc(){
    uint32_t w = pio_sm_get_blocking(gPuartPIO, gPuart_sm);
    return (char)(w>>24);
}

bool puart_is_readable(){
    return !pio_sm_is_rx_fifo_empty(gPuartPIO, gPuart_sm);
}
