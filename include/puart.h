#ifndef PUART_H
#define PUART_H

#include <stdbool.h>
#include <hardware/pio.h>

void puart_init(PIO pio, int uart_rx, int uart_tx);
void puart_deinit(void);

char puart_getc();
bool puart_is_readable();

#endif // PUART_H