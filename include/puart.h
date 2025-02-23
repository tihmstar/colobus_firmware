#ifndef PUART_H
#define PUART_H

#include <stdbool.h>

void puart_init(int uart_rx, int uart_tx);
void puart_deinit(void);

char puart_getc();
bool puart_is_readable();

#endif // PUART_H