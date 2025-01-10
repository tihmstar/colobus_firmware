#ifndef LIGHTNING_H
#define LIGHTNING_H

#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

void lightning_gpio_configure(int pin_sdq);
void lightning_init(int pin_sdq);
void lightning_cleanup(void);

// void lightning_read_blocking(void *buf, size_t bufSize);
void lightning_write_nonblocking(const void *buf, uint8_t bufSize);
void lightning_write_blocking(const void *buf, uint8_t bufSize);

typedef void (*lightning_rx_cb)(const void *buf, size_t bufSize);

void lightning_rx_callback_register(lightning_rx_cb cb);
void lightning_rx_callback_unregister();

#ifdef __cplusplus
}
#endif

#endif // LIGHTNING