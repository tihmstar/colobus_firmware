#ifndef PIO_I2C_H
#define PIO_I2C_H

#include <stdint.h>
#include <stdbool.h>

void pio_i2c_init(uint8_t sda, uint8_t scl);
void pio_i2c_deinit();

int pio_i2c_write_blocking(uint8_t addr, void *txbuf, uint8_t len, bool nonstop);
int pio_i2c_read_blocking(uint8_t addr, void *rxbuf, uint8_t len);

#endif // PIO_I2C_H