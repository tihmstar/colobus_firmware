#include "usbmux.h"

#include <hardware/gpio.h>
#include <stdint.h>

#pragma mark public
void usbmux_configure(enum Muxcfg cfg){
  gpio_init(USBMUX_SEL0);
  gpio_init(USBMUX_SEL1);
  uint32_t val =  ((((uint8_t)cfg) >> 0) & 1) << USBMUX_SEL0
                | ((((uint8_t)cfg) >> 1) & 1) << USBMUX_SEL1;
  uint32_t mask = (1<<USBMUX_SEL0) | (1<<USBMUX_SEL1);
  gpio_put_masked(mask, val);
  gpio_set_dir_out_masked(mask);
}
