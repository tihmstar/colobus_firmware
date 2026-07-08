#ifndef USBMUX_H
#define USBMUX_H

#define USBMUX_SEL0  24
#define USBMUX_SEL1  25

enum Muxcfg{
  kMuxcfg_iphone_disconnected = 0,
  kMuxcfg_iphone_to_hub = 1,
  kMuxcfg_iphone_to_gpio = 2,
  kMuxcfg_iphone_to_pico = 3,
};

void usbmux_configure(enum Muxcfg cfg);

#endif // USBMUX_H