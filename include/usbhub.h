#ifndef USBHUB_H
#define USBHUB_H

#define USBHUB_PIN_RESET  12
#define USBHUB_PIN_SCL    16
#define USBHUB_PIN_SDA    17

#pragma mark low level
void usbhub_init();
void usbhub_deinit();

#pragma mark easy-use
void usbhub_init_default();

#endif // USBHUB_H