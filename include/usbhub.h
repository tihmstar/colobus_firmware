#ifndef USBHUB_H
#define USBHUB_H

#define USBHUB_PIN_RESET  19
#define USBHUB_PIN_CFG1   15
#define USBHUB_PIN_SCL    11
#define USBHUB_PIN_SDA    10

#pragma mark low level
void usbhub_init();
void usbhub_deinit();

#pragma mark easy-use
void usbhub_init_default();

#endif // USBHUB_H