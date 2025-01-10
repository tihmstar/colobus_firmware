#ifndef TRISTAR_H
#define TRISTAR_H

#include <stdint.h>

#pragma mark TRISTAR REQUESTS
#define TRISTAR_REQUEST_GET_CABLE_TYPE 0x74

#pragma mark COLOBUS MODES
typedef enum {
    kCOLOBUS_MODE_RESET,
    kCOLOBUS_MODE_DFU,

    kCOLOBUS_MODE_USB_HOST,

    kCOLOBUS_MODE_USB_CHARGING,
    kCOLOBUS_MODE_USB_UART,
    kCOLOBUS_MODE_USB_JTAG_UART,
    kCOLOBUS_MODE_USB_JTAG_SPAM,

    kCOLOBUS_MODE_MAX,
    kCOLOBUS_MODE_DEFAULT      = kCOLOBUS_MODE_USB_JTAG_UART,
    kCOLOBUS_MODE_FIRST        = kCOLOBUS_MODE_USB_CHARGING,
} t_colobus_mode;

#pragma mark COLOBOS CABLE TYPES
#define CABLE_TYPE_RSP_MAX_SIZE 7

extern uint8_t colobus_cable_type_responses[kCOLOBUS_MODE_MAX][CABLE_TYPE_RSP_MAX_SIZE+1];

#endif // TRISTAR_H