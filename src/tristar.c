#include "tristar.h"

uint8_t colobus_cable_type_responses[kCOLOBUS_MODE_MAX][CABLE_TYPE_RSP_MAX_SIZE+1] = {
    [kCOLOBUS_MODE_RESET] = {0x75, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00},
    [kCOLOBUS_MODE_DFU] = {0x75, 0x20, 0x00, 0x02, 0x00, 0x00, 0x00},

    [kCOLOBUS_MODE_USB_HOST] = {0x75, 0x18, 0xFE, 0x09, 0xA1, 0x00, 0x00}, //lightning to usb camera adapter

    [kCOLOBUS_MODE_USB_CHARGING] = {0x75, 0x10, 0x0c, 0x00, 0x00, 0x00, 0x00},
    [kCOLOBUS_MODE_USB_UART] = {0x75, 0x20, 0x00, 0x10, 0x00, 0x00, 0x00},
    [kCOLOBUS_MODE_USB_JTAG_UART] = {0x75, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00},
    [kCOLOBUS_MODE_USB_JTAG_SPAM] = {0x75, 0xa0, 0x08, 0x10, 0x00, 0x00, 0x00},
};