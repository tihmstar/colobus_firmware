#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "pio_usb.h"
#include "pio_usb_ll.h"

typedef struct {
    pio_port_t   *pp;
    root_port_t  *root;
    usb_device_t *dev;
    uint8_t       maxpacket0;
} bus_t;

void bus_init(bus_t *b, bool skip_alarm_pool);

bool bus_wait_for_connect(bus_t *b);

void bus_reset_ep0_reopen(bus_t *b);

void bus_reset(bus_t *b, uint32_t hold_ms);

void bus_close_all(bus_t *b);

bool bus_open_ep0(bus_t *b, uint8_t maxpacket);

int bus_control_xfer(bus_t *b, const uint8_t setup[8],
                     uint8_t *data, uint16_t data_len, bool data_in,
                     uint32_t timeout_ms);
