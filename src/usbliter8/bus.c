#include "bus.h"

#include <pico/time.h>
#include <stdio.h>
#include <string.h>

#include <hardware/dma.h>
#include <hardware/pio.h>
#include <hardware/sync.h>
#include <pico/stdlib.h>

#include "pio_usb.h"
#include "pio_usb_ll.h"
#include "usb_definitions.h"

void bus_init(bus_t *b, bool skip_alarm_pool) {
    pio_usb_configuration_t cfg = PIO_USB_DEFAULT_CONFIG;
    cfg.skip_alarm_pool = skip_alarm_pool;
    b->dev  = pio_usb_host_init(&cfg);
    b->root = PIO_USB_ROOT_PORT(0);
    b->pp   = PIO_USB_PIO_PORT(0);
    b->maxpacket0 = 8;
}

bool bus_wait_for_connect(bus_t *b) {
    while (!b->root->connected) {
        sleep_ms(10);
        tight_loop_contents();
    }

    return true;
}

void bus_close_all(bus_t *b) {
    pio_usb_host_close_device(0, b->dev->address);
}

void bus_reset(bus_t *b, uint32_t hold_ms) {
    pio_usb_host_port_reset_start(0);
    sleep_ms(hold_ms);

    pio_usb_host_port_reset_end(0);
    sleep_ms(50);  // recovery + slack

    b->dev->connected    = true;
    b->dev->is_fullspeed = b->root->is_fullspeed;
    b->dev->is_root      = true;
    b->dev->root         = b->root;
    b->dev->address      = 0;
}

bool bus_open_ep0(bus_t *b, uint8_t maxpacket) {
    endpoint_descriptor_t ep0 = {
        .length   = 7,
        .type     = DESC_TYPE_ENDPOINT,
        .epaddr   = 0x00,
        .attr     = EP_ATTR_CONTROL,
        .max_size = {maxpacket, 0},
        .interval = 0,
    };

    if (!pio_usb_host_endpoint_open(0, b->dev->address, (const uint8_t *)&ep0, false)) {
        return false;
    }

    b->maxpacket0 = maxpacket;

    return true;
}

void bus_reset_ep0_reopen(bus_t *b) {
    bus_close_all(b);
    bus_reset(b, 20);
    bus_open_ep0(b, 64);
}

int bus_control_xfer(bus_t *b, const uint8_t setup[8], uint8_t *data, uint16_t data_len, bool data_in, uint32_t timeout_ms) {
    bool has_data = (data != NULL) && (data_len > 0);

    b->dev->control_pipe.stage          = STAGE_SETUP;
    b->dev->control_pipe.operation      = data_in ? CONTROL_IN : CONTROL_OUT;
    b->dev->control_pipe.rx_buffer      = data_in ? data : NULL;
    b->dev->control_pipe.request_length = data_in ? data_len : 0;
    b->dev->control_pipe.out_data_packet.tx_address = (!data_in && has_data) ? data : NULL;
    b->dev->control_pipe.out_data_packet.tx_length  = (!data_in && has_data) ? data_len : 0;

    if (!pio_usb_host_send_setup(0, b->dev->address, setup)) {
        return -1;
    }

    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (b->dev->control_pipe.operation != CONTROL_COMPLETE && b->dev->control_pipe.operation != CONTROL_ERROR) {
        if (time_reached(deadline)) {
            return -2;
        }
        tight_loop_contents();
    }

    return (b->dev->control_pipe.operation == CONTROL_COMPLETE) ? 0 : -1;
}
