#pragma once

#include "bus.h"

void usb_start(void);
int  usb_bus_init(void);
int  usb_bus_wait_for_device(void);
int  usb_bus_reset_open_ep0(void);

enum {
    USB_CMD_BUS_INIT = 0,
    USB_CMD_WAIT_FOR_DEVICE,
    USB_CMD_RESET_OPEN_EP0,
    USB_CMD_EXECUTE_FUNC
};

void usb_task_queue_cmd(int cmd);
bool usb_task_cmd_has_finished();
int usb_task_get_cmd_res();

typedef int (*usb_executee_t)(bus_t *b, void *ctx);

int usb_bus_execute(usb_executee_t func, void *ctx, uint64_t timeout);
