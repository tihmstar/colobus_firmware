#include <stdint.h>

#include "pico/multicore.h"

#include "usb.h"
#include "bus.h"
#include "log.h"

static struct {
    usb_executee_t func;
    void *ctx;
} usb_exec_ctx;

static bus_t gBus = { 0 };

void usb_task(void) {
    while (1) {
        uint32_t cmd = multicore_fifo_pop_blocking();
        uint32_t ret = -1;

        switch (cmd) {
            case USB_CMD_BUS_INIT: {
                bus_init(&gBus, false);
                ret = 0;
                break;
            }

            case USB_CMD_WAIT_FOR_DEVICE: {
                bus_wait_for_connect(&gBus);
                ret = 0;
                break;
            }

            case USB_CMD_RESET_OPEN_EP0: {
                bus_reset_ep0_reopen(&gBus);
                ret = 0;
                break;
            }

            case USB_CMD_EXECUTE_FUNC: {
                ret = usb_exec_ctx.func(&gBus, usb_exec_ctx.ctx);
                break;
            }
        }

        multicore_fifo_push_blocking(ret);
    }
}

void usb_start(void) {
    multicore_reset_core1();
    multicore_launch_core1(usb_task);
}

int _usb_task_execute_cmd(int cmd, uint64_t timeout) {
    multicore_fifo_push_blocking(cmd);

    uint32_t out = -1;

    if (timeout) {
        if (!multicore_fifo_pop_timeout_us(timeout, &out)) {
            INFO("TIMEOUT");
        }
    } else {
        out = multicore_fifo_pop_blocking();
    }

    return out;
}

void usb_task_queue_cmd(int cmd) {
  multicore_fifo_push_blocking(cmd);
}

bool usb_task_cmd_has_finished(){
  return multicore_fifo_rvalid();
}

int usb_task_get_cmd_res(){
  return multicore_fifo_pop_blocking();
}

#define DEFAULT_TIMEOUT_US  (100 * 1000)

int usb_bus_init(void) {
    return _usb_task_execute_cmd(USB_CMD_BUS_INIT, DEFAULT_TIMEOUT_US);
}

int usb_bus_wait_for_device(void) {
    INFO("waiting for device...");

    int ret = _usb_task_execute_cmd(USB_CMD_WAIT_FOR_DEVICE, 200);
    if (ret == 0) {
        INFO("connected, speed = %s", gBus.root->is_fullspeed ? "FS" : "LS");
    }

    return ret;
}

int usb_bus_reset_open_ep0(void) {
    int ret = _usb_task_execute_cmd(USB_CMD_RESET_OPEN_EP0, DEFAULT_TIMEOUT_US);
    if (ret == 0) {
        INFO("opened EP0");
    }

    return ret;
}

int usb_bus_execute(usb_executee_t func, void *ctx, uint64_t timeout) {
    usb_exec_ctx.func = func;
    usb_exec_ctx.ctx = ctx;

    return _usb_task_execute_cmd(USB_CMD_EXECUTE_FUNC, timeout);
}
