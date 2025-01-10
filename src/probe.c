#include "probe.h"
#include "swd.h"

#include <tusb.h>

#include <macros.h>

#pragma mark defines
enum COLOBUS_CMDS {
    kCOLOBUS_CMD_INVALID = 0,
    kCOLOBUS_CMD_READ,
    kCOLOBUS_CMD_WRITE,
    kCOLOBUS_CMD_RESET,
    kCOLOBUS_CMD_FREQ
};

struct __attribute__((__packed__)) colobus_cmd {
    uint8_t id;
    uint8_t cmd;
    uint8_t req;
    uint8_t res;
    uint32_t data;
};

#pragma mark globals
static struct colobus_cmd gCmands[0x100];


static uint8_t processCmd(struct colobus_cmd *cmd){
    int err = 0;
    switch (cmd->cmd){
    case kCOLOBUS_CMD_READ:
        err = swd_read(cmd->req, &cmd->data);
        break;

    case kCOLOBUS_CMD_WRITE:
        err = swd_write(cmd->req, cmd->data);
        break;

    case kCOLOBUS_CMD_RESET:
        err = swd_reset() == true;
        break;

    case kCOLOBUS_CMD_FREQ:
        swd_set_freq(cmd->data);
        err = 1;
        break;
    
    default:
        return __LINE__;
    }
error:
    return (uint8_t)err;
}

#pragma mark public
void probe_task(){
    int err = 0;
    uint32_t cmdsReadSize = 0;
    uint32_t cmdsCnt = 0;

    cassure(tud_vendor_available());

    cassure(cmdsReadSize = tud_vendor_read(gCmands, sizeof(gCmands)));
    cmdsCnt = cmdsReadSize/sizeof(*gCmands);

    for (size_t i = 0; i < cmdsCnt; i++){
        gCmands[i].res = processCmd(&gCmands[i]);
    }
    {
        uint32_t needSendSize = cmdsCnt*sizeof(*gCmands);
        uint32_t didSend = 0;
        for (size_t i = 0; i < 10 && didSend < needSendSize; i++){
            uint32_t curDidSend = tud_vendor_write(((uint8_t*)gCmands)+didSend, needSendSize-didSend);
            didSend += curDidSend;
            tud_vendor_flush();
        }
    }

error:
    return;
}