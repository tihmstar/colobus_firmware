#include "probe.h"
#include "swd.h"

#include <tusb.h>

#include <macros.h>
#define ARRAYOF(a) (sizeof(a)/sizeof(*a))

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
static bool gUseNewProtocol = true;
static uint8_t gSWDAckDelay = 0;

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
        swd_set_freq_hz(cmd->data*1000);
        err = 1;
        break;
    
    default:
        return __LINE__;
    }
    return (uint8_t)err;
}

struct __attribute__((__packed__)) newTypeCmd {
    uint16_t cmd;
    uint16_t len;
    uint16_t pad1;
    uint8_t id;
    uint8_t more;
    uint8_t data[0];
};

// struct __attribute__((__packed__)) ServerInfo {
//     uint32_t u1;
//     uint32_t bcd_fwvers : 12;
//     uint32_t u2_2 : 20;
//     uint32_t u3;
//     uint32_t u4;
//     uint32_t u5;
//     uint32_t u6;
// }

static bool reset_line_with_err(uint32_t *errcode){
    uint32_t dummy_flush = 0;
    if (!errcode) errcode = &dummy_flush; 
    for (size_t i = 0; i < 10; i++){
        if (!swd_reset()) continue;
        sleep_us(10);
        if (SWD_DP_read_CTRL(errcode) != SWD_RSP_OK) continue;
        if (SWD_DP_clear_error() != SWD_RSP_OK) continue;
        return true;
    }
    return false;
}

static bool reset_line(){
    return reset_line_with_err(NULL);
}

static bool processNewSWDCmd(void *buf, size_t bufSize){
    int err = 0;
    struct newTypeCmd *cmd = (struct newTypeCmd*)buf;
    if (bufSize < sizeof(*cmd)) return false;
#define doReply(v) do {cmd->cmd = 0;tud_vendor_write(cmd, sizeof(*cmd));uint16_t rsp[2] = {0,v};tud_vendor_write(rsp, sizeof(rsp));tud_vendor_write(&cmd->data[4], cmd->len-4);;tud_vendor_flush(); return true;} while (0)
#define replyWord(w) do {cmd->cmd = 0;uint32_t unk = w;cmd->len = sizeof(unk);tud_vendor_write(cmd, sizeof(*cmd));tud_vendor_write(&unk, sizeof(unk));tud_vendor_flush();return true;} while (0)
#define replyByte(b) do {cmd->cmd = 0;uint8_t unk = b;cmd->len = sizeof(unk);tud_vendor_write(cmd, sizeof(*cmd));tud_vendor_write(&unk, sizeof(unk));tud_vendor_flush();return true;} while (0)

    switch (cmd->cmd){
    case 0x8002:
        replyByte(reset_line());
    case 0x8003:
        if (cmd->more == 0) swd_set_freq_hz((*(uint32_t*)cmd->data));
        replyWord(swd_get_freq_hz());
    case 0x800B:
        replyWord(0);
    case 0x8007:
        replyWord((*(uint32_t*)cmd->data) & 0xffffff);
    case 0x8001:
    {
        uint32_t someID[] = {
            0x41410012,
            0x01028749,
            0x00000001,
            0,
            0,
            0x41424344
        };
        cmd->cmd = 0;
        cmd->len = sizeof(someID);
        tud_vendor_write(cmd, sizeof(*cmd));
        tud_vendor_write(someID, sizeof(someID));
        tud_vendor_flush();
        return true;
    }

    case 0x900a:
    {
        cmd->cmd = 0;
        cmd->len = 0;
        tud_vendor_write(cmd, sizeof(*cmd));
        tud_vendor_flush();
        return true;
    }

    case 0x2000:
    {
        cmd->cmd = 0;
        if (cmd->len == 0x14){
            cmd->len = 0x18;
            uint32_t rsp = 0;
            uint8_t fail = 0;
            fail |= (swd_read(BITS_DP_READ(cmd->data[0x08]<<1), &rsp) != SWD_RSP_OK);
            if (fail) reset_line();
            cmd->data[2] = fail ? 0xFE : 0x00;
            tud_vendor_write(cmd, sizeof(*cmd));
            tud_vendor_write(cmd->data, cmd->len-4);
            tud_vendor_write(&rsp, 4);
            tud_vendor_flush();
            return true;
        }else if (cmd->len == 0x18){
            uint8_t fail = 0;
            fail |= (swd_write(BITS_DP_WRITE(cmd->data[0x08]<<1), *(uint32_t*)&cmd->data[0x14]) != SWD_RSP_OK);
            if (fail) reset_line();
            cmd->data[2] = fail ? 0xFE : 0x00;
            tud_vendor_write(cmd, sizeof(*cmd));
            tud_vendor_write(cmd->data, cmd->len);
            tud_vendor_flush();
            return true;
        }
        break;
    }

    case 0x2001:
    {
        cmd->cmd = 0;
        uint32_t req = (((uint32_t)cmd->data[0x03]) << 24) | (cmd->data[0x08] & 0xF0);
        if (cmd->len == 0x14){
            cmd->len = 0x18;
            uint32_t rsp = 0;
            uint8_t fail = 0;
            for (int z=0; z<2; z++){
                fail |= (SWD_DP_write_SELECT(req) != SWD_RSP_OK);
                fail |= (swd_read(BITS_AP_READ((cmd->data[0x08] & 0xF)<<1),&rsp) != SWD_RSP_OK);
                fail |= (SWD_DP_read_RDBUFF(&rsp) != SWD_RSP_OK);
                cmd->data[2] = fail ? 0xFE : 0x00;
                if (fail){
                    reset_line_with_err((uint32_t*)(&cmd->data[0x04]));
                }else{
                    fail |= (SWD_DP_read_CTRL((uint32_t*)(&cmd->data[0x04])) != SWD_RSP_OK);
                    if (fail) reset_line_with_err((uint32_t*)(&cmd->data[0x04]));
                }
                if (!fail) break;
                fail = 0;
            }
            tud_vendor_write(cmd, sizeof(*cmd));
            tud_vendor_write(cmd->data, cmd->len-4);
            tud_vendor_write(&rsp, 4);
            tud_vendor_flush();
            return true;
        }else if (cmd->len == 0x18){
            uint8_t fail = 0;
            fail |= (SWD_DP_write_SELECT(req) != SWD_RSP_OK);
            fail |= (swd_write(BITS_AP_WRITE((cmd->data[0x08] & 0xF)<<1),*(uint32_t*)&cmd->data[0x14]) != SWD_RSP_OK);
            cmd->data[2] = fail ? 0xFE : 0x00;
            if (fail){
                reset_line_with_err((uint32_t*)(&cmd->data[0x04]));
            }else{
                fail |= (SWD_DP_read_CTRL((uint32_t*)(&cmd->data[0x04])) != SWD_RSP_OK);
                if (fail) reset_line_with_err((uint32_t*)(&cmd->data[0x04]));
            }
            cmd->len = 0x14;
            tud_vendor_write(cmd, sizeof(*cmd));
            tud_vendor_write(cmd->data, cmd->len);
            tud_vendor_flush();
            return true;
        }
        break;
    }

    case 0x2003:
    {
        cmd->cmd = 0;
        uint64_t addr = 0;
        addr = *(uint32_t*)&cmd->data[0x0C];
        addr <<= 32;
        addr |= *(uint32_t*)&cmd->data[0x08];
        uint32_t cnt = *(uint32_t*)&cmd->data[0x10];
        uint32_t req = (((uint32_t)cmd->data[0x03]) << 24);
        uint32_t *w = (uint32_t*)&cmd->data[0x14];
        if (cmd->len == 0x14){
            uint32_t rsp[0x10] = {};
            uint8_t didRead = 0;
            uint8_t fail = 0;
            fail |= (SWD_DP_write_SELECT(req) != SWD_RSP_OK);
            fail |= (SWD_AP_write_TAR_HIGH(addr>>32) != SWD_RSP_OK);
            fail |= (SWD_AP_write_TAR(addr) != SWD_RSP_OK);
            fail |= (SWD_AP_read_DRW(&rsp[0]) != SWD_RSP_OK);
            for (uint32_t curw = 1; curw < cnt && curw < ARRAYOF(rsp); curw++){
                fail |= (SWD_AP_read_DRW(&rsp[didRead]) != SWD_RSP_OK);
                if (fail) break;
                didRead++;
            }
            fail |= (SWD_DP_read_RDBUFF(&rsp[didRead++]) != SWD_RSP_OK);
            cmd->data[2] = fail ? 0xFE : 0x00;
            if (fail){
                reset_line_with_err((uint32_t*)(&cmd->data[0x04]));
            }else{
                fail |= (SWD_DP_read_CTRL((uint32_t*)(&cmd->data[0x04])) != SWD_RSP_OK);
                if (fail) reset_line_with_err((uint32_t*)(&cmd->data[0x04]));
            }
            cmd->len += didRead*4;
            tud_vendor_write(cmd, sizeof(*cmd));
            tud_vendor_write(cmd->data, 0x14);
            tud_vendor_write(rsp, 4*didRead);
            tud_vendor_flush();
            return true;
        }else if (cmd->len >= 0x18){
            uint8_t fail = 0;
            fail |= (SWD_DP_write_SELECT(req) != SWD_RSP_OK);
            fail |= (SWD_AP_write_TAR_HIGH(addr>>32) != SWD_RSP_OK);
            fail |= (SWD_AP_write_TAR(addr) != SWD_RSP_OK);
            for (uint32_t curw = 0; curw < cnt && curw*4 < cmd->len-0x14; curw++){
                fail |= (SWD_AP_write_DRW(w[curw]) != SWD_RSP_OK);
                if (fail) break;
            }
            cmd->data[2] = fail ? 0xFE : 0x00;
            if (fail){
                reset_line_with_err((uint32_t*)(&cmd->data[0x04]));
            }else{
                fail |= (SWD_DP_read_CTRL((uint32_t*)(&cmd->data[0x04])) != SWD_RSP_OK);
                if (fail) reset_line_with_err((uint32_t*)(&cmd->data[0x04]));
            }
            cmd->len = 0x14;
            tud_vendor_write(cmd, sizeof(*cmd));
            tud_vendor_write(cmd->data, cmd->len);
            tud_vendor_flush();
            return true;
        }

        break;
    }


    case 0x9003:
       {
        cmd->cmd = 0;
        uint32_t rsp[4]={};
        reset_line();
        rsp[1] = swd_read(BITS_DP_READ(BITS_DP_IDCODE), &rsp[0]);
        cmd->len = sizeof(rsp);
        tud_vendor_write(cmd, sizeof(*cmd));
        tud_vendor_write(rsp, sizeof(rsp));
        tud_vendor_flush();
        return true;
       } 
    
    case 0xC000: //get
    case 0xC001: //set
        cassure(cmd->len > 4);
        if (!strncmp((char*)&cmd->data[0x4], "connected",cmd->len-4)){
            doReply(1);
        } else if (!strncmp((char*)&cmd->data[0x4], "hostid",cmd->len-4)){
            doReply(7);
        } else if (!strncmp((char*)&cmd->data[0x4], "ptarget",cmd->len-4)){
            doReply(0x04b0);
        } else if (!strncmp((char*)&cmd->data[0x4], "swdackdelay",cmd->len-4)){
            if (cmd->cmd == 0xC001) gSWDAckDelay = cmd->data[0x02];
            doReply(gSWDAckDelay);
        } else if (!strncmp((char*)&cmd->data[0x4], "swdovrrun",cmd->len-4)){
            doReply(0);
        }
        break;
    
    default:
        break;
    }
error:
    (void)err;
    return false;
}

#pragma mark public
void probe_set_protocol(bool useNew){
    gUseNewProtocol = useNew;
}

int probe_task(bool dontRunSWDCommands){
    int err = 0;
    uint32_t cmdsReadSize = 0;
    uint32_t cmdsCnt = 0;
    uint32_t hasData = 0;

    cassure(hasData = tud_vendor_available());


    if (gUseNewProtocol){
        static uint8_t swdCmdsNew[0x100];
        while ((hasData = tud_vendor_available()) >= 8){
            cassure(cmdsReadSize = tud_vendor_read(swdCmdsNew, 8));
            if (cmdsReadSize == 8){
                uint8_t needsRead = swdCmdsNew[2];
                if (needsRead && tud_vendor_read(&swdCmdsNew[8], needsRead) != needsRead){
                    tud_vendor_read(swdCmdsNew, sizeof(swdCmdsNew));
                    return err;
                }
                cmdsReadSize+=needsRead;
            }
            cassure(dontRunSWDCommands == false);
            processNewSWDCmd(swdCmdsNew, cmdsReadSize);
            tud_task();    
        }        
    }else{
        static struct colobus_cmd swdCmdsColobus[0x100];
        cassure(cmdsReadSize = tud_vendor_read(swdCmdsColobus, sizeof(swdCmdsColobus)));
        cassure(dontRunSWDCommands == false);
        cmdsCnt = cmdsReadSize/sizeof(*swdCmdsColobus);
        for (size_t i = 0; i < cmdsCnt; i++){
            swdCmdsColobus[i].res = processCmd(&swdCmdsColobus[i]);
        }
        {
            uint32_t needSendSize = cmdsCnt*sizeof(*swdCmdsColobus);
            uint32_t didSend = 0;
            for (size_t i = 0; i < 10 && didSend < needSendSize; i++){
                uint32_t curDidSend = tud_vendor_write(((uint8_t*)swdCmdsColobus)+didSend, needSendSize-didSend);
                didSend += curDidSend;
                tud_vendor_flush();
            }
        }
    }
error:
    return err;
}