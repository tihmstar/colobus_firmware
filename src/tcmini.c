#include "tcmini.h"
#include "macros.h"
#include "fusb.h"

#include <pico/time.h>
#include <hardware/gpio.h>
#include <hardware/i2c.h>

#include <string.h>
#include <stdio.h>

#include <tusb.h>
#include <typec/pd_types.h>

#define TCMINI_DBG
#define TCMINI_VDM_LOG
#define TCMINI_PD_LOG

struct CDevice{
    bool isConnected;
    bool cc2Polarity;
    bool hasSentMessage;
    bool isInited;
    uint8_t msgID;

#define SND_MAX_QUEUE 4
    uint8_t sndBuf[SND_MAX_QUEUE][40];
    uint8_t sndSizes[SND_MAX_QUEUE];
    uint8_t sndNext;
    uint8_t sndDone;
    uint8_t sndDo;
};

union TCMinitIRQs{
    uint32_t irqAll;
    struct {
        uint8_t irq;
        uint8_t irqA;
        uint8_t irqB;
    };
};

#pragma mark globals
static struct CDevice gDev = {};
static bool gIRQIsPending = false;
static t_tcmini_vdm_cb gVDMCB = NULL;
static t_tcmini_dev_cb gDevCB = NULL;
static bool gIsDeviceMode = false;
static uint32_t gBorrowCap = 0;
static bool gIsCCManualMode = false;
static bool gManualModeIscc2Polarity = false;
static bool gIsPowerProxy = false;

#pragma mark defines
static int tcmini_readbyte(uint8_t addr, uint8_t *data);
static int tcmini_writebyte(uint8_t addr, uint8_t data);
static int tcmini_write(uint8_t *data, uint32_t size);
static int tcmini_cfg_clr_set(uint8_t reg, uint8_t clr, uint8_t set);
static int tcmini_update_connected_status();
static void tcmini_irq(uint gpio, uint32_t event_mask);
static int tcmini_deinitCDevice(struct CDevice *dev);
static int tcmini_initCDevice(struct CDevice *dev);
static int tcmini_populate_defaults();
static void tcmini_enable_irq();
static void tcmini_disable_irq();
static int tcmini_pd_send(pd_header_t header, const uint32_t *data, bool isVDM);
static int tcmini_pd_recv(pd_header_t *header, uint32_t *data, size_t dataCnt);
static int tcmini_handle_pd_packet();
static int tcmini_pd_write_push(uint8_t *data, uint8_t size);
static int tcmini_pd_write_perform();
static int tcmini_handle_vmd_internal(uint32_t *data, uint8_t cnt);

#pragma mark private
static uint32_t build_fixed_pdo(){
    // USB PD R2 0 V1.3 Table 6-6
    // Bit(s) Description
    // B31…30 Fixed supply
    // B29 Dual-Role Power
    // B28 USB Suspend Supported
    // B27 Unconstrained Power
    // B26 USB Communications Capable
    // B25 Dual-Role Data
    // B24…22 Reserved – Shall be set to zero.
    // B21…20 Peak Current
    // B19…10 Voltage in 50mV units
    // B9…0 Maximum Current in 10mA units
    uint32_t r = (0 << 31) | (1 << 26) | // usb comms capable
                 (100 << 10) |           // 5 volt = 100 * 50mV
                 (300 << 0);             // 3A = 300 * 10mA
    return r;
}

static int tcmini_pd_write_push(uint8_t *data, uint8_t size){
    int err = 0;

    uint8_t inc = (gDev.sndNext+1) % SND_MAX_QUEUE;
    cassure(inc != gDev.sndDone);
    gDev.sndSizes[gDev.sndNext] = size;
    memcpy(&gDev.sndBuf[gDev.sndNext], data, size);
    gDev.sndNext = inc;

error:
    return err;
}

static int tcmini_pd_write_perform(){
    int err = 0;
    if (gDev.sndNext != gDev.sndDone && gDev.sndDo == gDev.sndDone){
        cassure(!tcmini_write(gDev.sndBuf[gDev.sndDone], gDev.sndSizes[gDev.sndDone]));
        gDev.sndDo = (gDev.sndDo+1) % SND_MAX_QUEUE;
    }
error:
    return err;
}

static int tcmini_pd_send(pd_header_t header, const uint32_t *data, bool isVDM){
    int err = 0;
	uint8_t buf[40] = {};
	int16_t pos = 0;
    uint8_t dataLen = header.n_data_obj*sizeof(*data);
    header.msg_id = gDev.msgID++;

    cassure(!tcmini_cfg_clr_set(FUSB_REG_CONTROL0, 
        0,
                    FUSB_CONTROL0_VAL_TX_FLUSH
    ));

    buf[pos++] = FUSB_REG_FIFOS;
    if (isVDM){
        buf[pos++] = FUSB_FIFOS_TOK_SOP1;
        buf[pos++] = FUSB_FIFOS_TOK_RESET2;
        buf[pos++] = FUSB_FIFOS_TOK_SOP3;
        buf[pos++] = FUSB_FIFOS_TOK_SOP2;
    }else{
        buf[pos++] = FUSB_FIFOS_TOK_SOP1;
        buf[pos++] = FUSB_FIFOS_TOK_SOP1;
        buf[pos++] = FUSB_FIFOS_TOK_SOP1;
        buf[pos++] = FUSB_FIFOS_TOK_SOP2;
    }    

    buf[pos++] = FUSB_FIFOS_TOK_PACKSYM(sizeof(header)+dataLen);
    memcpy(&buf[pos], &header, sizeof(header)); pos += sizeof(header);
    memcpy(&buf[pos], data, dataLen); pos += dataLen;

    buf[pos++] = FUSB_FIFOS_TOK_JAM_CRC;
    buf[pos++] = FUSB_FIFOS_TOK_EOP;
    buf[pos++] = FUSB_FIFOS_TOK_TXOFF;
    buf[pos++] = FUSB_FIFOS_TOK_TXON;

    cassure(!tcmini_pd_write_push(buf, pos));
    
error:
    return err;
}

static int tcmini_pd_recv(pd_header_t *header, uint32_t *data, size_t dataCnt){
    int err = 0;
    uint8_t val = 0;
    struct __attribute__((__packed__)) {
        uint8_t sop;
        pd_header_t hdr;
    } rcv;
    uint32_t crc = 0;
    uint8_t dataLen = 0;

    do{
        cassure(!tcmini_readbyte(FUSB_REG_STATUS1, &val));
// #ifdef TCMINI_PD_LOG
//         {
//             char buf[0x100] = {};
//             snprintf(buf,sizeof(buf),"[tcmini] PD status1=0x%02x\r\n",val);
//             tud_cdc_n_write_str(2, buf);
//         }
// #endif
        cassure(!(val & FUSB_STATUS1_VAL_RX_EMPTY));
        val = FUSB_REG_FIFOS;
        cassure(i2c_write_blocking(i2c1, FUSB_DEVICE, &val, sizeof(val), true) == 1);
        cassure(i2c_read_blocking(i2c1, FUSB_DEVICE, (uint8_t*)&rcv, sizeof(rcv), true) == sizeof(rcv));
// #ifdef TCMINI_PD_LOG
//         {
//             char buf[0x100] = {};
//             snprintf(buf,sizeof(buf),"[tcmini] SOP=0x%02x PDHDR=0x%04x\r\n",rcv.sop,*(uint16_t*)&rcv.hdr);
//             tud_cdc_n_write_str(2, buf);
//         }
// #endif
        cassure(rcv.hdr.n_data_obj <= dataCnt);
        dataLen = rcv.hdr.n_data_obj*sizeof(*data);
        if (dataLen) cassure(i2c_read_blocking(i2c1, FUSB_DEVICE, (uint8_t*)data, dataLen, true) == dataLen);
        cassure(i2c_read_blocking(i2c1, FUSB_DEVICE, (uint8_t*)&crc, sizeof(crc), false) == sizeof(crc));
    }while (rcv.hdr.msg_type == PD_CTRL_GOOD_CRC && !gIsDeviceMode);
    memcpy(header, &rcv.hdr, sizeof(*header));

#ifdef TCMINI_DBG
    {
        char buf[0x100] = {};
        int didwrite = 0;
        didwrite = snprintf(buf,sizeof(buf),"[tcmini] tcmini_pd_recv(");
        uint8_t *hdr = (uint8_t*)header;
        for (size_t i = 0; i < 4; i++){
            didwrite += snprintf(&buf[didwrite],sizeof(buf)-didwrite,"%02x ",hdr[i]);
        }
        uint8_t *d = (uint8_t*)data;
        for (size_t i = 0; i < rcv.hdr.n_data_obj*4; i++){
            didwrite += snprintf(&buf[didwrite],sizeof(buf)-didwrite,"%02x ",d[i]);
        }
        uint8_t *c = (uint8_t*)&crc;
        for (size_t i = 0; i < 4; i++){
            didwrite += snprintf(&buf[didwrite],sizeof(buf)-didwrite,"%02x ",c[i]);
        }
        didwrite += snprintf(&buf[didwrite],sizeof(buf)-didwrite,")\r\n");
        tud_cdc_n_write_str(2, buf);
    }
#endif

error:
    if (err){
// #ifdef TCMINI_PD_LOG
//         {
//             char buf[0x100] = {};
//             snprintf(buf,sizeof(buf),"[tcmini] tcmini_pd_recv err=%d\r\n",err);
//             tud_cdc_n_write_str(2, buf);
//         }
// #endif        
    }
    return err;
}

#define uprintf(a...)do { char buf[0x100] = {}; snprintf(buf,sizeof(buf),a); tud_cdc_n_write_str(2, buf);} while(0)

static void parse_hdr(pd_header_t hdr){
    uprintf("\tmsg_type: ");
    #define PrintElem(e) case e: uprintf(#e); break
    switch (hdr.msg_type){
    PrintElem(PD_DATA_RESERVED);
    PrintElem(PD_DATA_SOURCE_CAP);
    PrintElem(PD_DATA_REQUEST);
    PrintElem(PD_DATA_BIST);
    PrintElem(PD_DATA_SINK_CAP);
    PrintElem(PD_DATA_BATTERY_STATUS);
    PrintElem(PD_DATA_ALERT);
    PrintElem(PD_DATA_GET_COUNTRY_INFO);
    PrintElem(PD_DATA_ENTER_USB);
    PrintElem(PD_DATA_EPR_REQUEST);
    PrintElem(PD_DATA_EPR_MODE);
    PrintElem(PD_DATA_SRC_INFO);
    PrintElem(PD_DATA_REVISION);
    PrintElem(PD_DATA_RESERVED_13);
    PrintElem(PD_DATA_RESERVED_14);
    PrintElem(PD_DATA_VENDOR_DEFINED);
    default:
        uprintf("UNKNOWN");
        break;
    }
    uprintf("\r\n");
    uprintf("\tdata_role: %s\r\n", hdr.data_role ? "UFP" : "DFP");
    uprintf("\tspecs_rev: %d\r\n", hdr.specs_rev);
    uprintf("\tpower_role: %s\r\n", hdr.power_role ? "Sink" : "Source");
    uprintf("\tmsg_id: %d\r\n", hdr.msg_id);
    uprintf("\tn_data_obj: %d\r\n", hdr.n_data_obj);
    uprintf("\textended: %d\r\n", hdr.extended);
}

static int parse_src_cap(uint32_t val){
    int fixedVoltage = -1;
    union pd_pdo_t{
        pd_pdo_fixed_t fixed;
        pd_pdo_battery_t bat;
        pd_pdo_variable_t var;
        pd_pdo_apdo_t apdo;
    };
    union pd_pdo_t *pdo = (union pd_pdo_t*)&val;

    uprintf("PDO:\r\n");
    switch (pdo->fixed.type){
    case PD_PDO_TYPE_FIXED:
    {
        uprintf("\ttype: PD_PDO_TYPE_FIXED\r\n");
        float curMax = pdo->fixed.current_max_10ma*0.01;
        uprintf("\tcurrent_max: %.03fA\r\n",curMax);
        float voltage = pdo->fixed.voltage_50mv*0.05;
        fixedVoltage = voltage;
        uprintf("\tvoltage: %.02fV\r\n",voltage);
        uprintf("\tcurrent_peak: %d\r\n",pdo->fixed.current_peak);
        uprintf("\tepr_mode_capable: %d\r\n",pdo->fixed.epr_mode_capable);
        uprintf("\tunchunked_ext_msg_support: %d\r\n",pdo->fixed.unchunked_ext_msg_support);
        uprintf("\tdual_role_data: %d\r\n",pdo->fixed.dual_role_data);
        uprintf("\tusb_comm_capable: %d\r\n",pdo->fixed.usb_comm_capable);
        uprintf("\tunconstrained_power: %d\r\n",pdo->fixed.unconstrained_power);
        uprintf("\tusb_suspend_supported: %d\r\n",pdo->fixed.usb_suspend_supported);
        uprintf("\tdual_role_power: %d\r\n",pdo->fixed.dual_role_power);
    }
        break;

    case PD_PDO_TYPE_APDO:
    {
        uprintf("\ttype: PD_PDO_TYPE_APDO\r\n");
        float curMax = pdo->apdo.current_max_50ma*0.05;
        uprintf("\tcurrent_max: %.02fA\r\n",curMax);
        float voltage_min = pdo->apdo.voltage_min_100mv*0.1;
        uprintf("\tvoltage_min: %.02fV\r\n",voltage_min);
        float voltage_max = pdo->apdo.voltage_max_100mv*0.1;
        uprintf("\tvoltage_max: %.02fV\r\n",voltage_max);
        uprintf("\tpps_power_limited: %d\r\n",pdo->apdo.pps_power_limited);
        uprintf("\tspr_programmable: %d\r\n",pdo->apdo.spr_programmable);
    }
    break;
    
    default:
        uprintf("unsupported PDO_TYPE: %d\r\n",pdo->fixed.type);
        break;
    }
    return fixedVoltage;
}

static void parse_request(uint32_t value){
    pd_rdo_fixed_variable_t *rdo = (pd_rdo_fixed_variable_t*)&value;

    float curMax = rdo->current_extremum_10ma*0.01;
    uprintf("\tmax current: %.2f\r\n",curMax);
    float curOp = rdo->current_operate_10ma*0.01;
    uprintf("\t    current: %.2f\r\n",curOp);
    uprintf("\tepr_mode_capable: %d\r\n",rdo->epr_mode_capable);
    uprintf("\tunchunked_ext_msg_support: %d\r\n",rdo->unchunked_ext_msg_support);
    uprintf("\tno_usb_suspend: %d\r\n",rdo->no_usb_suspend);
    uprintf("\tusb_comm_capable: %d\r\n",rdo->usb_comm_capable);
    uprintf("\tcapability_mismatch: %d\r\n",rdo->capability_mismatch);
    uprintf("\tgive_back_flag: %d\r\n",rdo->give_back_flag);
    uprintf("\tobject_position: %d\r\n",rdo->object_position);
}

static int tcmini_handle_pd_packet(){
    int err = 0;
    pd_header_t hdr = {};
    uint32_t pkts[8] = {};

    cassure(!tcmini_pd_recv(&hdr, pkts, ARRAYOF(pkts)));

    if (gIsDeviceMode){
        switch (hdr.msg_type){
        case PD_DATA_SOURCE_CAP:
        {
            parse_hdr(hdr);
            cassure(gIsDeviceMode);
            cassure(hdr.n_data_obj >= 1);
            uint8_t minIdx = 0xFF;
            uint8_t minVoltage = 15; //limit
            for (int i=0; i<hdr.n_data_obj; i++){
    #ifdef TCMINI_PD_LOG
                {
                    char buf[0x100] = {};
                    snprintf(buf,sizeof(buf),"[tcmini] PD<SOURCE_CAP: 0x%08x\r\n",pkts[i]);
                    tud_cdc_n_write_str(2, buf);
                }
    #endif
                int curv = parse_src_cap(pkts[i]);
                if (curv > 0){
                    if (curv < minVoltage || curv <= 9){
                        minIdx = i;
                        minVoltage = curv;
                        gBorrowCap = pkts[i];
                    }
                }
            }
            if (minIdx != 0xFF){
    #ifdef TCMINI_PD_LOG
                {
                    char buf[0x100] = {};
                    snprintf(buf,sizeof(buf),"[tcmini] found suitable voltage %dV at idx %d\r\n",minVoltage,minIdx);
                    tud_cdc_n_write_str(2, buf);
                }
    #endif
                {
                    pd_header_t req = {
                        .msg_type = PD_DATA_REQUEST,
                        .power_role = PD_POWER_ROLE_SINK,
                        .data_role = PD_DATA_ROLE_UFP,
                        .msg_id = 0,
                        .n_data_obj = 1,
                        .specs_rev = PD_REV_20,
                        .extended = 0,
                    };
                    pd_rdo_fixed_variable_t pdo = {
                        .current_extremum_10ma = 100, //3A
                        .current_operate_10ma = 100, //3A
                        .reserved = 0,
                        .epr_mode_capable = 0,
                        .unchunked_ext_msg_support = 0,
                        .no_usb_suspend = 1,
                        .usb_comm_capable = 0,
                        .capability_mismatch = 0,
                        .give_back_flag = 0,
                        .object_position = minIdx+1,
                    };
                    parse_request(*(uint32_t*)&pdo);
                    cassure(!tcmini_pd_send(req, (uint32_t*)&pdo, false));
                }

            }
        }
        break;

        case PD_CTRL_ACCEPT:
#ifdef TCMINI_PD_LOG
            {
                char buf[0x100] = {};
                snprintf(buf,sizeof(buf),"[tcmini] PD<ACCEPT\r\n");
                tud_cdc_n_write_str(2, buf);
            }
#endif
            break;

        case PD_CTRL_PS_READY:
#ifdef TCMINI_PD_LOG
            {
                char buf[0x100] = {};
                snprintf(buf,sizeof(buf),"[tcmini] PD<PS_READY\r\n");
                tud_cdc_n_write_str(2, buf);
            }
#endif
            break;            

        default:
    #ifdef TCMINI_PD_LOG
            {
                char buf[0x100] = {};
                snprintf(buf,sizeof(buf),"[tcmini] PD<UNK %d\r\n",hdr.msg_type);
                tud_cdc_n_write_str(2, buf);
            }
    #endif
            break;
        }
    }else{
        switch (hdr.msg_type){

        case PD_DATA_REQUEST:
        {
            cassure(hdr.n_data_obj >= 1);    
    #ifdef TCMINI_PD_LOG
            {
                char buf[0x100] = {};
                snprintf(buf,sizeof(buf),"[tcmini] PD<REQUEST: 0x%08x\r\n",pkts[0]);
                tud_cdc_n_write_str(2, buf);
            }
            parse_request(pkts[0]);
    #endif
            {
                pd_header_t rsp = {
                    .msg_type = PD_CTRL_ACCEPT,
                    .power_role = PD_POWER_ROLE_SOURCE,
                    .data_role = PD_DATA_ROLE_DFP,
                    .msg_id = 0,
                    .n_data_obj = 0,
                    .specs_rev = PD_REV_20,
                    .extended = 0,
                };
                cassure(!tcmini_pd_send(rsp, NULL, false));
            }
    #ifdef TCMINI_PD_LOG
            {
                char buf[0x100] = {};
                snprintf(buf,sizeof(buf),"[tcmini] PD>ACCEPT\r\n");
                tud_cdc_n_write_str(2, buf);
            }
    #endif
            {
                pd_header_t rsp = {
                    .msg_type = PD_CTRL_PS_READY,
                    .power_role = PD_POWER_ROLE_SOURCE,
                    .data_role = PD_DATA_ROLE_DFP,
                    .msg_id = 0,
                    .n_data_obj = 0,
                    .specs_rev = PD_REV_20,
                    .extended = 0,
                };
                cassure(!tcmini_pd_send(rsp, NULL, false));
            }
    #ifdef TCMINI_PD_LOG
            {
                char buf[0x100] = {};
                snprintf(buf,sizeof(buf),"[tcmini] PD>PS_RDY\r\n");
                tud_cdc_n_write_str(2, buf);
            }
    #endif
            gDev.isInited = true;
            tcmini_vmd_apple_send_map_uart(kTCMINI_PIN_MAPPING_SBU);
            if (gDevCB) gDevCB(true, gDev.cc2Polarity);
        }
            break;
        
        case PD_DATA_VENDOR_DEFINED:
            tcmini_handle_vmd_internal(pkts,hdr.n_data_obj);
            break;

        default:
    #ifdef TCMINI_PD_LOG
            {
                char buf[0x100] = {};
                snprintf(buf,sizeof(buf),"[tcmini] PD>UNK %d\r\n",hdr.msg_type);
                tud_cdc_n_write_str(2, buf);
            }
    #endif
            break;
        }
    }

error:
    return err;
}

static int tcmini_handle_vmd_internal(uint32_t *data, uint8_t cnt){
#ifdef TCMINI_VDM_LOG
    {
        char buf[0x100] = {};
        int didwrite = 0;
        didwrite = snprintf(buf,sizeof(buf),"[tcmini] received VMD:");
        for (size_t i = 0; i < cnt; i++){
            didwrite += snprintf(&buf[didwrite],sizeof(buf)-didwrite," 0x%08x",data[i]);
        }
        didwrite += snprintf(&buf[didwrite],sizeof(buf)-didwrite,"\r\n");
        tud_cdc_n_write_str(2, buf);
    }
#endif
    if (gVDMCB){
        gVDMCB(data,cnt);
    }
    return 0;
}

static int tcmini_pd_send_source_cap(){
    int err = 0;
    pd_header_t hdr = {
        .msg_type = PD_DATA_SOURCE_CAP,
        .power_role = PD_POWER_ROLE_SOURCE,
        .data_role = PD_DATA_ROLE_DFP,
        .msg_id = 0,
        .n_data_obj = 1,
        .specs_rev = PD_REV_20,
        .extended = 0,
    };
    // uint32_t cap = 1UL << 31; /* Variable non-battery PS, 0V, 0mA */
    uint32_t cap = build_fixed_pdo();

    if (gBorrowCap) cap = gBorrowCap;
    cassure(!tcmini_pd_send(hdr, &cap, false));
error:
    return err;
}

static void tcmini_enable_irq(){
    if (!gpio_get(TCMINI_PIN_IRQ)){
        gIRQIsPending = true;
    }else{
        gpio_set_irq_enabled_with_callback(TCMINI_PIN_IRQ, GPIO_IRQ_EDGE_FALL, true, tcmini_irq);
    }
}

static void tcmini_disable_irq(){
    gpio_set_irq_enabled_with_callback(TCMINI_PIN_IRQ, GPIO_IRQ_EDGE_FALL, false, tcmini_irq);
}

static int tcmini_readbyte(uint8_t addr, uint8_t *data){
    int err = 0;
    cassure(i2c_write_blocking(i2c1, FUSB_DEVICE, &addr, sizeof(addr), true) == 1);
    cassure(i2c_read_blocking(i2c1, FUSB_DEVICE, data, sizeof(*data), false) == 1);
error:
  return err;
}

static int tcmini_writebyte(uint8_t addr, uint8_t data){
  uint8_t d[2]={
    addr,
    data
  };
  return i2c_write_blocking(i2c1, FUSB_DEVICE, d, sizeof(d), false) != 2;
}

static int tcmini_write(uint8_t *data, uint32_t size){
#ifdef TCMINI_DBG
    {
        char buf[0x100] = {};
        int didwrite = 0;
        didwrite = snprintf(buf,sizeof(buf),"[tcmini] tcmini_write(");
        for (size_t i = 0; i < size; i++){
            didwrite += snprintf(&buf[didwrite],sizeof(buf)-didwrite,"%02x ",data[i]);
        }
        didwrite += snprintf(&buf[didwrite],sizeof(buf)-didwrite,")\r\n");
        tud_cdc_n_write_str(2, buf);
    }
#endif
  return i2c_write_blocking(i2c1, FUSB_DEVICE, data, size, false) != size;
}

static int tcmini_cfg_clr_set(uint8_t reg, uint8_t clr, uint8_t set){
    int err = 0;
    uint8_t val = 0;
    cassure(!tcmini_readbyte(reg, &val));
    val &= ~clr;
    val |= set;
#ifdef TCMINI_DBG
    {
        char buf[0x100] = {};
        snprintf(buf,sizeof(buf),"[tcmini] reg=0x%02x val=0x%02x\r\n",reg,val);
        tud_cdc_n_write_str(2, buf);
    }
#endif
    cassure(!tcmini_writebyte(reg, val)); 
error:
    return err;
}

static int tcmini_update_connected_status(){
    int err = 0;
    uint8_t val = 0;
    bool nowIsConnected = false;
    cassure(!tcmini_readbyte(FUSB_REG_STATUS0, &val));
#ifdef TCMINI_DBG
    {
        char buf[0x100] = {};
        snprintf(buf,sizeof(buf),"[tcmini] conn=0x%02x\r\n",val);
        tud_cdc_n_write_str(2, buf);
    }
#endif
    nowIsConnected = !(val & FUSB_STATUS0_VAL_COMP);
    if (gIsDeviceMode) nowIsConnected ^=1;
    if (nowIsConnected == gDev.isConnected) return 0;
    gDev.isConnected = nowIsConnected;
#ifdef TCMINI_DBG
    {
        char buf[0x100] = {};
        snprintf(buf,sizeof(buf),"[tcmini] connected=%d\r\n",nowIsConnected);
        tud_cdc_n_write_str(2, buf);
    }
#endif
    if (gDev.isConnected){
        tcmini_initCDevice(&gDev);
    }else{
        tcmini_deinitCDevice(&gDev);
    }
error:
    return err;
}

static void tcmini_irq(uint gpio, uint32_t event_mask){
    if (gpio != TCMINI_PIN_IRQ) return;
    gIRQIsPending = true;
    tcmini_disable_irq();
}

static int tcmini_deinitCDevice(struct CDevice *dev){
    int err = 0;
    if (gDevCB) gDevCB(false, false);
    memset(dev, 0, sizeof(*dev));
    cassure(!tcmini_cfg_clr_set(FUSB_REG_SWITCHES0, 
        0,
                    FUSB_SWITCHES0_VAL_PU_EN1
                    | FUSB_SWITCHES0_VAL_PU_EN2
                    | FUSB_SWITCHES0_VAL_MEAS_CC1
                    | FUSB_SWITCHES0_VAL_MEAS_CC2
    ));

    cassure(!tcmini_cfg_clr_set(FUSB_REG_SWITCHES1, 
        FUSB_SWITCHES1_VAL_TXCC1
        | FUSB_SWITCHES1_VAL_TXCC2,
                0
    ));

    cassure(!tcmini_cfg_clr_set(FUSB_REG_MASK, 
        FUSB_MASK_VAL_M_BC_LVL,
                0
    )); 

    cassure(!tcmini_cfg_clr_set(FUSB_REG_SWITCHES1, 
        FUSB_SWITCHES1_VAL_AUTO_CRC
        | FUSB_SWITCHES1_VAL_POWERROLE
        | FUSB_SWITCHES1_VAL_DATAROLE,
                0
    )); 
error:
    return err;
}

static int tcmini_initCDevice(struct CDevice *dev){
    int err = 0;
    uint8_t val = 0;
    static uint8_t isCC1 = 0;
    static uint8_t isCC2 = 0;

    if (gIsCCManualMode){
        isCC1 = !gManualModeIscc2Polarity;
        isCC2 = gManualModeIscc2Polarity;
    }else{
        if (gIsDeviceMode){
            cassure(!tcmini_cfg_clr_set(FUSB_REG_SWITCHES0, 
                FUSB_SWITCHES0_VAL_MEAS_CC1
                | FUSB_SWITCHES0_VAL_MEAS_CC2
                | FUSB_SWITCHES0_VAL_PDWN1
                | FUSB_SWITCHES0_VAL_PDWN2,
                                FUSB_SWITCHES0_VAL_MEAS_CC1
                                | FUSB_SWITCHES0_VAL_PDWN1
            ));
            sleep_us(250);
            cassure(!tcmini_readbyte(FUSB_REG_STATUS0, &val));
            isCC1 = !!(val & FUSB_STATUS0_VAL_COMP);
            cassure(!tcmini_cfg_clr_set(FUSB_REG_SWITCHES0, 
                FUSB_SWITCHES0_VAL_MEAS_CC1
                | FUSB_SWITCHES0_VAL_MEAS_CC2
                | FUSB_SWITCHES0_VAL_PDWN1
                | FUSB_SWITCHES0_VAL_PDWN2,
                                FUSB_SWITCHES0_VAL_MEAS_CC2
                                | FUSB_SWITCHES0_VAL_PDWN2
            ));
            sleep_us(250);
            cassure(!tcmini_readbyte(FUSB_REG_STATUS0, &val));
            isCC2 = !!(val & FUSB_STATUS0_VAL_COMP);
        }else{
            cassure(!tcmini_cfg_clr_set(FUSB_REG_SWITCHES0, 
                FUSB_SWITCHES0_VAL_MEAS_CC1
                | FUSB_SWITCHES0_VAL_MEAS_CC2
                | FUSB_SWITCHES0_VAL_PU_EN1
                | FUSB_SWITCHES0_VAL_PU_EN2,
                                FUSB_SWITCHES0_VAL_MEAS_CC1
                                | FUSB_SWITCHES0_VAL_PU_EN1
            ));
            sleep_us(250);
            cassure(!tcmini_readbyte(FUSB_REG_STATUS0, &val));

            isCC1 = !(val & FUSB_STATUS0_VAL_COMP);
            cassure(!tcmini_cfg_clr_set(FUSB_REG_SWITCHES0, 
                FUSB_SWITCHES0_VAL_MEAS_CC1
                | FUSB_SWITCHES0_VAL_MEAS_CC2
                | FUSB_SWITCHES0_VAL_PU_EN1
                | FUSB_SWITCHES0_VAL_PU_EN2,
                                FUSB_SWITCHES0_VAL_MEAS_CC2
                                | FUSB_SWITCHES0_VAL_PU_EN2
            ));
            sleep_us(250);
            cassure(!tcmini_readbyte(FUSB_REG_STATUS0, &val));
            isCC2 = !(val & FUSB_STATUS0_VAL_COMP);
        }
    }

#ifdef TCMINI_DBG
    {
        char buf[0x100] = {};
        snprintf(buf,sizeof(buf),"[tcmini] isCC1:0x%x isCC2:0x%x\r\n",isCC1,isCC2);
        tud_cdc_n_write_str(2, buf);
    }
#endif

    if (gIsDeviceMode){
        cassure(!tcmini_cfg_clr_set(FUSB_REG_SWITCHES0, 
            0,
                            FUSB_SWITCHES0_VAL_PDWN1
                            | FUSB_SWITCHES0_VAL_PDWN2
        ));        
    }else{
        if (!gIsCCManualMode){
            cassure(!tcmini_cfg_clr_set(FUSB_REG_SWITCHES0, 
                0,
                                FUSB_SWITCHES0_VAL_PU_EN1
                                | FUSB_SWITCHES0_VAL_PU_EN2
            ));
        }else{
            if (gManualModeIscc2Polarity){
                cassure(!tcmini_cfg_clr_set(FUSB_REG_SWITCHES0, 
                    FUSB_SWITCHES0_VAL_PDWN2
                    | FUSB_SWITCHES0_VAL_PU_EN1
                    | FUSB_SWITCHES0_VAL_PU_EN2,
                                    FUSB_SWITCHES0_VAL_PU_EN2
                ));
            }else{
                cassure(!tcmini_cfg_clr_set(FUSB_REG_SWITCHES0, 
                    FUSB_SWITCHES0_VAL_PDWN1
                    | FUSB_SWITCHES0_VAL_PU_EN1
                    | FUSB_SWITCHES0_VAL_PU_EN2,
                                    FUSB_SWITCHES0_VAL_PU_EN1
                ));
            }
        }
    }

    if (isCC1 && !isCC2){
        gDev.cc2Polarity = false;
    }else if (!isCC1 && isCC2){
        gDev.cc2Polarity = true;
    }else{
        /*
            thunderbolt cable have both connected?
        */
        gDev.cc2Polarity = false;
    }

    if (gDev.cc2Polarity){
        cassure(!tcmini_cfg_clr_set(FUSB_REG_SWITCHES0, 
            FUSB_SWITCHES0_VAL_MEAS_CC1
            | FUSB_SWITCHES0_VAL_MEAS_CC2,
                        FUSB_SWITCHES0_VAL_MEAS_CC2
        ));
        cassure(!tcmini_cfg_clr_set(FUSB_REG_SWITCHES1, 
            FUSB_SWITCHES1_VAL_TXCC1
            | FUSB_SWITCHES1_VAL_TXCC2,
                    FUSB_SWITCHES1_VAL_TXCC2
                    | FUSB_SWITCHES1_VAL_AUTO_CRC
        ));
    }else{
        cassure(!tcmini_cfg_clr_set(FUSB_REG_SWITCHES0, 
            FUSB_SWITCHES0_VAL_MEAS_CC1
            | FUSB_SWITCHES0_VAL_MEAS_CC2,
                        FUSB_SWITCHES0_VAL_MEAS_CC1
        ));
        cassure(!tcmini_cfg_clr_set(FUSB_REG_SWITCHES1, 
            FUSB_SWITCHES1_VAL_TXCC1
            | FUSB_SWITCHES1_VAL_TXCC2,
                    FUSB_SWITCHES1_VAL_TXCC1
                    | FUSB_SWITCHES1_VAL_AUTO_CRC
        ));
    }
    


    cassure(!tcmini_cfg_clr_set(FUSB_REG_MASK, 
        0,
                FUSB_MASK_VAL_M_BC_LVL
    ));
    cassure(!tcmini_cfg_clr_set(FUSB_REG_CONTROL1, 
        0,
                FUSB_CONTROL1_VAL_RX_FLUSH
    ));

    if (gIsDeviceMode){
        cassure(!tcmini_cfg_clr_set(FUSB_REG_SWITCHES1, 
                FUSB_SWITCHES1_VAL_POWERROLE
                | FUSB_SWITCHES1_VAL_DATAROLE,
                        0
        ));
    }else{
        cassure(!tcmini_cfg_clr_set(FUSB_REG_SWITCHES1, 
                0,
                        FUSB_SWITCHES1_VAL_POWERROLE
                        | FUSB_SWITCHES1_VAL_DATAROLE
        ));
    }


    cassure(!tcmini_writebyte(FUSB_REG_RESET, FUSB_RESET_VAL_PD_RESET));
    if (!gIsDeviceMode && gDev.isConnected){        
        cassure(!tcmini_pd_send_source_cap());
    }

error:
    if (err){
        cassure(!tcmini_cfg_clr_set(FUSB_REG_SWITCHES0, 
            0,
                        FUSB_SWITCHES0_VAL_MEAS_CC1
                        | FUSB_SWITCHES0_VAL_MEAS_CC2
        ));
    }
    return err;
}

static int tcmini_populate_defaults(){
    int err = 0;
    uint8_t val = 0;
        
    cassure(!tcmini_writebyte(FUSB_REG_RESET, FUSB_RESET_VAL_SW_RES));

    cassure(!tcmini_readbyte(FUSB_REG_DEVICE_ID, &val)); 
    cassure(val & 0x80);

    if (!gIsDeviceMode){
        //config
        {
            //setup device detection based on CC lines
            cassure(!tcmini_cfg_clr_set(FUSB_REG_MEASURE, 
                FUSB_MEASURE_VAL_MEAS_VBUS 
                | FUSB_MEASURE_MASK_MDAC,
                        FUSB_MEASURE_VAL_MDAC(0x20)
            ));
            if (!gIsCCManualMode){
                cassure(!tcmini_cfg_clr_set(FUSB_REG_SWITCHES0, 
                    FUSB_SWITCHES0_VAL_VCONN_CC1
                    | FUSB_SWITCHES0_VAL_VCONN_CC2
                    | FUSB_SWITCHES0_VAL_PDWN1
                    | FUSB_SWITCHES0_VAL_PDWN2
                    | FUSB_SWITCHES0_VAL_PU_EN1
                    | FUSB_SWITCHES0_VAL_PU_EN2,
                            FUSB_SWITCHES0_VAL_PU_EN1
                            | FUSB_SWITCHES0_VAL_PU_EN2
                            | FUSB_SWITCHES0_VAL_MEAS_CC1
                            | FUSB_SWITCHES0_VAL_MEAS_CC2
                ));                
            }else{
                if (gManualModeIscc2Polarity){
                    cassure(!tcmini_cfg_clr_set(FUSB_REG_SWITCHES0, 
                        FUSB_SWITCHES0_VAL_VCONN_CC1
                        | FUSB_SWITCHES0_VAL_VCONN_CC2
                        | FUSB_SWITCHES0_VAL_PU_EN1
                        | FUSB_SWITCHES0_VAL_PU_EN2,
                                FUSB_SWITCHES0_VAL_PU_EN2
                                | FUSB_SWITCHES0_VAL_MEAS_CC2
                    ));   
                }else{
                    cassure(!tcmini_cfg_clr_set(FUSB_REG_SWITCHES0, 
                        FUSB_SWITCHES0_VAL_VCONN_CC1
                        | FUSB_SWITCHES0_VAL_VCONN_CC2
                        | FUSB_SWITCHES0_VAL_PU_EN1
                        | FUSB_SWITCHES0_VAL_PU_EN2,
                                FUSB_SWITCHES0_VAL_PU_EN1
                                | FUSB_SWITCHES0_VAL_MEAS_CC1
                    ));    
                }
            }
        }

        cassure(!tcmini_cfg_clr_set(FUSB_REG_CONTROL3, 
            FUSB_CONTROL3_MASK_RETRIES,
                    FUSB_CONTROL3_VAL_AUTO_RETRY
                    | FUSB_CONTROL3_VAL_AUTO_HARDRESET
                    | FUSB_CONTROL3_VAL_RETRIES(FUSB_CONTROL3_MAX_RETRIES)
        ));    

        cassure(!tcmini_cfg_clr_set(FUSB_REG_MASK, 
            FUSB_MASK_VAL_M_VBUSOK
            | FUSB_MASK_VAL_M_BC_LVL
            | FUSB_MASK_VAL_M_COMP_CHNG
            | FUSB_MASK_VAL_M_COLLISION
            | FUSB_MASK_VAL_M_ALERT
            | FUSB_MASK_VAL_M_CRC_CHK,
                    0
        ));   

        cassure(!tcmini_cfg_clr_set(FUSB_REG_MASKA, 
            FUSB_MASKA_VAL_M_RETRYFAIL
            | FUSB_MASKA_VAL_M_HARDSENT
            | FUSB_MASKA_VAL_M_TXSENT
            | FUSB_MASKA_VAL_M_HARDRST,
                    0
        ));

        cassure(!tcmini_cfg_clr_set(FUSB_REG_MASKB, 
            FUSB_MASKB_VAL_M_GCRCSENT,
                    0
        ));

        cassure(!tcmini_cfg_clr_set(FUSB_REG_CONTROL0, 
            FUSB_CONTROL0_VAL_INT_MASK
            | FUSB_CONTROL0_HOST_CUR_MASK,
                    FUSB_CONTROL0_VAL_HOST_CUR(FUSB_CONTROL0_HOST_CUR_DEFAULT_USB)
        ));

        cassure(!tcmini_cfg_clr_set(FUSB_REG_CONTROL1, 
            0,
                    FUSB_CONTROL1_VAL_RX_FLUSH
                    | FUSB_CONTROL1_VAL_ENSOP1DB
                    | FUSB_CONTROL1_VAL_ENSOP2DB
                    | FUSB_CONTROL1_VAL_ENSOP1
                    | FUSB_CONTROL1_VAL_ENSOP2
        ));    
        
        cassure(!tcmini_cfg_clr_set(FUSB_REG_SWITCHES1, 
            FUSB_SWITCHES1_VAL_AUTO_CRC
            | FUSB_SWITCHES1_MASK_SPECREV,
                    FUSB_SWITCHES1_VAL_SPECREV(FUSB_SWITCHES1_SPECREV_1_0)
        ));  

        cassure(!tcmini_cfg_clr_set(FUSB_REG_POWER, 
            0,
                    FUSB_POWER_VAL_PWR_ALL
        ));  
    }else{
        cassure(!tcmini_cfg_clr_set(FUSB_REG_MEASURE, 
            FUSB_MEASURE_VAL_MEAS_VBUS 
            | FUSB_MEASURE_MASK_MDAC,
                    FUSB_MEASURE_VAL_MDAC(0x4)
        ));
        cassure(!tcmini_writebyte(FUSB_REG_SWITCHES0, 
            FUSB_SWITCHES0_VAL_PDWN1 
            | FUSB_SWITCHES0_VAL_PDWN2
            | FUSB_SWITCHES0_VAL_MEAS_CC1
            | FUSB_SWITCHES0_VAL_MEAS_CC2
            ));

        cassure(!tcmini_cfg_clr_set(FUSB_REG_MASK, 
            FUSB_MASK_VAL_M_VBUSOK
            | FUSB_MASK_VAL_M_BC_LVL
            | FUSB_MASK_VAL_M_COMP_CHNG
            | FUSB_MASK_VAL_M_COLLISION
            | FUSB_MASK_VAL_M_ALERT
            | FUSB_MASK_VAL_M_CRC_CHK,
                    0
        ));   

        cassure(!tcmini_cfg_clr_set(FUSB_REG_MASKA, 
            FUSB_MASKA_VAL_M_RETRYFAIL
            | FUSB_MASKA_VAL_M_HARDSENT
            | FUSB_MASKA_VAL_M_TXSENT
            | FUSB_MASKA_VAL_M_HARDRST,
                    0
        ));

        cassure(!tcmini_cfg_clr_set(FUSB_REG_MASKB, 
            FUSB_MASKB_VAL_M_GCRCSENT,
                    0
        ));

        cassure(!tcmini_cfg_clr_set(FUSB_REG_CONTROL0, 
            FUSB_CONTROL0_VAL_INT_MASK
            | FUSB_CONTROL0_HOST_CUR_MASK,
                    FUSB_CONTROL0_VAL_HOST_CUR(FUSB_CONTROL0_HOST_CUR_DISABLED)
        ));

        cassure(!tcmini_cfg_clr_set(FUSB_REG_POWER, 
            0,
                    FUSB_POWER_VAL_PWR_ALL
        ));  
    }

error:
    return -err;
}

#pragma mark public
int tcmini_init(){
    int err = 0;
    i2c_init(i2c1, 50e3);
    gpio_pull_up(TCMINI_PIN_SDA);
    gpio_pull_up(TCMINI_PIN_SCL);
    gpio_set_function(TCMINI_PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(TCMINI_PIN_SCL, GPIO_FUNC_I2C);

    cassure(!tcmini_populate_defaults());

    gpio_init(TCMINI_PIN_IRQ);
    gpio_pull_up(TCMINI_PIN_IRQ);
    gpio_set_dir(TCMINI_PIN_IRQ, GPIO_IN);

    tcmini_update_connected_status();
    tcmini_enable_irq();
    gVDMCB = NULL;

error:
    if (err){
        tcmini_deinit();
    }
    return -err;
}

void tcmini_deinit(){
    tcmini_disable_irq();
    i2c_deinit(i2c1);
    gpio_set_function(TCMINI_PIN_SDA, GPIO_FUNC_NULL);
    gpio_set_function(TCMINI_PIN_SCL, GPIO_FUNC_NULL);
    gVDMCB = NULL;
}

#pragma mark functions
void tcmini_test(){
    tcmini_update_connected_status();
}

void tcmini_stop(){
    tcmini_disable_irq();
    tcmini_writebyte(FUSB_REG_POWER, 0);
}

void tcmini_set_mode(bool isDevice){
    gIsDeviceMode = isDevice;
    memset(&gDev, 0, sizeof(gDev));
    if (!gIsCCManualMode) tcmini_populate_defaults();
}

void tcmini_ccManualMode(bool on, bool cc2Polarity){
    gIsCCManualMode = on;
    gManualModeIscc2Polarity = cc2Polarity;
    tcmini_populate_defaults();
    tcmini_initCDevice(&gDev);
    tcmini_update_connected_status();
}

void tcmini_set_cap(uint32_t cap){
    gBorrowCap = cap;
}

int tcmini_powerproxy(bool enable){
    int err = 0;
    gIsPowerProxy = enable;
    if (gIsDeviceMode){
        cassure(gDev.isConnected);
        gIsDeviceMode = false;
        memset(&gDev, 0, sizeof(gDev));
        tcmini_ccManualMode(true, !gDev.cc2Polarity);
    }
error:
    return err;
}

void tcmini_task(){
    tcmini_pd_write_perform();
    if (!gIRQIsPending) return;
    int err = 0;
    static union TCMinitIRQs irqs = {};
    bool needsHandlePDPacket = false;

    cassure(!tcmini_readbyte(FUSB_REG_INTERRUPT, &irqs.irq));
    cassure(!tcmini_readbyte(FUSB_REG_INTERRUPTA, &irqs.irqA));
    cassure(!tcmini_readbyte(FUSB_REG_INTERRUPTB, &irqs.irqB));

#ifdef TCMINI_DBG
    {
        char buf[0x100] = {};
        snprintf(buf,sizeof(buf),"[tcmini] task run irq=0x%x irqa=0x%x irqb=0x%x\r\n",irqs.irq,irqs.irqA,irqs.irqB);
        tud_cdc_n_write_str(2, buf);
    }
#endif

    if (irqs.irqA & FUSB_INTERRUPTA_VAL_I_HARDRST){
        irqs.irqA &= ~FUSB_INTERRUPTA_VAL_I_HARDRST;
        tcmini_populate_defaults();
        tcmini_update_connected_status();
        tcmini_enable_irq();
        return;
    }

    if (irqs.irqA & FUSB_INTERRUPTA_VAL_I_HARDSENT){
        irqs.irqA &= ~FUSB_INTERRUPTA_VAL_I_HARDSENT;
        //ignore
    }

    if (irqs.irq & FUSB_INTERRUPT_VAL_I_ALERT){
        irqs.irq &= ~FUSB_INTERRUPT_VAL_I_ALERT;
        uint8_t val = 0;
        tcmini_readbyte(FUSB_REG_STATUS1, &val);
        if (!(val & FUSB_STATUS1_VAL_RX_EMPTY)){
            needsHandlePDPacket = true;
#ifdef TCMINI_DBG
        {
            char buf[0x100] = {};
            snprintf(buf,sizeof(buf),"[tcmini] status1=0x%02x\r\n",val);
            tud_cdc_n_write_str(2, buf);
        }
#endif
        }
    }
    
    if (irqs.irq & (FUSB_INTERRUPT_VAL_I_BC_LVL | FUSB_INTERRUPT_VAL_I_COMP_CHNG | FUSB_INTERRUPT_VAL_I_ACTIVITY)){
        irqs.irq &= ~(FUSB_INTERRUPT_VAL_I_BC_LVL | FUSB_INTERRUPT_VAL_I_COMP_CHNG | FUSB_INTERRUPT_VAL_I_ACTIVITY);
        tcmini_update_connected_status();
    }

    if ((irqs.irqA & FUSB_INTERRUPTA_VAL_I_RETRYFAIL)){
        irqs.irqA &= ~FUSB_INTERRUPTA_VAL_I_RETRYFAIL;
        gDev.sndDo = (gDev.sndDo+SND_MAX_QUEUE-1) % SND_MAX_QUEUE;
    }

    if (irqs.irqA & FUSB_INTERRUPTA_VAL_I_TXSENT){
        irqs.irqA &= ~FUSB_INTERRUPTA_VAL_I_TXSENT;
        gDev.hasSentMessage = true;
        gDev.sndDo = gDev.sndDone = (gDev.sndDone+1) % SND_MAX_QUEUE;
    }

    if ((irqs.irqB & FUSB_INTERRUPTB_VAL_I_GCRCSENT) || (irqs.irq & FUSB_INTERRUPT_VAL_I_CRC_CHK)){
        irqs.irqB &= ~FUSB_INTERRUPTB_VAL_I_GCRCSENT;
        irqs.irq &= ~FUSB_INTERRUPT_VAL_I_CRC_CHK;
        needsHandlePDPacket = true;
    }

    if (needsHandlePDPacket){
        tcmini_handle_pd_packet();
    }

#ifdef TCMINI_DBG
    if (irqs.irqAll){
        char buf[0x100] = {};
        snprintf(buf,sizeof(buf),"[tcmini] ------- unhandled irq=0x%x irqa=0x%x irqb=0x%x\r\n",irqs.irq,irqs.irqA,irqs.irqB);
        tud_cdc_n_write_str(2, buf);
    }
#endif
error:
    if (!err){
        gIRQIsPending = false;
        tcmini_enable_irq();
    }
}

bool tcmini_is_device_connected(){
    return gDev.isConnected;
}

void tcmini_register_vdm_cb(t_tcmini_vdm_cb cb){
    gVDMCB = cb;
}

void tcmini_register_dev_cb(t_tcmini_dev_cb cb){
    gDevCB = cb;
}

int tcmini_vmd_send(const uint32_t *data, uint8_t cnt){
    int err = 0;
    pd_header_t req = {
        .msg_type = PD_DATA_VENDOR_DEFINED,
        .power_role = PD_POWER_ROLE_SOURCE,
        .data_role = PD_DATA_ROLE_DFP,
        .msg_id = 0,
        .n_data_obj = cnt,
        .specs_rev = PD_REV_20,
        .extended = 0,
    };
#ifdef TCMINI_VDM_LOG
    {
        char buf[0x100] = {};
        int didwrite = 0;
        didwrite = snprintf(buf,sizeof(buf),"[tcmini] sending VDM:");
        for (size_t i = 0; i < cnt; i++){
            didwrite += snprintf(&buf[didwrite],sizeof(buf)-didwrite," 0x%08x",data[i]);
        }
        didwrite += snprintf(&buf[didwrite],sizeof(buf)-didwrite,"\r\n");
        tud_cdc_n_write_str(2, buf);
    }
#endif
    cassure(!tcmini_pd_send(req, data, true));
error:
    return err;
}

int tcmini_vdm_apple_perform_action(bool exit, bool persist, bool exit_conflicting, enum TCMINI_PIN_MAPPING mapping, uint16_t action_id, uint32_t arguments[], uint8_t arguments_len){
    int err = 0;
    uint32_t vdm[16] = {
        0x5ac8012,
    };
    uint32_t action =
        (exit << 25) |
        (persist << 24) |
        (exit_conflicting << 23) |
        (((uint8_t)mapping) << 16) |
        action_id;
    cassure(arguments_len < 14);
    vdm[1] = action;
    for (int i = 0; i < arguments_len; i++){
        vdm[2 + i] = arguments[i];
    }
    return tcmini_vmd_send(vdm, 2+arguments_len);
error:
    return err;
}


int tcmini_vmd_apple_send_reboot(){
    uint32_t arg = 0x8000UL << 16;
    return tcmini_vdm_apple_perform_action(0,0,0,kTCMINI_PIN_MAPPING_NONE,0x105,&arg,1);
}

int tcmini_vmd_apple_send_dfu(){
    uint32_t arg = 0x8001UL << 16;
    return tcmini_vdm_apple_perform_action(0,0,0,kTCMINI_PIN_MAPPING_NONE,0x106,&arg,1);
}

int tcmini_vmd_apple_send_map_uart(enum TCMINI_PIN_MAPPING mapping){
    return tcmini_vdm_apple_perform_action(0,0,0,mapping,0x306,NULL,0);
}

int tcmini_pd_sendreq(int pos){
    int err = 0;
    pd_header_t req = {
        .msg_type = PD_DATA_REQUEST,
        .power_role = PD_POWER_ROLE_SINK,
        .data_role = PD_DATA_ROLE_UFP,
        .msg_id = 0,
        .n_data_obj = 1,
        .specs_rev = PD_REV_20,
        .extended = 0,
    };
    pd_rdo_fixed_variable_t pdo = {
        .current_extremum_10ma = 100, //3A
        .current_operate_10ma = 100, //3A
        .reserved = 0,
        .epr_mode_capable = 0,
        .unchunked_ext_msg_support = 0,
        .no_usb_suspend = 1,
        .usb_comm_capable = 0,
        .capability_mismatch = 0,
        .give_back_flag = 0,
        .object_position = pos,
    };
    parse_request(*(uint32_t*)&pdo);
    cassure(!tcmini_pd_send(req, (uint32_t*)&pdo, false));    
error:
    return err;
}