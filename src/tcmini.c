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
        cassure(!(val & FUSB_STATUS1_VAL_RX_EMPTY));
        val = FUSB_REG_FIFOS;
        cassure(i2c_write_blocking(i2c1, FUSB_DEVICE, &val, sizeof(val), true) == 1);
        cassure(i2c_read_blocking(i2c1, FUSB_DEVICE, (uint8_t*)&rcv, sizeof(rcv), true) == sizeof(rcv));
        cassure(rcv.hdr.n_data_obj <= dataCnt);
        dataLen = rcv.hdr.n_data_obj*sizeof(*data);
        if (dataLen) cassure(i2c_read_blocking(i2c1, FUSB_DEVICE, (uint8_t*)data, dataLen, true) == dataLen);
        cassure(i2c_read_blocking(i2c1, FUSB_DEVICE, (uint8_t*)&crc, sizeof(crc), false) == sizeof(crc));
    }while (rcv.hdr.msg_type == PD_CTRL_GOOD_CRC);
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
    return err;
}

#define uprintf(a...)do { char buf[0x100] = {}; snprintf(buf,sizeof(buf),a); tud_cdc_n_write_str(2, buf);} while(0)

static void parse_request(uint32_t value){
    // Check reserved bits
    if (value & (1 << 31) || (value & 0x00F00000))
    {
        uprintf("Error: Reserved bits are not set to zero.\r\n");
        return;
    }

    // Extract object position
    uint32_t objectPosition = (value >> 28) & 0x07;
    if (objectPosition == 0)
    {
        uprintf("Error: Object position is set to a reserved value.\r\n");
        return;
    }

    // Extract flags
    uint32_t giveBackFlag = (value >> 27) & 0x01;
    uint32_t capabilityMismatch = (value >> 26) & 0x01;
    uint32_t usbCommCapable = (value >> 25) & 0x01;
    uint32_t noUSBSuspend = (value >> 24) & 0x01;

    // Extract currents
    uint32_t operatingCurrent = (value >> 10) & 0x03FF; // 10 bits for operating current
    uint32_t maxOperatingCurrent = value & 0x03FF;      // 10 bits for max operating current

    // Print details
    uprintf("\tObject Position: %u\r\n", objectPosition);
    uprintf("\tGiveBack Flag: %u\r\n", giveBackFlag);
    uprintf("\tCapability Mismatch: %u\r\n", capabilityMismatch);
    uprintf("\tUSB Communications Capable: %u\r\n", usbCommCapable);
    uprintf("\tNo USB Suspend: %u\r\n", noUSBSuspend);
    uprintf("\tOperating Current: %u mA\r\n", operatingCurrent * 10);
    uprintf("\tMaximum Operating Current: %u mA\r\n", maxOperatingCurrent * 10);
}

static int tcmini_handle_pd_packet(){
    int err = 0;
    pd_header_t hdr = {};
    uint32_t pkts[8] = {};

    cassure(!tcmini_pd_recv(&hdr, pkts, ARRAYOF(pkts)));
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
    }
        break;
    
    case PD_DATA_VENDOR_DEFINED:
        tcmini_handle_vmd_internal(pkts,hdr.n_data_obj);
        break;

    default:
        break;
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

#ifdef TCMINI_DBG
    {
        char buf[0x100] = {};
        snprintf(buf,sizeof(buf),"[tcmini] isCC1:0x%x isCC2:0x%x\r\n",isCC1,isCC2);
        tud_cdc_n_write_str(2, buf);
    }
#endif

    cassure(!tcmini_cfg_clr_set(FUSB_REG_SWITCHES0, 
        0,
                        FUSB_SWITCHES0_VAL_PU_EN1
                        | FUSB_SWITCHES0_VAL_PU_EN2
    ));

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
            | FUSB_SWITCHES1_VAL_TXCC2
            | FUSB_SWITCHES0_VAL_VCONN_CC1
            | FUSB_SWITCHES0_VAL_VCONN_CC2,
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
            | FUSB_SWITCHES1_VAL_TXCC2
            | FUSB_SWITCHES0_VAL_VCONN_CC1
            | FUSB_SWITCHES0_VAL_VCONN_CC2,
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

    cassure(!tcmini_cfg_clr_set(FUSB_REG_SWITCHES1, 
            0,
                    FUSB_SWITCHES1_VAL_POWERROLE
                    | FUSB_SWITCHES1_VAL_DATAROLE
    ));


    cassure(!tcmini_writebyte(FUSB_REG_RESET, FUSB_RESET_VAL_PD_RESET));
    cassure(!tcmini_pd_send_source_cap());

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

    //config
    {
        //setup device detection based on CC lines
        cassure(!tcmini_cfg_clr_set(FUSB_REG_MEASURE, 
            FUSB_MEASURE_VAL_MEAS_VBUS 
            | FUSB_MEASURE_MASK_MDAC,
                    FUSB_MEASURE_VAL_MDAC(0x20)
        ));
        cassure(!tcmini_cfg_clr_set(FUSB_REG_SWITCHES0, 
            FUSB_SWITCHES0_VAL_VCONN_CC1
            | FUSB_SWITCHES0_VAL_VCONN_CC2
            | FUSB_SWITCHES0_VAL_PDWN1
            | FUSB_SWITCHES0_VAL_PDWN2,
                    FUSB_SWITCHES0_VAL_PU_EN1
                    | FUSB_SWITCHES0_VAL_PU_EN2
                    | FUSB_SWITCHES0_VAL_MEAS_CC1
                    | FUSB_SWITCHES0_VAL_MEAS_CC2
        ));
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
    i2c_deinit(i2c1);
    gpio_set_function(TCMINI_PIN_SDA, GPIO_FUNC_NULL);
    gpio_set_function(TCMINI_PIN_SCL, GPIO_FUNC_NULL);
    gVDMCB = NULL;
}

#pragma mark functions
void tcmini_task(){
    tcmini_pd_write_perform();
    if (!gIRQIsPending) return;
    int err = 0;
    static union TCMinitIRQs irqs = {};

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

    if (irqs.irq & FUSB_INTERRUPT_VAL_I_CRC_CHK){
        irqs.irq &= ~FUSB_INTERRUPT_VAL_I_CRC_CHK;
        //ignore
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

    if (irqs.irqB & FUSB_INTERRUPTB_VAL_I_GCRCSENT){
        irqs.irqB &= ~FUSB_INTERRUPTB_VAL_I_GCRCSENT;
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