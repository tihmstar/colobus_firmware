#include "lightning.h"
#include "swd.h"
#include "tristar.h"
#include "crc.h"
#include "probe.h"
#include "tusb_config.h"

#include <pico.h>
#include <pico/multicore.h>
#include <hardware/uart.h>
#include <hardware/gpio.h>
#include <hardware/structs/ioqspi.h>
#include <hardware/structs/sio.h>
#include "bsp/board.h"

#include <tusb.h>

#include <macros.h>

#include <stdio.h>
#include <string.h>

#define ENABLE_BOOTSEL_BUTTON_CHECKING 

#define USEC_PER_SEC  1000000L
#define USEC_PER_MSEC    1000L

#define PIN_SDQ 3
#define PIN_BUTTON 5
#define PIN_LED 25


#define PIN_SWDBASE 2
#define PIN_SWDCLK  PIN_SWDBASE+0
#define PIN_SWDIO   PIN_SWDBASE+1

extern int main(void);

#define DCSD_UART uart0
#define DCSD_TX_PIN 0
#define DCSD_RX_PIN 1


#define ITF_SERIAL0 0
#define ITF_SERIAL1 1

static t_colobus_mode gActiveMode = kCOLOBUS_MODE_DEFAULT;
static bool gWantTristarReset = false;
static bool gWantTristarDFU = false;

static int gWantDCSDInit = 0;
static uint64_t gWantSWDInitTime = 0;

static bool gDCSDIsInited = false;
static bool gSWDIsInited = false;
static volatile bool gIsSWDTaskActive = false;

static int gDPIDR = 0;

static bool gSWDModeIsSpam = true;

void dcsd_init(void){
    if (!gDCSDIsInited){
        uart_init(DCSD_UART, 115200);
        gpio_set_function(DCSD_TX_PIN, GPIO_FUNC_UART);
        gpio_set_function(DCSD_RX_PIN, GPIO_FUNC_UART);
        gDCSDIsInited = true;
    }
}

void dcsd_deinit(void){
    if (gDCSDIsInited){
        gDCSDIsInited = false;
        uart_deinit(DCSD_UART);
    }
}

void lightning_callback(const void *buf, size_t bufSize){
    uint8_t *req8 = (uint8_t*)buf;
    uint32_t *req32 = (uint32_t*)buf;

    if (bufSize == 4 && req8[0] == TRISTAR_REQUEST_GET_CABLE_TYPE){
        gSWDIsInited = false;
        lightning_gpio_configure(PIN_SDQ);

        if (gWantTristarReset){
            lightning_write_nonblocking(colobus_cable_type_responses[kCOLOBUS_MODE_RESET], sizeof(colobus_cable_type_responses[kCOLOBUS_MODE_RESET]));
        }else if (gWantTristarDFU){
            lightning_write_nonblocking(colobus_cable_type_responses[kCOLOBUS_MODE_DFU], sizeof(colobus_cable_type_responses[kCOLOBUS_MODE_DFU]));
        }else{
            lightning_write_nonblocking(colobus_cable_type_responses[gActiveMode], sizeof(colobus_cable_type_responses[gActiveMode]));

            if (gActiveMode == kCOLOBUS_MODE_USB_UART ||
                gActiveMode == kCOLOBUS_MODE_USB_JTAG_UART){
                gWantDCSDInit = 1;
            }else{
                gWantDCSDInit = -1;
            }

            if (gActiveMode == kCOLOBUS_MODE_USB_JTAG_UART ||
                gActiveMode == kCOLOBUS_MODE_USB_JTAG_SPAM){
                gWantSWDInitTime = time_us_64() + USEC_PER_MSEC*1;
            }
        }

        if (gWantTristarReset){
            gWantTristarReset = false;
        } else if (gWantTristarDFU){
            gWantTristarDFU = false;
        }
    }
}

void colobus_set_active_mode(t_colobus_mode activeMode){
    gActiveMode = activeMode;
}

void colobus_init(void){
    for (size_t i = 0; i < kCOLOBUS_MODE_MAX; i++){
        colobus_cable_type_responses[i][CABLE_TYPE_RSP_MAX_SIZE] = crc(colobus_cable_type_responses[i],CABLE_TYPE_RSP_MAX_SIZE);
    }    
}


static bool get_my_bootsel_button(void){
    bool button_state = false;
#ifdef ENABLE_BOOTSEL_BUTTON_CHECKING
    if (!((uint32_t)main >= SRAM_BASE && (uint32_t)main < SRAM_END)){
        //to use this function you need to compile with 'set(PICO_COPY_TO_RAM 1)'
        return false;
    }

	const uint CS_PIN_INDEX = 1;

    static bool isFlashDisabled = false;
    if (!isFlashDisabled){
        hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl, GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB, IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    	for (volatile int i = 0; i < 1000; ++i) tight_loop_contents();
    }
    button_state = !(sio_hw->gpio_hi_in & (1u << CS_PIN_INDEX));
#endif
    return button_state;
}

void button_init(){
    gpio_init(PIN_BUTTON);
    gpio_set_dir(PIN_BUTTON, 0);
    gpio_pull_up(PIN_BUTTON);
}

bool get_button(void){
    if (get_my_bootsel_button()) return true;
    return !gpio_get(PIN_BUTTON);
}

typedef enum {
    kButtonPressTypeNone = 0,
    kButtonPressTypeShort,  // 0.1 - 1.5 sec
    kButtonPressTypeMid,    // 1.5 - 3 sec
    kButtonPressTypeLong,   // >= 3 sec

    kButtonPressLast,
} t_buttonPressType;

void init_led(){
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, true);
    gpio_put(PIN_LED, 0);
}

void led_blink_fast(int cnt){
    for (size_t i = 0; i < cnt; i++){
        gpio_put(PIN_LED, 1);
        sleep_ms(USEC_PER_MSEC*0.01);
        gpio_put(PIN_LED, 0);        
        sleep_ms(USEC_PER_MSEC*0.1);
    }
}

void led_blink_slow(int cnt){
    for (size_t i = 0; i < cnt; i++){
        gpio_put(PIN_LED, 1);
        sleep_ms(USEC_PER_MSEC*0.2);
        gpio_put(PIN_LED, 0);        
        sleep_ms(USEC_PER_MSEC*0.4);
    }
}

t_buttonPressType detectButtonPress(void){
    static uint64_t lastButtonSwitchTime = 0;
    static bool lastButtonState = false;
    static t_buttonPressType lastRet = kButtonPressTypeNone;

    t_buttonPressType ret = kButtonPressTypeNone;

    uint64_t curTime = time_us_64();
    bool curState = get_button();

    /*
        Perform some manual debouncing
    */
    uint64_t tdiff = curTime - lastButtonSwitchTime;
    if (tdiff > USEC_PER_MSEC*1){
        float secsPressed = (float)tdiff / USEC_PER_SEC;
        if (secsPressed < 1.5){
            ret = kButtonPressTypeShort;
        }else if (secsPressed < 3){
            ret = kButtonPressTypeMid;
        }else {
            ret = kButtonPressTypeLong;
        }

        if (lastButtonState){
            if (ret != lastRet){
                led_blink_fast(ret);
            }
            if (lastRet == kButtonPressLast-1){
                ret = kButtonPressTypeNone;
            }else{
                lastRet = ret;
            }
        }

        if (!lastButtonState || (curState && ret != kButtonPressLast-1)){
            ret = kButtonPressTypeNone;
        }
        if (curState != lastButtonState){
            lastButtonSwitchTime = curTime;
            lastButtonState = curState;
            lastRet = kButtonPressTypeNone;
        }        
    }

    return ret;
}

static bool gColobusWantsWake = false;
void colobus_wake_runloop(){
    if (gColobusWantsWake){
        while (gIsSWDTaskActive){
            tight_loop_contents();
        }
        gColobusWantsWake = false;
        gSWDIsInited = false;
        gpio_init(PIN_SDQ);
        gpio_set_dir(PIN_SDQ, GPIO_OUT);
        gpio_put(PIN_SDQ, 0);
        sleep_ms(10);
        gpio_set_dir(PIN_SDQ, GPIO_IN);
    }
}

void colobus_perform_wake(){
    gColobusWantsWake = true;
}

int task_spam(){
    int err = 0;

    uint8_t dstitf = ITF_SERIAL1;

    int ack = 0;
    bool hasdata = false;



    if (!gDPIDR){
        cassure(swd_reset());
        SWD_DP_clear_error();
        {
            uint32_t data = 0;
            ack = SWD_DP_read_IDCODE(&data);
            if (data != 0 && ack == SWD_RSP_OK){
                gDPIDR = data;
                cassure((ack = SWD_DP_write_CTRL(0x50000000)) == SWD_RSP_OK);
                cassure((ack = SWD_DP_write_SELECT(0x01000000)) == SWD_RSP_OK);
                cassure((ack = SWD_AP_write_CSW(0xA2000012)) == SWD_RSP_OK);
            }
        }
    }
    
    {
        uint32_t uart_ctrl_reg = 0;
        if (gDPIDR == 0x5ba02477){
            //s7002
            uart_ctrl_reg = 0xc6e00004;
        }else if (gDPIDR == 0x4ba02477){
            //s8002
            uart_ctrl_reg = 0xC83B401C;
        }

        if (uart_ctrl_reg){
            {
                int cnt = 1;

                while (cnt > 0 && tud_cdc_n_write_available(dstitf) > 1){
                    uint32_t data = 0;
                    
                    cassure((ack = SWD_readmem(uart_ctrl_reg + 0x00, &data)) == SWD_RSP_OK);
                    cnt = data & 0x7f;
                    if (!cnt--) break;
                    hasdata = true;
                    tud_cdc_n_write_char(dstitf, data >> 8);
                }
            }
        }
    }
error:
    if (ack == SWD_RSP_LINE_ERROR) gDPIDR = 0;
    if (hasdata) tud_cdc_n_write_flush(dstitf);
    return err;
}

void task_swd(){
    static uint32_t sSpamFails = 0;
    gIsSWDTaskActive = true;

    if (gSWDModeIsSpam){
        if (!task_spam()){
            sSpamFails = 0;
        }else{
            if (sSpamFails++ >= 10){
                gWantSWDInitTime = time_us_64() + 10*USEC_PER_MSEC;
                /*
                    One tristar poll cycle is ~7.6ms.
                */
                gSWDIsInited = false;
                sSpamFails = 0;
            }            
        }
    }else{
        probe_task();
        sSpamFails = 0;
    }
    gIsSWDTaskActive = false;
}

void usb_loop() {
    tusb_init();

    while (1){
        tud_task();

        if (gDCSDIsInited){
            while (uart_is_readable(DCSD_UART) && tud_cdc_n_write_available(ITF_SERIAL0)) {
                tud_cdc_n_write_char(ITF_SERIAL0, uart_getc(DCSD_UART));
            }
            
            if (tud_cdc_n_available(ITF_SERIAL0)) {
                uart_putc_raw(DCSD_UART, tud_cdc_n_read_char(ITF_SERIAL0));
            }
            tud_cdc_n_write_flush(ITF_SERIAL0);
        }
    }
}

int main(){
    board_init(); //THIS IS MANDATORY, OTHERWISE SWD DOESN'T WORK!!!


    multicore_launch_core1(usb_loop);


    swd_init(PIN_SWDBASE);
    button_init();
    colobus_init();
    colobus_set_active_mode(kCOLOBUS_MODE_DEFAULT);
    lightning_rx_callback_register(lightning_callback);
    lightning_init(PIN_SDQ);
    colobus_perform_wake();

    init_led();

    while (1){
        t_buttonPressType bpress = detectButtonPress();
        if (bpress){
            led_blink_slow(bpress);
            if (bpress == kButtonPressTypeShort){
                gSWDModeIsSpam = !gSWDModeIsSpam;
                
            }else if (bpress == kButtonPressTypeMid){
                gWantTristarDFU = true;
                gWantTristarReset = true;
                colobus_perform_wake();
            }else if (bpress == kButtonPressTypeLong){
                gWantTristarReset = true;
                colobus_perform_wake();
            }
        
            if (!gSWDModeIsSpam){
                gpio_put(PIN_LED, 1);
            }
        }
        colobus_wake_runloop();
        if (gWantDCSDInit){
            if (gWantDCSDInit > 0){
                dcsd_init();
            }else{
                dcsd_deinit();
            }
            gWantDCSDInit = 0;
        }

        if (gWantSWDInitTime && (gWantSWDInitTime < time_us_64())){
            swd_gpio_configure(PIN_SDQ);
            swd_reset();
            gWantSWDInitTime = 0;
            gDPIDR = 0;
            gSWDIsInited = true;
        }

        if (gSWDIsInited){
            task_swd();
        }
    }
}   
