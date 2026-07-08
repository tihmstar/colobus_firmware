#include "lightning.h"
#include "swd.h"
#include "tristar.h"
#include "crc.h"
#include "probe.h"
#include "tusb_config.h"
#include "puart.h"
#include "usbhub.h"
#include "usbmux.h"

#include <pico.h>
#include <pico/multicore.h>
#include <hardware/uart.h>
#include <hardware/gpio.h>
#include <hardware/structs/ioqspi.h>
#include <hardware/structs/sio.h>
#include <hardware/watchdog.h>
#include <pico/bootrom.h>
#include "bsp/board.h"

#include <tusb.h>

#include <macros.h>

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#define SPAM_ATTEMPTS_TIMEOUT 2000

#define USEC_PER_SEC  1000000L
#define USEC_PER_MSEC    1000L


#define PIN_BUTTON1 5
#define PIN_BUTTON2 6
#define PIN_BUTTON3 7

#define PIN_LED1  28
#define PIN_LED2  27
#define PIN_LED3  26


#define PIN_SDQ_INVERTED(isInverted)        (isInverted ? 3 : 2)

#define PIN_SWDIO_INVERTED(isInverted)      (isInverted ? 3 : 2)
#define PIN_SWDCLK_INVERTED(isInverted)     (isInverted ? 2 : 3)

#define PIN_PUART_TX_INVERTED(isInverted)   (isInverted ? 3 : 2)
#define PIN_PUART_RX_INVERTED(isInverted)   (isInverted ? 2 : 3)



#define ARRAYOF(a) (sizeof(a)/sizeof(*a))

extern int main(void);

#define DCSD_UART uart0
#define DCSD_TX_PIN 0
#define DCSD_RX_PIN 1


#define ITF_SERIAL0 0
#define ITF_SERIAL1 1
#define ITF_CONTROL 2

#define ITF_UART_PRIMARY    ((gActiveMode == kCOLOBUS_MODE_USB_UART2_UART) ? ITF_SERIAL1 : ITF_SERIAL0)
#define ITF_UART_SECONDARY  ((gActiveMode == kCOLOBUS_MODE_USB_UART2_UART) ? ITF_SERIAL0 : ITF_SERIAL1)

static t_colobus_mode gActiveMode = kCOLOBUS_MODE_DEFAULT;
static bool gWantTristarReset = false;
static bool gWantTristarDFU = false;

static uint64_t gWantSWDInitTime = 0;
static uint64_t gLastLightningCheckin = 0;
static uint32_t gSpamFails = 0;

static bool gDCSDIsInited = false;
static bool gPUARTIsInited = false;
static bool gSWDIsInited = false;
static volatile bool gIsSWDTaskActive = false;

static int gWantPUARTInit = 0;
static int gWantDCSDInit = 0;
static bool gSWDWantDeinit = false;

static int gDPIDR = 0;

static bool gSWDModeIsSpam = true;
static bool gCableIsInverted = false;

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
        lightning_gpio_configure(PIN_SDQ_INVERTED(gCableIsInverted));
        gSWDWantDeinit = false;

        if (gWantTristarReset){
            lightning_write_blocking(colobus_cable_type_responses[kCOLOBUS_MODE_RESET], sizeof(colobus_cable_type_responses[kCOLOBUS_MODE_RESET]));
        }else if (gWantTristarDFU){
            lightning_write_blocking(colobus_cable_type_responses[kCOLOBUS_MODE_DFU], sizeof(colobus_cable_type_responses[kCOLOBUS_MODE_DFU]));
        }else{
            lightning_write_blocking(colobus_cable_type_responses[gActiveMode], sizeof(colobus_cable_type_responses[gActiveMode]));

            gLastLightningCheckin = time_us_64();

            if (gActiveMode == kCOLOBUS_MODE_USB_UART ||
                gActiveMode == kCOLOBUS_MODE_USB_UART2_UART ||
                gActiveMode == kCOLOBUS_MODE_USB_JTAG_UART){
                gWantDCSDInit = 1;
                if (gActiveMode == kCOLOBUS_MODE_USB_UART2_UART){
                    gWantPUARTInit = 1;
                }else{
                    gWantPUARTInit = -1;
                }
            }else{
                gWantDCSDInit = -1;
                gWantPUARTInit = -1;
            }

            if (gActiveMode == kCOLOBUS_MODE_USB_JTAG_UART ||
                gActiveMode == kCOLOBUS_MODE_USB_JTAG_SPAM){
                gWantSWDInitTime = time_us_64() + USEC_PER_MSEC*1;
                gSpamFails = 0;
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

static inline void init_button_pin(uint32_t pin_button){
  gpio_init(pin_button);
  gpio_set_dir(pin_button, GPIO_IN);
  gpio_pull_up(pin_button);
}

void button_init(){
  init_button_pin(PIN_BUTTON1);
  init_button_pin(PIN_BUTTON2);
  init_button_pin(PIN_BUTTON3);
}

bool button_get_edge(int btn){
    static bool lastState[32] = {[0 ... 31] = true};
    bool ret = false;
    if (btn < 0 || btn >= ARRAYOF(lastState)) return false;
    bool curState = gpio_get(btn);
    if (!curState && lastState[btn]) ret = true;
    lastState[btn] = curState;
    return ret;
}

typedef enum {
    kButtonPressTypeNone = 0,
    kButtonPressTypeShort,  // 0.1 - 1.5 sec
    kButtonPressTypeMid,    // 1.5 - 3 sec
    kButtonPressTypeLong,   // >= 3 sec

    kButtonPressLast,
} t_buttonPressType;

static inline void init_led_pin(uint32_t pin_led){
  gpio_init(pin_led);
  gpio_set_dir(pin_led, GPIO_OUT);
  gpio_put(pin_led, 0);
}


void init_led(){
  init_led_pin(PIN_LED1);
  init_led_pin(PIN_LED2);
  init_led_pin(PIN_LED3);
}

void led_blink_id(int id){
  int pin = 0;
  switch (id) {
    case 1:
      pin = PIN_LED1;
      break;
    case 2:
      pin = PIN_LED2;
      break;
    case 3:
      pin = PIN_LED3;
      break;
    default:
      return;
  }
  gpio_put(pin, 1);
  sleep_ms(USEC_PER_MSEC*0.5);
  gpio_put(pin, 0);
}

t_buttonPressType detectButtonPress(void){
    static uint64_t lastButtonSwitchTime = 0;
    static bool lastButtonState = false;
    static t_buttonPressType lastRet = kButtonPressTypeNone;

    t_buttonPressType ret = kButtonPressTypeNone;

    uint64_t curTime = time_us_64();
    bool curState = !gpio_get(PIN_BUTTON1);

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

        if (button_get_edge(PIN_BUTTON1)){
          return kButtonPressTypeShort;
        }

        if (button_get_edge(PIN_BUTTON2)){
          return kButtonPressTypeMid;
        }

        if (button_get_edge(PIN_BUTTON3)){
          return kButtonPressTypeLong;
        }

        if (lastButtonState){
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
        gSWDWantDeinit = false;
        gpio_init(PIN_SDQ_INVERTED(gCableIsInverted));
        gpio_set_dir(PIN_SDQ_INVERTED(gCableIsInverted), GPIO_OUT);
        gpio_put(PIN_SDQ_INVERTED(gCableIsInverted), 0);
        sleep_ms(50);
        gpio_set_dir(PIN_SDQ_INVERTED(gCableIsInverted), GPIO_IN);
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
            {
                uint32_t memdata = 0;
                if (SWD_readmem(0x80000004, &memdata) == SWD_RSP_OK){
                    if (memdata == 0x50300003){
                        //actually t8011 (ATV4K)
                        uart_ctrl_reg = 0xd063401C;
                    }
                }
            }
        }else{
            uint32_t data = 0;
            ack = SWD_DP_read_IDCODE(&data);
            if (ack == SWD_RSP_OK){
                gDPIDR = data;
            }else{
                gDPIDR = 0;
            }
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
    gIsSWDTaskActive = true;

    probe_task(gSWDModeIsSpam);
    if (gSWDModeIsSpam){
        if (gSpamFails < SPAM_ATTEMPTS_TIMEOUT){
            if (!task_spam()){
                gSpamFails = 0;
            }else{
                if ((gSpamFails++) & 0xF >= 10){
                    gWantSWDInitTime = time_us_64() + 10*USEC_PER_MSEC;
                    /*
                        One tristar poll cycle is ~7.6ms.
                    */
                }            
            }
        }        
    }else{
        gSpamFails = 0;
    }
    gIsSWDTaskActive = false;
}

void myusb_task(){
  tud_task();
  if (gDCSDIsInited){
      int itf_primary = ITF_UART_PRIMARY;
      while (uart_is_readable(DCSD_UART) && tud_cdc_n_write_available(itf_primary)) {
          tud_cdc_n_write_char(itf_primary, uart_getc(DCSD_UART));
      }
      
      if (tud_cdc_n_available(itf_primary)) {
          uart_putc_raw(DCSD_UART, tud_cdc_n_read_char(itf_primary));
      }
      tud_cdc_n_write_flush(itf_primary);
  }        

  if (gPUARTIsInited){
      int itf_tgt = ITF_SERIAL0;
      while (puart_is_readable() && tud_cdc_n_write_available(itf_tgt)) {
          tud_cdc_n_write_char(itf_tgt, puart_getc());
      }
      if (tud_cdc_n_available(itf_tgt)) {
          uart_putc_raw(DCSD_UART, tud_cdc_n_read_char(itf_tgt));
      }
      tud_cdc_n_write_flush(itf_tgt);
  }
  static char line[0x100] = {};
  static size_t didRead = 0;
  int didBreak = 0;
  if (didRead >= sizeof(line)-1){
    didRead = 0;
    memset(line, 0, sizeof(line));
  }
  for (; didRead < sizeof(line)-1; didRead++){
    if (!tud_cdc_n_available(ITF_CONTROL)) break;
    char c = tud_cdc_n_read_char(ITF_CONTROL);
    if (!isprint(c) && c != '\n' && c != '\r') {
      didRead--;
      continue;
    }
    tud_cdc_n_write_char(ITF_CONTROL, c);
    line[didRead] = c;
    tud_cdc_n_write_flush(ITF_CONTROL);
    if (line[didRead] == '\r'){
        didBreak = 1;
    }
    if (line[didRead] == '\n'){
      didBreak = 2;
      break;
    }
  }
  if (didBreak == 1){
    tud_cdc_n_write_char(ITF_CONTROL, '\n');
  }
  while (didRead > 0){
    char c = line[didRead-1];
    if (c == '\n' || c == '\r') didRead--;
    else break;
  }
  if (didBreak){
    line[didRead] = '\0';
    if (!strcmp(line, "reset")){
      tud_cdc_n_write_str(ITF_CONTROL, "resetting device!\r\n");
      gWantTristarReset = true;
      colobus_perform_wake();
    } else if (!strcmp(line, "dfu")){
      tud_cdc_n_write_str(ITF_CONTROL, "entering DFU!\r\n");
      gWantTristarReset = true;
      gWantTristarDFU = true;
      colobus_perform_wake();
    } else if (!strcmp(line, "rb")){
      watchdog_reboot(0,0,0);
    } else if (!strcmp(line, "bl")){
      reset_usb_boot(0,0);
    } else {
      tud_cdc_n_write_str(ITF_CONTROL, "got unrecognised command: '");
      tud_cdc_n_write_str(ITF_CONTROL, line);
      tud_cdc_n_write_str(ITF_CONTROL, "'\r\n");
    }
    didRead = 0;
    memset(line, 0, sizeof(line));
  }
  tud_cdc_n_write_flush(ITF_CONTROL);
  tud_task();
}

int main(){
    board_init(); //THIS IS MANDATORY, OTHERWISE SWD DOESN'T WORK!!!

    usbhub_init_default();
    usbmux_configure(kMuxcfg_iphone_to_hub);

    button_init();
    colobus_init();
    colobus_set_active_mode(kCOLOBUS_MODE_DEFAULT);
    lightning_rx_callback_register(lightning_callback);
    tusb_init();

    init_led();

    gCableIsInverted = button_get_edge(PIN_BUTTON1);

    bool isKisMode = false;
    if (button_get_edge(PIN_BUTTON2)){
      colobus_set_active_mode(kCOLOBUS_MODE_KIS_UART);
      isKisMode = true;
    }
    
    lightning_init(PIN_SDQ_INVERTED(gCableIsInverted));

    colobus_perform_wake();

    while (1){
        {
          //Boot message
          static int didgreet = 0;
          if (!didgreet && time_us_64() > USEC_PER_MSEC*2200){
            didgreet = 1;
            tud_cdc_n_write_str(ITF_CONTROL, "\r\nHello from Colobus!\r\n");
            tud_cdc_n_write_flush(ITF_CONTROL);            
          }
        }
        t_buttonPressType bpress = detectButtonPress();
        if (bpress){
            led_blink_id(bpress);
            if (bpress == kButtonPressTypeShort){
                gSWDModeIsSpam = !gSWDModeIsSpam;      
                if (isKisMode){
                  if (gSWDModeIsSpam){
                    colobus_set_active_mode(kCOLOBUS_MODE_KIS_UART);
                  }else{
                    colobus_set_active_mode(kCOLOBUS_MODE_DEFAULT);
                  }    
                  colobus_perform_wake();              
                }          
                
            }else if (bpress == kButtonPressTypeMid && !isKisMode){
                gWantTristarReset = true;
                colobus_perform_wake();

            }else if (bpress == kButtonPressTypeLong){
                if (gWantTristarReset){
                    gWantTristarReset = false;
                    gWantTristarDFU = false;
                }else{
                    gWantTristarReset = true;
                    gWantTristarDFU = true;
                    colobus_perform_wake();
                }
            }        
        }
        gpio_put(PIN_LED1, !gSWDModeIsSpam || isKisMode);
        if (isKisMode){
          gpio_put(PIN_LED2, gSWDModeIsSpam);
        }else{
          gpio_put(PIN_LED2, gCableIsInverted);
        }
        gpio_put(PIN_LED3, gWantTristarReset);
        
        colobus_wake_runloop();
        if (gWantDCSDInit){
            if (gWantDCSDInit > 0){
                dcsd_init();
            }else{
                dcsd_deinit();
            }
            gWantDCSDInit = 0;
        }

        if (gWantPUARTInit){
            if (gWantPUARTInit > 0){
                puart_init(PIN_PUART_RX_INVERTED(gCableIsInverted),PIN_PUART_TX_INVERTED(gCableIsInverted));
                gPUARTIsInited = true;
            }else{
                gPUARTIsInited = false;
                puart_deinit();
            }
            gWantPUARTInit = 0;
        }

        if (gSWDWantDeinit){
            gSWDWantDeinit = false;
            gSWDIsInited = false;
            swd_deinit();
        }

        if (!isKisMode && gWantSWDInitTime && (gWantSWDInitTime < time_us_64())){
            if (!gSWDIsInited){
                swd_init(PIN_SWDIO_INVERTED(gCableIsInverted), PIN_SWDCLK_INVERTED(gCableIsInverted));
                gSWDIsInited = true;
            }
            swd_reset();
            gWantSWDInitTime = 0;
            gDPIDR = 0;
        }

        if (gWantSWDInitTime == 0 && gSWDIsInited){
            task_swd();
        }
        myusb_task();
    }
}