#include "tusb_config.h"
#include "tusb.h"


#define USB_BCD 0x0200
#define USB_VID 0x6874
#define USB_PID 0x6263

#define MAX_PACKET_SIZE 0x40


#pragma mark usb default stuff
//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const * tud_descriptor_device_cb(void){
  return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+

#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + (CFG_TUD_CDC * TUD_CDC_DESC_LEN) + (TUD_VENDOR_DESC_LEN * CFG_TUD_VENDOR))
#define EPNUM_UART_NOTIF            0x81
#define EPNUM_UART_OUT              0x02
#define EPNUM_UART_IN               0x83

#define EPNUM_DOCKCHANNEL_NOTIF     0x84
#define EPNUM_DOCKCHANNEL_OUT       0x05
#define EPNUM_DOCKCHANNEL_IN        0x86

#define EPNUM_JTAG_OUT              0x07
#define EPNUM_JTAG_IN               0x88

enum{
  ITF_NUM_CDC_UART = 0,
  ITF_NUM_CDC_UART_DATA,
  ITF_NUM_CDC_DOCKCHANNEL,
  ITF_NUM_CDC_DOCKCHANNEL_DATA,
  ITF_NUM_JTAG,

  ITF_NUM_TOTAL
};

uint8_t const desc_configuration[] ={
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // 1 CDC: UART
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_UART, 4, EPNUM_UART_NOTIF, 8, EPNUM_UART_OUT, EPNUM_UART_IN, MAX_PACKET_SIZE),

    // 2 CDC: DOCKCHANNEL
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_DOCKCHANNEL, 5, EPNUM_DOCKCHANNEL_NOTIF, 8, EPNUM_DOCKCHANNEL_OUT, EPNUM_DOCKCHANNEL_IN, MAX_PACKET_SIZE),

    // 3 VENDOR: JTAG
    TUD_VENDOR_DESCRIPTOR(ITF_NUM_JTAG, 0, EPNUM_JTAG_OUT, EPNUM_JTAG_IN, MAX_PACKET_SIZE),
};


// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const * tud_descriptor_configuration_cb(uint8_t index){
  (void) index; // for multiple configurations

  // This example use the same configuration for both high and full speed mode
  return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+
char const* string_desc_arr [] ={
  (const char[]) { 0x09, 0x04 }, // 0: is supported language is English (0x0409)
  "tihmstar",                    // 1: Manufacturer
  "Colobus Cable",               // 2: Product
  "colobus",                     // 3: Serial
  "colobus_serial0",             // 4: uart
  "colobus_serial1",             // 5: dockchannel
  "colobus_jtag",                // 6: jtag
};

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid){
    (void) langid;
    static uint16_t _desc_str[32];
    uint8_t chr_count;

    if ( index == 0){
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {    
        if (!(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) )  return NULL;

        const char* str = string_desc_arr[index];

        // Cap at max char
        chr_count = strlen(str);
        if ( chr_count > 31 ) chr_count = 31;

            // Convert ASCII string into UTF-16
            for(uint8_t i=0; i<chr_count; i++){
            _desc_str[1+i] = str[i];
        }
    }

    // first byte is length (including header), second byte is string type
    _desc_str[0] = (TUSB_DESC_STRING << 8 ) | (2*chr_count + 2);

    return _desc_str;
}