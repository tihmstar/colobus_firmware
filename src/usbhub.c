#include "usbhub.h"

#include "pio_i2c.h"

#include <pico/time.h>
#include <hardware/gpio.h>

#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#define ARRAYOF(a) (sizeof(a)/sizeof(*a))

#define USB_VID 0x6874
#define USB_PID 0x6268
#define USB_DID 0x105
#define USB_LANG_ID_ENG 0x0904

#define USBHUB_SMBUS_DEVICE 0x2C
#define USBHUB_MAX_BURST_BYTES 0x10
#define USBHUB_STRING_BUF_LEN 62

#pragma mark private
static void usbhub_reset(bool defaultcfg){
  gpio_init(USBHUB_PIN_SCL);
  gpio_set_dir(USBHUB_PIN_SCL, GPIO_OUT);
  gpio_put(USBHUB_PIN_SCL, !defaultcfg);
  gpio_init(USBHUB_PIN_RESET);
  gpio_set_dir(USBHUB_PIN_RESET, GPIO_OUT);
  gpio_put(USBHUB_PIN_RESET, 0);
  sleep_us(10);
  gpio_put(USBHUB_PIN_RESET, 1);
  gpio_set_dir(USBHUB_PIN_RESET, GPIO_IN);
  sleep_us(800);
}

static uint8_t usbhub_readbyte(uint8_t addr){
  uint8_t ret[2] = {};
  int asd = pio_i2c_write_blocking(USBHUB_SMBUS_DEVICE, &addr, sizeof(addr), true);
  int dsa = pio_i2c_read_blocking(USBHUB_SMBUS_DEVICE, &ret, sizeof(ret));
  return ret[1];
}

static void usbhub_writebyte(uint8_t addr, uint8_t data){
  uint8_t d[3]={
    addr,
    1,
    data
  };
  pio_i2c_write_blocking(USBHUB_SMBUS_DEVICE, d, sizeof(d), false);
}

static void usbhub_writeword(uint8_t addr, uint16_t data){
  uint8_t d[4]={
    addr,
    2,
    data,
    data >> 8
  };
  pio_i2c_write_blocking(USBHUB_SMBUS_DEVICE, d, sizeof(d), false);
}

static void usbhub_write_internal(uint8_t addr, void *data, uint8_t datalen){
  assert(datalen <= USBHUB_MAX_BURST_BYTES);
  uint8_t d[datalen+2];
  d[0] = addr;
  d[1] = datalen;
  memcpy(&d[2], data, datalen);
  pio_i2c_write_blocking(USBHUB_SMBUS_DEVICE, d, sizeof(d), false);
}

static void usbhub_write(uint8_t addr, void *data, uint8_t datalen){
  uint8_t *ptr = (uint8_t*)data;
  for (size_t i = 0; i < datalen; i+=USBHUB_MAX_BURST_BYTES){
    uint8_t wlen = MIN(USBHUB_MAX_BURST_BYTES, datalen);
    usbhub_write_internal(addr+i, ptr+i, wlen);
  }
}

static void usbhub_populate_defaults(){
  usbhub_writeword(0x00, USB_VID);
  usbhub_writeword(0x02, USB_PID);
  usbhub_writeword(0x04, USB_DID);
  usbhub_writebyte(0x06, 0
    | (1/*BUS powered*/<<7)
    | (0/*reserved*/   <<6)
    | (0/*HS enabled*/ <<5)
    | (1/*MTT_ENABLE*/ <<4)
    | (1/*EOP_DISABLE*/<<3)
    | (0b10/*CURRENT_SNS off*/<<1)
    | (1/*PORT_PWR individual*/<<0)
  );
  usbhub_writebyte(0x07, 0
    | (0/*DYNAMIC off*/<<7)
    | (0/*reserved*/   <<6)
    | (0b10/*OC_TIMER 8.0 ms*/<<4)
    | (0b000/*reserved*/<<0)
  );
  usbhub_writebyte(0x08, 0
    | (0/*reserved*/   <<4)
    | (0/*PRTMAP_EN standard*/<<3)
    | (0b00/*reserved*/<<1)
    | (1/*STRING_EN*/<<0)
  );
  usbhub_writebyte(0x09, 0
    | (1/*PORT 2 removable*/<<2)
    | (0/*PORT 1 non-removable*/<<1)
  );
  // usbhub_writebyte(0x0A,
  //   | (0/*PORT 2 SP enabled*/<<2)
  //   | (0/*PORT 1 SP enabled*/<<1)
  // );
  usbhub_writebyte(0x0B, 0
    | (0/*PORT 2 BP enabled*/<<2)
    | (0/*PORT 1 BP enabled*/<<1)
  );

  usbhub_writebyte(0x0C, /*MAX_PWR_BP*/ 0x32/*100mA*/);
  usbhub_writebyte(0x0F, /*HC_MAX_C_BP*/ 0x32/*100mA*/);
  usbhub_writebyte(0x10, /*POWER_ON_TIME*/ 0x32/*100ms*/);
  usbhub_writeword(0x11, USB_LANG_ID_ENG);
  {
    //manufactor string
    uint16_t str[] = {'t','i','h','m','s','t','a','r'};
    usbhub_writebyte(0x13, ARRAYOF(str));
    usbhub_write(0x16, str, sizeof(str));
  }
  {
    //product string
    uint16_t str[] = {'C','o','l','o','b','u','s',' ','H','u','b'};
    usbhub_writebyte(0x14, ARRAYOF(str));
    usbhub_write(0x54, str, sizeof(str));
  }
  {
    //serial string
    uint16_t str[] = {'r','e','v','5'};
    usbhub_writebyte(0x15, ARRAYOF(str));
    usbhub_write(0x92, str, sizeof(str));
  }
}

static void usbhub_lock_and_attach(){
  usbhub_writebyte(0xFF, 0x01);
}


#pragma mark public
#pragma mark public low level
void usbhub_init(){
  usbhub_reset(false);
  pio_i2c_init(USBHUB_PIN_SDA, USBHUB_PIN_SCL);
}

void usbhub_deinit(){
  pio_i2c_deinit();
}

#pragma mark public easy-use
void usbhub_init_default(){
  usbhub_init();
  usbhub_populate_defaults();
  usbhub_lock_and_attach();
  usbhub_deinit();
}
