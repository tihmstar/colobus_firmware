#ifndef SWD_H
#define SWD_H

#include <stdint.h>
#include <stdbool.h>

void swd_init(int swd_io, int swd_clk);
void swd_gpio_configure(int pin_swdio);
void swd_deinit();

void swd_set_freq(uint32_t freq_khz);
uint32_t swd_get_freq();

bool swd_reset();
int swd_read(uint8_t req, uint32_t *val);
int swd_write(uint8_t req, uint32_t val);

int SWD_readmem(uint32_t addr, uint32_t *data);


#define SWD_RSP_OK          0b001
#define SWD_RSP_WAIT        0b010
#define SWD_RSP_FAULT       0b100
#define SWD_RSP_LINE_ERROR  0b111

#define BITS_ALWYS  0x81

#define BITS_AP     (1<<1)
#define BITS_DP     (0<<1)

#define BITS_RD     (1<<2)
#define BITS_WR     (0<<2)

#define PARITY(i)   (i<<5)

#define BITS_DP_IDCODE  (0b00 << 3)
#define BITS_DP_ABORT   (0b00 << 3)
#define BITS_DP_CTRL    (0b01 << 3)
#define BITS_DP_RESEND  (0b10 << 3)
#define BITS_DP_SELECT  (0b10 << 3)
#define BITS_DP_RDBUFF  (0b11 << 3)

#define BITS_AP_CSW     (0b00 << 3)
#define BITS_AP_TAR     (0b01 << 3)
#define BITS_AP_DRW     (0b11 << 3)

#define BITS_DP_READ(addr) (((addr) == 0 || ((addr)>>3) == 3) ? PARITY(1) : PARITY(0)) | addr | BITS_RD | BITS_DP | BITS_ALWYS
#define BITS_DP_WRITE(addr) (((addr) == 0 || ((addr)>>3) == 3) ? PARITY(0) : PARITY(1)) | addr | BITS_WR | BITS_DP | BITS_ALWYS

#define BITS_AP_READ(addr) (((addr) == 0 || ((addr)>>3) == 3) ? PARITY(0) : PARITY(1)) | addr | BITS_RD | BITS_AP | BITS_ALWYS
#define BITS_AP_WRITE(addr) (((addr) == 0 || ((addr)>>3) == 3) ? PARITY(1) : PARITY(0)) | addr | BITS_WR | BITS_AP | BITS_ALWYS

#define SWD_DP_read_IDCODE(val)   swd_read(BITS_DP_READ(BITS_DP_IDCODE),val)
#define SWD_DP_read_CTRL(val)     swd_read(BITS_DP_READ(BITS_DP_CTRL),val)
#define SWD_DP_read_RESEND(val)   swd_read(BITS_DP_READ(BITS_DP_RESEND),val)
#define SWD_DP_read_RDBUFF(val)   swd_read(BITS_DP_READ(BITS_DP_RDBUFF),val)

#define SWD_DP_write_ABORT(val)  swd_write(BITS_DP_WRITE(BITS_DP_ABORT),val)
#define SWD_DP_write_CTRL(val)   swd_write(BITS_DP_WRITE(BITS_DP_CTRL),val)
#define SWD_DP_write_SELECT(val) swd_write(BITS_DP_WRITE(BITS_DP_SELECT),val)

#define SWD_AP_read_CSW(val) swd_read(BITS_AP_READ(BITS_AP_CSW),val)
#define SWD_AP_read_TAR(val) swd_read(BITS_AP_READ(BITS_AP_TAR),val)
#define SWD_AP_read_DRW(val) swd_read(BITS_AP_READ(BITS_AP_DRW),val)

#define SWD_AP_write_CSW(val) swd_write(BITS_AP_WRITE(BITS_AP_CSW),val)
#define SWD_AP_write_TAR(val) swd_write(BITS_AP_WRITE(BITS_AP_TAR),val)
#define SWD_AP_write_DRW(val) swd_write(BITS_AP_WRITE(BITS_AP_DRW),val)

#define SWD_DP_clear_error() SWD_DP_write_ABORT(0x1e)


#endif // SWD_H