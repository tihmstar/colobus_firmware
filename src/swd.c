#include "swd.h"

#include "swd.pio.h"
#include <hardware/clocks.h>
#include <hardware/pio.h>
#include <hardware/dma.h>

#include <macros.h>

#include <string.h>
#include <stdio.h>

#define SWD_PIO pio1
#define SWD_SM 0


#define ARRAYOF(arr) (sizeof(arr)/sizeof(*arr))

/*
    SWD_CLK = swd_base + 0
    SWD_IO  = swd_base + 1
*/
#define PIN_SWDCLK_OFFSET   0
#define PIN_SWDIO_OFFSET    1

static int gSWDBasePin = -1;

static int gSWD_pio_pc = -1;

#pragma mark public

void swd_init(int swd_base){
    gSWDBasePin = swd_base;
    pio_gpio_init(SWD_PIO, swd_base + PIN_SWDCLK_OFFSET);
    pio_gpio_init(SWD_PIO, swd_base + PIN_SWDIO_OFFSET);
    gpio_set_pulls(swd_base + PIN_SWDCLK_OFFSET, false, true);
    gpio_set_pulls(swd_base + PIN_SWDIO_OFFSET, true, false);


    if (gSWD_pio_pc == -1){
        pio_sm_set_enabled(SWD_PIO, SWD_SM, false);

        gSWD_pio_pc = pio_add_program(SWD_PIO, &swd_program);
        pio_sm_config c = swd_program_get_default_config(gSWD_pio_pc);

        sm_config_set_in_pins(&c, swd_base+PIN_SWDIO_OFFSET);
        sm_config_set_out_pins(&c, swd_base+PIN_SWDIO_OFFSET, 1);

        sm_config_set_sideset_pins(&c, swd_base+PIN_SWDCLK_OFFSET);
        sm_config_set_sideset(&c, 1, false, false);
        pio_sm_set_consecutive_pindirs(SWD_PIO, SWD_SM, swd_base+PIN_SWDCLK_OFFSET, 1, true);

        sm_config_set_set_pins(&c, swd_base+PIN_SWDIO_OFFSET, 1);

        sm_config_set_in_shift(&c, true, true, 32);
        sm_config_set_out_shift(&c, true, true, 8);

        pio_sm_init(SWD_PIO, SWD_SM, gSWD_pio_pc + swd_offset_start, &c);
        pio_sm_set_enabled(SWD_PIO, SWD_SM, true);
    }

    swd_set_freq(1000);
    swd_reset();
}

void swd_gpio_configure(int pin_swdio){
    gpio_set_pulls(pin_swdio, true, false);
    pio_gpio_init(SWD_PIO, pin_swdio);
}

void swd_deinit(){
    if (gSWD_pio_pc != -1){
        pio_sm_set_enabled(SWD_PIO, SWD_SM, false);
        pio_remove_program(SWD_PIO, &swd_program, gSWD_pio_pc);
        gSWD_pio_pc = -1;
    }
}

void swd_set_freq(uint32_t freq_khz){
    float div = ((float)clock_get_hz(clk_sys)/1e3) / (freq_khz<<1);
    if (div < 1) div = 1;
    pio_sm_set_clkdiv(SWD_PIO, SWD_SM, div);
}

uint32_t swd_get_freq(){
    uint32_t piocldif = SWD_PIO->sm[SWD_SM].clkdiv;
    uint8_t div_frac8 = piocldif >> PIO_SM0_CLKDIV_FRAC_LSB;
    uint32_t div_int = piocldif >> PIO_SM0_CLKDIV_INT_LSB;
    const int frac_bit_count = REG_FIELD_WIDTH(PIO_SM0_CLKDIV_FRAC);
    float div = (float)div_int + ((float)div_frac8/(1u << frac_bit_count));
    uint32_t freq_khz = (((float)clock_get_hz(clk_sys)/1e3) / div);
    return freq_khz>>1;
}

bool swd_reset(){
    uint8_t reqbuf[] = {
        (gSWD_pio_pc + swd_offset_write),
        (56-1),
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
        
        (gSWD_pio_pc + swd_offset_write),
        (16-1),
        0x9e, 0xe7,

        (gSWD_pio_pc + swd_offset_write),
        (56-1),
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 

        (gSWD_pio_pc + swd_offset_write),
        (16-1),
        0x00, 0x00,
    };

    int dSend = dma_claim_unused_channel(true);

    {
        dma_channel_config c = dma_channel_get_default_config(dSend);
        channel_config_set_dreq(&c, pio_get_dreq(SWD_PIO, SWD_SM, true)); //false=read data from SM, true=write data to SM
        channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        dma_channel_configure(  dSend,
                                &c,
                                &SWD_PIO->txf[SWD_SM],
                                reqbuf,
                                sizeof(reqbuf),
                                true
        );
    }

    dma_channel_wait_for_finish_blocking(dSend);

error:
    dma_channel_unclaim(dSend);

    uint32_t idcode = 0;
    int ack = swd_read(BITS_DP_READ(BITS_DP_IDCODE), &idcode);
    return ack == 0b001; //ACK OK
}

int swd_read(uint8_t req, uint32_t *val){
    uint8_t reqbuf[] = {
        (gSWD_pio_pc + swd_offset_write),
        32 -1, //number of bits to write 
        0x00, 0x00, 0x00, 0x00, 

        (gSWD_pio_pc + swd_offset_write),
        8 -1, //number of bits to write 
        req,

        (gSWD_pio_pc + swd_offset_read),
        4 -1,

        (gSWD_pio_pc + swd_offset_read),
        33 -1,
    };
    uint32_t rsp[1+2] = {};

    int dSend = dma_claim_unused_channel(true);
    int dRecv = dma_claim_unused_channel(true);

    uint8_t ack = 0;

    for (int i=0; i < 5; i++) {
        {
            dma_channel_config c = dma_channel_get_default_config(dSend);
            channel_config_set_dreq(&c, pio_get_dreq(SWD_PIO, SWD_SM, true)); //false=read data from SM, true=write data to SM
            channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
            channel_config_set_read_increment(&c, true);
            channel_config_set_write_increment(&c, false);
            dma_channel_configure(  dSend,
                                    &c,
                                    &SWD_PIO->txf[SWD_SM],
                                    reqbuf,
                                    sizeof(reqbuf),
                                    true
            );
        }

        {
            dma_channel_config c = dma_channel_get_default_config(dRecv);
            channel_config_set_dreq(&c, pio_get_dreq(SWD_PIO, SWD_SM, false)); //false=read data from SM, true=write data to SM
            channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
            channel_config_set_read_increment(&c, false);
            channel_config_set_write_increment(&c, true);
            dma_channel_configure(  dRecv,
                                    &c,
                                    rsp,
                                    &SWD_PIO->rxf[SWD_SM],
                                    ARRAYOF(rsp),
                                    true
            );
        }
        dma_channel_wait_for_finish_blocking(dRecv);
        dma_channel_wait_for_finish_blocking(dSend);

        ack = rsp[0]>>29;
        if (ack != SWD_RSP_WAIT) break;
    };

error:
    dma_channel_unclaim(dRecv);
    dma_channel_unclaim(dSend);    

    if (val) *val = rsp[1];
    return ack;
}

int swd_write(uint8_t req, uint32_t val){
    uint32_t parity = __builtin_parity(val);
    uint8_t reqbuf[] = {
        (gSWD_pio_pc + swd_offset_write),
        32 -1, //number of bits to write 
        0x00, 0x00, 0x00, 0x00, 
        (gSWD_pio_pc + swd_offset_write),
        8 -1, //number of bits to write 
        req,

        (gSWD_pio_pc + swd_offset_read),
        5 -1,

        (gSWD_pio_pc + swd_offset_write),
        32 -1,
        (val >> 0), (val >> 8), (val >> 16), (val >> 24), 

        (gSWD_pio_pc + swd_offset_write),
        8 -1, //number of bits to write 
        parity & 1,
    };
    uint32_t rsp = 0;

    int dSend = dma_claim_unused_channel(true);
    uint8_t ack = 0;

    for (int i=0; i < 5; i++) {
        {
            dma_channel_config c = dma_channel_get_default_config(dSend);
            channel_config_set_dreq(&c, pio_get_dreq(SWD_PIO, SWD_SM, true)); //false=read data from SM, true=write data to SM
            channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
            channel_config_set_read_increment(&c, true);
            channel_config_set_write_increment(&c, false);
            dma_channel_configure(  dSend,
                                    &c,
                                    &SWD_PIO->txf[SWD_SM],
                                    reqbuf,
                                    sizeof(reqbuf),
                                    true
            );
        }

        dma_channel_wait_for_finish_blocking(dSend);
        rsp = pio_sm_get_blocking(SWD_PIO, SWD_SM);

        ack = (rsp>>28) & 0b111;
        if (ack != SWD_RSP_WAIT) break;
    };

error:
    dma_channel_unclaim(dSend);    

    return ack;
}

int SWD_readmem(uint32_t addr, uint32_t *data){
    int err = 0;
    int ack = 0;
    cassure((ack = SWD_AP_write_TAR(addr)) == SWD_RSP_OK);
    cassure((ack = SWD_AP_read_DRW(data)) == SWD_RSP_OK);
    cassure((ack = SWD_DP_read_RDBUFF(data)) == SWD_RSP_OK);
error:
    return ack;
}