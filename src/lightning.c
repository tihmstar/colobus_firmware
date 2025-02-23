
#include "lightning.h"

#include "lightning.pio.h"

#include <hardware/pio.h>
#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/irq.h>
#include <hardware/gpio.h>
#include <hardware/sync.h>

#include <pico/bit_ops.h>

#include <stdio.h>
#include <string.h>

#define LIGHTNING_PIO pio0
#define LIGHTNING_RX_SM 0
#define LIGHTNING_TX_SM 1


static int gLightning_rx_pio_pc = -1;
static int gLightning_tx_pio_pc = -1;

static int gRxDMAChannel = -1;
static int gTxDMAChannel = -1;
static lightning_rx_cb gUserRxCB = NULL;
static uint8_t gRxBuf[0x40] = {};

#pragma mark private
static void rx_irq_handler(void){
    // dma_channel_abort(gRxDMAChannel);
    dma_channel_hw_t *dmachan =  dma_channel_hw_addr(gRxDMAChannel);
    size_t didRead = dmachan->write_addr - (uint32_t)gRxBuf;
    if (gUserRxCB && didRead) gUserRxCB(gRxBuf, didRead);
    dmachan->transfer_count = sizeof(gRxBuf);
    dmachan->al2_write_addr_trig = (uint32_t)gRxBuf;
    dma_hw->ints0 = 1u << gRxDMAChannel;
    pio_interrupt_clear(LIGHTNING_PIO, 0);
}

#pragma mark public
void lightning_gpio_configure(int pin_sdq){
    pio_gpio_init(LIGHTNING_PIO, pin_sdq);
    gpio_set_pulls(pin_sdq, true, false);
}

void lightning_init(int pin_sdq){
    lightning_gpio_configure(pin_sdq);
    if (gLightning_rx_pio_pc == -1){
        pio_sm_set_enabled(LIGHTNING_PIO, LIGHTNING_RX_SM, false);

        gLightning_rx_pio_pc = pio_add_program(LIGHTNING_PIO, &lightning_rx_program);
        pio_sm_config c = lightning_rx_program_get_default_config(gLightning_rx_pio_pc);

        {
            float freq = 2e6; //Hz
    
            float div = (float)clock_get_hz(clk_sys) / freq;
            if (div < 1) div = 1;
            sm_config_set_clkdiv(&c, div);
        }

        sm_config_set_in_pins(&c, pin_sdq);
        sm_config_set_out_pins(&c, pin_sdq, 1);
        sm_config_set_set_pins(&c, pin_sdq, 1);
        sm_config_set_in_shift(&c, true, true, 8);
        sm_config_set_out_shift(&c, false, false, 8);
        sm_config_set_jmp_pin(&c, pin_sdq);

        pio_sm_set_consecutive_pindirs(LIGHTNING_PIO, LIGHTNING_RX_SM, pin_sdq, 1, false);

        pio_sm_init(LIGHTNING_PIO, LIGHTNING_RX_SM, gLightning_rx_pio_pc + lightning_rx_offset_reset, &c);
        pio_sm_set_enabled(LIGHTNING_PIO, LIGHTNING_RX_SM, true);
    }

    if (gLightning_tx_pio_pc == -1){
        pio_sm_set_enabled(LIGHTNING_PIO, LIGHTNING_TX_SM, false);

        gLightning_tx_pio_pc = pio_add_program(LIGHTNING_PIO, &lightning_tx_program);
        pio_sm_config c = lightning_tx_program_get_default_config(gLightning_tx_pio_pc);

        {
            float freq = 1e6; //Hz
    
            float div = (float)clock_get_hz(clk_sys) / freq;
            if (div < 1) div = 1;
            sm_config_set_clkdiv(&c, div);
        }

        sm_config_set_in_pins(&c, pin_sdq);
        sm_config_set_out_pins(&c, pin_sdq, 1);
        sm_config_set_sideset_pins(&c, pin_sdq);
        sm_config_set_sideset(&c, 2, true, true);
        sm_config_set_set_pins(&c, pin_sdq, 1);
        sm_config_set_out_shift(&c, true, true, 8);
        sm_config_set_jmp_pin(&c, pin_sdq);

        pio_sm_init(LIGHTNING_PIO, LIGHTNING_TX_SM, gLightning_tx_pio_pc + lightning_tx_offset_start, &c);
        pio_sm_set_enabled(LIGHTNING_PIO, LIGHTNING_TX_SM, true);
    }
    if (gTxDMAChannel == -1){
        gTxDMAChannel = dma_claim_unused_channel(true);
    }
    
    if (gRxDMAChannel == -1){
        gRxDMAChannel = dma_claim_unused_channel(true);

        {
            dma_channel_config c = dma_channel_get_default_config(gRxDMAChannel);
            channel_config_set_dreq(&c, pio_get_dreq(LIGHTNING_PIO, LIGHTNING_RX_SM, false)); //false=read data from SM, true=write data to SM
            channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
            channel_config_set_read_increment(&c, false);
            channel_config_set_write_increment(&c, true);
            dma_channel_configure(  gRxDMAChannel,
                                    &c,
                                    gRxBuf,
                                    &((uint8_t*)(&LIGHTNING_PIO->rxf[LIGHTNING_RX_SM]))[3],
                                    sizeof(gRxBuf),
                                    false
            );
        }
        dma_channel_start(gRxDMAChannel);

        //Enable irq interrupts for up and down actions.
        irq_set_exclusive_handler(PIO0_IRQ_0, rx_irq_handler);
        pio_set_irq0_source_enabled(LIGHTNING_PIO, (enum pio_interrupt_source)(pis_interrupt0 + LIGHTNING_RX_SM), true);
        irq_set_enabled(PIO0_IRQ_0, true);

        irq_set_exclusive_handler(DMA_IRQ_0, rx_irq_handler);   // Set interrupt handler
        dma_channel_set_irq0_enabled(gRxDMAChannel, true);      // Enable IRQ for DMA
        irq_set_enabled(DMA_IRQ_0, true);                       // Enable the IRQ
    }
}

void lightning_cleanup(void){
    if (gLightning_rx_pio_pc != -1){
        pio_sm_set_enabled(LIGHTNING_PIO, LIGHTNING_RX_SM, false);
        pio_remove_program(LIGHTNING_PIO, &lightning_rx_program, gLightning_rx_pio_pc);
        gLightning_rx_pio_pc = -1;
    }
    
    if (gLightning_tx_pio_pc != -1){
        pio_sm_set_enabled(LIGHTNING_PIO, LIGHTNING_TX_SM, false);
        pio_remove_program(LIGHTNING_PIO, &lightning_tx_program, gLightning_tx_pio_pc);
        gLightning_tx_pio_pc = -1;
    }

    irq_set_enabled(PIO0_IRQ_0, false);
    pio_set_irq0_source_enabled(LIGHTNING_PIO, (enum pio_interrupt_source)(pis_interrupt0 + LIGHTNING_RX_SM), false);
    irq_remove_handler(PIO0_IRQ_0, rx_irq_handler);


    {
        int dmaActiveChannelsMask = 0;

        if (gTxDMAChannel != -1) dmaActiveChannelsMask |= 1u << gTxDMAChannel;
        if (gRxDMAChannel != -1) dmaActiveChannelsMask |= 1u << gRxDMAChannel;

        dma_hw->abort = dmaActiveChannelsMask;

        if (gTxDMAChannel != -1) while (dma_hw->ch[gTxDMAChannel].ctrl_trig & DMA_CH0_CTRL_TRIG_BUSY_BITS) tight_loop_contents();
        if (gRxDMAChannel != -1) while (dma_hw->ch[gRxDMAChannel].ctrl_trig & DMA_CH0_CTRL_TRIG_BUSY_BITS) tight_loop_contents();

        dma_unclaim_mask(dmaActiveChannelsMask);
        gTxDMAChannel = -1;
        gRxDMAChannel = -1;
    }
}

// void lightning_read_blocking(void *buf, size_t bufSize){
//     int dChan = -1;

//     dChan = dma_claim_unused_channel(true);
    
//     {
//         dma_channel_config c = dma_channel_get_default_config(dChan);
//         channel_config_set_dreq(&c, pio_get_dreq(LIGHTNING_PIO, LIGHTNING_RX_SM, false)); //false=read data from SM, true=write data to SM
//         channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
//         channel_config_set_read_increment(&c, false);
//         channel_config_set_write_increment(&c, true);
//         dma_channel_configure(  dChan,
//                                 &c,
//                                 buf,
//                                 &((uint8_t*)(&LIGHTNING_PIO->rxf[LIGHTNING_RX_SM]))[3],
//                                 bufSize,
//                                 true
//         );
//     }

//     dma_channel_wait_for_finish_blocking(dChan);

// error:
//     dma_channel_unclaim(dChan);
// }

void lightning_write_nonblocking(const void *buf, uint8_t bufSize){
    if (!bufSize) return;
    {
        dma_channel_config c = dma_channel_get_default_config(gTxDMAChannel);
        channel_config_set_dreq(&c, pio_get_dreq(LIGHTNING_PIO, LIGHTNING_TX_SM, true)); //false=read data from SM, true=write data to SM
        channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        dma_channel_configure(  gTxDMAChannel,
                                &c,
                                &LIGHTNING_PIO->txf[LIGHTNING_TX_SM],
                                buf,
                                bufSize,
                                true
        );
    }
}

void lightning_write_blocking(const void *buf, uint8_t bufSize){
    lightning_write_nonblocking(buf, bufSize);
    dma_channel_wait_for_finish_blocking(gTxDMAChannel);
}

void lightning_rx_callback_register(lightning_rx_cb cb){
    gUserRxCB = cb;
}

void lightning_rx_callback_unregister(){
    gUserRxCB = NULL;
}