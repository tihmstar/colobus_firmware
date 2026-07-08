#include "pio_i2c.h"
#include "i2c.pio.h"

#include <hardware/pio.h>
#include <hardware/gpio.h>
#include <hardware/clocks.h>
#include <hardware/irq.h>


#define I2C_PIO pio1

#pragma mark static vars
static int gI2C_sm = -1;
static int gI2C_pio_pc = -1;

#pragma mark private

const int PIO_I2C_ICOUNT_LSB = 10;
const int PIO_I2C_FINAL_LSB  = 9;
const int PIO_I2C_DATA_LSB   = 1;
const int PIO_I2C_NAK_LSB    = 0;

enum {
    I2C_SC0_SD0 = 0,
    I2C_SC0_SD1,
    I2C_SC1_SD0,
    I2C_SC1_SD1
};

#pragma mark private declaration
bool pio_i2c_check_error();
void pio_i2c_resume_after_error();
void pio_i2c_rx_enable(bool en);
static inline void pio_i2c_put16(uint16_t data);
void pio_i2c_put_or_err(uint16_t data);
uint8_t pio_i2c_get();
void pio_i2c_start();
void pio_i2c_stop();
void pio_i2c_repstart();
static void pio_i2c_wait_idle();

#pragma mark private implementation
bool pio_i2c_check_error() {
  return pio_interrupt_get(I2C_PIO, gI2C_sm);
}

void pio_i2c_resume_after_error() {
  pio_sm_drain_tx_fifo(I2C_PIO, gI2C_sm);
  pio_sm_exec(I2C_PIO, gI2C_sm, (I2C_PIO->sm[gI2C_sm].execctrl & PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS) >> PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB);
  pio_interrupt_clear(I2C_PIO, gI2C_sm);
}

void pio_i2c_rx_enable(bool en) {
  if (en)
      hw_set_bits(&I2C_PIO->sm[gI2C_sm].shiftctrl, PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS);
  else
      hw_clear_bits(&I2C_PIO->sm[gI2C_sm].shiftctrl, PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS);
}

static inline void pio_i2c_put16(uint16_t data) {
    while (pio_sm_is_tx_fifo_full(I2C_PIO, gI2C_sm))
        ;
    // some versions of GCC dislike this
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif
    *(io_rw_16 *)&I2C_PIO->txf[gI2C_sm] = data;
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
}


// If I2C is ok, block and push data. Otherwise fall straight through.
void pio_i2c_put_or_err(uint16_t data) {
    while (pio_sm_is_tx_fifo_full(I2C_PIO, gI2C_sm))
        if (pio_i2c_check_error(I2C_PIO, gI2C_sm))
            return;
    if (pio_i2c_check_error(I2C_PIO, gI2C_sm))
        return;
    // some versions of GCC dislike this
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif
    *(io_rw_16 *)&I2C_PIO->txf[gI2C_sm] = data;
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
}

uint8_t pio_i2c_get() {
    return (uint8_t)pio_sm_get(I2C_PIO, gI2C_sm);
}

void pio_i2c_start() {
    pio_i2c_put_or_err(2u << PIO_I2C_ICOUNT_LSB);                         // Escape code for 3 instruction sequence
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC1_SD0]);    // We are already in idle state, just pull SDA low
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC0_SD0]);    // Also pull clock low so we can present data
    pio_i2c_put_or_err(pio_encode_mov(pio_isr, pio_null));                // Ensure ISR counter is clear following a write
}

void pio_i2c_stop() {
    pio_i2c_put_or_err(2u << PIO_I2C_ICOUNT_LSB);
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC0_SD0]);    // SDA is unknown; pull it down
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC1_SD0]);    // Release clock
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC1_SD1]);    // Release SDA to return to idle state
};

void pio_i2c_repstart() {
    pio_i2c_put_or_err(4u << PIO_I2C_ICOUNT_LSB);
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC0_SD1]);
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC1_SD1]);
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC1_SD0]);
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC0_SD0]);
    pio_i2c_put_or_err(pio_encode_mov(pio_isr, pio_null));
}

static void pio_i2c_wait_idle() {
    // Finished when TX runs dry or SM hits an IRQ
    I2C_PIO->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + gI2C_sm);
    while (!(I2C_PIO->fdebug & 1u << (PIO_FDEBUG_TXSTALL_LSB + gI2C_sm) || pio_i2c_check_error()))
        tight_loop_contents();
}

#pragma mark public
/*
  In colobus rev 5 i fucked up wiring and swapped SDA and SCL pins :/
*/
void pio_i2c_init(uint8_t pin_sda, uint8_t pin_scl){
    assert(pin_sda == pin_scl + 1);
    pio_gpio_init(I2C_PIO, pin_sda);
    pio_gpio_init(I2C_PIO, pin_scl);
    gpio_pull_up(pin_sda);
    gpio_pull_up(pin_scl);
    
    if (gI2C_sm == -1){
      gI2C_sm = pio_claim_unused_sm(I2C_PIO, true);
    }

    if (gI2C_pio_pc == -1){
        pio_sm_set_enabled(I2C_PIO, gI2C_sm, false);

        gI2C_pio_pc = pio_add_program(I2C_PIO, &i2c_program);
        pio_sm_config c = i2c_program_get_default_config(gI2C_pio_pc);

        // IO mapping
        sm_config_set_out_pins(&c, pin_sda, 1);
        sm_config_set_set_pins(&c, pin_sda, 1);
        sm_config_set_in_pins(&c, pin_sda);
        sm_config_set_sideset_pins(&c, pin_scl);
        sm_config_set_jmp_pin(&c, pin_sda);

        sm_config_set_out_shift(&c, false, true, 16);
        sm_config_set_in_shift(&c, false, true, 8);

        {
            float freq = 60e3; //Hz
    
            float div = (float)clock_get_hz(clk_sys) / freq;
            if (div < 1) div = 1;
            sm_config_set_clkdiv(&c, div);
        }
        
        // Try to avoid glitching the bus while connecting the IOs. Get things set
        // up so that pin is driven down when PIO asserts OE low, and pulled up
        // otherwise.
        gpio_pull_up(pin_scl);
        gpio_pull_up(pin_sda);
        uint32_t both_pins = (1u << pin_sda) | (1u << pin_scl);
        pio_sm_set_pins_with_mask(I2C_PIO, gI2C_sm, both_pins, both_pins);
        pio_sm_set_pindirs_with_mask(I2C_PIO, gI2C_sm, both_pins, both_pins);
        pio_gpio_init(I2C_PIO, pin_sda);
        gpio_set_oeover(pin_sda, GPIO_OVERRIDE_INVERT);
        pio_gpio_init(I2C_PIO, pin_scl);
        gpio_set_oeover(pin_scl, GPIO_OVERRIDE_INVERT);
        pio_sm_set_pins_with_mask(I2C_PIO, gI2C_sm, 0, both_pins);

        // Clear IRQ flag before starting, and make sure flag doesn't actually
        // assert a system-level interrupt (we're using it as a status flag)
        pio_set_irq0_source_enabled(I2C_PIO, (enum pio_interrupt_source) ((uint) pis_interrupt0 + gI2C_sm), false);
        pio_set_irq1_source_enabled(I2C_PIO, (enum pio_interrupt_source) ((uint) pis_interrupt0 + gI2C_sm), false);
        pio_interrupt_clear(I2C_PIO, gI2C_sm);

        // Configure and start SM
        pio_sm_init(I2C_PIO, gI2C_sm, gI2C_pio_pc + i2c_offset_entry_point, &c);
        pio_sm_set_enabled(I2C_PIO, gI2C_sm, true);
      }
}

void pio_i2c_deinit(){
  if (gI2C_pio_pc != -1){
      pio_sm_set_enabled(I2C_PIO, gI2C_sm, false);
      pio_remove_program(I2C_PIO, &i2c_program, gI2C_pio_pc);
      gI2C_pio_pc = -1;
  }
  if (gI2C_sm != -1){
    pio_sm_unclaim(I2C_PIO, gI2C_sm); gI2C_sm = -1;
  }
}

int pio_i2c_write_blocking(uint8_t addr, void *txbuf_, uint8_t len, bool nonstop){
  uint8_t *txbuf = (uint8_t *)txbuf_;
  int err = 0;
  pio_i2c_start();
  pio_i2c_rx_enable(false);
  pio_i2c_put16((addr << 2) | 1u);
  while (len && !pio_i2c_check_error()) {
      if (!pio_sm_is_tx_fifo_full(I2C_PIO, gI2C_sm)) {
          --len;
          pio_i2c_put_or_err((*txbuf++ << PIO_I2C_DATA_LSB) | ((len == 0) << PIO_I2C_FINAL_LSB) | 1u);
      }
  }
  if (nonstop) return err;
  pio_i2c_stop();
  pio_i2c_wait_idle();
  if (pio_i2c_check_error()) {
      err = -1;
      pio_i2c_resume_after_error();
      pio_i2c_stop();
  }
  return err;
}

int pio_i2c_read_blocking(uint8_t addr, void *rxbuf_, uint8_t len){
  uint8_t *rxbuf = (uint8_t *)rxbuf_;
  int err = 0;
  pio_i2c_repstart();
  pio_i2c_rx_enable(true);
  while (!pio_sm_is_rx_fifo_empty(I2C_PIO, gI2C_sm))
      (void)pio_i2c_get();
  pio_i2c_put16((addr << 2) | 3u);
  uint32_t tx_remain = len; // Need to stuff 0xff bytes in to get clocks

  bool first = true;

  while ((tx_remain || len) && !pio_i2c_check_error()) {
      if (tx_remain && !pio_sm_is_tx_fifo_full(I2C_PIO, gI2C_sm)) {
          --tx_remain;
          pio_i2c_put16((0xffu << 1) | (tx_remain ? 0 : (1u << PIO_I2C_FINAL_LSB) | (1u << PIO_I2C_NAK_LSB)));
      }
      if (!pio_sm_is_rx_fifo_empty(I2C_PIO, gI2C_sm)) {
          if (first) {
              // Ignore returned address byte
              (void)pio_i2c_get();
              first = false;
          }
          else {
              --len;
              *rxbuf++ = pio_i2c_get();
          }
      }
  }
  pio_i2c_stop();
  pio_i2c_wait_idle();
  if (pio_i2c_check_error()) {
      err = -1;
      pio_i2c_resume_after_error();
      pio_i2c_stop();
  }
  return err;
}
