#ifndef FUSB_H
#define FUSB_H

#define FUSB_DEVICE 0x22

#define FUSB_REG_DEVICE_ID  0x01
#define FUSB_REG_RESET      0x0C
#   define FUSB_RESET_VAL_PD_RESET              (1<<1)
#   define FUSB_RESET_VAL_SW_RES                (1<<0)
#define FUSB_REG_SWITCHES0  0x02
#   define FUSB_SWITCHES0_VAL_PU_EN2            (1<<7)
#   define FUSB_SWITCHES0_VAL_PU_EN1            (1<<6)
#   define FUSB_SWITCHES0_VAL_VCONN_CC2         (1<<5)
#   define FUSB_SWITCHES0_VAL_VCONN_CC1         (1<<4)
#   define FUSB_SWITCHES0_VAL_MEAS_CC2          (1<<3)
#   define FUSB_SWITCHES0_VAL_MEAS_CC1          (1<<2)
#   define FUSB_SWITCHES0_VAL_PDWN2             (1<<1)
#   define FUSB_SWITCHES0_VAL_PDWN1             (1<<0)
#define FUSB_REG_SWITCHES1  0x03
#   define FUSB_SWITCHES1_MASK_RESERVED         (1<<3)
#   define FUSB_SWITCHES1_VAL_POWERROLE         (1<<7)
#   define FUSB_SWITCHES1_VAL_SPECREV(n)        (((n) ? 0b11 : 0b00) << 5)
#   define FUSB_SWITCHES1_SPECREV_1_0           0b00
#   define FUSB_SWITCHES1_SPECREV_2_0           0b01
#   define FUSB_SWITCHES1_MASK_SPECREV          (0b11 << 5)
#   define FUSB_SWITCHES1_VAL_DATAROLE          (1<<4)
#   define FUSB_SWITCHES1_VAL_AUTO_CRC          (1<<2)
#   define FUSB_SWITCHES1_VAL_TXCC2             (1<<1)
#   define FUSB_SWITCHES1_VAL_TXCC1             (1<<0)
#define FUSB_REG_MEASURE    0x04
#   define FUSB_MEASURE_VAL_MEAS_VBUS           (1<<6)
#   define FUSB_MEASURE_MASK_MDAC               (0x3F)
#   define FUSB_MEASURE_VAL_MDAC(n)             (n & 0x3F)
#define FUSB_REG_CONTROL0   0x06
#   define FUSB_CONTROL0_VAL_TX_FLUSH           (1<<6)
#   define FUSB_CONTROL0_VAL_INT_MASK           (1<<5)
#   define FUSB_CONTROL0_VAL_HOST_CUR(n)        ((n&0b11)<<2)
#   define FUSB_CONTROL0_HOST_CUR_DISABLED      0b00
#   define FUSB_CONTROL0_HOST_CUR_DEFAULT_USB   0b01
#   define FUSB_CONTROL0_HOST_CUR_MED_1_5_A     0b10
#   define FUSB_CONTROL0_HOST_CUR_HIGH_3_A      0b11
#   define FUSB_CONTROL0_HOST_CUR_MASK          (0b11<<2)
#   define FUSB_CONTROL0_VAL_AUTO_PRE           (1<<1)
#   define FUSB_CONTROL0_VAL_TX_START           (1<<0)
#define FUSB_REG_CONTROL1   0x07
#   define FUSB_CONTROL1_MASK_RESERVED          ((1<<7) | (1<<3))
#   define FUSB_CONTROL1_VAL_ENSOP2DB           (1<<6)
#   define FUSB_CONTROL1_VAL_ENSOP1DB           (1<<5)
#   define FUSB_CONTROL1_VAL_BIST_MODE2         (1<<4)
#   define FUSB_CONTROL1_VAL_RX_FLUSH           (1<<2)
#   define FUSB_CONTROL1_VAL_ENSOP2             (1<<1)
#   define FUSB_CONTROL1_VAL_ENSOP1             (1<<0)
#define FUSB_REG_CONTROL3   0x09
#   define FUSB_CONTROL3_VAL_SEND_HARD_RESET    (1<<6)
#   define FUSB_CONTROL3_VAL_BIST_TMODE         (1<<5)
#   define FUSB_CONTROL3_VAL_AUTO_HARDRESET     (1<<4)
#   define FUSB_CONTROL3_VAL_AUTO_SOFTRESET     (1<<3)
#   define FUSB_CONTROL3_MASK_RETRIES           (0b11 << 1)
#   define FUSB_CONTROL3_VAL_RETRIES(n)         ((n&0b11)<<1) //max 3 retries!
#   define FUSB_CONTROL3_MAX_RETRIES            3
#   define FUSB_CONTROL3_VAL_AUTO_RETRY         (1<<0)
#define FUSB_REG_MASK       0x0A
#   define FUSB_MASK_VAL_M_VBUSOK               (1<<7)
#   define FUSB_MASK_VAL_M_ACTIVITY             (1<<6)
#   define FUSB_MASK_VAL_M_COMP_CHNG            (1<<5)
#   define FUSB_MASK_VAL_M_CRC_CHK              (1<<4)
#   define FUSB_MASK_VAL_M_ALERT                (1<<3)
#   define FUSB_MASK_VAL_M_WAKE                 (1<<2)
#   define FUSB_MASK_VAL_M_COLLISION            (1<<1)
#   define FUSB_MASK_VAL_M_BC_LVL               (1<<0)
#define FUSB_REG_POWER      0x0B
#   define FUSB_POWER_VAL_MASK_RESERVED         (0xF<<4)
#   define FUSB_POWER_VAL_PWR_ALL               (0xF<<0)
#   define FUSB_POWER_VAL_PWR_INTERNAL_OSC      (1<<3)
#   define FUSB_POWER_VAL_PWR_MEASURE_BLK       (1<<2)
#   define FUSB_POWER_VAL_PWR_RCV_MEASURE_REF   (1<<1)
#   define FUSB_POWER_VAL_PWR_BNDGAP_WAKE       (1<<0)
#define FUSB_REG_MASKA      0x0E
#   define FUSB_MASKA_VAL_M_OCP_TEMP            (1<<7)
#   define FUSB_MASKA_VAL_M_TOGDONE             (1<<6)
#   define FUSB_MASKA_VAL_M_SOFTFAIL            (1<<5)
#   define FUSB_MASKA_VAL_M_RETRYFAIL           (1<<4)
#   define FUSB_MASKA_VAL_M_HARDSENT            (1<<3)
#   define FUSB_MASKA_VAL_M_TXSENT              (1<<2)
#   define FUSB_MASKA_VAL_M_SOFTRST             (1<<1)
#   define FUSB_MASKA_VAL_M_HARDRST             (1<<0)
#define FUSB_REG_MASKB      0x0C
#   define FUSB_MASKB_VAL_M_GCRCSENT            (1<<0)
#define FUSB_REG_INTERRUPTA 0x3E
#   define FUSB_INTERRUPTA_VAL_I_OCP_TEMP       (1<<7)
#   define FUSB_INTERRUPTA_VAL_I_TOGDONE        (1<<6)
#   define FUSB_INTERRUPTA_VAL_I_SOFTFAIL       (1<<5)
#   define FUSB_INTERRUPTA_VAL_I_RETRYFAIL      (1<<4)
#   define FUSB_INTERRUPTA_VAL_I_HARDSENT       (1<<3)
#   define FUSB_INTERRUPTA_VAL_I_TXSENT         (1<<2)
#   define FUSB_INTERRUPTA_VAL_I_SOFTRST        (1<<1)
#   define FUSB_INTERRUPTA_VAL_I_HARDRST        (1<<0)
#define FUSB_REG_INTERRUPTB 0x3F
#   define FUSB_INTERRUPTB_VAL_I_GCRCSENT       (1<<0)
#define FUSB_REG_STATUS0    0x40
#   define FUSB_STATUS0_VAL_VBUSOK              (1<<7)
#   define FUSB_STATUS0_VAL_ACTIVITY            (1<<6)
#   define FUSB_STATUS0_VAL_COMP                (1<<5)
#   define FUSB_STATUS0_VAL_CRC_CHK             (1<<4)
#   define FUSB_STATUS0_VAL_ALERT               (1<<3)
#   define FUSB_STATUS0_VAL_WAKE                (1<<2)
#   define FUSB_STATUS0_MASK_BC_LVL             (0b11)
#define FUSB_REG_STATUS1    0x41
#   define FUSB_STATUS1_VAL_RXSOP2              (1<<7)
#   define FUSB_STATUS1_VAL_RXSOP1              (1<<6)
#   define FUSB_STATUS1_VAL_RX_EMPTY            (1<<5)
#   define FUSB_STATUS1_VAL_RX_FULL             (1<<4)
#   define FUSB_STATUS1_VAL_TX_EMPTY            (1<<3)
#   define FUSB_STATUS1_VAL_TX_FULL             (1<<2)
#   define FUSB_STATUS1_VAL_OVRTEMP             (1<<1)
#   define FUSB_STATUS1_VAL_OCP                 (1<<0)
#define FUSB_REG_INTERRUPT  0x42
#   define FUSB_INTERRUPT_VAL_I_VBUSOK          (1<<7)
#   define FUSB_INTERRUPT_VAL_I_ACTIVITY        (1<<6)
#   define FUSB_INTERRUPT_VAL_I_COMP_CHNG       (1<<5)
#   define FUSB_INTERRUPT_VAL_I_CRC_CHK         (1<<4)
#   define FUSB_INTERRUPT_VAL_I_ALERT           (1<<3)
#   define FUSB_INTERRUPT_VAL_I_WAKE            (1<<2)
#   define FUSB_INTERRUPT_VAL_I_COLLISION       (1<<1)
#   define FUSB_INTERRUPT_VAL_I_BC_LVL          (1<<0)
#define FUSB_REG_FIFOS      0x43
#define FUSB_FIFOS_TOK_TXON                     0xA1
#define FUSB_FIFOS_TOK_SOP1                     0x12
#define FUSB_FIFOS_TOK_SOP2                     0x13
#define FUSB_FIFOS_TOK_SOP3                     0x1B
#define FUSB_FIFOS_TOK_RESET1                   0x15
#define FUSB_FIFOS_TOK_RESET2                   0x16
#define FUSB_FIFOS_TOK_PACKSYM(n)               (0x80 | ((n)&0b11111))
#define FUSB_FIFOS_TOK_JAM_CRC                  0xFF
#define FUSB_FIFOS_TOK_EOP                      0x14
#define FUSB_FIFOS_TOK_TXOFF                    0xFE

#endif //FUSB_H