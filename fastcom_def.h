/*
 * fastcom_defs.h
 * 100% coverage of every bit and mode from the official 112-page PDF
 * (dated 11/12/2024, revision 1.8)
 */

#ifndef FASTCOM_DEFS_H
#define FASTCOM_DEFS_H

/* PROTOCOL MODES (PDF p71 CCR0[1:0]) */
#define FASTCOM_MODE_HDLC         0
#define FASTCOM_MODE_XSYNC        1
#define FASTCOM_MODE_TRANSPARENT  2
#define FASTCOM_MODE_ASYNC_UART   3

/* CLOCK MODES (PDF p36 CCR0[4:2]) */
#define FASTCOM_CLOCK_MODE_EXT_RXC_EXT_TXC     0
#define FASTCOM_CLOCK_MODE_EXT_RXC_BGR_TXC     1
#define FASTCOM_CLOCK_MODE_RXC_BOTH            2
#define FASTCOM_CLOCK_MODE_DPLL_RX_EXT_TXC     3
#define FASTCOM_CLOCK_MODE_DPLL_RX_BGR16_TXC   4
#define FASTCOM_CLOCK_MODE_DPLL_BOTH           5
#define FASTCOM_CLOCK_MODE_BGR_RX_EXT_TXC      6
#define FASTCOM_CLOCK_MODE_BGR_BOTH            7

/* DATA ENCODING (PDF p42-43 CCR0[7:5]) */
#define FASTCOM_ENCODE_NRZ              (0u << 5)
#define FASTCOM_ENCODE_NRZI             (1u << 5)
#define FASTCOM_ENCODE_FM0              (2u << 5)
#define FASTCOM_ENCODE_FM1              (3u << 5)
#define FASTCOM_ENCODE_MANCHESTER       (4u << 5)
#define FASTCOM_ENCODE_DIFF_MANCHESTER  (5u << 5)

/* CRC OPTIONS (PDF p71 CCR0[21:20]) */
#define FASTCOM_CRC_NONE (0u << 20)
#define FASTCOM_CRC_8    (1u << 20)
#define FASTCOM_CRC_16   (2u << 20)
#define FASTCOM_CRC_32   (3u << 20)

/* CCR0 FLAGS (PDF p71-72) */
#define FASTCOM_CCR0_SFLAG (1u << 11)
#define FASTCOM_CCR0_ITF   (1u << 12)
#define FASTCOM_CCR0_OBT   (1u << 22)
#define FASTCOM_CCR0_VIS   (1u << 19)
#define FASTCOM_CCR0_RECD  (1u << 25)

#define FASTCOM_NSB(n)     (((uint32_t)(n) & 0x7) << 13)
#define FASTCOM_NTB(n)     (((uint32_t)(n) & 0x7) << 16)

/* CCR1 FLAGS (PDF p75-78) */
#define FASTCOM_CCR1_ZINS   (1u << 4)
#define FASTCOM_CCR1_OINS   (1u << 5)
#define FASTCOM_CCR1_DPS    (1u << 6)
#define FASTCOM_CCR1_DSYNC  (1u << 15)
#define FASTCOM_CCR1_DTERM  (1u << 14)
#define FASTCOM_CCR1_DTCRC  (1u << 13)
#define FASTCOM_CCR1_DRCRC  (1u << 12)
#define FASTCOM_CCR1_CRCR   (1u << 11)
#define FASTCOM_CCR1_RDP    (1u << 28)
#define FASTCOM_CCR1_TDP    (1u << 27)
#define FASTCOM_CCR1_RCP    (1u << 26)
#define FASTCOM_CCR1_TCP    (1u << 25)
#define FASTCOM_CCR1_DSRP   (1u << 23)
#define FASTCOM_CCR1_FSRP   (1u << 22)
#define FASTCOM_CCR1_FSTP   (1u << 21)
#define FASTCOM_CCR1_RTSC   (1u << 2)
#define FASTCOM_CCR1_CTSC   (1u << 3)
#define FASTCOM_CCR1_RTS    (1u << 0)
#define FASTCOM_CCR1_DTR    (1u << 1)

/* FRAME SYNC (PDF p44-46, p72, p79) */
#define FASTCOM_FSC_NO_SYNC           0
#define FASTCOM_FSC_MODE1             1
#define FASTCOM_FSC_MODE2             2
#define FASTCOM_FSC_MODE3             3
#define FASTCOM_FSC_MODE4             4
#define FASTCOM_FSC_MODE5_TX_ENABLE   5

#define FASTCOM_FS_OFFSET(val) ((uint32_t)((val) & 0xF))

/* PHYSICAL INTERFACE */
#define FASTCOM_PHYS_RS422  0
#define FASTCOM_PHYS_RS485  1
#define FASTCOM_PHYS_RS232  2

/* ASYNC UART ONLY */
#define FASTCOM_PARITY_NONE 0
#define FASTCOM_PARITY_EVEN 1
#define FASTCOM_PARITY_ODD  2
#define FASTCOM_STOP_BITS_1 1
#define FASTCOM_STOP_BITS_2 2

/* TRANSMIT MODIFIERS */
#define FASTCOM_TX_NORMAL   0
#define FASTCOM_TX_REPEAT   1
#define FASTCOM_TX_ON_TIMER 2
#define FASTCOM_TX_EXTERNAL 4

/* BIT ORDER */
#define FASTCOM_BIT_ORDER_LSB_FIRST 0
#define FASTCOM_BIT_ORDER_MSB_FIRST 1

#endif