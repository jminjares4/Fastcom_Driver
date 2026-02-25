#ifndef FASTCOM_DEFS_H
#define FASTCOM_DEFS_H

#include <stdint.h>

/* FSCC and SuperFSCC manual: https://fastcomproducts.com/Manuals/FSCC_and_SuperFSCC_Product_Family.pdf */

/***********************************
 *  COMMAND REGISTER (CMDR)
 ***********************************/
#define FC_TIMR    (1u << 0)  /* Start Timer Command */
#define FC_STIMR   (1u << 1)  /* Stop Timer Command */
#define FC_HUNT    (1u << 16) /* Enter HUNT State Command */
#define FC_RRES    (1u << 17) /* Receiver Reset Command */
#define FC_DRC     (1u << 18) /* Detect Receive Clock */
#define FC_XF      (1u << 24) /* Transmit Frame */
#define FC_XREP    (1u << 25) /* Transmit Repeatedly */
#define FC_SXREP   (1u << 26) /* Stop Transmitting Repeatedly */
#define FC_TRES    (1u << 27) /* Transmitter Reset */
#define FC_TXT     (1u << 28) /* Transmit on Timer */
#define FC_TXEXT   (1u << 29) /* Transmit on External Signal */

/******************************************
 *  CHANNEL CONFIGURATION REGISTER 0 (CCR0)
 *****************************************/

/* Fastcom Modes */
#define FC_CCR0_HDLC_MODE        (0u << 0)        /* HDLC/SDLC Mode */
#define FC_CCR0_XSYNC_MODE       (1u << 0)        /* Character Oriented Synchronous (X-Sync) Mode */
#define FC_CCR0_TRANSPARENT_MODE (2u << 0)        /* Transparent Mode */
#define FC_CCR0_TRUE_ASYNC_MODE  (3u << 0)        /* True-Asynchronous Mode */

/* Clock Modes: 4.1.2 pg 36 */
#define FC_CCR0_CLOCK_MODE_0     (0u << 2) /* External RxClk and External TXClkIn */
#define FC_CCR0_CLOCK_MODE_1     (1u << 2) /* External RxClk and Internal BGR */
#define FC_CCR0_CLOCK_MODE_2     (2u << 2) /* External RxClk and Effective RxClk */
#define FC_CCR0_CLOCK_MODE_3     (3u << 2) /* DPLL and External RxClkIn */
#define FC_CCR0_CLOCK_MODE_4     (4u << 2) /* DPLL and Internal BGR */
#define FC_CCR0_CLOCK_MODE_5     (5u << 2) /* DPLL and Effective RxClk */
#define FC_CCR0_CLOCK_MODE_6     (6u << 2) /* Internal BGR and External TxClkIn */
#define FC_CCR0_CLOCK_MODE_7     (7u << 2) /* Internal BGR and Internal BGR (TxClkOut = BGR) */

/* Data Encoding: 4.2 pg 42 */
#define FC_ENCODE_NRZ               (0u << 5) /* Non-Return-To-Zero (NRZ) */
#define FC_ENCODE_NRZI              (1u << 5) /* Non-Return-To-Zero-Inverted (NRZI) */
#define FC_ENCODE_FM0               (2u << 5) /* Bi-Phase Space */
#define FC_ENCODE_FM1               (3u << 5) /* Bi-Phase Mark */
#define FC_ENCODE_MANCHESTER        (4u << 5) /* Bi-Phase */
#define FC_ENCODE_DIFF_MANCHESTER   (5u << 5) /* Differential Manchester */

/* Frame Sync Signals: 4.5 pg 44 */
#define FC_FST_NO_SYNC   (0u << 8) /* No Frame Sync */
#define FC_FST_MODE_1    (1u << 8) /* The sync signal completely frames the data */
#define FC_FST_MODE_2    (2u << 8) /* The sync signal is found at the beginning of data */
#define FC_FST_MODE_3    (3u << 8) /* The sync signal is found at the end of data */
#define FC_FST_MODE_4    (4u << 8) /* Both Mode 2 and Mode 3 at the same time */
#define FC_FST_MODE_5    (5u << 8) /* Similar to mode 1, except there are independent controls for both leading and trailing edge */

/* Shared Flags */
#define FC_SHARED_FLAG_DISABLE (0u << 11) /* Do not share flags */
#define FC_SHARED_FLAG_ENABLE  (1u << 11) /* Enable shared flags */

/* ITF */
#define FC_ITF_LOGICAL_1 (0u << 12) /* Continuous logical '1' is sent during idle periods */
#define FC_ITF_SYNC      (1u << 12) /* Continuous SYNC sequences are sent during idle periods */

/* Number of Sync Bytes */
#define FC_NSB_MASK      0x0000E000u
#define FC_NSB_NO_SYNC   (0u << 13) /* No synchronization bytes used (Transparent Mode) */
#define FC_NSB_1         (1u << 13) /* One byte */
#define FC_NSB_2         (2u << 13) /* Two bytes */
#define FC_NSB_3         (3u << 13) /* Three bytes */
#define FC_NSB_4         (4u << 13) /* Four bytes */

/* Number of Termination Bytes */
#define FC_NTB_MASK      0x00070000u
#define FC_NTB_NO_TERM   (0u << 16) /* No termination bytes used (Transparent Mode) */
#define FC_NTB_1         (1u << 16) /* One byte */
#define FC_NTB_2         (2u << 16) /* Two bytes */
#define FC_NTB_3         (3u << 16) /* Three bytes */
#define FC_NTB_4         (4u << 16) /* Four bytes */

/* Visible Mask */
#define FC_VIS_NO_VISIBLE (0u << 19) /* Masked interrupt status bits not visible */
#define FC_VIS_VISIBLE    (1u << 19) /* Masked interrupt status bits are visible */

/* CRC Frame Mode */
#define FC_CRC_MODE_8    (0u << 20) /* Use CRC-8 algorithm */
#define FC_CRC_MODE_16   (2u << 20) /* Use CRC-16 algorithm */
#define FC_CRC_MODE_32   (3u << 20) /* Use CRC-32 algorithm */

/* Order Bit Transmission */
#define FC_OBT_LSB       (0u << 22) /* LSB first */
#define FC_OBT_MSB       (1u << 22) /* MSB first */

/* Address Mode: HDLC ONLY */
#define FC_ADM_MODE_0    (0u << 23) /* No address checking */
#define FC_ADM_MODE_1    (1u << 23) /* 1 byte address checking */
#define FC_ADM_MODE_2    (2u << 23) /* 2 byte address checking */

/* Receiver Disable */
#define FC_RECD_ENABLE   (0u << 25) /* Receiver enabled */
#define FC_RECD_DISABLE  (1u << 25) /* Receiver disabled */

/* External Signal Select */
#define FC_EXTS_CTS      (0u << 28) /* CTS selected */
#define FC_EXTS_DSR      (1u << 28) /* DSR selected */
#define FC_EXTS_RI       (2u << 28) /* RI selected */
#define FC_EXTS_CD       (3u << 28) /* CD selected */

/******************************************
 *  CHANNEL CONFIGURATION REGISTER 1 (CCR1)
 *****************************************/
#define FC_RTS      (1u << 0)  /* Request to Send Pin State */
#define FC_DTR      (1u << 1)  /* Data Terminal Ready Pin State */
#define FC_RTSC     (1u << 2)  /* RTS Control */
#define FC_CTSC     (1u << 3)  /* CTS Control */
#define FC_ZINS     (1u << 4)  /* Zero Insertion Control */
#define FC_OINS     (1u << 5)  /* Ones Insertion Control */
#define FC_DPS      (1u << 6)  /* DPLL Phase Shift Disable */
#define FC_SYNC2F   (1u << 7)  /* Transfer SYNC character(s) to RxFIFO (X-Sync mode only) */
#define FC_TERM2F   (1u << 8)  /* Transfer TERM character(s) to RxFIFO (X-Sync mode only) */
#define FC_ADD2F    (1u << 9)  /* Address byte(s) to RxFIFO (HDLC mode only) */
#define FC_CRC2F    (1u << 10) /* CRC bytes to RxFIFO */
#define FC_CRCR     (1u << 11) /* CRC reset level */
#define FC_DRCRC    (1u << 12) /* Disable receive CRC checking */
#define FC_DTCRC    (1u << 13) /* Disable transmit CRC insertion */
#define FC_DTERM    (1u << 14) /* Disable appending TERM to transmit data */
#define FC_DSYNC    (1u << 15) /* Disable appending SYNC to transmit data */

#define FC_RIP_INV  (1u << 16) /* RI pin polarity */
#define FC_CDP_INV  (1u << 17) /* CD pin polarity */
#define FC_RTSP_INV (1u << 18) /* RTS pin polarity */
#define FC_CTSP_INV (1u << 19) /* CTS pin polarity */
#define FC_DTRP_INV (1u << 20) /* DTR pin polarity */
#define FC_DSRP_INV (1u << 21) /* DSR pin polarity */
#define FC_FSRP_INV (1u << 22) /* Frame Sync Receive pin polarity */
#define FC_FSTP_INV (1u << 23) /* Frame Sync Transmit pin polarity */
#define FC_TCOP_INV (1u << 24) /* Transmit Clock Output pin polarity */
#define FC_TCIP_INV (1u << 25) /* Transmit Clock Input pin polarity */
#define FC_RCP_INV  (1u << 26) /* RxClk pin polarity */
#define FC_TDP_INV  (1u << 27) /* Transmit Data pin polarity */
#define FC_RDP_INV  (1u << 28) /* Receive Data pin polarity */

/******************************************
 *  CHANNEL CONFIGURATION REGISTER 2 (CCR2)
 *****************************************/
#define FC_CCR2_FSRO_MASK   0x0000000Fu
#define FC_CCR2_FSRO_0      (0u << 0)
#define FC_CCR2_FSRO_1      (1u << 0)
#define FC_CCR2_FSRO_2      (2u << 0)
#define FC_CCR2_FSRO_3      (3u << 0)
#define FC_CCR2_FSRO_4      (4u << 0)
#define FC_CCR2_FSRO_5      (5u << 0)
#define FC_CCR2_FSRO_6      (6u << 0)
#define FC_CCR2_FSRO_7      (7u << 0)
#define FC_CCR2_FSRO_M1     (15u << 0) /* -1 clock (two's complement) */

#define FC_CCR2_FSTOL_MASK  0x000000F0u
#define FC_CCR2_FSTOL_0     (0u << 4)
#define FC_CCR2_FSTOL_1     (1u << 4)
#define FC_CCR2_FSTOL_7     (7u << 4)
#define FC_CCR2_FSTOL_M1    (15u << 4)

#define FC_CCR2_FSTOT_MASK  0x00000F00u
#define FC_CCR2_FSTOT_0     (0u << 8)
#define FC_CCR2_FSTOT_1     (1u << 8)
#define FC_CCR2_FSTOT_7     (7u << 8)
#define FC_CCR2_FSTOT_M1    (15u << 8)

#define FC_CCR2_RLC_MASK    0xFFFF0000u
#define FC_CCR2_RLC_0       (0u << 16) /* disabled */
#define FC_CCR2_RLC_65535   (65535u << 16) /* max */

/******************************************
 * TIMER CONTROL REGISTER (TCR)
 *****************************************/
/* Timer Clock Source Select */
#define FC_TSRC_OSC     (0u << 0) /* Default. OSC input (the clock that drives BGR) */
#define FC_TSRC_TXCLK   (1u << 0) /* Effective transmit clock */
#define FC_TSRC_RXCLK   (2u << 0) /* Effective receive clock */
#define FC_TSRC_PCI     (3u << 0) /* PCI bus clock */

/* Timer Trigger Select */
#define FC_TTRIG_RECYCLE  (0u << 2)  /* Timer recycles and triggers an interrupt each time it expires */
#define FC_TTRIG_ONE_SHOT (1u << 2)  /* Timer will count down to zero and trigger an interrupt one time */

/* Timer Expiration Count */
#define FC_TCNT(X)      (((X) & 0x1FFFFFFFu) << 3)

/******************************************
 * DPLL RESET CONTROL REGISTER (DPLLR)
 *****************************************/
#define FC_DPLLR_VALUE(X)  ((X) & 0x1FFu)

/******************************************
 * FEATURE CONTROL REGISTER (FCR) – BAR2 0x00
 *****************************************/
#define FC_FCR_DTA_A     (1u << 0)
#define FC_FCR_CLK_A     (1u << 1)
#define FC_FCR_STRB_A    (1u << 3)
#define FC_FCR_DTA_B     (1u << 8)
#define FC_FCR_CLK_B     (1u << 9)
#define FC_FCR_STRB_B    (1u << 11)

#define FC_FCR_RECHOA    (1u << 16)
#define FC_FCR_TC485_A   (1u << 17)
#define FC_FCR_TD485_A   (1u << 18)
#define FC_FCR_FSTDTRA   (1u << 19)   /* 0 = FST strobe, 1 = DTR */
#define FC_FCR_RECHOB    (1u << 20)
#define FC_FCR_TC485_B   (1u << 21)
#define FC_FCR_TD485_B   (1u << 22)
#define FC_FCR_FSTDTRB   (1u << 23)

#define FC_FCR_UART_A    (1u << 24)   /* ASYNC driver control for port A */
#define FC_FCR_UART_B    (1u << 25)   /* ASYNC driver control for port B */

/* ================================================================
 * DMA (SuperFSCC)
 * ================================================================ */
#define FC_DMACCR_ENABLE    0x03000000u

#endif /* FASTCOM_DEFS_H */