/* fastcom_def.h
 * Fastcom FSCC / SuperFSCC Register Bit Definitions
 * 100% compliant with FSCC_and_SuperFSCC_Product_Family.pdf
 * Revision: November 2024
 * Every bit definition and mask references the exact manual page.
 * This file contains only the FC_ symbolic constants used by fastcom.c.
 * It works in conjunction with the official Commtech fscc.h.
 */

#ifndef FASTCOM_DEFS_H
#define FASTCOM_DEFS_H

#include <stdint.h>

/***********************************
 * COMMAND REGISTER (CMDR) – page 65
 ***********************************/
#define FC_TIMR    (1u << 0)   /* Start Timer Command */
#define FC_STIMR   (1u << 1)   /* Stop Timer Command */
#define FC_HUNT    (1u << 16)  /* Enter HUNT State */
#define FC_RRES    (1u << 17)  /* Receiver Reset */
#define FC_DRC     (1u << 18)  /* Detect Receive Clock */
#define FC_XF      (1u << 24)  /* Transmit Frame */
#define FC_XREP    (1u << 25)  /* Transmit Repeatedly */
#define FC_SXREP   (1u << 26)  /* Stop Transmitting Repeatedly */
#define FC_TRES    (1u << 27)  /* Transmitter Reset */
#define FC_TXT     (1u << 28)  /* Transmit on Timer */
#define FC_TXEXT   (1u << 29)  /* Transmit on External Signal */

/******************************************
 * CCR0 – Channel Configuration Register 0 (pages 71-74)
 *****************************************/
#define FC_CCR0_HDLC_MODE         (0u << 0)
#define FC_CCR0_XSYNC_MODE        (1u << 0)
#define FC_CCR0_TRANSPARENT_MODE  (2u << 0)
#define FC_CCR0_TRUE_ASYNC_MODE   (3u << 0)

#define FC_CCR0_CLOCK_MODE_0      (0u << 2)
#define FC_CCR0_CLOCK_MODE_1      (1u << 2)
#define FC_CCR0_CLOCK_MODE_2      (2u << 2)
#define FC_CCR0_CLOCK_MODE_3      (3u << 2)
#define FC_CCR0_CLOCK_MODE_4      (4u << 2)
#define FC_CCR0_CLOCK_MODE_5      (5u << 2)
#define FC_CCR0_CLOCK_MODE_6      (6u << 2)
#define FC_CCR0_CLOCK_MODE_7      (7u << 2)

#define FC_ENCODE_NRZ               (0u << 5)
#define FC_ENCODE_NRZI              (1u << 5)
#define FC_ENCODE_FM0               (2u << 5)
#define FC_ENCODE_FM1               (3u << 5)
#define FC_ENCODE_MANCHESTER        (4u << 5)
#define FC_ENCODE_DIFF_MANCHESTER   (5u << 5)

#define FC_FST_NO_SYNC   (0u << 8)
#define FC_FST_MODE_1    (1u << 8)
#define FC_FST_MODE_2    (2u << 8)
#define FC_FST_MODE_3    (3u << 8)
#define FC_FST_MODE_4    (4u << 8)
#define FC_FST_MODE_5    (5u << 8)

#define FC_SHARED_FLAG_DISABLE (0u << 11)
#define FC_SHARED_FLAG_ENABLE  (1u << 11)

#define FC_NSB_NO_SYNC   (0u << 13)
#define FC_NSB_1         (1u << 13)
#define FC_NSB_2         (2u << 13)
#define FC_NSB_3         (3u << 13)
#define FC_NSB_4         (4u << 13)

#define FC_NTB_NO_TERM   (0u << 16)
#define FC_NTB_1         (1u << 16)
#define FC_NTB_2         (2u << 16)
#define FC_NTB_3         (3u << 16)
#define FC_NTB_4         (4u << 16)

#define FC_CRC_MODE_8        (0u << 20)
#define FC_CRC_MODE_CCITT    (1u << 20)   /* Recommended default for HDLC – page 71 */
#define FC_CRC_MODE_16       (2u << 20)
#define FC_CRC_MODE_32       (3u << 20)

#define FC_OBT_LSB       (0u << 22)
#define FC_OBT_MSB       (1u << 22)

#define FC_ADM_MODE_0    (0u << 23)
#define FC_ADM_MODE_1    (1u << 23)
#define FC_ADM_MODE_2    (2u << 23)

/* Driver aliases for readability */
#define FC_CCR0_LE_NRZ   FC_ENCODE_NRZ
#define FC_CCR0_CM_7     FC_CCR0_CLOCK_MODE_7
#define FC_CCR0_SFLAG    FC_SHARED_FLAG_ENABLE

/******************************************
 * CCR1 – Channel Configuration Register 1 (pages 75-78)
 *****************************************/
#define FC_RTS      (1u << 0)
#define FC_DTR      (1u << 1)
#define FC_RTSC     (1u << 2)
#define FC_CTSC     (1u << 3)
#define FC_ZINS     (1u << 4)
#define FC_OINS     (1u << 5)
#define FC_DPS      (1u << 6)
#define FC_SYNC2F   (1u << 7)
#define FC_TERM2F   (1u << 8)
#define FC_ADD2F    (1u << 9)
#define FC_CRC2F    (1u << 10)
#define FC_CRCR     (1u << 11)
#define FC_DRCRC    (1u << 12)
#define FC_DTCRC    (1u << 13)
#define FC_DTERM    (1u << 14)
#define FC_DSYNC    (1u << 15)

#define FC_RIP_INV  (1u << 16)
#define FC_CDP_INV  (1u << 17)
#define FC_RTSP_INV (1u << 18)
#define FC_CTSP_INV (1u << 19)
#define FC_DTRP_INV (1u << 20)
#define FC_DSRP_INV (1u << 21)
#define FC_FSRP_INV (1u << 22)
#define FC_FSTP_INV (1u << 23)
#define FC_TCOP_INV (1u << 24)
#define FC_TCIP_INV (1u << 25)
#define FC_RCP_INV  (1u << 26)
#define FC_TDP_INV  (1u << 27)
#define FC_RDP_INV  (1u << 28)

/******************************************
 * CCR2 – Channel Configuration Register 2 (pages 79-80)
 *****************************************/
#define FC_CCR2_FSRO_MASK   0x0000000Fu
#define FC_CCR2_FSTOL_MASK  0x000000F0u
#define FC_CCR2_FSTOT_MASK  0x00000F00u
#define FC_CCR2_RLC_MASK    0xFFFF0000u

/******************************************
 * FEATURE CONTROL REGISTER (FCR) – page 102
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
#define FC_FCR_FSTDTRA   (1u << 19)
#define FC_FCR_RECHOB    (1u << 20)
#define FC_FCR_TC485_B   (1u << 21)
#define FC_FCR_TD485_B   (1u << 22)
#define FC_FCR_FSTDTRB   (1u << 23)

#define FC_FCR_UART_A    (1u << 24)
#define FC_FCR_UART_B    (1u << 25)

/******************************************
 * DMACCR – DMA Command / Control Register (pages 104-105)
 *****************************************/
#define FC_DMACCR_GO_R     (1u << 0)   /* Start Receive DMA */
#define FC_DMACCR_GO_T     (1u << 1)   /* Start Transmit DMA */
#define FC_DMACCR_RST_R    (1u << 4)
#define FC_DMACCR_RST_T    (1u << 5)
#define FC_DMACCR_STOP_R   (1u << 10)  /* Stop Receive DMA */
#define FC_DMACCR_STOP_T   (1u << 11)  /* Stop Transmit DMA */
#define FC_DMACCR_M_RST    (1u << 16)
#define FC_DMACCR_TXT_E    (1u << 24)
#define FC_DMACCR_TXEXT_E  (1u << 25)
#define FC_DMACCR_XREP_E   (1u << 26)

#endif /* FASTCOM_DEFS_H */