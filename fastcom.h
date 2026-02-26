/* fastcom.h
 * Fastcom FSCC / SuperFSCC User-Space Driver Header
 * 100% compliant with FSCC_and_SuperFSCC_Product_Family.pdf
 * Revision: November 2024
 * This header works directly with the official Commtech fscc.h and calculate-clock-bits.h.
 * It declares all public API functions and configuration structures.
 */

#ifndef FASTCOM_H
#define FASTCOM_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "fastcom_def.h"
#include "fscc.h"          /* Official Commtech header – provides struct fscc_registers, ioctl definitions, and FSCC_REGISTERS_INIT */

typedef struct fastcom_port fastcom_port_t;

/* ================================================================
 * EASY ONE-LINE API
 * ================================================================ */
int fastcom_configure_clocked_transparent(fastcom_port_t* p, uint32_t bitrate_hz, uint32_t fst_mode);
int fastcom_configure_transparent(fastcom_port_t* p, uint32_t bitrate_hz);
int fastcom_configure_true_async(fastcom_port_t* p, uint32_t baud);
int fastcom_configure_hdlc(fastcom_port_t* p, uint32_t bitrate_hz);
int fastcom_configure_xsync(fastcom_port_t* p, uint32_t bitrate_hz);

/* ================================================================
 * CONFIGURATION STRUCTURES
 * ================================================================ */

typedef struct {
    uint32_t bitrate_hz;
    uint32_t clock_mode;          /* 0-7 – page 36 */
    uint32_t encoding;            /* page 42 */
    uint32_t fst_mode;            /* page 44 */
    int8_t   fst_receive_offset;  /* FSRO – page 79 */
    int8_t   fst_lead_offset;     /* FSTOL – page 79 */
    int8_t   fst_trail_offset;    /* FSTOT – page 79 */
    uint32_t rlc;                 /* Receive Length Check – pages 44, 79 */
    bool     invert_txclkout;
    bool     invert_fst;
    bool     rs485_txclkout;      /* TC485 bits – page 102 */
    uint32_t extra_ccr1;
    bool     use_dma;             /* SuperFSCC DMA flag – pages 28, 104 */
} fc_clocked_transparent_config_t;

void fc_clocked_transparent_default(fc_clocked_transparent_config_t* cfg, uint32_t bitrate_hz);

typedef struct {
    uint32_t bitrate_hz;
    uint32_t encoding;
    uint32_t crc_mode;            /* defaults to CCITT – page 71 */
    bool     shared_flag;
    uint32_t bit_order;
    uint32_t address_mode;
    uint32_t ssr, smr, tsr, tmr;
    uint32_t rar, ramr, ppr;
    uint32_t extra_ccr1;
    bool     use_dma;
} fc_hdlc_config_t;

void fc_hdlc_default(fc_hdlc_config_t* cfg, uint32_t bitrate_hz);

typedef struct {
    uint32_t bitrate_hz;
    uint32_t nsb, ntb;
    uint32_t ssr, smr, tsr, tmr;
    uint32_t extra_ccr1;
    bool     use_dma;
} fc_xsync_config_t;

void fc_xsync_default(fc_xsync_config_t* cfg, uint32_t bitrate_hz);

typedef struct {
    uint32_t bitrate_hz;
    uint32_t rlc;
    bool     use_dma;
} fc_transparent_config_t;

void fc_transparent_default(fc_transparent_config_t* cfg, uint32_t bitrate_hz);

typedef struct {
    uint32_t baud;
    bool     use_dma;   /* ignored in True-Async mode – page 60 */
} fc_true_async_config_t;

void fc_true_async_default(fc_true_async_config_t* cfg, uint32_t baud);

/* ================================================================
 * CORE API
 * ================================================================ */
fastcom_port_t* fastcom_open(unsigned port_number);   /* ports 0-3 */
void fastcom_close(fastcom_port_t* p);

int fastcom_setup_clocked_transparent(fastcom_port_t* p, const fc_clocked_transparent_config_t* cfg);
int fastcom_setup_hdlc(fastcom_port_t* p, const fc_hdlc_config_t* cfg);
int fastcom_setup_xsync(fastcom_port_t* p, const fc_xsync_config_t* cfg);
int fastcom_setup_transparent(fastcom_port_t* p, const fc_transparent_config_t* cfg);
int fastcom_setup_true_async(fastcom_port_t* p, const fc_true_async_config_t* cfg);

int fastcom_tx_frame(fastcom_port_t* p, const void* data, size_t len);
ssize_t fastcom_rx_frame(fastcom_port_t* p, void* buf, size_t max_len, bool wait);
int fastcom_purge(fastcom_port_t* p, bool tx, bool rx);

/* SuperFSCC DMA API – DMACCR register (pages 104-105) */
int fastcom_enable_dma(fastcom_port_t* p);
int fastcom_disable_dma(fastcom_port_t* p);
int fastcom_set_memory_cap(fastcom_port_t* p, int input_bytes, int output_bytes);

int fastcom_set_registers(fastcom_port_t* p, struct fscc_registers* regs);

#endif