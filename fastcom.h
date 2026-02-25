#ifndef FASTCOM_H
#define FASTCOM_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "fastcom_def.h"
#include "calculate-clock-bits.h"
#include "fscc.h"
#include "serialfc.h"

// ================================================================
// COMMON CLOCK + STROBE
// ================================================================
typedef struct {
    uint32_t freq_hz;
    uint32_t fst_mode;
    int rx_offset;
    int tx_lead_offset;
    int tx_trail_offset;
} fc_clock_strobe_t;

// ================================================================
// MODE 0: HDLC – FULL CONFIG
// ================================================================
typedef struct {
    fc_clock_strobe_t clock_strobe;
    uint32_t encoding;
    uint32_t crc_mode;
    bool rx_crc_check;
    bool tx_crc_insert;
    uint32_t bit_order;
    uint32_t address_mode;
    bool shared_flag;
    uint32_t ssr;
    uint32_t smr;
    uint32_t tsr;
    uint32_t tmr;
    uint32_t rar;
    uint32_t ramr;
    uint32_t ppr;
    uint32_t tcr;
    uint32_t dpllr;
} fc_hdlc_config_t;

// ================================================================
// MODE 1: X-SYNC – FULL CONFIG
// ================================================================
typedef struct {
    fc_clock_strobe_t clock_strobe;
    uint32_t nsb;
    uint32_t ntb;
    uint32_t itf;
    uint32_t ssr;
    uint32_t smr;
    uint32_t tsr;
    uint32_t tmr;
} fc_xsync_config_t;

// ================================================================
// MODE 2: TRANSPARENT – FULL CONFIG
// ================================================================
typedef struct {
    fc_clock_strobe_t clock_strobe;
    uint32_t rlc;
} fc_transparent_config_t;

// ================================================================
// MODE 3: TRUE-ASYNC – FULL CONFIG
// ================================================================
typedef struct {
    int data_bits;
    char parity;
    int stop_bits;
    uint32_t baud;
} fc_true_async_config_t;

// ================================================================
// CUSTOM: TRANSPARENT WITH ASYNC FRAMING
// ================================================================
typedef struct {
    fc_clock_strobe_t clock_strobe;
    uint32_t rlc;
} fc_transparent_async_framing_config_t;

// ================================================================
// API
// ================================================================
typedef struct fastcom_port fastcom_port_t;

fastcom_port_t* fastcom_open(unsigned port_number);
void fastcom_close(fastcom_port_t* port);

int fastcom_setup_hdlc(fastcom_port_t* port, const fc_hdlc_config_t* cfg);
int fastcom_setup_xsync(fastcom_port_t* port, const fc_xsync_config_t* cfg);
int fastcom_setup_transparent(fastcom_port_t* port, const fc_transparent_config_t* cfg);
int fastcom_setup_true_async(fastcom_port_t* port, const fc_true_async_config_t* cfg);
int fastcom_setup_transparent_async_framing(fastcom_port_t* port, const fc_transparent_async_framing_config_t* cfg);

// SuperFSCC DMA Support
int fastcom_enable_dma(fastcom_port_t* port);
int fastcom_set_memory_cap(fastcom_port_t* port, int input, int output);

int fastcom_tx_frame(fastcom_port_t* port, const void* data, size_t len);
ssize_t fastcom_rx_frame(fastcom_port_t* port, void* buf, size_t max_len, bool wait);

int fastcom_purge(fastcom_port_t* port, bool tx, bool rx);

#endif