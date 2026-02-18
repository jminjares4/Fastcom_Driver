/*
 * fastcom.h
 * Fastcom FSCC / SuperFSCC High-Level User-Space API
 * 100% coverage of the 112-page PDF + official Commtech kernel driver
 */

#ifndef FASTCOM_H
#define FASTCOM_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "fastcom_defs.h"
#include "fscc.h"                    /* Official Commtech kernel header */

typedef struct fastcom_handle *fastcom_t;

/* Configuration structure (1:1 with PDF registers) */
typedef struct {
    int mode;                    /* FASTCOM_MODE_* */
    int clock_mode;              /* FASTCOM_CLOCK_MODE_* */
    int physical;                /* FASTCOM_PHYS_* */

    unsigned long baud_rate;           /* 0 = external clock */
    unsigned long external_clock_hz;   /* 0 = internal 50 MHz */

    bool dpll_enable;
    bool dsync;

    unsigned encoding;
    unsigned crc;

    bool zero_bit_insertion;
    bool one_bit_insertion;
    bool shared_flag;

    /* Async UART only */
    int data_bits;   /* 7 or 8 */
    int parity;      /* FASTCOM_PARITY_* */
    int stop_bits;   /* FASTCOM_STOP_BITS_* */

    /* Frame formatting */
    uint32_t preamble;
    uint8_t  preamble_length;
    uint32_t postamble;
    uint8_t  postamble_length;
    uint8_t  interframe_fill;
    bool     invert_data;
    bool     invert_clock;

    unsigned tx_modifiers;   /* FASTCOM_TX_* */

    /* High-level RX features */
    bool append_status;
    bool append_timestamp;
    bool rx_multiple_frames;

    /* Sync / Address registers */
    uint32_t sync_sequence, sync_mask;
    uint32_t termination_sequence, termination_mask;
    uint32_t receive_address, receive_address_mask;
    uint32_t timer_control;

    int bit_order;
    uint16_t receive_length_check;

    /* FRAME SYNC */
    int  frame_sync_mode;                     /* FASTCOM_FSC_* */
    int8_t frame_sync_rx_offset;              /* -8..+7 */
    int8_t frame_sync_tx_offset_leading;      /* -8..+7 */
    int8_t frame_sync_tx_offset_trailing;     /* -8..+7 (Mode 5) */

    bool use_tx_frame_sync;                   /* Use hardware FS pin for TX */

    /* Modem polarities */
    bool dsr_active_high;
    bool fsr_active_high;
    bool fst_active_high;

    /* DMA tuning for 50 Mbit/s sustained */
    int input_buffer_frames;
    int output_buffer_frames;

    /* Extra bits from PDF p71, p75 */
    uint8_t num_sync_bytes;   /* 0-7 */
    uint8_t num_term_bytes;   /* 0-7 */
    bool    vis;
    bool    recd;
    bool    dterm;
    bool    dtcrc;
    bool    drcrc;
    bool    crcr;
    bool    dps;
    bool    manual_rts;
    bool    rts_asserted;
    bool    dtr_asserted;
} fastcom_config_t;

/* PUBLIC API */
fastcom_t fastcom_open(int port_number);   /* 0..3 for SuperFSCC/4-PCIe */

int fastcom_setup(fastcom_t dev, const fastcom_config_t *cfg);

ssize_t fastcom_write(fastcom_t dev, const void *buf, size_t len);
ssize_t fastcom_read(fastcom_t dev, void *buf, size_t len);
ssize_t fastcom_read_one_frame(fastcom_t dev, void *payload_buf, size_t payload_buf_size,
                               uint64_t *timestamp_out, uint16_t *status_out);

int fastcom_purge(fastcom_t dev, bool purge_tx, bool purge_rx);
int fastcom_set_memory_cap(fastcom_t dev, int input_frames, int output_frames);

/* Raw register access (100% coverage) */
int fastcom_get_registers(fastcom_t dev, struct fscc_registers *regs);
int fastcom_set_registers(fastcom_t dev, const struct fscc_registers *regs);

uint32_t fastcom_get_status(fastcom_t dev);      /* STAR */
uint32_t fastcom_get_version(fastcom_t dev);     /* VSTR */
uint32_t fastcom_get_dma_status(fastcom_t dev);  /* DSTAR */

int fastcom_set_rts(fastcom_t dev, bool asserted);
int fastcom_get_cts(fastcom_t dev);
int fastcom_enable_tx_frame_sync(fastcom_t dev, bool enable);

void fastcom_close(fastcom_t dev);

#endif