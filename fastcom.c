#include "fastcom.h"
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>

struct fastcom_port {
    int fd;
    unsigned number;
    bool dma_enabled;
};

fastcom_port_t* fastcom_open(unsigned port_number)
{
    char path[32];
    fastcom_port_t* p = calloc(1, sizeof(*p));
    if (!p) return NULL;
    sprintf(path, "/dev/fscc%u", port_number);
    p->fd = open(path, O_RDWR);
    if (p->fd < 0) { free(p); return NULL; }
    p->number = port_number;
    p->dma_enabled = false;
    return p;
}

void fastcom_close(fastcom_port_t* p) { if (p) { close(p->fd); free(p); } }

static int reg_get(fastcom_port_t* p, struct fscc_registers* r)
{
    FSCC_REGISTERS_INIT(*r);
    return ioctl(p->fd, FSCC_GET_REGISTERS, r) == 0 ? 0 : -1;
}

static int reg_set(fastcom_port_t* p, struct fscc_registers* r)
{
    return ioctl(p->fd, FSCC_SET_REGISTERS, r) == 0 ? 0 : -1;
}

// ================================================================
// COMMON CLOCK + STROBE
// ================================================================
static int apply_clock_strobe(fastcom_port_t* p, const fc_clock_strobe_t* cs)
{
    unsigned char bits[20];
    if (calculate_clock_bits(cs->freq_hz, 10, bits) != 0) return -1;
    ioctl(p->fd, FSCC_SET_CLOCK_BITS, bits);

    struct fscc_registers r;
    reg_get(p, &r);
    r.CCR0 = (r.CCR0 & ~0x1C) | FC_CCR0_CLOCK_MODE_7;
    r.CCR0 = (r.CCR0 & ~FC_FST_MODE_MASK) | (cs->fst_mode << 8);
    r.CCR2 = (cs->rx_offset & 0xF) | ((cs->tx_lead_offset & 0xF)<<4) | ((cs->tx_trail_offset & 0xF)<<8);

    if (p->number % 2 == 0) r.FCR &= ~FC_FCR_FSTDTRA;
    else                    r.FCR &= ~FC_FCR_FSTDTRB;

    return reg_set(p, &r);
}

// ================================================================
// MODE SETUP – COMPLETE CONFIGURATION
// ================================================================
int fastcom_setup_hdlc(fastcom_port_t* p, const fc_hdlc_config_t* cfg)
{
    fastcom_set_mode(p, FC_MODE_HDLC);
    apply_clock_strobe(p, &cfg->clock_strobe);

    struct fscc_registers r;
    reg_get(p, &r);

    r.CCR0 = (r.CCR0 & ~0xE0) | cfg->encoding;
    r.CCR0 = (r.CCR0 & ~0x300000) | (cfg->crc_mode << 20);
    r.CCR0 = (r.CCR0 & ~(1u<<22)) | (cfg->bit_order << 22);
    r.CCR0 = (r.CCR0 & ~(3u<<23)) | (cfg->address_mode << 23);
    if (cfg->shared_flag) r.CCR0 |= FC_SHARED_FLAG_ENABLE;

    if (!cfg->rx_crc_check) r.CCR1 |= FC_DRCRC;
    if (!cfg->tx_crc_insert) r.CCR1 |= FC_DTCRC;

    r.SSR = cfg->ssr;
    r.SMR = cfg->smr;
    r.TSR = cfg->tsr;
    r.TMR = cfg->tmr;
    r.RAR = cfg->rar;
    r.RAMR = cfg->ramr;
    r.PPR = cfg->ppr;
    r.TCR = cfg->tcr;
    r.DPLLR = cfg->dpllr;

    return reg_set(p, &r);
}

int fastcom_setup_xsync(fastcom_port_t* p, const fc_xsync_config_t* cfg)
{
    fastcom_set_mode(p, FC_MODE_XSYNC);
    apply_clock_strobe(p, &cfg->clock_strobe);

    struct fscc_registers r;
    reg_get(p, &r);
    r.CCR0 = (r.CCR0 & ~FC_NSB_MASK) | (cfg->nsb << 13);
    r.CCR0 = (r.CCR0 & ~FC_NTB_MASK) | (cfg->ntb << 16);
    if (cfg->itf) r.CCR0 |= FC_ITF_SYNC;

    r.SSR = cfg->ssr;
    r.SMR = cfg->smr;
    r.TSR = cfg->tsr;
    r.TMR = cfg->tmr;

    return reg_set(p, &r);
}

int fastcom_setup_transparent(fastcom_port_t* p, const fc_transparent_config_t* cfg)
{
    fastcom_set_mode(p, FC_MODE_TRANSPARENT);
    apply_clock_strobe(p, &cfg->clock_strobe);

    struct fscc_registers r;
    reg_get(p, &r);
    r.CCR2 = (r.CCR2 & ~0xFFFF0000u) | (cfg->rlc << 16);
    return reg_set(p, &r);
}

int fastcom_setup_true_async(fastcom_port_t* p, const fc_true_async_config_t* cfg)
{
    fastcom_set_mode(p, FC_MODE_TRUE_ASYNC);
    return 0;
}

int fastcom_setup_transparent_async_framing(fastcom_port_t* p, const fc_transparent_async_framing_config_t* cfg)
{
    fastcom_set_mode(p, FC_MODE_TRANSPARENT);
    apply_clock_strobe(p, &cfg->clock_strobe);

    struct fscc_registers r;
    reg_get(p, &r);
    r.CCR2 = (r.CCR2 & ~0xFFFF0000u) | (cfg->rlc << 16);
    return reg_set(p, &r);
}

// ================================================================
// SUPERFSCC DMA
// ================================================================
int fastcom_enable_dma(fastcom_port_t* p)
{
    struct fscc_registers r;
    reg_get(p, &r);
    r.DMACCR = FC_DMACCR_ENABLE;
    p->dma_enabled = true;
    return reg_set(p, &r);
}

int fastcom_set_memory_cap(fastcom_port_t* p, int input, int output)
{
    struct fscc_memory_cap cap = {input, output};
    return ioctl(p->fd, FSCC_SET_MEMORY_CAP, &cap);
}

// ================================================================
// I/O
// ================================================================
int fastcom_tx_frame(fastcom_port_t* p, const void* data, size_t len)
{
    return write(p->fd, data, len) == (ssize_t)len ? 0 : -1;
}

ssize_t fastcom_rx_frame(fastcom_port_t* p, void* buf, size_t max_len, bool wait)
{
    if (wait) {
        fd_set set; FD_ZERO(&set); FD_SET(p->fd, &set);
        select(p->fd+1, &set, NULL, NULL, NULL);
    }
    return read(p->fd, buf, max_len);
}

int fastcom_purge(fastcom_port_t* p, bool tx, bool rx)
{
    if (tx) ioctl(p->fd, FSCC_PURGE_TX, 0);
    if (rx) ioctl(p->fd, FSCC_PURGE_RX, 0);
    return 0;
}