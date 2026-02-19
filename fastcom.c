/*
 * fastcom.c
 * COMPLETE USER-SPACE DRIVER FOR FASTCOM FSCC / SuperFSCC FAMILY
 * Properly handles sync_fd / data_fd / ctrl_fd separation
 * Runtime mode switching between sync and async/UART
 * No dependency on fscc_enable_async=1 at load time
 */

#include "fastcom.h"
#include "fastcom_defs.h"
#include "calculate-clock-bits.h"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <serialfc.h>

struct fastcom_handle {
    int port;
    fastcom_mode_t mode;
    int sync_fd;       // always /dev/fscc# — for FCR, purge, status
    int data_fd;       // active read/write: fscc# (sync) or ttyS# (async)
    int ctrl_fd;       // /dev/serialfc# — async extras only
};

static int tty_number_for_port(int port) {
    // Typical Commtech mapping — verify with dmesg | grep ttyS
    // port 0 → ttyS4, port 1 → ttyS5, etc.
    return 4 + port;
}

fastcom_t fastcom_open(int port, fastcom_mode_t initial_mode) {
    if (port < 0 || port > 3) return NULL;

    fastcom_t h = calloc(1, sizeof(*h));
    if (!h) return NULL;

    h->port = port;
    h->mode = initial_mode;
    h->sync_fd = h->data_fd = h->ctrl_fd = -1;

    char path[64];

    // Always open sync_fd — needed for mode control
    sprintf(path, "/dev/fscc%d", port);
    h->sync_fd = open(path, O_RDWR);
    if (h->sync_fd < 0) {
        perror("open /dev/fscc#");
        goto fail;
    }

    // Set initial data path
    if (initial_mode == FASTCOM_MODE_ASYNC_UART) {
        sprintf(path, "/dev/ttyS%d", tty_number_for_port(port));
        h->data_fd = open(path, O_RDWR | O_NOCTTY);
        if (h->data_fd < 0) goto fail;

        sprintf(path, "/dev/serialfc%d", port);
        h->ctrl_fd = open(path, O_RDWR);
        if (h->ctrl_fd < 0) goto fail;

        // Ensure UART mode is active
        fastcom_set_mode(h, FASTCOM_MODE_ASYNC_UART);
    } else {
        h->data_fd = h->sync_fd;
    }

    return h;

fail:
    fastcom_close(h);
    return NULL;
}

int fastcom_set_mode(fastcom_t h, fastcom_mode_t new_mode) {
    if (!h || h->mode == new_mode) return 0;

    char path[64];

    // Close current async fds if switching away
    if (h->mode == FASTCOM_MODE_ASYNC_UART) {
        if (h->data_fd >= 0 && h->data_fd != h->sync_fd) close(h->data_fd);
        if (h->ctrl_fd >= 0) close(h->ctrl_fd);
        h->data_fd = -1;
        h->ctrl_fd = -1;
    }

    // Update FCR via sync_fd
    __u32 uart_bit = (h->port == 0) ? 0x01000000UL : 0x02000000UL;
    struct fscc_registers regs = { .FCR = FSCC_UPDATE_VALUE };

    if (ioctl(h->sync_fd, FSCC_GET_REGISTERS, &regs) < 0) {
        perror("GET FCR");
        return -1;
    }

    if (new_mode == FASTCOM_MODE_ASYNC_UART) {
        regs.FCR |= uart_bit;
    } else {
        regs.FCR &= ~uart_bit;
    }

    if (ioctl(h->sync_fd, FSCC_SET_REGISTERS, &regs) < 0) {
        perror("SET FCR");
        return -1;
    }

    // Purge after mode change
    ioctl(h->sync_fd, FSCC_PURGE_RX);
    ioctl(h->sync_fd, FSCC_PURGE_TX);
    usleep(20000);

    // Open new path if going async
    if (new_mode == FASTCOM_MODE_ASYNC_UART) {
        sprintf(path, "/dev/ttyS%d", tty_number_for_port(h->port));
        h->data_fd = open(path, O_RDWR | O_NOCTTY);
        if (h->data_fd < 0) {
            perror("open ttyS#");
            return -1;
        }

        sprintf(path, "/dev/serialfc%d", h->port);
        h->ctrl_fd = open(path, O_RDWR);
        if (h->ctrl_fd < 0) {
            close(h->data_fd);
            h->data_fd = -1;
            perror("open serialfc#");
            return -1;
        }
    } else {
        h->data_fd = h->sync_fd;
    }

    h->mode = new_mode;
    return 0;
}

int fastcom_set_config(fastcom_t h, const fastcom_config_t *cfg) {
    if (!h || !cfg) return -EINVAL;

    if (h->mode == FASTCOM_MODE_ASYNC_UART) {
        struct termios tio;
        if (tcgetattr(h->data_fd, &tio) < 0) return -1;

        cfmakeraw(&tio);
        tio.c_cflag &= ~(PARENB | PARODD | CSTOPB | CSIZE);
        tio.c_cflag |= CREAD | CLOCAL;

        switch (cfg->data_bits) {
            case 5: tio.c_cflag |= CS5; break;
            case 6: tio.c_cflag |= CS6; break;
            case 7: tio.c_cflag |= CS7; break;
            default: tio.c_cflag |= CS8; break;
        }

        if (cfg->parity == FASTCOM_PARITY_ODD)  tio.c_cflag |= PARENB | PARODD;
        if (cfg->parity == FASTCOM_PARITY_EVEN) tio.c_cflag |= PARENB;
        if (cfg->stop_bits == FASTCOM_STOP_BITS_2) tio.c_cflag |= CSTOPB;

        speed_t baud = B9600;
        if (cfg->baud_rate == 1000000) baud = B1000000;
        else if (cfg->baud_rate == 115200) baud = B115200;
        cfsetispeed(&tio, baud);
        cfsetospeed(&tio, baud);

        if (tcsetattr(h->data_fd, TCSANOW, &tio) < 0) return -1;

        if (cfg->strobe_per_byte) {
            ioctl(h->ctrl_fd, IOCTL_FASTCOM_SET_FRAME_LENGTH, 1);
        }
        if (cfg->aux_clock_hz > 0) {
            unsigned char bits[20];
            if (calculate_clock_bits(cfg->aux_clock_hz, 10, bits) == 0) {
                ioctl(h->ctrl_fd, IOCTL_FASTCOM_SET_CLOCK_BITS, bits);
            }
        }
    } else {
        struct fscc_registers regs = {0};
        regs.CCR0 = FSCC_UPDATE_VALUE;
        if (ioctl(h->sync_fd, FSCC_GET_REGISTERS, &regs) < 0) return -1;

        regs.CCR0 &= ~0x03;
        switch (cfg->mode) {
            case FASTCOM_MODE_HDLC:       regs.CCR0 |= 0x00; break;
            case FASTCOM_MODE_XSYNC:      regs.CCR0 |= 0x01; break;
            case FASTCOM_MODE_TRANSPARENT: regs.CCR0 |= 0x02; break;
            default: break;
        }

        if (cfg->baud_rate > 0) {
            unsigned char bits[20];
            if (calculate_clock_bits(cfg->baud_rate, 10, bits) == 0) {
                ioctl(h->sync_fd, FSCC_SET_CLOCK_BITS, bits);
            }
        }

        if (cfg->strobe_per_byte && cfg->mode == FASTCOM_MODE_TRANSPARENT) {
            regs.CCR0 = (regs.CCR0 & ~(0x7u << 8)) | (FASTCOM_FSC_MODE5_TX_ENABLE << 8);
        }

        if (ioctl(h->sync_fd, FSCC_SET_REGISTERS, &regs) < 0) return -1;

        fastcom_purge(h, true, true);
    }

    return 0;
}

ssize_t fastcom_write(fastcom_t h, const void *buf, size_t len) {
    if (!h || h->data_fd < 0) return -EINVAL;
    return write(h->data_fd, buf, len);
}

ssize_t fastcom_read(fastcom_t h, void *buf, size_t len) {
    if (!h || h->data_fd < 0) return -EINVAL;
    return read(h->data_fd, buf, len);
}

int fastcom_purge(fastcom_t h, bool tx, bool rx) {
    if (!h || h->sync_fd < 0) return -EINVAL;
    if (tx) ioctl(h->sync_fd, FSCC_PURGE_TX);
    if (rx) ioctl(h->sync_fd, FSCC_PURGE_RX);
    return 0;
}

uint32_t fastcom_get_status(fastcom_t h) {
    if (!h || h->sync_fd < 0) return 0;
    struct fscc_registers regs = { .STAR = FSCC_UPDATE_VALUE };
    ioctl(h->sync_fd, FSCC_GET_REGISTERS, &regs);
    return regs.STAR;
}

uint32_t fastcom_get_version(fastcom_t h) {
    if (!h || h->sync_fd < 0) return 0;
    struct fscc_registers regs = { .VSTR = FSCC_UPDATE_VALUE };
    ioctl(h->sync_fd, FSCC_GET_REGISTERS, &regs);
    return regs.VSTR;
}

uint32_t fastcom_get_dma_status(fastcom_t h) {
    if (!h || h->sync_fd < 0) return 0;
    struct fscc_registers regs = { .DSTAR = FSCC_UPDATE_VALUE };
    ioctl(h->sync_fd, FSCC_GET_REGISTERS, &regs);
    return regs.DSTAR;
}

void fastcom_close(fastcom_t h) {
    if (!h) return;
    if (h->ctrl_fd >= 0) close(h->ctrl_fd);
    if (h->data_fd >= 0 && h->data_fd != h->sync_fd) close(h->data_fd);
    if (h->sync_fd >= 0) close(h->sync_fd);
    free(h);
}