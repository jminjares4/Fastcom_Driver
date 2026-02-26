/* example_1mhz_8n1_write_read.c
 * Full bidirectional example:
 *   • 1 MHz Clocked Transparent mode
 *   • Clock Output (TXCLKOUT = 1 MHz)
 *   • Strobe Output (FST pin pulses per character)
 *   • 8N1 async framing (software)
 *   • SuperFSCC DMA enabled
 */

#include "fastcom.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* ================================================================
 * Pack one byte into 10-bit 8N1 frame (LSB first)
 * ================================================================ */
static void pack_8n1(uint8_t byte, uint8_t* bits) {
    bits[0] = 0;                    /* Start bit */
    for (int i = 0; i < 8; i++) {
        bits[1 + i] = (byte >> i) & 1;   /* Data bits LSB first */
    }
    bits[9] = 1;                    /* Stop bit */
}

/* ================================================================
 * Simple 8N1 decoder for received raw bits
 * ================================================================ */
static int decode_8n1(const uint8_t* bits, size_t len, uint8_t* out_bytes, size_t max_bytes) {
    size_t count = 0;
    for (size_t i = 0; i + 9 < len && count < max_bytes; i++) {
        if (bits[i] == 0) {                 /* Found start bit */
            uint8_t byte = 0;
            for (int b = 0; b < 8; b++) {
                byte |= bits[i + 1 + b] << b;
            }
            if (bits[i + 9] == 1) {         /* Valid stop bit */
                out_bytes[count++] = byte;
                i += 9;                     /* Skip to next possible frame */
            }
        }
    }
    return count;
}

int main(void) {
    /* Change port number if needed (0-3) */
    fastcom_port_t* p = fastcom_open(0);
    if (!p) {
        perror("fastcom_open failed");
        return 1;
    }

    /* =============== 1 MHz CONFIGURATION =============== */
    fc_clocked_transparent_config_t cfg;
    fc_clocked_transparent_default(&cfg, 1000000UL);   /* 1 MHz bit clock */

    cfg.clock_mode = 7;           /* Mode 7 → TXCLKOUT = internal BGR (Clock Output) */
    cfg.fst_mode   = FC_FST_MODE_1; /* Full frame strobe → Strobe Output on FST pin */
    cfg.use_dma    = true;        /* Enable SuperFSCC DMA */

    if (fastcom_setup_clocked_transparent(p, &cfg) != 0) {
        fprintf(stderr, "Configuration failed\n");
        fastcom_close(p);
        return 1;
    }

    fastcom_enable_dma(p);
    fastcom_purge(p, true, true);   /* Clear buffers */

    printf("=== 1 MHz 8N1 Transparent Mode with Clock + Strobe + DMA ===\n");
    printf("TXCLKOUT = 1 MHz continuous clock output\n");
    printf("FST      = strobe pulse for every 10-bit character\n");
    printf("DMA      = Enabled\n\n");

    /* =============== WRITE (Transmit) =============== */
    const char* msg = "Hello from Fastcom 1MHz 8N1!\r\n";
    uint8_t frame[10];

    printf("Sending: %s", msg);

    for (size_t i = 0; i < strlen(msg); i++) {
        pack_8n1((uint8_t)msg[i], frame);
        fastcom_tx_frame(p, frame, 10);
        usleep(500);               /* Small delay between characters */
    }

    /* =============== READ (Receive) =============== */
    printf("\nWaiting for reply... (send something from the other end)\n");

    uint8_t rx_bits[4096];
    uint8_t rx_bytes[512];

    ssize_t bits_rx = fastcom_rx_frame(p, rx_bits, sizeof(rx_bits), true);  /* blocking read */

    if (bits_rx > 0) {
        int bytes_rx = decode_8n1(rx_bits, bits_rx, rx_bytes, sizeof(rx_bytes) - 1);
        if (bytes_rx > 0) {
            rx_bytes[bytes_rx] = '\0';
            printf("Received: %s\n", rx_bytes);
        } else {
            printf("Received %zd raw bits (no valid 8N1 frames found)\n", bits_rx);
        }
    } else {
        printf("No data received.\n");
    }

    fastcom_close(p);
    return 0;
}