#ifndef SWEEPER_NRF24_TX_H
#define SWEEPER_NRF24_TX_H
#include <stdint.h>

typedef struct {
    void *ctx;
    void (*select)(void *, int selected);
    void (*ce)(void *, int high);
    int (*transfer)(void *, uint8_t out); /* byte or negative on IO error */
} nrf24_io_t;

typedef struct {
    nrf24_io_t io;
    uint32_t started_us, io_errors;
    uint8_t phase, last_status, config[8];
} nrf24_tx_t;

/* Vendor reference profile: channel 60, 250 kbps, 0dBm, 1-byte HW CRC,
 * five-byte address 34 43 10 10 01, pipe-0 hardware ACK and dynamic payload.
 * This is a candidate profile until real USB RX receipt is demonstrated.
 */
int nrf24_tx_init(nrf24_tx_t *radio, const nrf24_io_t *io);
int nrf24_tx_start(nrf24_tx_t *radio, const uint8_t *payload, unsigned length, uint32_t now_us);
/* 0=busy, 1=TX_DS, -1=MAX_RT/timeout/IO error; call repeatedly without blocking. */
int nrf24_tx_poll(nrf24_tx_t *radio, uint32_t now_us);
void nrf24_tx_power_down(nrf24_tx_t *radio);
#endif
