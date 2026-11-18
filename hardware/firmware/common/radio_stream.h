#ifndef SWEEPER_RADIO_STREAM_H
#define SWEEPER_RADIO_STREAM_H
#include "nrf24_tx.h"
#include <stdint.h>

#define RADIO_STREAM_DEPTH 4u
#define RADIO_STREAM_MAX_AGE_US 100000u
#define RADIO_STREAM_TX_DEADLINE_US 100000u
#define RADIO_STREAM_SETTLE_US 2000u

typedef struct {
    uint8_t wire[32];
    uint32_t received_us;
} radio_stream_sample_t;

typedef struct {
    radio_stream_sample_t samples[RADIO_STREAM_DEPTH];
    unsigned head, count;
} radio_stream_queue_t;

/* All calls are from the main loop, never a UART ISR. No dynamic allocation. */
typedef struct {
    nrf24_tx_t radio;
    radio_stream_queue_t queues[2];
    uint32_t boot_id, state, ready_us, packets_ok, packets_failed, packets_started;
    uint32_t encode_errors, overflow_drops, stale_drops, high_water, init_error;
    unsigned next_sensor, settled;
} radio_stream_t;

int radio_stream_init(radio_stream_t *s, const nrf24_io_t *io, uint32_t boot_id, uint32_t now_us);
int radio_stream_frame(radio_stream_t *s, unsigned sensor, const uint8_t frame[32],
                       uint32_t received_us, uint32_t sequence);
void radio_stream_poll(radio_stream_t *s, uint32_t now_us);
void radio_stream_diagnostic(const radio_stream_t *s, volatile uint32_t words[16]);
int radio_stream_busy(const radio_stream_t *s);
#endif
