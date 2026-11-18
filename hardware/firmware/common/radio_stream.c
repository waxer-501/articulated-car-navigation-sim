#include "radio_stream.h"
#include "telemetry_v1.h"
#include <string.h>

int radio_stream_init(radio_stream_t *s, const nrf24_io_t *io, uint32_t boot_id, uint32_t now)
{
    if (!s) return -1;
    memset(s, 0, sizeof(*s));
    s->boot_id = boot_id;
    s->state = 2;
    if (!boot_id) { s->init_error = 100; return -1; }
    int result = nrf24_tx_init_stream(&s->radio, io);
    if (result) {
        s->init_error = (uint32_t)-result;
        if (s->radio.io.ce && s->radio.io.select && s->radio.io.transfer)
            nrf24_tx_power_down(&s->radio);
        return result;
    }
    /* The board wrapper sets this again from the clock AFTER SPI init; keeping
     * the argument also makes the hardware-independent scheduler testable.
     */
    s->ready_us = now + RADIO_STREAM_SETTLE_US;
    s->state = 1;
    return 0;
}

int radio_stream_frame(radio_stream_t *s, unsigned sensor, const uint8_t frame[32],
                       uint32_t received_us, uint32_t sequence)
{
    if (s->state != 1 || sensor > 1) return -1;
    radio_stream_sample_t sample;
    if (telemetry_v1_from_imu406(frame, 32, (uint8_t)(sensor+1u), (uint16_t)sequence,
                               received_us, s->boot_id, sample.wire)) {
        ++s->encode_errors;
        return -1;
    }
    sample.received_us = received_us;
    radio_stream_queue_t *q = &s->queues[sensor];
    if (q->count == RADIO_STREAM_DEPTH) {
        q->head = (q->head+1u) % RADIO_STREAM_DEPTH;
        --q->count;
        ++s->overflow_drops; /* Keep newest samples, never hide queue loss. */
    }
    q->samples[(q->head+q->count) % RADIO_STREAM_DEPTH] = sample;
    ++q->count;
    unsigned depth = s->queues[0].count+s->queues[1].count;
    if (depth > s->high_water) s->high_water = depth;
    return 0;
}

void radio_stream_poll(radio_stream_t *s, uint32_t now)
{
    if (s->state != 1) return;
    if (s->radio.phase) {
        int result = nrf24_tx_poll_deadline(&s->radio, now, RADIO_STREAM_TX_DEADLINE_US);
        if (!result) return;
        if (result > 0) ++s->packets_ok;
        else ++s->packets_failed;
    }
    if (!s->settled) {
        if ((int32_t)(now-s->ready_us) < 0) return;
        s->settled = 1; /* Do not compare a fixed boot deadline after time wrap. */
    }
    for (unsigned attempt = 0; attempt < 2; ++attempt) {
        unsigned id = s->next_sensor;
        s->next_sensor ^= 1u;
        radio_stream_queue_t *q = &s->queues[id];
        while (q->count) {
            radio_stream_sample_t *sample = &q->samples[q->head];
            q->head = (q->head+1u) % RADIO_STREAM_DEPTH;
            --q->count;
            if ((uint32_t)(now-sample->received_us) > RADIO_STREAM_MAX_AGE_US) {
                ++s->stale_drops;
                continue;
            }
            ++s->packets_started;
            /* SPI copies the full packet before returning, so dequeuing cannot
             * allow a new UART frame to overwrite an in-flight radio payload.
             */
            if (nrf24_tx_start(&s->radio, sample->wire, 32, now)) ++s->packets_failed;
            return;
        }
    }
}

int radio_stream_busy(const radio_stream_t *s)
{
    return s->state == 1 && (s->radio.phase || s->queues[0].count || s->queues[1].count);
}

void radio_stream_diagnostic(const radio_stream_t *s, volatile uint32_t d[16])
{
    d[0] = 3; d[1] = s->boot_id; d[2] = s->state;
    d[3] = s->packets_ok; d[4] = s->packets_failed; d[5] = s->packets_started;
    d[6] = s->encode_errors; d[7] = s->radio.io_errors;
    d[8] = s->overflow_drops; d[9] = s->stale_drops;
    d[10] = s->radio.last_status;
    d[11] = s->queues[0].count+s->queues[1].count;
    d[12] = s->high_water; d[13] = RADIO_STREAM_MAX_AGE_US;
    d[14] = s->init_error; d[15] = RADIO_STREAM_TX_DEADLINE_US;
}
