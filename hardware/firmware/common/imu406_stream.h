#ifndef SWEEPER_IMU406_STREAM_H
#define SWEEPER_IMU406_STREAM_H
#include <stdint.h>

typedef struct {
    uint8_t bytes[32];
    uint32_t used;
    uint32_t rejected;
} imu406_stream_t;

/* Main-loop parser, not an ISR routine. Returns 1 only for a validated frame.
 * Corrupt/noisy prefixes are discarded; embedded 0x68 bytes are supported.
 * output is untouched until an entire valid frame is received.
 */
int imu406_stream_push(imu406_stream_t *stream, uint8_t byte, uint8_t output[32]);
/* Abandon a partial frame after a UART error/queue loss; keep error counters. */
void imu406_stream_discontinuity(imu406_stream_t *stream);
#endif
