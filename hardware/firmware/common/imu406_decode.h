#ifndef SWEEPER_IMU406_DECODE_H
#define SWEEPER_IMU406_DECODE_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t address;
    float euler_deg[3]; /* Diagnostic only; not an absolute heading reference. */
    float accel_m_s2[3]; /* Sensor axes, before mounting transform. */
    float gyro_rad_s[3];
} imu406_sample_t;

/* Receive-only decoder for a complete 32-byte 0x84 frame.
 * Return 0 on success; negative on invalid length/header/checksum/format/BCD.
 * Output remains unchanged on failure. No HAL, IO, allocation or commands.
 */
int imu406_decode(const uint8_t *frame, size_t length, imu406_sample_t *output);

#ifdef __cplusplus
}
#endif
#endif
