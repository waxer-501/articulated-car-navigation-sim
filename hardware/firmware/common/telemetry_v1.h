#ifndef SWEEPER_TELEMETRY_V1_H
#define SWEEPER_TELEMETRY_V1_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Encode one validated IMU frame as data-only 32-byte telemetry.
 * sensor_id 1=front, 2=rear; each sensor has an independent sequence.
 * rx_time_us is MCU UART frame completion, not the IMU sample instant.
 * boot_id must be newly generated each MCU boot (board implementation pending).
 * Output remains untouched on error. No radio/HAL/control IO is performed.
 */
int telemetry_v1_from_imu406(const uint8_t *imu_frame, size_t length,
                           uint8_t sensor_id, uint16_t sequence,
                           uint32_t rx_time_us, uint32_t boot_id,
                           uint8_t output[32]);

#ifdef __cplusplus
}
#endif
#endif
