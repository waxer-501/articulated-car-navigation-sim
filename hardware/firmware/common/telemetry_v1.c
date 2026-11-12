#include "telemetry_v1.h"
#include <string.h>

static void put16(uint8_t *p, uint16_t x)
{
    p[0] = (uint8_t)x;
    p[1] = (uint8_t)(x >> 8);
}

static void put32(uint8_t *p, uint32_t x)
{
    put16(p, (uint16_t)x);
    put16(p + 2, (uint16_t)(x >> 16));
}

static uint16_t crc16(const uint8_t *p, size_t length)
{
    uint16_t crc = 0xffffu;
    size_t i;
    unsigned int bit;
    for (i = 0; i < length; ++i) {
        crc ^= (uint16_t)p[i] << 8;
        for (bit = 0; bit < 8; ++bit) {
            crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000u) ? 0x1021u : 0u));
        }
    }
    return crc;
}

int telemetry_v1_from_imu406(const uint8_t *frame, size_t length,
                           uint8_t sensor_id, uint16_t sequence,
                           uint32_t rx_time_us, uint32_t boot_id,
                           uint8_t output[32])
{
    uint8_t packet[32] = {0};
    unsigned int sum = 0;
    size_t i, j;
    if (frame == NULL || output == NULL || length != 32) return -1;
    if (sensor_id != 1u && sensor_id != 2u) return -2;
    if (frame[0] != 0x68u || frame[1] != 0x1fu || frame[3] != 0x84u) return -3;
    for (i = 1; i < 31; ++i) sum += frame[i];
    if ((sum & 255u) != frame[31]) return -4;
    packet[0] = 0xa6u;
    packet[1] = (uint8_t)(0x10u | sensor_id);
    put16(packet + 2, sequence);
    put32(packet + 4, rx_time_us);
    put32(packet + 8, boot_id);
    for (i = 0; i < 9; ++i) {
        const uint8_t *p = frame + 4 + i*3;
        unsigned int digits[5] = {p[0] & 15u, p[1] >> 4, p[1] & 15u, p[2] >> 4, p[2] & 15u};
        unsigned int magnitude = 0;
        int32_t value;
        if ((p[0] >> 4) > 1u) return -5;
        for (j = 0; j < 5; ++j) {
            if (digits[j] > 9u) return -5;
            magnitude = magnitude*10u+digits[j];
        }
        value = (p[0] >> 4) ? -(int32_t)magnitude : (int32_t)magnitude;
        if (value < -32768 || value > 32767) return -6;
        put16(packet + 12 + i*2, (uint16_t)(int16_t)value);
    }
    put16(packet + 30, crc16(packet, 30));
    memcpy(output, packet, sizeof(packet));
    return 0;
}
