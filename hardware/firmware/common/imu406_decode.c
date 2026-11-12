#include "imu406_decode.h"

static int bcd3(const uint8_t *data, float divisor, float *result)
{
    unsigned int sign = data[0] >> 4;
    unsigned int digits[5] = {data[0] & 15u, data[1] >> 4,
                             data[1] & 15u, data[2] >> 4, data[2] & 15u};
    unsigned int value = 0;
    size_t i;
    if (sign > 1u) return -5;
    for (i = 0; i < 5; ++i) {
        if (digits[i] > 9u) return -5;
        value = value * 10u + digits[i];
    }
    *result = ((sign != 0u) ? -(float)value : (float)value) / divisor;
    return 0;
}

int imu406_decode(const uint8_t *frame, size_t length, imu406_sample_t *output)
{
    imu406_sample_t sample = {0};
    unsigned int sum = 0;
    float value;
    size_t i;
    if (frame == NULL || output == NULL || length != 32u) return -1;
    if (frame[0] != 0x68u || frame[1] != 0x1fu) return -2;
    for (i = 1; i < length - 1; ++i) sum += frame[i];
    if ((sum & 255u) != frame[length - 1]) return -3;
    if (frame[3] != 0x84u) return -4;
    sample.address = frame[2];
    for (i = 0; i < 9; ++i) {
        float divisor = (i >= 3 && i < 6) ? 1000.0f : 100.0f;
        if (bcd3(frame + 4 + 3*i, divisor, &value) != 0) return -5;
        if (i < 3) sample.euler_deg[i] = value;
        else if (i < 6) sample.accel_m_s2[i - 3] = value * 9.80665f;
        else sample.gyro_rad_s[i - 6] = value * 0.01745329251994329577f;
    }
    *output = sample;
    return 0;
}
