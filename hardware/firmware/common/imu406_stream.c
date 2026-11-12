#include "imu406_stream.h"
#include "imu406_decode.h"
#include <string.h>

static void advance(imu406_stream_t *s)
{
    uint32_t i = 1;
    while (i < s->used && s->bytes[i] != 0x68u) ++i;
    s->used -= i;
    memmove(s->bytes, s->bytes + i, s->used);
}

int imu406_stream_push(imu406_stream_t *s, uint8_t byte, uint8_t output[32])
{
    imu406_sample_t sample;
    if (s == NULL || output == NULL) return 0;
    if (s->used == 0u && byte != 0x68u) return 0;
    s->bytes[s->used++] = byte;
    while (s->used != 0u) {
        if ((s->used >= 2u && s->bytes[1] != 0x1fu) ||
            (s->used >= 4u && s->bytes[3] != 0x84u)) {
            ++s->rejected;
            advance(s);
        } else if (s->used == 32u) {
            if (imu406_decode(s->bytes, 32u, &sample) == 0) {
                memcpy(output, s->bytes, 32u);
                s->used = 0;
                return 1;
            }
            ++s->rejected;
            advance(s);
        } else break;
    }
    return 0;
}

void imu406_stream_discontinuity(imu406_stream_t *s)
{
    if (s != NULL) s->used = 0;
}
