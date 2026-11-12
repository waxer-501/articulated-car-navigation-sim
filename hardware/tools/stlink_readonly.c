/* Read-only memory access. No halt, reset, run, erase, memory-write or flash API.
 * Link against the pinned stlink 1.7.0 ABI, not an arbitrary library version.
 */
#include <stdint.h>
#include <string.h>
#include <stlink.h>
#include <usb.h>

static unsigned unstable_reads;
unsigned capture_unstable_reads(void) { return unstable_reads; }

void *capture_open(const char *serial)
{
    char chosen[STLINK_SERIAL_BUFFER_SIZE] = {0};
    if (!serial || strlen(serial) >= sizeof(chosen)) return NULL;
    strcpy(chosen, serial);
    stlink_t *sl = stlink_open_usb(UERROR, CONNECT_HOT_PLUG, chosen, 100);
    if (!sl) return NULL;
    if (sl->chip_id != 0x412u || sl->flash_size != 32768u) {
        stlink_exit_debug_mode(sl);
        stlink_close(sl);
        return NULL;
    }
    return sl;
}

void capture_close(void *handle)
{
    if (handle) {
        stlink_exit_debug_mode(handle);
        stlink_close(handle);
    }
}

int capture_snapshot(void *handle, uint8_t output[256])
{
    stlink_t *sl = handle;
    uint8_t before[4];
    if (!sl || !output) return -1;
    if (stlink_read_mem32(sl, 0x2000000cu, 4) != 0) return -1;
    memcpy(before, sl->q_buf, 4);
    if (before[0] & 1u) return 0;
    if (stlink_read_mem32(sl, 0x20000000u, 256) != 0) return -1;
    memcpy(output, sl->q_buf, 256);
    /* A consistent sequence alone cannot detect corrupted debug-transfer data. */
    if (stlink_read_mem32(sl, 0x20000000u, 256) != 0) return -1;
    if (memcmp(output, sl->q_buf, 256)) {
        if (memcmp(output + 12, sl->q_buf + 12, 4) == 0) ++unstable_reads;
        return 0;
    }
    if (stlink_read_mem32(sl, 0x2000000cu, 4) != 0) return -1;
    if (memcmp(before, sl->q_buf, 4) || memcmp(before, output + 12, 4)) return 0;
    return 1;
}

int capture_flash_backup(void *handle, uint8_t output[32768])
{
    stlink_t *sl = handle;
    if (!sl || !output) return -1;
    for (unsigned i = 0; i < 32768u; i += 1024u) {
        unsigned matching = 0;
        uint8_t previous[1024];
        for (unsigned attempt = 0; attempt < 12u; ++attempt) {
            if (stlink_read_mem32(sl, 0x08000000u + i, 1024) != 0) return -1;
            if (attempt != 0u && memcmp(previous, sl->q_buf, 1024) == 0) ++matching;
            else {
                if (attempt != 0u) ++unstable_reads;
                matching = 1;
            }
            memcpy(previous, sl->q_buf, 1024);
            if (matching == 3u) break;
        }
        /* Require three consecutive identical reads, never silently use last. */
        if (matching != 3u) return -1;
        memcpy(output + i, previous, 1024);
    }
    return 0;
}
