/* Read-only diagnosis even when target identity reads as zero.
 * Never halt, reset, run, erase or write target memory/registers.
 */
#include <stdio.h>
#include <stdint.h>
#include <stlink.h>
#include <usb.h>

int main(void)
{
    char serial[STLINK_SERIAL_BUFFER_SIZE] = "33FF72064243323456261357";
    stlink_t *sl = stlink_open_usb(UERROR, CONNECT_HOT_PLUG, serial, 100);
    if (!sl) { puts("open_failed"); return 1; }
    printf("chip_id=0x%08x flash_bytes=%zu core_id=0x%08x voltage_tool_mV=%d\n",
           sl->chip_id, sl->flash_size, sl->core_id, stlink_target_voltage(sl));
    int status = stlink_status(sl);
    printf("status_rc=%d core_state=%d\n", status, sl->core_stat);
    const uint32_t addresses[] = {0xe0042000u,0xe000ed00u,0xe000edf0u,0xe000ed28u,
        0xe000ed2cu,0xe000ed38u,0x40021000u,0x40021004u,0x40010804u,0x40010800u,
        0x40010004u,0x4002200cu,0x40022010u,0x20000000u,0x08007800u,0x08007804u,0x08000000u};
    for (unsigned i = 0; i < sizeof(addresses)/sizeof(addresses[0]); ++i) {
        uint32_t value = 0xdeadbeefu;
        int rc = stlink_read_debug32(sl, addresses[i], &value);
        printf("addr=0x%08x rc=%d value=0x%08x\n",addresses[i],rc,value);
    }
    stlink_exit_debug_mode(sl);
    stlink_close(sl);
    return 0;
}
