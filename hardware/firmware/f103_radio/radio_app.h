#ifndef SWEEPER_RADIO_APP_H
#define SWEEPER_RADIO_APP_H
#include <stdint.h>
uint32_t radio_boot_id(void);
void radio_app_init(uint32_t (*clock_us)(void), uint32_t boot_id);
void radio_app_frame(unsigned id, const uint8_t frame[32], uint32_t us, uint32_t sequence);
void radio_app_poll(uint32_t us, uint32_t ms, volatile uint32_t diagnostic[16]);
#endif
