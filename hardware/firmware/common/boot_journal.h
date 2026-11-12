#ifndef SWEEPER_BOOT_JOURNAL_H
#define SWEEPER_BOOT_JOURNAL_H
#include <stdint.h>

typedef struct {
    void *ctx;
    uint32_t (*read)(void *, unsigned page, unsigned word);
    int (*erase)(void *, unsigned page);
    int (*program)(void *, unsigned page, unsigned word, uint32_t value);
} boot_flash_t;

/* Two 1-KiB pages, records {counter, ~counter}. Commit before any RF output.
 * Return 0 on errors/exhaustion; never reuse a successfully committed boot ID.
 * Flash pages are dedicated to this journal and must not contain program data.
 */
uint32_t boot_journal_next(const boot_flash_t *flash);
#endif
