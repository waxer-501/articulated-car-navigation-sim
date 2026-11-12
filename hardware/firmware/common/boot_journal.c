#include "boot_journal.h"
#include <stddef.h>

uint32_t boot_journal_next(const boot_flash_t *f)
{
    uint32_t highest = 0;
    unsigned active = 0, page, slot;
    int empty[2] = {-1, -1};
    if (!f || !f->read || !f->erase || !f->program) return 0;
    for (page = 0; page < 2; ++page) {
        for (slot = 0; slot < 128; ++slot) {
            uint32_t value = f->read(f->ctx, page, slot*2);
            uint32_t inverse = f->read(f->ctx, page, slot*2+1);
            if (value != 0 && value != UINT32_MAX && inverse == ~value && value > highest) {
                highest = value;
                active = page;
            }
            if (value == UINT32_MAX && inverse == UINT32_MAX && empty[page] < 0) empty[page] = (int)slot;
        }
    }
    if (highest >= UINT32_MAX-1u) return 0;
    if (empty[active] < 0) {
        active = 1u-active;
        if (f->erase(f->ctx, active) != 0) return 0; /* Latest valid page stays intact. */
        for (slot = 0; slot < 256; ++slot) if (f->read(f->ctx, active, slot) != UINT32_MAX) return 0;
        empty[active] = 0;
    }
    uint32_t next = highest+1u;
    slot = (unsigned)empty[active]*2u;
    if (f->program(f->ctx, active, slot, next) != 0) return 0;
    if (f->program(f->ctx, active, slot+1u, ~next) != 0) return 0;
    if (f->read(f->ctx, active, slot) != next || f->read(f->ctx, active, slot+1u) != ~next) return 0;
    return next;
}
