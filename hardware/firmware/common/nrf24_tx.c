#include "nrf24_tx.h"
#include <stddef.h>
#include <string.h>

static int command(nrf24_tx_t *r, uint8_t cmd, const uint8_t *out, uint8_t *in, unsigned n)
{
    int status, value;
    r->io.select(r->io.ctx, 1);
    status = r->io.transfer(r->io.ctx, cmd);
    if (status >= 0) for (unsigned i = 0; i < n; ++i) {
        value = r->io.transfer(r->io.ctx, out ? out[i] : 0xffu);
        if (value < 0) { status = -1; break; }
        if (in) in[i] = (uint8_t)value;
    }
    r->io.select(r->io.ctx, 0);
    if (status < 0) ++r->io_errors;
    else r->last_status = (uint8_t)status;
    return status;
}

static int write_reg(nrf24_tx_t *r, uint8_t reg, uint8_t value)
{
    return command(r, (uint8_t)(0x20u | reg), &value, NULL, 1);
}

static int read_reg(nrf24_tx_t *r, uint8_t reg)
{
    uint8_t value = 0;
    if (command(r, reg, NULL, &value, 1) < 0) return -1;
    return value;
}

int nrf24_tx_init(nrf24_tx_t *r, const nrf24_io_t *io)
{
    static const uint8_t address[] = {0x34,0x43,0x10,0x10,0x01};
    static const uint8_t regs[] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x1c};
    static const uint8_t values[] = {0x0a,0x01,0x01,0x03,0xff,60,0x26,1};
    uint8_t check[5];
    if (!r || !io || !io->select || !io->ce || !io->transfer) return -1;
    memset(r, 0, sizeof(*r));
    r->io = *io;
    r->io.ce(r->io.ctx, 0);
    if (write_reg(r, 0x00, 0x08) < 0) return -1;
    if (command(r, 0x30, address, NULL, 5) < 0 || command(r, 0x10, NULL, check, 5) < 0 ||
        memcmp(address, check, 5)) return -2; /* Missing/wrong SPI device never enables RF. */
    if (command(r, 0x2a, address, NULL, 5) < 0 || command(r, 0x0a, NULL, check, 5) < 0 ||
        memcmp(address, check, 5)) return -2;
    if (write_reg(r, 0x1d, 7) < 0) return -1;
    if (read_reg(r, 0x1d) != 7) {
        uint8_t key = 0x73;
        if (command(r, 0x50, &key, NULL, 1) < 0 || write_reg(r, 0x1d, 7) < 0) return -1;
    }
    if (read_reg(r, 0x1d) != 7) return -3;
    if (write_reg(r, 0x11, 16) < 0) return -1;
    for (unsigned i = 0; i < sizeof(regs); ++i) {
        if (write_reg(r, regs[i], values[i]) < 0) return -1;
        int value = read_reg(r, regs[i]);
        if (value < 0 || value != values[i]) return -3;
        r->config[i] = (uint8_t)value;
    }
    if (write_reg(r, 0x07, 0x70) < 0 || command(r, 0xe1, NULL, NULL, 0) < 0 ||
        command(r, 0xe2, NULL, NULL, 0) < 0) return -1;
    return 0;
}

int nrf24_tx_start(nrf24_tx_t *r, const uint8_t *payload, unsigned length, uint32_t now)
{
    if (!payload || !length || length > 32 || r->phase) return -1;
    r->io.ce(r->io.ctx, 0);
    if (write_reg(r, 0x07, 0x70) < 0 || command(r, 0xe1, NULL, NULL, 0) < 0 ||
        command(r, 0xa0, payload, NULL, length) < 0) return -1;
    r->started_us = now;
    r->phase = 1;
    r->io.ce(r->io.ctx, 1);
    return 0;
}

int nrf24_tx_poll(nrf24_tx_t *r, uint32_t now)
{
    if (r->phase == 0) return -1;
    if (r->phase == 1) {
        /* now passed to start preceded SPI loading. Start pulse timing here,
         * so SPI transfer time can never shorten the CE pulse below 10 us. */
        r->started_us = now;
        r->phase = 2;
        return 0;
    }
    if (r->phase == 2) {
        if ((uint32_t)(now-r->started_us) < 15u) return 0;
        r->io.ce(r->io.ctx, 0);
        r->phase = 3;
    }
    int status = command(r, 0xff, NULL, NULL, 0);
    if (status < 0 || (status & 0x10) || (uint32_t)(now-r->started_us) >= 1000000u) {
        r->io.ce(r->io.ctx, 0);
        r->phase = 0;
        write_reg(r, 0x07, 0x70);
        command(r, 0xe1, NULL, NULL, 0);
        return -1;
    }
    if (status & 0x20) {
        r->phase = 0;
        write_reg(r, 0x07, 0x70);
        return 1;
    }
    return 0;
}

void nrf24_tx_power_down(nrf24_tx_t *r)
{
    r->io.ce(r->io.ctx, 0);
    r->phase = 0;
    write_reg(r, 0x00, 0x08);
}
