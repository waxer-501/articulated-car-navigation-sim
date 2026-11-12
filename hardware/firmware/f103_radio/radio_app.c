#include "stm32f103x6.h"
#include "radio_app.h"
#include "nrf24_tx.h"
#include "boot_journal.h"
#include "telemetry_v1.h"
#include <string.h>

#define JOURNAL_BASE 0x08007800u
#define TRANSMIT_START_MS 5000u
#define TRANSMIT_END_MS 65000u
#define FRAGMENT_GAP_US 300000u

static uint32_t (*clock_us)(void);
static uint32_t boot_id, state, fragments_ok, fragments_failed, packets_done, packets_started;
static uint32_t encode_errors, next_us, last_status, init_error;
static uint8_t latest[2][32], packet[32], have[2], half, active, next_sensor;
static uint32_t timestamps[2], sequences[2], last_selected[2];
static nrf24_tx_t radio;

static uint32_t flash_read(void *unused, unsigned page, unsigned word)
{
    (void)unused;
    return *(volatile uint32_t *)(JOURNAL_BASE + page*1024u + word*4u);
}

static int flash_ready(void)
{
    for (unsigned spin = 0; spin < 32000000u; ++spin) {
        if (!(FLASH->SR & FLASH_SR_BSY)) return (FLASH->SR & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)) ? -1 : 0;
    }
    return -1;
}

static void flash_unlock(void)
{
    if (FLASH->CR & FLASH_CR_LOCK) { FLASH->KEYR = 0x45670123u; FLASH->KEYR = 0xcdef89abu; }
    FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR;
}

static int flash_erase(void *unused, unsigned page)
{
    (void)unused;
    if (page > 1) return -1;
    flash_unlock();
    if (flash_ready()) return -1;
    FLASH->CR = FLASH_CR_PER;
    FLASH->AR = JOURNAL_BASE + page*1024u;
    FLASH->CR |= FLASH_CR_STRT;
    int rc = flash_ready();
    FLASH->CR = FLASH_CR_LOCK;
    return rc;
}

static int flash_program(void *unused, unsigned page, unsigned word, uint32_t value)
{
    (void)unused;
    if (page > 1 || word > 255) return -1;
    uint32_t address = JOURNAL_BASE + page*1024u + word*4u;
    if (*(volatile uint32_t *)address != UINT32_MAX) return -1;
    flash_unlock();
    for (unsigned i = 0; i < 2; ++i) {
        if (flash_ready()) { FLASH->CR = FLASH_CR_LOCK; return -1; }
        FLASH->CR = FLASH_CR_PG;
        *(volatile uint16_t *)(address + i*2u) = (uint16_t)(value >> (i*16u));
    }
    int rc = flash_ready();
    FLASH->CR = FLASH_CR_LOCK;
    return rc;
}

uint32_t radio_boot_id(void)
{
    boot_flash_t storage = {0, flash_read, flash_erase, flash_program};
    return boot_journal_next(&storage);
}

static void select_chip(void *unused, int selected)
{
    (void)unused;
    GPIOA->BSRR = selected ? (1u << 20) : (1u << 4);
}

static void enable_radio(void *unused, int high)
{
    (void)unused;
    GPIOB->BSRR = high ? 1u : (1u << 16);
}

static int spi_byte(void *unused, uint8_t byte)
{
    (void)unused;
    uint32_t started = clock_us();
    while (!(SPI1->SR & SPI_SR_TXE)) if ((uint32_t)(clock_us()-started) > 100000u) return -1;
    *(volatile uint8_t *)&SPI1->DR = byte;
    while (!(SPI1->SR & SPI_SR_RXNE)) if ((uint32_t)(clock_us()-started) > 100000u) return -1;
    return *(volatile uint8_t *)&SPI1->DR;
}

void radio_app_init(uint32_t (*clock)(void), uint32_t id)
{
    clock_us = clock;
    boot_id = id;
    state = 2;
    if (!id) { init_error = 100; return; }
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    /* PA5/PA7 alternate push-pull, PA6 input. SPI mode 0, nominal 500 kHz. */
    GPIOA->CRL = (GPIOA->CRL & ~((15u << 20) | (15u << 24) | (15u << 28))) |
                  (10u << 20) | (4u << 24) | (10u << 28);
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (6u << 3) | SPI_CR1_SPE;
    nrf24_io_t io = {0, select_chip, enable_radio, spi_byte};
    int result = nrf24_tx_init(&radio, &io);
    if (result) {
        init_error = (uint32_t)-result;
        nrf24_tx_power_down(&radio);
        return;
    }
    state = 1; /* Configured and read back; wait five seconds before first TX. */
}

void radio_app_frame(unsigned id, const uint8_t frame[32], uint32_t us, uint32_t sequence)
{
    if (id > 1) return;
    memcpy(latest[id], frame, 32);
    timestamps[id] = us;
    sequences[id] = sequence;
    have[id] = 1;
}

static void update_diagnostic(volatile uint32_t d[16])
{
    d[0] = 2; d[1] = boot_id; d[2] = state;
    d[3] = fragments_ok; d[4] = fragments_failed; d[5] = packets_done;
    d[6] = encode_errors; d[7] = radio.io_errors; d[8] = TRANSMIT_END_MS;
    d[9] = packets_started; d[10] = last_status;
    d[11] = (uint32_t)radio.config[0] | ((uint32_t)radio.config[5] << 8) | ((uint32_t)radio.config[6] << 16) | (7u << 24);
    d[12] = (uint32_t)radio.config[1] | ((uint32_t)radio.config[3] << 8) | ((uint32_t)radio.config[7] << 16);
    d[13] = FRAGMENT_GAP_US; d[14] = init_error; d[15] = TRANSMIT_START_MS;
}

void radio_app_poll(uint32_t now, uint32_t ms, volatile uint32_t d[16])
{
    if (state != 1) { update_diagnostic(d); return; }
    if (radio.phase) {
        int result = nrf24_tx_poll(&radio, now);
        last_status = radio.last_status;
        if (result == 0) { update_diagnostic(d); return; }
        if (result > 0) {
            ++fragments_ok;
            if (half == 0) half = 1;
            else { ++packets_done; active = 0; half = 0; }
        } else {
            ++fragments_failed;
            active = 0; half = 0;
        }
        next_us = now + FRAGMENT_GAP_US;
    }
    if (!active && ms >= TRANSMIT_END_MS) {
        nrf24_tx_power_down(&radio);
        state = 4;
    } else if (ms >= TRANSMIT_START_MS && (int32_t)(now-next_us) >= 0) {
        if (!active) {
            for (unsigned attempt = 0; attempt < 2; ++attempt) {
                unsigned id = next_sensor;
                next_sensor ^= 1u;
                if (!have[id] || sequences[id] == last_selected[id] || (uint32_t)(now-timestamps[id]) > 2000000u) continue;
                last_selected[id] = sequences[id];
                if (telemetry_v1_from_imu406(latest[id], 32, (uint8_t)(id+1u), (uint16_t)sequences[id],
                                             timestamps[id], boot_id, packet) != 0) { ++encode_errors; continue; }
                active = 1; half = 0; ++packets_started;
                break;
            }
        }
        if (active && nrf24_tx_start(&radio, packet+16u*half, 16, now) != 0) {
            ++fragments_failed; active = 0; half = 0; next_us = now+FRAGMENT_GAP_US;
        }
    }
    update_diagnostic(d);
}
