/* Dual-IMU receive-only bring-up. No IMU commands, RF transmission or USB stack.
 * Only USART1/2 RX, TIM2 and SysTick run. PA4=CSN high, PB0=CE low.
 * MCU remains accessible over SWD; all measurement payloads originate at UART.
 */
#include "stm32f103x6.h"
#include "imu406_stream.h"
#include <stddef.h>
#ifdef RADIO_TELEMETRY
#include "radio_app.h"
#endif

#define CAPACITY 128u
#define UART_ERRORS (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)

typedef struct {
    uint32_t bytes, frames, rejected, uart_errors, queue_drops;
    uint32_t last_frame_us, tail_pos, max_queue_depth;
    uint8_t frame[32], tail[16];
} sensor_diag_t;

typedef struct {
    uint32_t magic, abi, size, lock, uptime_ms, core_hz, fault, reset_flags;
    sensor_diag_t sensor[2];
    uint32_t reserved[16];
} diagnostic_t;

_Static_assert(sizeof(diagnostic_t) == 256, "Diagnostic ABI");
_Static_assert(offsetof(diagnostic_t, sensor[1]) == 112, "Rear ABI offset");
__attribute__((section(".diagnostic"), used)) volatile diagnostic_t diagnostic;
static volatile diagnostic_t working;

typedef struct { uint32_t us; uint16_t errors; uint8_t byte, unused; } event_t;
typedef struct {
    volatile uint32_t head, tail, drops;
    event_t events[CAPACITY];
    imu406_stream_t stream;
    uint32_t handled_drops, last_byte_us;
} channel_t;

static channel_t channels[2];
static volatile uint32_t timer_high;
uint32_t SystemCoreClock = 64000000u;

static uint32_t lock_diagnostic(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    ++working.lock;
    __DMB();
    return mask;
}

static void unlock_diagnostic(uint32_t mask)
{
    __DMB();
    ++working.lock;
    __set_PRIMASK(mask);
}

void SystemInit(void)
{
    uint32_t n;
    RCC->CR |= RCC_CR_HSION;
    /* Reset path uses the internal oscillator; do not assume board HSE wiring. */
    RCC->CFGR = 0;
    RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_HSEON | RCC_CR_CSSON);
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;
    RCC->CFGR = RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PLLMULL16;
    RCC->CR |= RCC_CR_PLLON; /* HSI/2 * 16 = nominal 64 MHz */
    for (n = 0; n < 1000000u && !(RCC->CR & RCC_CR_PLLRDY); ++n) {}
    if (RCC->CR & RCC_CR_PLLRDY) {
        RCC->CFGR |= RCC_CFGR_SW_PLL;
        for (n = 0; n < 1000000u &&
             (RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL; ++n) {}
    }
    SCB->VTOR = 0x08000000u;
}

/* ST startup calls this; this C-only build has no constructors. */
void __libc_init_array(void) {}

void SystemCoreClockUpdate(void)
{
    SystemCoreClock = (RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL ? 64000000u : 8000000u;
}

static void publish_diagnostic(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint32_t version = (diagnostic.lock + 2u) & ~1u;
    diagnostic.lock = version | 1u;
    __DMB();
    for (unsigned i = 0; i < 64u; ++i) {
        if (i != 3u) ((volatile uint32_t *)&diagnostic)[i] = ((volatile uint32_t *)&working)[i];
    }
    __DMB();
    diagnostic.lock = version + 2u;
    __set_PRIMASK(mask);
}

static uint32_t now_us(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint32_t high = timer_high;
    uint32_t count = TIM2->CNT;
    if ((TIM2->SR & TIM_SR_UIF) && count < 32768u) high += 65536u;
    __set_PRIMASK(mask);
    return high + count;
}

void TIM2_IRQHandler(void)
{
    TIM2->SR = ~TIM_SR_UIF;
    timer_high += 65536u;
}

void SysTick_Handler(void)
{
    uint32_t mask = lock_diagnostic();
    ++working.uptime_ms;
    unlock_diagnostic(mask);
}

static void receive_byte(unsigned id, USART_TypeDef *uart)
{
    uint32_t status = uart->SR;
    if (status & (USART_SR_RXNE | UART_ERRORS)) {
        uint8_t byte = (uint8_t)uart->DR; /* SR then DR clears UART errors. */
        uint32_t us = now_us();
        channel_t *c = &channels[id];
        uint32_t head = c->head;
        uint32_t depth = head - c->tail;
        uint32_t mask = lock_diagnostic();
        volatile sensor_diag_t *d = &working.sensor[id];
        ++d->bytes;
        d->tail[d->tail_pos++ % 16u] = byte;
        if (status & UART_ERRORS) ++d->uart_errors;
        if (depth < CAPACITY) {
            c->events[head % CAPACITY] = (event_t){us, (uint16_t)(status & UART_ERRORS), byte, 0};
            __DMB();
            c->head = head + 1u;
            if (depth + 1u > d->max_queue_depth) d->max_queue_depth = depth + 1u;
        } else {
            ++c->drops;
            ++d->queue_drops;
        }
        unlock_diagnostic(mask);
    }
}

void USART1_IRQHandler(void) { receive_byte(0, USART1); }
void USART2_IRQHandler(void) { receive_byte(1, USART2); }

static void process_channel(unsigned id)
{
    channel_t *c = &channels[id];
    uint32_t tail = c->tail;
    uint8_t frame[32];
    event_t e;
    uint32_t drops, mask;
    if (tail == c->head) return;
    __DMB();
    e = c->events[tail % CAPACITY];
    __DMB();
    c->tail = tail + 1u;
    drops = c->drops;
    if (drops != c->handled_drops) {
        /* Discard queued fragments when their byte continuity is unknown. */
        __disable_irq();
        c->tail = c->head;
        c->handled_drops = c->drops;
        __enable_irq();
        imu406_stream_discontinuity(&c->stream);
        return;
    }
    if (e.errors || (uint32_t)(e.us - c->last_byte_us) > 100000u) {
        imu406_stream_discontinuity(&c->stream);
    }
    c->last_byte_us = e.us;
    if (e.errors) return;
    int valid = imu406_stream_push(&c->stream, e.byte, frame);
    mask = lock_diagnostic();
    working.sensor[id].rejected = c->stream.rejected;
    if (valid) {
        ++working.sensor[id].frames;
        working.sensor[id].last_frame_us = e.us;
        for (unsigned i = 0; i < 32u; ++i) working.sensor[id].frame[i] = frame[i];
    }
    unlock_diagnostic(mask);
#ifdef RADIO_TELEMETRY
    if (valid) radio_app_frame(id, frame, e.us, working.sensor[id].frames);
#endif
}

void HardFault_Handler(void)
{
    __disable_irq();
#ifdef RADIO_TELEMETRY
    GPIOB->BRR = 1u; /* Drop RF CE even if the diagnostic main loop faults. */
#endif
    working.fault = 0x80000003u;
    publish_diagnostic();
    while (1) __WFI();
}

int main(void)
{
    __disable_irq();
    diagnostic.magic = 0;
    for (unsigned i = 0; i < sizeof(diagnostic) / 4u; ++i) ((volatile uint32_t *)&working)[i] = 0;
    working.abi = 1;
    working.size = sizeof(diagnostic);
    working.reset_flags = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;
    SystemCoreClockUpdate();
    working.core_hz = SystemCoreClock;
    if (SystemCoreClock != 64000000u) working.fault = 1; /* Visible fallback. */
#ifdef RADIO_TELEMETRY
    /* Commit a new boot identifier before enabling UART interrupts or RF. */
    uint32_t boot = radio_boot_id();
#endif
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_USART1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN | RCC_APB1ENR_TIM2EN;
    /* SPI is not started. Keep the wired radio deselected and CE low. */
    GPIOA->BSRR = 1u << 4;
    GPIOA->CRL = (GPIOA->CRL & ~(15u << 16)) | (2u << 16);
    GPIOB->BRR = 1u;
    GPIOB->CRL = (GPIOB->CRL & ~15u) | 2u;
    /* Both IMU TX pins connect to pulled-up receive inputs. MCU TX stays input. */
    GPIOA->BSRR = (1u << 10) | (1u << 3);
    GPIOA->CRH = (GPIOA->CRH & ~(15u << 8)) | (8u << 8);
    GPIOA->CRL = (GPIOA->CRL & ~(15u << 12)) | (8u << 12);
    TIM2->PSC = SystemCoreClock / 1000000u - 1u;
    TIM2->ARR = 65535u;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0;
    TIM2->DIER = TIM_DIER_UIE;
    TIM2->CR1 = TIM_CR1_CEN;
    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);
    USART1->BRR = (SystemCoreClock + 57600u) / 115200u;
    USART2->BRR = (SystemCoreClock / 2u + 57600u) / 115200u;
    USART1->CR2 = USART2->CR2 = 0;
    USART1->CR3 = USART2->CR3 = USART_CR3_EIE;
    USART1->CR1 = USART2->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE;
    NVIC_SetPriority(USART1_IRQn, 2);
    NVIC_SetPriority(USART2_IRQn, 2);
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(USART2_IRQn);
    SysTick_Config(SystemCoreClock / 1000u);
    NVIC_SetPriority(SysTick_IRQn, 3);
    working.magic = 0x31444d49u; /* Little-endian 'IMD1'; set after init. */
    publish_diagnostic();
    uint32_t published_ms = working.uptime_ms;
    __enable_irq();
#ifdef RADIO_TELEMETRY
    radio_app_init(now_us, boot);
#endif
    while (1) {
        process_channel(0);
        process_channel(1);
#ifdef RADIO_TELEMETRY
        radio_app_poll(now_us(), working.uptime_ms, working.reserved);
#endif
        if ((uint32_t)(working.uptime_ms - published_ms) >= 200u) {
            publish_diagnostic();
            published_ms = working.uptime_ms;
        }
        /* UART interrupts wake WFI, SysTick bounds a missed-wakeup to 1 ms. */
        if (channels[0].head == channels[0].tail && channels[1].head == channels[1].tail) __WFI();
    }
}
