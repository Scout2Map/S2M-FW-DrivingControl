/*
 * File   : led.c
 * Purpose: Onboard status LED control, isolated so the board specific
 *          port, pin and polarity live in exactly one place.
 * Author : jihoonkimtech
 *
 * This particular STM32F103C8T6 core board is NOT a standard Blue Pill.
 * The schematic routes the user LED D2 through R6 to PB12, while a
 * stock Blue Pill uses PC13. Chasing that difference cost a full
 * debugging session, hence this module: the mapping is now a pair of
 * macros in board_config.h and nothing else touches the port directly.
 *
 * Note: LED_ACTIVE_LOW selects the polarity. If the LED reads inverted
 * on a different board revision, flip that macro rather than editing
 * the logic here.
 */

#include "stm32f1xx.h"
#include "board_config.h"
#include "led.h"

// Maps a port pointer to its RCC enable bit without a lookup table
static void led_clock_enable(void)
{
    if (LED_PORT == GPIOA)      RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    else if (LED_PORT == GPIOB) RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    else                        RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
}

void led_init(void)
{
    led_clock_enable();

    // CRL covers pins 0..7, CRH covers 8..15, four bits each
    volatile uint32_t *cr = (LED_PIN < 8U) ? &LED_PORT->CRL : &LED_PORT->CRH;
    uint32_t shift = (LED_PIN % 8U) * 4U;

    *cr &= ~(0xFU << shift);
    *cr |=  (0x2U << shift);    // output push pull, 2MHz is plenty

    led_off();
}

void led_on(void)
{
#if LED_ACTIVE_LOW
    LED_PORT->BSRR = (1U << (LED_PIN + 16U));
#else
    LED_PORT->BSRR = (1U << LED_PIN);
#endif
}

void led_off(void)
{
#if LED_ACTIVE_LOW
    LED_PORT->BSRR = (1U << LED_PIN);
#else
    LED_PORT->BSRR = (1U << (LED_PIN + 16U));
#endif
}

void led_toggle(void)
{
    LED_PORT->ODR ^= (1U << LED_PIN);
}

void led_set(uint8_t on)
{
    if (on) {
        led_on();
    } else {
        led_off();
    }
}

// Blocking blink used during bring-up and fault reporting.
// Runs off a spin loop so it works before SysTick is configured.
void led_blink_blocking(uint8_t times)
{
    for (uint8_t i = 0; i < times; i++) {
        led_on();
        for (volatile uint32_t d = 0; d < 300000U; d++) { }
        led_off();
        for (volatile uint32_t d = 0; d < 300000U; d++) { }
    }
}
