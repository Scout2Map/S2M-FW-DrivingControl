/*
 * File   : clock.c
 * Purpose: System clock tree setup for the STM32F103C8T6.
 *          HSE 8MHz -> PLL x9 -> 72MHz sysclk, with the USB prescaler
 *          set to /1.5 so the USB peripheral sees exactly 48MHz.
 * Author : jihoonkimtech
 *
 * Clock domains after init
 *   SYSCLK  72MHz   AHB   72MHz   APB2  72MHz
 *   APB1    36MHz   but the timer clock doubles back to 72MHz
 *   ADC     12MHz   (/6, the 14MHz ceiling is a hard limit)
 *   USB     48MHz   (/1.5, any other ratio breaks enumeration)
 *
 * Note: flash needs two wait states above 48MHz, set before the switch.
 * If the crystal or the PLL fails to lock, clock_fault_handler() traps
 * and reports which stage failed through the LED blink count. Motors are
 * not initialised at that point, so halting is safe here in a way it
 * would not be later.
 */

#include "stm32f1xx.h"
#include "board_config.h"
#include "clock.h"
#include "led.h"

// ============================================================
// Clock setup for STM32F103C8T6
// HSE 8MHz -> PLL x9 -> 72MHz sysclk
// USB gets exactly 48MHz via the /1.5 prescaler
// Any other combination breaks USB enumeration
// ============================================================

// Bit patterns written directly rather than via HAL style symbols.
// The bare CMSIS device header only defines the field masks, not the
// per-value constants that come with the ST HAL, so the values below
// are taken straight from RM0008.
#define CFGR_PLLSRC_HSE     (1U << 16)  // PLL fed from HSE
#define CFGR_PLLMULL_9      (7U << 18)  // multiplier 9, encoding is n-2
#define CFGR_HPRE_DIV1      (0U << 4)
#define CFGR_PPRE1_DIV2     (4U << 8)
#define CFGR_PPRE2_DIV1     (0U << 11)
#define CFGR_ADCPRE_DIV6    (2U << 14)
#define CFGR_SW_PLL         (2U << 0)
#define CFGR_SWS_PLL        (2U << 2)
#define ACR_LATENCY_2       (2U << 0)
#define MAPR_SWJ_JTAGDISABLE (2U << 24) // SWD kept, JTAG pins released

// Startup budgets. Generous on purpose, a clone crystal can take tens
// of milliseconds to stabilise and a premature give up looks exactly
// like a dead crystal.
#define HSE_START_TIMEOUT   0x1000000U
#define PLL_LOCK_TIMEOUT    0x0100000U

void clock_init(void)
{
    // Flash needs two wait states above 48MHz
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= ACR_LATENCY_2;
    // The prefetch buffer hides most of the wait state cost
    FLASH->ACR |= FLASH_ACR_PRFTBE;

    // Crystal mode rather than an external clock input. The reset value
    // is already 0, but being explicit rules out a stale bit surviving
    // a soft reset.
    RCC->CR &= ~RCC_CR_HSEBYP;
    RCC->CR |= RCC_CR_HSEON;

    uint32_t guard = 0;
    while (!(RCC->CR & RCC_CR_HSERDY)) {
        // Clone boards sometimes carry a slow starting crystal, so this
        // budget is deliberately generous. Giving up early looks exactly
        // like a dead crystal and sends debugging down the wrong path.
        if (++guard > HSE_START_TIMEOUT) {
            clock_fault_handler(CLOCK_FAULT_HSE);
        }
    }

    // Bus prescalers, set before the switch so nothing is overclocked
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= CFGR_HPRE_DIV1;    // AHB  = 72MHz
    RCC->CFGR |= CFGR_PPRE1_DIV2;   // APB1 = 36MHz, timer clock still 72MHz
    RCC->CFGR |= CFGR_PPRE2_DIV1;   // APB2 = 72MHz

    // ADC tops out at 14MHz, /6 lands on 12MHz
    RCC->CFGR &= ~RCC_CFGR_ADCPRE;
    RCC->CFGR |= CFGR_ADCPRE_DIV6;

    // PLL source HSE, no prediv, multiply by 9
    // PLLSRC and PLLMULL may only be written while the PLL is off
    RCC->CR &= ~RCC_CR_PLLON;
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
    RCC->CFGR |= CFGR_PLLSRC_HSE;
    RCC->CFGR |= CFGR_PLLMULL_9;

    // USB prescaler /1.5 -> 72 / 1.5 = 48MHz
    // On F103 a cleared bit selects the 1.5 divider
    RCC->CFGR &= ~RCC_CFGR_USBPRE;

    RCC->CR |= RCC_CR_PLLON;
    guard = 0;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {
        if (++guard > PLL_LOCK_TIMEOUT) {
            clock_fault_handler(CLOCK_FAULT_PLL);
        }
    }

    // Switch the system clock over to the PLL
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != CFGR_SWS_PLL) {
        // Wait for the hardware to confirm the switch
    }

    // Peripheral clocks used by this firmware
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN
                  | RCC_APB2ENR_IOPBEN
                  | RCC_APB2ENR_IOPCEN
                  | RCC_APB2ENR_AFIOEN
                  | RCC_APB2ENR_ADC1EN;

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN
                  | RCC_APB1ENR_TIM3EN
                  | RCC_APB1ENR_TIM4EN
                  | RCC_APB1ENR_I2C2EN;

    // Release PB3/PB4/PA15 from JTAG while keeping SWD alive
    // Must follow the AFIO clock enable, and SWD itself is never disabled
    AFIO->MAPR &= ~AFIO_MAPR_SWJ_CFG;
    AFIO->MAPR |= MAPR_SWJ_JTAGDISABLE;
}

// Reports a clock failure through the LED and never returns.
// Motors are not initialised at this point, so halting is safe here in
// a way it would not be once the drivers are live.
//
// The blink count identifies which stage failed, which matters because
// a dead crystal and a bad PLL configuration need different fixes:
//   2 blinks -> HSE never became ready, suspect the crystal or its caps
//   4 blinks -> PLL never locked, suspect the multiplier or source bits
void clock_fault_handler(clock_fault_t reason)
{
    led_init();

    uint8_t count = (reason == CLOCK_FAULT_HSE) ? 2U : 4U;

    while (1) {
        led_blink_blocking(count);
        // Long gap so the group boundary is obvious to the eye
        for (volatile uint32_t d = 0; d < 1500000U; d++) {
            IWDG->KR = IWDG_REFRESH_KEY;
        }
    }
}
