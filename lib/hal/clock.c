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
 * with the LED on. Motors are not initialised at that point, so a halt
 * is safe here in a way it would not be later.
 */

#include "stm32f1xx.h"
#include "board_config.h"
#include "clock.h"

// ============================================================
// Clock setup for STM32F103C8T6
// HSE 8MHz -> PLL x9 -> 72MHz sysclk
// USB gets exactly 48MHz via the /1.5 prescaler
// Any other combination breaks USB enumeration
// ============================================================

void clock_init(void)
{
    // Flash needs 2 wait states above 48MHz
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_2;
    // Prefetch buffer hides most of the wait state cost
    FLASH->ACR |= FLASH_ACR_PRFTBE;

    // Start the external crystal
    RCC->CR |= RCC_CR_HSEON;
    uint32_t guard = 0;
    while (!(RCC->CR & RCC_CR_HSERDY)) {
        // If the crystal never starts we stay on HSI and USB will not work
        if (++guard > 0x10000U) {
            clock_fault_handler();
        }
    }

    // Bus prescalers, set before switching so nothing is overclocked
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;    // AHB  = 72MHz
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;   // APB1 = 36MHz, timer clock still 72MHz
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;   // APB2 = 72MHz

    // ADC max clock is 14MHz, /6 gives 12MHz
    RCC->CFGR &= ~RCC_CFGR_ADCPRE;
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;

    // PLL source HSE, no prediv, multiply by 9
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
    RCC->CFGR |= RCC_CFGR_PLLSRC_HSE;
    RCC->CFGR |= RCC_CFGR_PLLMULL9;

    // USB prescaler /1.5 -> 72 / 1.5 = 48MHz
    // On F103 this bit cleared means divide by 1.5
    RCC->CFGR &= ~RCC_CFGR_USBPRE;

    RCC->CR |= RCC_CR_PLLON;
    guard = 0;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {
        if (++guard > 0x10000U) {
            clock_fault_handler();
        }
    }

    // Switch the system clock over to the PLL
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
        // Wait for the hardware to confirm the switch
    }

    // Enable the peripheral clocks used by this firmware
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN
                  | RCC_APB2ENR_IOPBEN
                  | RCC_APB2ENR_IOPCEN
                  | RCC_APB2ENR_AFIOEN
                  | RCC_APB2ENR_ADC1EN;

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN
                  | RCC_APB1ENR_TIM3EN
                  | RCC_APB1ENR_TIM4EN
                  | RCC_APB1ENR_I2C2EN;

    // Free PA13/PA14 SWD only, release PB3/PB4/PA15 from JTAG
    // Do this after AFIO clock is on, and never disable SWD itself
    AFIO->MAPR &= ~AFIO_MAPR_SWJ_CFG;
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
}

// Called when the crystal or PLL fails to lock
// Motors are not running yet at this point so a halt is safe
void clock_fault_handler(void)
{
    // Drive the onboard LED on and stop
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    GPIOC->CRH &= ~(0xFU << 20);
    GPIOC->CRH |= (0x2U << 20);   // output push pull 2MHz
    GPIOC->BSRR = (1U << (LED_PIN + 16));  // active low, LED on
    while (1) {
        __NOP();
    }
}
