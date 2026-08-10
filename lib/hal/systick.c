/*
 * File   : systick.c
 * Purpose: 1ms timebase driving the cooperative scheduler.
 * Author : jihoonkimtech
 *
 * Note: SysTick runs at the lowest interrupt priority (15) so USB and
 * any future encoder overflow handler preempt it. A 32 bit read of the
 * counter is atomic on Cortex-M3, so no critical section is needed.
 */

#include "stm32f1xx.h"
#include "board_config.h"
#include "systick.h"

// ============================================================
// 1ms timebase for the cooperative scheduler
// Kept deliberately simple, no RTOS on this build
// ============================================================

static volatile uint32_t s_millis;

void systick_init(void)
{
    // Reload for a 1ms tick at 72MHz
    SysTick->LOAD = (SYSCLK_FREQ_HZ / 1000U) - 1U;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                  | SysTick_CTRL_TICKINT_Msk
                  | SysTick_CTRL_ENABLE_Msk;

    // Lowest urgency, USB and encoder overflow must preempt this
    NVIC_SetPriority(SysTick_IRQn, 15);
}

void SysTick_Handler(void)
{
    s_millis++;
}

uint32_t systick_millis(void)
{
    // 32 bit read is atomic on Cortex-M3
    return s_millis;
}

void systick_delay_ms(uint32_t ms)
{
    uint32_t start = s_millis;
    while ((s_millis - start) < ms) {
        __WFI();
    }
}
