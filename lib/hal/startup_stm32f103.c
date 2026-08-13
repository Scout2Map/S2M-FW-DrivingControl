/*
 * File   : startup_stm32f103.c
 * Purpose: Minimal C startup for the STM32F103C8T6. Provides the vector
 *          table, the reset handler, and .data / .bss initialisation.
 * Author : jihoonkimtech
 *
 * Written in C rather than assembly to keep the whole project in one
 * language. Symbols come from ld/stm32f103c8.ld.
 *
 * Note: only the vectors this firmware actually uses are named. Every
 * other entry aliases Default_Handler, which traps in a loop and lets
 * the IWDG reset the board.
 */

#include <stdint.h>

// ============================================================
// Minimal startup for STM32F103C8T6
// Vector table, reset handler, data and bss initialisation
// ============================================================

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;

extern int main(void);
void Reset_Handler(void);
void Default_Handler(void);
void SysTick_Handler(void);

// Every unused vector falls back to a trap loop
void NMI_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void USB_LP_CAN1_RX0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_UP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM4_IRQHandler(void)    __attribute__((weak, alias("Default_Handler")));

__attribute__((section(".isr_vector"), used))
void (* const g_vectors[])(void) = {
    (void (*)(void))&_estack,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0, 0, 0, 0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
    // External interrupts start here, only the ones we use are named
    [16 + 25] = TIM1_UP_IRQHandler,
    [16 + 20] = USB_LP_CAN1_RX0_IRQHandler,
    [16 + 30] = TIM4_IRQHandler,
};

void Reset_Handler(void)
{
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;

    // Copy initialised data from flash to ram
    while (dst < &_edata) {
        *dst++ = *src++;
    }
    // Zero the bss section
    dst = &_sbss;
    while (dst < &_ebss) {
        *dst++ = 0;
    }

    main();

    // main must never return, trap if it does
    while (1) { }
}

void Default_Handler(void)
{
    // Deliberately does not refresh the watchdog. An unexpected vector
    // means the firmware is in an undefined state, so letting the IWDG
    // reset the board is the correct response rather than spinning here.
    while (1) { }
}
