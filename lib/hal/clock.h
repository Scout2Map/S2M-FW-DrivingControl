/*
 * File   : clock.h
 * Purpose: System clock initialisation entry points.
 * Author : jihoonkimtech
 */

#ifndef CLOCK_H
#define CLOCK_H

typedef enum {
    CLOCK_FAULT_HSE = 0,   // crystal never signalled ready
    CLOCK_FAULT_PLL        // PLL never locked
} clock_fault_t;

void clock_init(void);
void clock_fault_handler(clock_fault_t reason);

#endif
