/*
 * File   : systick.h
 * Purpose: Millisecond timebase interface.
 * Author : jihoonkimtech
 */

#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>

void     systick_init(void);
uint32_t systick_millis(void);
void     systick_delay_ms(uint32_t ms);
void     systick_delay_us(uint32_t us);   // busy wait, for bit banging

#endif
