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

#endif
