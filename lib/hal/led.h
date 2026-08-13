/*
 * File   : led.h
 * Purpose: Status LED interface. Board specific port, pin and polarity
 *          are configured in board_config.h.
 * Author : jihoonkimtech
 */

#ifndef LED_H
#define LED_H

#include <stdint.h>

void led_init(void);
void led_on(void);
void led_off(void);
void led_toggle(void);
void led_set(uint8_t on);
void led_blink_blocking(uint8_t times);   // works before SysTick exists

#endif
