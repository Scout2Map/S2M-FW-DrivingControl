/*
 * File   : debug_uart.h
 * Purpose: Blocking USART2 debug output for bring-up and calibration.
 * Author : jihoonkimtech
 */

#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include <stdint.h>
#include <stdarg.h>

void debug_uart_init(void);
void debug_putc(char c);
void debug_puts(const char *s);
void debug_milli(int32_t milli);           // 1234 prints as 1.234
void debug_printf(const char *fmt, ...);   // %d %u %x %s %c %m %%

#endif
