/*
 * File   : debug_uart.c
 * Purpose: Minimal blocking USART2 transmitter for bring-up messages,
 *          plus a tiny formatter that avoids pulling in newlib printf.
 * Author : jihoonkimtech
 *
 * Pin map
 *   USART2 TX = PA2   -> connect to the RX pin of a USB-TTL adapter
 *   GND               -> common ground with the adapter
 *   115200 8N1, transmit only
 *
 * USART1 would be the usual choice but its TX sits on PA9, which now
 * belongs to the left encoder. USART3 lands on PB10/PB11, taken by the
 * IMU. PA2 is the only conflict free option left on this pin map.
 *
 * Note: printing is blocking. That is fine during calibration but it
 * would wreck the 200Hz control loop, so keep debug_uart out of the
 * normal drive path.
 */

#include "stm32f1xx.h"
#include "board_config.h"
#include "debug_uart.h"

#define DBG_BAUD    115200UL
// APB1 runs at 36MHz, BRR holds the divider in 12.4 fixed point
#define APB1_CLK_HZ (SYSCLK_FREQ_HZ / 2U)

void debug_uart_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    // PA2 as alternate function push pull, 50MHz
    GPIOA->CRL &= ~(0xFU << (2U * 4U));
    GPIOA->CRL |=  (0xBU << (2U * 4U));

    USART2->BRR = (APB1_CLK_HZ + (DBG_BAUD / 2U)) / DBG_BAUD;
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
}

void debug_putc(char c)
{
    while (!(USART2->SR & USART_SR_TXE)) {
        // Wait for the transmit register to drain
    }
    USART2->DR = (uint8_t)c;
}

void debug_puts(const char *s)
{
    while (*s) {
        if (*s == '\n') {
            debug_putc('\r');
        }
        debug_putc(*s++);
    }
}

static void put_uint(uint32_t v, uint32_t base, int width, char pad)
{
    char buf[12];
    int  n = 0;

    if (v == 0U) {
        buf[n++] = '0';
    }
    while (v > 0U) {
        uint32_t d = v % base;
        buf[n++] = (char)((d < 10U) ? ('0' + d) : ('a' + d - 10U));
        v /= base;
    }
    while (n < width) {
        buf[n++] = pad;
    }
    while (n > 0) {
        debug_putc(buf[--n]);
    }
}

static void put_int(int32_t v, int width, char pad)
{
    if (v < 0) {
        debug_putc('-');
        put_uint((uint32_t)(-v), 10U, width - 1, pad);
    } else {
        put_uint((uint32_t)v, 10U, width, pad);
    }
}

// Prints a value carried in thousandths as a decimal, 1234 -> 1.234
void debug_milli(int32_t milli)
{
    if (milli < 0) {
        debug_putc('-');
        milli = -milli;
    }
    put_uint((uint32_t)(milli / 1000), 10U, 0, ' ');
    debug_putc('.');
    put_uint((uint32_t)(milli % 1000), 10U, 3, '0');
}

// Supports %d %u %x %s %c %% and a width prefix such as %6d
// Deliberately no %f, soft float formatting is not worth the flash
void debug_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);

    while (*fmt) {
        if (*fmt != '%') {
            if (*fmt == '\n') {
                debug_putc('\r');
            }
            debug_putc(*fmt++);
            continue;
        }
        fmt++;

        char pad   = ' ';
        int  width = 0;
        if (*fmt == '0') {
            pad = '0';
            fmt++;
        }
        while (*fmt >= '0' && *fmt <= '9') {
            width = width * 10 + (*fmt++ - '0');
        }

        switch (*fmt++) {
        case 'd': put_int(va_arg(ap, int32_t), width, pad);          break;
        case 'u': put_uint(va_arg(ap, uint32_t), 10U, width, pad);   break;
        case 'x': put_uint(va_arg(ap, uint32_t), 16U, width, pad);   break;
        case 'c': debug_putc((char)va_arg(ap, int));                 break;
        case 's': debug_puts(va_arg(ap, const char *));              break;
        case 'm': debug_milli(va_arg(ap, int32_t));                  break;
        case '%': debug_putc('%');                                   break;
        default:  debug_putc('?');                                   break;
        }
    }
    va_end(ap);
}
