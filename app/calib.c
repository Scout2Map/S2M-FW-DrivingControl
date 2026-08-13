/*
 * File   : calib.c
 * Purpose: Encoder calibration mode. Reports raw counts so that
 *          COUNTS_PER_WHEEL_REV, the encoder PPR and the counting
 *          direction can all be confirmed on the bench.
 * Author : jihoonkimtech
 *
 * The motors are never enabled here. Both BTS7960 enable pins stay low
 * for the whole session so a stray command cannot spin a wheel while
 * hands are on it.
 *
 * Two readouts are provided so the mode is usable with or without a
 * USB-TTL adapter:
 *   USART2 PA2 at 115200 gives the full numeric trace
 *   The onboard LED lights once the left encoder passes ten wheel
 *   revolutions under the 11 PPR assumption, which is enough on its own
 *   to tell 11 PPR from 13 PPR
 *
 * Note: turn the wheel by hand slowly. Spinning it fast is fine for the
 * counter but makes it hard to see where the LED actually lights.
 */

#include "stm32f1xx.h"
#include "board_config.h"
#include "calib.h"
#include "debug_uart.h"
#include "encoder.h"
#include "led.h"
#include "systick.h"

// Candidate resolutions, the whole point of the exercise
#define CPR_11PPR   (11 * 4 * GEAR_RATIO)   // 5764 at 131:1
#define CPR_13PPR   (13 * 4 * GEAR_RATIO)   // 6812 at 131:1

#define CALIB_REVS  10
#define LED_TRIGGER (CPR_11PPR * CALIB_REVS)

static void print_banner(void)
{
    debug_printf("\n");
    debug_printf("=====================================================\n");
    debug_printf(" Scout2Map encoder calibration\n");
    debug_printf("=====================================================\n");
    debug_printf(" Gear ratio in config : %d:1\n", (int32_t)GEAR_RATIO);
    debug_printf(" Counts per wheel rev\n");
    debug_printf("   if 11 PPR : %d\n", (int32_t)CPR_11PPR);
    debug_printf("   if 13 PPR : %d\n", (int32_t)CPR_13PPR);
    debug_printf(" Currently configured : %d\n",
                 (int32_t)COUNTS_PER_WHEEL_REV);
    debug_printf("-----------------------------------------------------\n");
    debug_printf(" Lift the chassis so both wheels spin freely.\n");
    debug_printf(" Mark a start point on one wheel.\n");
    debug_printf(" Turn it slowly, exactly %d revolutions, forward.\n",
                 (int32_t)CALIB_REVS);
    debug_printf("\n");
    debug_printf(" LED lights at %d counts, ten revs if 11 PPR.\n",
                 (int32_t)LED_TRIGGER);
    debug_printf("   LED at exactly 10 rev -> 11 PPR, use %d\n",
                 (int32_t)CPR_11PPR);
    debug_printf("   LED near  8.5 rev     -> 13 PPR, use %d\n",
                 (int32_t)CPR_13PPR);
    debug_printf("\n");
    debug_printf(" Counts must RISE when the wheel turns forward.\n");
    debug_printf(" If a side falls instead, flip ENC_x_INVERT.\n");
    debug_printf("=====================================================\n\n");
}

void calib_run(void)
{
    debug_uart_init();
    encoder_init();
    encoder_reset();

    // Three blinks mark calibration mode, distinct from the two blinks
    // main emits on boot, so the active mode is visible without a terminal
    led_blink_blocking(3);
    led_off();

    print_banner();

    uint32_t t_print = 0;
    uint32_t t_loop  = 0;
    uint8_t  latched = 0;

    while (1) {
        uint32_t now = systick_millis();

        // Sample at the normal control rate so wrap handling is exercised
        if ((now - t_loop) >= LOOP_PERIOD_MS) {
            t_loop = now;
            encoder_update();
        }

        if ((now - t_print) >= 250U) {
            t_print = now;

            int32_t lt = encoder_get_total(ENC_LEFT);
            int32_t rt = encoder_get_total(ENC_RIGHT);

            // Revolutions implied by each hypothesis, in thousandths
            int32_t l_rev11 = (lt * 1000) / CPR_11PPR;
            int32_t l_rev13 = (lt * 1000) / CPR_13PPR;
            int32_t r_rev11 = (rt * 1000) / CPR_11PPR;
            int32_t r_rev13 = (rt * 1000) / CPR_13PPR;

            debug_printf("L %8d (11PPR %mrev  13PPR %mrev)   ",
                         lt, l_rev11, l_rev13);
            debug_printf("R %8d (11PPR %mrev  13PPR %mrev)\n",
                         rt, r_rev11, r_rev13);

            int32_t mag = (lt < 0) ? -lt : lt;
            if (!latched && mag >= LED_TRIGGER) {
                latched = 1;
                led_on();
                debug_printf("\n>>> LED ON at %d counts. Read the wheel now.\n",
                             lt);
                debug_printf(">>> Exactly 10 revolutions -> ENC_PPR = 11\n");
                debug_printf(">>> Short of 10 revolutions -> ENC_PPR = 13\n\n");
            }
        }

        IWDG->KR = IWDG_REFRESH_KEY;
    }
}
