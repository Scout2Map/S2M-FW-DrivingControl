/*
 * File   : main.c
 * Purpose: Scout2Map drive control MCU entry point (STM32F103C8T6).
 *          Runs a cooperative superloop that services the velocity
 *          control loop, sensor polling and telemetry on independent
 *          periods, all timed against a 1ms SysTick base.
 * Author : jihoonkimtech
 *
 * Boot order is safety critical. motor_init() drives both BTS7960
 * enable pins low and zeroes every PWM compare register before
 * motor_enable() is ever reached, so a reset cannot kick the wheels.
 *
 * Note: the IWDG is refreshed only from the healthy path at the end
 * of the loop. A task that hangs will stop the refresh and force a
 * reset rather than leaving the robot driving unattended.
 */

#include "stm32f1xx.h"
#include "board_config.h"
#include "board_io.h"
#include "clock.h"
#include "systick.h"
#include "motor.h"
#include "encoder.h"
#include "led.h"
#include "drive.h"
#include "calib.h"

// ============================================================
// Cooperative scheduler, no RTOS
// Each task compares its own deadline against the 1ms tick
// ============================================================

static void iwdg_init(void)
{
    // Independent watchdog on the 40kHz LSI
    // Prescaler 32 with reload 500 lands near 400ms
    IWDG->KR  = 0x5555U;
    IWDG->PR  = 3U;
    IWDG->RLR = 500U;
    IWDG->KR  = 0xAAAAU;
    IWDG->KR  = 0xCCCCU;
}

int main(void)
{
    // Order matters, the motors must be silent before anything else runs
    clock_init();
    systick_init();
    led_init();
    // Two blinks on entry answer the first bring-up question, namely
    // whether the firmware reached main at all
    led_blink_blocking(2);
    motor_init();
#if ENCODER_AVAILABLE
    encoder_init();
#endif
#if CALIB_MODE
    // Bench mode. Motors are left disabled and this never returns.
    iwdg_init();
    calib_run();
#endif

    drive_init(board_io_get());

    // Let the drivers and the UBEC rail settle
    systick_delay_ms(200);

    iwdg_init();
    motor_enable();

    uint32_t t_loop  = 0;
    uint32_t t_imu   = 0;
    uint32_t t_adc   = 0;
    uint32_t t_telem = 0;
    uint32_t t_led   = 0;

    while (1) {
        uint32_t now = systick_millis();

        if ((now - t_loop) >= LOOP_PERIOD_MS) {
            t_loop = now;
            drive_update();
        }

        if ((now - t_imu) >= IMU_PERIOD_MS) {
            t_imu = now;
            // imu_poll();  non blocking state machine, added next
        }

        if ((now - t_adc) >= ADC_PERIOD_MS) {
            t_adc = now;
            // dist_sample();  GP2D120 median filter, added next
        }

        if ((now - t_telem) >= TELEM_PERIOD_MS) {
            t_telem = now;
            // telem_send();  binary frame over USB CDC, added next
        }

        // Heartbeat, a frozen LED means the loop died
        if ((now - t_led) >= 500U) {
            t_led = now;
            led_toggle();
        }

        // Refresh the watchdog only from the healthy path
        IWDG->KR = 0xAAAAU;
    }
}
