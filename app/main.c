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
#include "link.h"
#include "bno055.h"
#include "adc.h"
#include "calib.h"

// ============================================================
// Cooperative scheduler, no RTOS
// Each task compares its own deadline against the 1ms tick
// ============================================================

static void iwdg_init(void)
{
    // Independent watchdog on the 40kHz LSI, see board_config.h
    // Any blocking routine longer than IWDG_TIMEOUT_MS must refresh
    // the key register itself
    IWDG->KR  = IWDG_UNLOCK_KEY;
    IWDG->PR  = IWDG_PRESCALER;
    IWDG->RLR = IWDG_RELOAD;
    IWDG->KR  = IWDG_REFRESH_KEY;
    IWDG->KR  = IWDG_START_KEY;
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

    // Starts the boot wait, the device needs ~700ms before it answers.
    // Init continues in the background from bno055_poll().
    bno055_init();
    adc_init();

    drive_init(board_io_get());

    // USB comes up before the watchdog starts. Enumeration involves a
    // deliberate D+ pulldown so a host that already saw this device
    // re-enumerates after a firmware reset, and that pulldown outlasts
    // IWDG_TIMEOUT_MS.
    link_init();

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
            bno055_poll();
        }

        if ((now - t_adc) >= ADC_PERIOD_MS) {
            t_adc = now;
            adc_poll();

            // Battery state is reported, not acted on. Returning to the
            // start point and flushing buffered events both require
            // driving, so cutting power here would make the very
            // policies the SBC needs to run impossible, and would strand
            // the robot along with its data.
            //
            // A dead SBC is already covered: no commands means the
            // command timeout stops the robot within CMD_TIMEOUT_MS.
            //
            // The one exception below is pack protection, not mission
            // policy. See BATT_DEAD_MV.
            if (batt_get_state() == BATT_DEAD) {
                motor_estop();
            }
        }

        if ((now - t_telem) >= TELEM_PERIOD_MS) {
            t_telem = now;
            link_send_telemetry();
        }

        // Drained every pass rather than on a timer. The USB ring is
        // 256 bytes and a burst of commands must not be allowed to
        // overflow it between scheduler slots.
        link_poll_rx();

        // Same reasoning: an I2C phase completes in tens of
        // microseconds and is abandoned after I2C_TIMEOUT_MS, so the
        // transfer has to be advanced far more often than the 10ms
        // slot that decides when to start one.
        bno055_pump();

        // Heartbeat, a frozen LED means the loop died
        if ((now - t_led) >= 500U) {
            t_led = now;
            led_toggle();
        }

        // Refresh the watchdog only from the healthy path
        IWDG->KR = IWDG_REFRESH_KEY;
    }
}
