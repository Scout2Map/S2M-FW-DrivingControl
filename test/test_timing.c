/*
 * File   : test_timing.c
 * Purpose: Guards the scheduling assumptions that the periods in
 *          board_config.h depend on.
 * Author : jihoonkimtech
 *
 * These are arithmetic checks, not behavioural ones. They exist because
 * a mismatch between how often a state machine is serviced and how long
 * it is willing to wait produces a silent, total failure: the BNO055
 * state machine was driven from the 10ms IMU slot while abandoning any
 * phase older than 5ms, so every transfer failed on its first step and
 * the device looked absent.
 */

#include <stdio.h>
#include "board_config.h"

// Mirrors I2C_TIMEOUT_MS in lib/hal/i2c.c
#define I2C_TIMEOUT_MS  5U

static int fails;

static void check(const char *name, int cond)
{
    printf("%-52s %s\n", name, cond ? "PASS" : "FAIL");
    if (!cond) fails++;
}

int main(void)
{
    printf("\n=== Scout2Map scheduling invariants ===\n\n");

    // The IMU slot only decides when to START a transfer. If the same
    // slot also had to advance it, the phase would already be stale by
    // the next call. This asserts the two are not the same rate, which
    // is what forces bno055_pump() to live in the main loop.
    printf("   IMU slot %ums, I2C phase timeout %ums\n",
           IMU_PERIOD_MS, I2C_TIMEOUT_MS);
    check("IMU slot alone cannot service an I2C phase in time",
          IMU_PERIOD_MS > I2C_TIMEOUT_MS);
    printf("   -> transfers must be advanced from the main loop, not the slot\n");

    // The control loop must fit inside the command timeout with room to
    // spare, otherwise a single missed slot stops the robot
    printf("   control loop %ums, command timeout %ums\n",
           LOOP_PERIOD_MS, CMD_TIMEOUT_MS);
    check("command timeout spans many control loops",
          CMD_TIMEOUT_MS >= LOOP_PERIOD_MS * 20U);

    // Telemetry must not outpace the control loop that fills it
    check("telemetry period is a multiple of the control loop",
          TELEM_PERIOD_MS % LOOP_PERIOD_MS == 0U);

    // The IMU cannot produce data faster than 100Hz
    check("IMU polling does not exceed the sensor output rate",
          IMU_PERIOD_MS >= 10U);

    // Every blocking routine must finish inside the watchdog window
    printf("   watchdog %ums\n", IWDG_TIMEOUT_MS);
    check("control loop fits well inside the watchdog window",
          LOOP_PERIOD_MS * 10U < IWDG_TIMEOUT_MS);

    // Velocity averaging must not lag the plant it controls
    {
        float window_ms = (float)SPEED_WINDOW * LOOP_PERIOD_MS;
        printf("   speed window %.0fms\n", window_ms);
        check("speed window stays short against the plant time constant",
              window_ms < 110.0f * 0.5f);
    }

    printf("\n%s (%d failure%s)\n\n",
           fails ? "SOME TESTS FAILED" : "ALL TESTS PASSED",
           fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
