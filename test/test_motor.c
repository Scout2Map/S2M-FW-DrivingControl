/*
 * File   : test_motor.c
 * Purpose: Host tests for the motor duty state machine: slew limiting,
 *          direction change deadtime and shoot-through prevention.
 * Author : jihoonkimtech
 *
 * The logic is transcribed rather than linked, because motor.c touches
 * timer registers directly. Any edit to motor_update() must be mirrored
 * here, which is the price of testing it at all without a board.
 *
 * Note: the deadlock this file guards against was real. A reversal used
 * to leave a wheel permanently stationary, and it only showed up once
 * one motor was configured as mirrored on actual hardware.
 */

#include <stdio.h>
#include <stdint.h>
#include "board_config.h"

typedef struct {
    int16_t  target, applied;
    int8_t   last_sign;
    uint32_t dead_until;
    uint16_t ccr_fwd, ccr_rev;
} ch_t;

static int fails;

static void check(const char *name, int cond)
{
    printf("%-46s %s\n", name, cond ? "PASS" : "FAIL");
    if (!cond) fails++;
}

// Mirrors motor_update() for one channel
static void step(ch_t *m, uint32_t now)
{
    m->ccr_fwd = m->ccr_rev = 0;

    if (now < m->dead_until) {
        m->applied = 0;
        return;
    }

    int8_t want = (m->target > 0) ? 1 : ((m->target < 0) ? -1 : 0);
    if (want != 0 && m->last_sign != 0 && want != m->last_sign) {
        m->dead_until = now + DIR_CHANGE_DEADTIME_MS;
        m->applied    = 0;
        m->last_sign  = 0;
        return;
    }

    int16_t d = m->target - m->applied;
    if (d >  DUTY_SLEW_PER_LOOP) d =  DUTY_SLEW_PER_LOOP;
    if (d < -DUTY_SLEW_PER_LOOP) d = -DUTY_SLEW_PER_LOOP;
    m->applied += d;

    int8_t sign = (m->applied > 0) ? 1 : ((m->applied < 0) ? -1 : 0);
    if (sign != 0) m->last_sign = sign;

    uint32_t mag = (uint32_t)(m->applied < 0 ? -m->applied : m->applied);
    uint32_t ccr = (mag * (PWM_ARR + 1U)) / 1000U;
    if (m->applied >= 0) m->ccr_fwd = (uint16_t)ccr;
    else                 m->ccr_rev = (uint16_t)ccr;
}

static int settle(ch_t *m, int loops)
{
    uint32_t now = 0;
    for (int i = 0; i < loops; i++) {
        now += LOOP_PERIOD_MS;
        step(m, now);
    }
    return m->applied;
}

int main(void)
{
    printf("\n=== Scout2Map motor state machine, host tests ===\n\n");

    // --- the deadlock regression ---
    // A reversal used to pin applied at zero forever
    {
        ch_t m = { .target = -300, .last_sign = 1 };
        check("reversal from forward reaches the target",
              settle(&m, 60) == -300);
    }
    {
        ch_t m = { .target = 300, .last_sign = -1 };
        check("reversal from backward reaches the target",
              settle(&m, 60) == 300);
    }
    {
        ch_t m = { .target = 300, .last_sign = 0 };
        check("start from stopped reaches the target",
              settle(&m, 60) == 300);
    }

    // --- shoot-through prevention ---
    {
        ch_t m = { .target = -300, .last_sign = 0 };
        uint32_t now = 0;
        int both_live = 0;
        for (int i = 0; i < 60; i++) {
            now += LOOP_PERIOD_MS;
            step(&m, now);
            if (m.ccr_fwd != 0 && m.ccr_rev != 0) both_live = 1;
        }
        check("never drives both bridge sides at once", !both_live);
    }

    // --- coast window during a reversal ---
    {
        ch_t m = { .target = -300, .last_sign = 1 };
        uint32_t now = LOOP_PERIOD_MS;
        step(&m, now);
        check("reversal opens a coast window",
              m.ccr_fwd == 0 && m.ccr_rev == 0 && m.dead_until > now);
    }

    // --- slew limiting ---
    {
        ch_t m = { .target = 1000, .last_sign = 0 };
        uint32_t now = 0;
        int16_t prev = 0, worst = 0;
        for (int i = 0; i < 100; i++) {
            now += LOOP_PERIOD_MS;
            step(&m, now);
            int16_t jump = (int16_t)(m.applied - prev);
            if (jump < 0) jump = (int16_t)-jump;
            if (jump > worst) worst = jump;
            prev = m.applied;
        }
        printf("   largest single step %d, limit %d\n", worst, DUTY_SLEW_PER_LOOP);
        check("slew rate is enforced", worst <= DUTY_SLEW_PER_LOOP);
        check("full scale is still reachable", m.applied == 1000);
    }

    // --- duty to compare value mapping ---
    {
        ch_t m = { .target = 1000, .last_sign = 0 };
        settle(&m, 100);
        printf("   duty 1000 maps to CCR %u, ARR %lu\n",
               m.ccr_fwd, (unsigned long)PWM_ARR);
        check("full duty maps to full scale", m.ccr_fwd == PWM_ARR + 1U);
    }

    // --- configured direction matches the bench measurement ---
    check("left motor is not inverted",  MOTL_INVERT == 0);
    check("right motor is inverted",     MOTR_INVERT == 1);
    check("left encoder is inverted",    ENC_L_INVERT == 1);
    check("right encoder is not inverted", ENC_R_INVERT == 0);

    printf("\n%s (%d failure%s)\n\n",
           fails ? "SOME TESTS FAILED" : "ALL TESTS PASSED",
           fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
