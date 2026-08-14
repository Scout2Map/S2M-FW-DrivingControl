/*
 * File   : test_drive.c
 * Purpose: Host side harness for the control layer. Builds with plain
 *          gcc and proves lib/control carries no hidden dependency on
 *          the MCU, so kinematics and PID can be verified without the
 *          board being present.
 * Author : jihoonkimtech
 *
 * The stub IO table models the motors as a first order lag plant and
 * republishes the command every loop, mirroring how the RPi5 streams
 * cmd_vel. A separate silent path exercises the command timeout.
 *
 * Note: run this after changing any PID gain, before flashing.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "drive.h"
#include "board_config.h"

// ============================================================
// Host side harness for the control layer
// Builds with plain gcc, no board and no cross toolchain needed
// Proves that lib/control has no hidden hardware dependency
// ============================================================

static int16_t  g_duty[2];
static float    g_speed[2];
static int32_t  g_counts[2];
static uint32_t g_now;

static void     stub_set_duty(int i, int16_t d) { g_duty[i] = d; }
static void     stub_commit(void)               { }
static void     stub_sample(void)               { }
static float    stub_speed(int i)               { return g_speed[i]; }
static int32_t  stub_counts(int i)              { return g_counts[i]; }
static float    stub_yaw(void)                  { return 0.0f; }
static uint32_t stub_millis(void)               { return g_now; }

static const drive_io_t stub_io = {
    stub_set_duty, stub_commit, stub_sample,
    stub_speed, stub_counts, stub_yaw, stub_millis,
};

static int fails;

static void check(const char *name, int cond)
{
    printf("%-42s %s\n", name, cond ? "PASS" : "FAIL");
    if (!cond) fails++;
}

// Simple first order plant, duty drives speed toward a steady value
static void step_plant(void)
{
    for (int i = 0; i < 2; i++) {
        float target = (float)g_duty[i] / PID_FF;
        g_speed[i] += (target - g_speed[i]) * 0.25f;
        g_counts[i] = (int32_t)(g_speed[i] * 1000.0f
                                * LOOP_PERIOD_S / MM_PER_COUNT);
    }
}

// The RPi5 republishes cmd_vel continuously, mirror that here so the
// command timeout does not fire during a long run
static float g_v_hold, g_w_hold;

static void run_loops(int n)
{
    for (int i = 0; i < n; i++) {
        g_now += LOOP_PERIOD_MS;
        drive_command(g_v_hold, g_w_hold);
        drive_update();
        step_plant();
    }
}

// Runs without refreshing the command, used to exercise the timeout
static void run_loops_silent(int n)
{
    for (int i = 0; i < n; i++) {
        g_now += LOOP_PERIOD_MS;
        drive_update();
        step_plant();
    }
}

static void hold(float v, float w)
{
    g_v_hold = v;
    g_w_hold = w;
    drive_command(v, w);
}

int main(void)
{
    printf("\n=== Scout2Map drive control, host tests ===\n\n");

    // --- command timeout ---
    memset(g_speed, 0, sizeof g_speed);
    g_now = 0;
    drive_init(&stub_io);
    hold(0.2f, 0.0f);
    run_loops(10);
    check("straight command produces forward duty", g_duty[0] > 0 && g_duty[1] > 0);

    run_loops_silent((CMD_TIMEOUT_MS / LOOP_PERIOD_MS) + 10);
    check("stale command stops both wheels", g_duty[0] == 0 && g_duty[1] == 0);

    // --- velocity tracking ---
    memset(g_speed, 0, sizeof g_speed);
    g_now = 0;
    drive_init(&stub_io);
    hold(0.2f, 0.0f);
    run_loops(200);
    printf("   settled speed=%.4f duty=%d counts/loop=%d\n",
           g_speed[0], g_duty[0], g_counts[0]);
    check("PID converges on the target speed", fabsf(g_speed[0] - 0.2f) < 0.02f);

    // --- speed clamp ---
    memset(g_speed, 0, sizeof g_speed);
    g_now = 0;
    drive_init(&stub_io);
    hold(5.0f, 0.0f);              // far beyond the gearbox limit
    run_loops(200);
    check("request above the gearbox limit is clamped",
          g_speed[0] < MAX_WHEEL_SPEED_MPS + 0.03f);

    // --- turning kinematics ---
    memset(g_speed, 0, sizeof g_speed);
    g_now = 0;
    drive_init(&stub_io);
    hold(0.0f, 0.5f);              // spin in place, counterclockwise
    run_loops(50);
    check("in place rotation drives the wheels apart", g_duty[0] < 0 && g_duty[1] > 0);

    // --- angular ceiling ---
    memset(g_speed, 0, sizeof g_speed);
    g_now = 0;
    drive_init(&stub_io);
    drive_reset_odom();
    hold(0.0f, 5.0f);              // far beyond the LiDAR friendly rate
    run_loops(120);                // 0.6s, short enough to avoid pi wrap
    {
        float ax, ay, ath;
        drive_get_odom(&ax, &ay, &ath);
        // Heading rate must stay under the configured ceiling
        float rate = fabsf(ath) / 0.6f;
        printf("   spin rate under a 5 rad/s request: %.3f rad/s\n", rate);
        check("angular request is capped for scan quality",
              rate < MAX_ANGULAR_RATE + 0.1f);
    }

    // --- odometry ---
    memset(g_speed, 0, sizeof g_speed);
    g_now = 0;
    drive_init(&stub_io);
    drive_reset_odom();
    hold(0.2f, 0.0f);
    run_loops(400);                // 2 seconds of travel
    float x, y, th;
    drive_get_odom(&x, &y, &th);
    printf("   odom after 2s at 0.2 m/s: x=%.3f y=%.3f th=%.3f\n", x, y, th);
    check("straight run accumulates x only", x > 0.25f && fabsf(y) < 0.01f);
    check("straight run leaves heading unchanged", fabsf(th) < 0.01f);

    // --- raw duty injection ---
    // Bring-up path: must bypass the PID entirely and report back
    // exactly what was asked for
    memset(g_speed, 0, sizeof g_speed);
    g_now = 0;
    drive_init(&stub_io);
    drive_set_raw(400, -400);
    g_now += LOOP_PERIOD_MS;
    drive_update();
    check("raw mode applies the requested duty verbatim",
          g_duty[0] == 400 && g_duty[1] == -400);
    check("raw duty is reported back in telemetry",
          drive_get_duty(0) == 400 && drive_get_duty(1) == -400);

    // A normal velocity command must end raw mode, otherwise a forgotten
    // bring-up command would silently override the controller
    hold(0.1f, 0.0f);
    run_loops(20);
    check("velocity command cancels raw mode",
          g_duty[0] != 400 && g_duty[0] > 0);

    // --- command timeout applies to raw mode too ---
    drive_init(&stub_io);
    drive_set_raw(400, 400);
    run_loops_silent((CMD_TIMEOUT_MS / LOOP_PERIOD_MS) + 10);
    check("raw mode still honours the command timeout",
          g_duty[0] == 0 && g_duty[1] == 0);

    // --- verified encoder constants ---
    // Guards the bench measurement of 2026-08-13 against accidental edits
    check("encoder resolution matches the bench measurement",
          COUNTS_PER_WHEEL_REV == 5764);
    {
        // Ten wheel revolutions must equal the trip point observed on hardware
        int32_t ten_revs = COUNTS_PER_WHEEL_REV * 10;
        printf("   10 revs = %d counts, %.1f mm\n",
               ten_revs, ten_revs * MM_PER_COUNT);
        check("ten revolutions land on the observed trip point",
              ten_revs == 57640);
    }
    {
        // The 5ms poll must stay far from the int16 wrap limit
        float per_loop = MAX_WHEEL_SPEED_MPS * 1000.0f
                       * LOOP_PERIOD_S / MM_PER_COUNT;
        printf("   counts per loop at top speed: %.1f\n", per_loop);
        check("poll rate keeps wrap arithmetic unambiguous",
              per_loop < 16000.0f);
        check("resolution is fine enough for velocity PID",
              per_loop > 5.0f);
    }

    printf("\n%s (%d failure%s)\n\n",
           fails ? "SOME TESTS FAILED" : "ALL TESTS PASSED",
           fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
