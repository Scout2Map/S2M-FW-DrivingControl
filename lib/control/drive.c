/*
 * File   : drive.c
 * Purpose: Differential drive kinematics, per wheel velocity PID and
 *          odometry integration. Converts a (v, w) command into left
 *          and right duty values.
 * Author : jihoonkimtech
 *
 * This file touches no peripheral register. Motors, encoders, the IMU
 * and the clock are all reached through the injected drive_io_t table,
 * which lets the whole module compile and run on a host PC. See test/.
 *
 * Control structure
 *   (v, w) -> inverse kinematics -> per wheel target speed
 *          -> feedforward (PID_FF) + velocity PID
 *          -> signed permille duty
 *
 * The feedforward carries most of the command and the PID corrects only
 * the residual, which keeps the gains small and the step response damped.
 *
 * Note: the open loop path is a live fallback, not dead code. The
 * encoder health check switches to it when a commanded wheel reports no
 * motion for a second, so a cut encoder wire degrades the robot instead
 * of stopping it dead in the field.
 *
 * Note: odometry integrated here is for slip detection and debugging.
 * The SBC owns the TF tree and the SLAM input.
 */

#include <math.h>
#include <stddef.h>
#include "board_config.h"
#include "drive.h"

// ============================================================
// Drive layer
//
// With encoders present this runs a per wheel velocity PID with a
// feedforward term. The feedforward carries most of the command and
// the PID only corrects the residual, which keeps the gains small
// and the step response well damped.
//
// The open loop path stays intact as a fallback. It engages when
// ENCODER_AVAILABLE is 0, or when the encoder health check trips at
// runtime, so a broken encoder wire degrades the robot rather than
// stopping it dead in the field.
//
// No peripheral register is touched here. Everything reaches the
// hardware through the injected drive_io_t table.
// ============================================================

#define IDX_LEFT    0
#define IDX_RIGHT   1
#define IDX_COUNT   2

typedef struct {
    float integral;
    float prev_err;
} pid_state_t;

static const drive_io_t *s_io;

static float       s_v_cmd;         // requested linear velocity, m/s
static float       s_w_cmd;         // requested angular velocity, rad/s
static uint32_t    s_cmd_stamp;     // arrival time of the last command
static pid_state_t s_pid[IDX_COUNT];
static uint8_t     s_openloop;      // 1 while the fallback path is active

static float       s_heading_ref;
static uint8_t     s_heading_lock;

// Pose integrated on the MCU for slip detection and debugging
// The SBC owns the TF tree, this is not the SLAM input
static float s_odom_x, s_odom_y, s_odom_th;

static float wrap_pi(float a)
{
    while (a >  3.14159265f) a -= 6.28318531f;
    while (a < -3.14159265f) a += 6.28318531f;
    return a;
}

static float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

void drive_init(const drive_io_t *io)
{
    s_io        = io;
    s_v_cmd     = 0.0f;
    s_w_cmd     = 0.0f;
    s_cmd_stamp = (io && io->millis) ? io->millis() : 0U;

    for (int i = 0; i < IDX_COUNT; i++) {
        s_pid[i].integral = 0.0f;
        s_pid[i].prev_err = 0.0f;
    }

#if ENCODER_AVAILABLE
    s_openloop = 0;
#else
    s_openloop = 1;
#endif

    s_heading_ref  = 0.0f;
    s_heading_lock = 0;
    s_odom_x = s_odom_y = s_odom_th = 0.0f;
}

void drive_command(float v_mps, float w_radps)
{
    s_v_cmd     = v_mps;
    s_w_cmd     = w_radps;
    s_cmd_stamp = s_io->millis();
}

void drive_stop(void)
{
    s_v_cmd = 0.0f;
    s_w_cmd = 0.0f;
    s_heading_lock = 0;
    for (int i = 0; i < IDX_COUNT; i++) {
        s_pid[i].integral = 0.0f;
        s_pid[i].prev_err = 0.0f;
    }
    s_io->set_duty(IDX_LEFT, 0);
    s_io->set_duty(IDX_RIGHT, 0);
}

void drive_get_odom(float *x, float *y, float *th)
{
    if (x)  *x  = s_odom_x;
    if (y)  *y  = s_odom_y;
    if (th) *th = s_odom_th;
}

void drive_reset_odom(void)
{
    s_odom_x = s_odom_y = s_odom_th = 0.0f;
}

uint8_t drive_is_openloop(void)
{
    return s_openloop;
}

#if ENCODER_AVAILABLE
// Integrate wheel travel into a pose estimate
// Differential drive model, valid while slip stays small
static void odom_integrate(void)
{
    float dl = (float)s_io->get_delta_counts(IDX_LEFT)  * MM_PER_COUNT * 0.001f;
    float dr = (float)s_io->get_delta_counts(IDX_RIGHT) * MM_PER_COUNT * 0.001f;

    float ds  = (dl + dr) * 0.5f;
    float dth = (dr - dl) / (WHEEL_BASE_MM * 0.001f);

    // Midpoint heading keeps the error small on curved segments
    float th_mid = s_odom_th + dth * 0.5f;
    s_odom_x  += ds * cosf(th_mid);
    s_odom_y  += ds * sinf(th_mid);
    s_odom_th  = wrap_pi(s_odom_th + dth);
}

// One wheel of velocity PID, output in permille
static int16_t pid_step(int idx, float target_mps, float actual_mps)
{
    pid_state_t *p = &s_pid[idx];
    float err = target_mps - actual_mps;

    // Feedforward carries the bulk of the command
    float out = target_mps * PID_FF;
    out += err * PID_KP;

    p->integral += err * PID_KI * LOOP_PERIOD_S;
    p->integral  = clampf(p->integral, -PID_I_LIMIT, PID_I_LIMIT);
    out += p->integral;

#if PID_USE_D
    out += ((err - p->prev_err) / LOOP_PERIOD_S) * PID_KD;
#endif
    p->prev_err = err;

    // Stop the integrator charging further while the output saturates
    if (out > (float)DUTY_MAX || out < -(float)DUTY_MAX) {
        p->integral = clampf(p->integral, -PID_I_LIMIT * 0.5f, PID_I_LIMIT * 0.5f);
    }

    return (int16_t)clampf(out, -(float)DUTY_MAX, (float)DUTY_MAX);
}

// A commanded wheel reporting no motion means a jam or a cut encoder
// wire. Closing the loop on absent feedback is worse than open loop.
static void encoder_health_check(void)
{
    static uint32_t quiet_ms[IDX_COUNT];
    uint8_t commanded = (fabsf(s_v_cmd) > 0.02f) || (fabsf(s_w_cmd) > 0.05f);

    for (int i = 0; i < IDX_COUNT; i++) {
        int32_t d = s_io->get_delta_counts(i);
        if (d < 0) d = -d;

        if (commanded && d < STALL_MIN_COUNTS) {
            quiet_ms[i] += LOOP_PERIOD_MS;
            if (quiet_ms[i] > 1000U) {
                s_openloop = 1;
            }
        } else {
            quiet_ms[i] = 0;
        }
    }
}
#endif // ENCODER_AVAILABLE

void drive_update(void)
{
    // A silent RPi5 must not leave the robot driving
    if ((s_io->millis() - s_cmd_stamp) > CMD_TIMEOUT_MS) {
        drive_stop();
        s_io->commit();
        return;
    }

    // Clamp the request to what the gearbox can actually deliver
    float v = clampf(s_v_cmd, -MAX_WHEEL_SPEED_MPS, MAX_WHEEL_SPEED_MPS);
    float w = s_w_cmd;

    // Differential drive kinematics, half the track width per side
    float half_base = WHEEL_BASE_MM * 0.0005f;
    float v_left    = v - w * half_base;
    float v_right   = v + w * half_base;

#if ENCODER_AVAILABLE
    s_io->sample_encoders();
    odom_integrate();
    encoder_health_check();

    if (!s_openloop) {
        s_io->set_duty(IDX_LEFT,
                       pid_step(IDX_LEFT,  v_left,  s_io->get_speed_mps(IDX_LEFT)));
        s_io->set_duty(IDX_RIGHT,
                       pid_step(IDX_RIGHT, v_right, s_io->get_speed_mps(IDX_RIGHT)));
        s_io->commit();
        return;
    }
#endif

    // ---- Open loop fallback ----
    float base = v * DUTY_PER_MPS;
    float diff = w * DUTY_PER_RADPS;

#if HEADING_ASSIST_ENABLE
    // Straight line hold only, never fight a deliberate turn
    if (fabsf(w) < 0.05f && fabsf(v) > 0.01f) {
        if (!s_heading_lock) {
            s_heading_ref  = s_io->get_yaw();
            s_heading_lock = 1;
        }
        float err = wrap_pi(s_heading_ref - s_io->get_yaw());
        // Proportional only, an integral term winds up badly without feedback
        diff += err * HEADING_KP;
    } else {
        s_heading_lock = 0;
    }
#endif

    float left  = base - diff;
    float right = base + diff;

    // Step over the dead zone so small commands still produce motion
    if (fabsf(left) > 1.0f && fabsf(left) < (float)DUTY_MIN_MOVE) {
        left = (left > 0) ? (float)DUTY_MIN_MOVE : -(float)DUTY_MIN_MOVE;
    }
    if (fabsf(right) > 1.0f && fabsf(right) < (float)DUTY_MIN_MOVE) {
        right = (right > 0) ? (float)DUTY_MIN_MOVE : -(float)DUTY_MIN_MOVE;
    }

    s_io->set_duty(IDX_LEFT,
                   (int16_t)clampf(left,  -(float)DUTY_MAX, (float)DUTY_MAX));
    s_io->set_duty(IDX_RIGHT,
                   (int16_t)clampf(right, -(float)DUTY_MAX, (float)DUTY_MAX));
    s_io->commit();
}
