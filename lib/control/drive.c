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
static uint8_t     s_raw_mode;         // direct duty injection active
static int16_t     s_raw[IDX_COUNT];
// Logical duty, before motor.c applies the mounting inversion. Reported
// as is so a forward command reads positive on both wheels, which is
// what someone reading telemetry expects. The electrical sign is an
// implementation detail of how the motors happen to be bolted on.
static int16_t     s_duty[IDX_COUNT];

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
    s_raw_mode     = 0;
    s_raw[IDX_LEFT] = s_raw[IDX_RIGHT] = 0;
    s_duty[IDX_LEFT] = s_duty[IDX_RIGHT] = 0;
    s_odom_x = s_odom_y = s_odom_th = 0.0f;
}

void drive_command(float v_mps, float w_radps)
{
    s_v_cmd     = v_mps;
    s_w_cmd     = w_radps;
    s_cmd_stamp = s_io->millis();
    // A normal velocity command always ends raw mode, so a forgotten
    // bring-up command cannot keep overriding the controller
    s_raw_mode  = 0;
}

void drive_set_raw(int16_t left_permille, int16_t right_permille)
{
    s_raw[IDX_LEFT]  = left_permille;
    s_raw[IDX_RIGHT] = right_permille;
    s_raw_mode       = 1;
    s_cmd_stamp      = s_io->millis();
}

uint8_t drive_cmd_expired(void)
{
    return (uint8_t)((s_io->millis() - s_cmd_stamp) > CMD_TIMEOUT_MS);
}

int16_t drive_get_duty(int idx)
{
    return (idx >= 0 && idx < IDX_COUNT) ? s_duty[idx] : 0;
}

// Single point where duty reaches the hardware, so the reported value
// and the applied value can never disagree
static void apply_duty(int16_t left, int16_t right)
{
    s_duty[IDX_LEFT]  = left;
    s_duty[IDX_RIGHT] = right;
    s_io->set_duty(IDX_LEFT,  left);
    s_io->set_duty(IDX_RIGHT, right);
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
    apply_duty(0, 0);
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

    // Feedforward carries the bulk of the command, the terms below only
    // trim the residual
    float ff = target_mps * PID_FF;

    // Conditional integration. The accumulator only grows when doing so
    // would not push an already saturated output further into the rail.
    //
    // An earlier version instead halved the accumulator whenever the
    // output saturated, which discarded correction the loop had already
    // earned and left a standing error the integrator could never close.
    float unsat = ff + err * PID_KP + p->integral;
    uint8_t saturated_high = (unsat >=  (float)DUTY_MAX);
    uint8_t saturated_low  = (unsat <= -(float)DUTY_MAX);

    if (!((saturated_high && err > 0.0f) || (saturated_low && err < 0.0f))) {
        p->integral += err * PID_KI * LOOP_PERIOD_S;
        p->integral  = clampf(p->integral, -PID_I_LIMIT, PID_I_LIMIT);
    }

    float out = ff + err * PID_KP + p->integral;

#if PID_USE_D
    out += ((err - p->prev_err) / LOOP_PERIOD_S) * PID_KD;
#endif
    p->prev_err = err;

    return (int16_t)clampf(out, -(float)DUTY_MAX, (float)DUTY_MAX);
}

// A commanded wheel reporting no motion means a jam or a cut encoder
// wire. Closing the loop on absent feedback is worse than open loop.
//
// The fallback is reversible. A wheel that was briefly jammed, or a
// connector that was reseated, otherwise left the robot open loop until the
// next MCU reset, which silently degrades odometry for the rest of a mission
// and cannot be diagnosed from the outside.
static void encoder_health_check(void)
{
    static uint32_t quiet_ms[IDX_COUNT];
    static uint32_t healthy_ms[IDX_COUNT];

    uint8_t commanded = (fabsf(s_v_cmd) > 0.02f) || (fabsf(s_w_cmd) > 0.05f);
    uint8_t all_healthy = 1;

    for (int i = 0; i < IDX_COUNT; i++) {
        int32_t d = s_io->get_delta_counts(i);
        if (d < 0) d = -d;

        if (commanded && d < STALL_MIN_COUNTS) {
            healthy_ms[i] = 0;
            quiet_ms[i] += LOOP_PERIOD_MS;
            if (quiet_ms[i] > ENC_FAULT_MS) {
                s_openloop = 1;
            }
        } else {
            quiet_ms[i] = 0;

            // Only motion proves an encoder works. A standing robot produces
            // no counts either, so idle time must not count as recovery.
            if (commanded && d >= STALL_MIN_COUNTS) {
                if (healthy_ms[i] < ENC_RECOVER_MS) {
                    healthy_ms[i] += LOOP_PERIOD_MS;
                }
            }
        }

        if (healthy_ms[i] < ENC_RECOVER_MS) {
            all_healthy = 0;
        }
    }

    // One good wheel is not enough: closing the loop with a dead encoder on
    // the other side would drive the chassis in a circle.
    if (s_openloop && all_healthy) {
        s_openloop = 0;

        // The integrators wound up against an error the open loop path never
        // acted on. Resuming with that history would kick the wheels.
        for (int i = 0; i < IDX_COUNT; i++) {
            s_pid[i].integral = 0.0f;
            s_pid[i].prev_err = 0.0f;
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

#if ENCODER_AVAILABLE
    // Raw mode still samples encoders so telemetry and odometry stay
    // live while the controller is bypassed
    if (s_raw_mode) {
        s_io->sample_encoders();
        odom_integrate();
        apply_duty(s_raw[IDX_LEFT], s_raw[IDX_RIGHT]);
        s_io->commit();
        return;
    }
#else
    if (s_raw_mode) {
        apply_duty(s_raw[IDX_LEFT], s_raw[IDX_RIGHT]);
        s_io->commit();
        return;
    }
#endif

    // Clamp the request to what the gearbox can actually deliver
    float v = clampf(s_v_cmd, -MAX_WHEEL_SPEED_MPS, MAX_WHEEL_SPEED_MPS);
    // Rotating faster than this smears a LiDAR scan badly enough to hurt
    // scan matching, so the ceiling is set below the mechanical limit
    float w = clampf(s_w_cmd, -MAX_ANGULAR_RATE, MAX_ANGULAR_RATE);

    // Differential drive kinematics, half the track width per side
    float half_base = WHEEL_BASE_MM * 0.0005f;
    float v_left    = v - w * half_base;
    float v_right   = v + w * half_base;

#if ENCODER_AVAILABLE
    s_io->sample_encoders();
    odom_integrate();
    encoder_health_check();

    if (!s_openloop) {
        apply_duty(pid_step(IDX_LEFT,  v_left,  s_io->get_speed_mps(IDX_LEFT)),
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

    apply_duty((int16_t)clampf(left,  -(float)DUTY_MAX, (float)DUTY_MAX),
               (int16_t)clampf(right, -(float)DUTY_MAX, (float)DUTY_MAX));
    s_io->commit();
}
