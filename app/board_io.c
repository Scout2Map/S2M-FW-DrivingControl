/*
 * File   : board_io.c
 * Purpose: Adapter binding the hardware independent control layer to
 *          the concrete drivers on this board. Every function here is
 *          a thin shim; all real logic lives in lib/control.
 * Author : jihoonkimtech
 *
 * This file is the only place that knows both sides. Swapping the
 * motor driver or the encoder scheme means editing board_io.c alone,
 * the control layer never notices the change.
 *
 * Note: imu_get_yaw() is a weak stub returning 0.0 until the BNO055
 * driver lands. Heading assist is inert but harmless until then.
 */

#include <stddef.h>
#include "board_config.h"
#include "board_io.h"
#include "drive.h"
#include "motor.h"
#include "encoder.h"
#include "systick.h"

// ============================================================
// Adapter between the hardware independent control layer and the
// concrete drivers on this board. Every function here is a thin
// shim, all real logic lives in lib/control.
//
// Swapping a motor driver or an encoder scheme means editing this
// file only, the control layer never notices.
// ============================================================

// Supplied by the BNO055 driver once it lands
// The weak stub keeps the build green until then
float imu_get_yaw(void) __attribute__((weak));
float imu_get_yaw(void)
{
    return 0.0f;
}

static void io_set_duty(int idx, int16_t permille)
{
    motor_set((idx == 0) ? MOTOR_LEFT : MOTOR_RIGHT, permille);
}

static void io_commit(void)
{
    motor_update();
}

static void io_sample_encoders(void)
{
#if ENCODER_AVAILABLE
    encoder_update();
#endif
}

static float io_get_speed_mps(int idx)
{
#if ENCODER_AVAILABLE
    return encoder_get_speed_mps((idx == 0) ? ENC_LEFT : ENC_RIGHT);
#else
    (void)idx;
    return 0.0f;
#endif
}

static int32_t io_get_delta_counts(int idx)
{
#if ENCODER_AVAILABLE
    return encoder_get_delta((idx == 0) ? ENC_LEFT : ENC_RIGHT);
#else
    (void)idx;
    return 0;
#endif
}

static float io_get_yaw(void)
{
    return imu_get_yaw();
}

static uint32_t io_millis(void)
{
    return systick_millis();
}

static const drive_io_t s_board_io = {
    .set_duty         = io_set_duty,
    .commit           = io_commit,
    .sample_encoders  = io_sample_encoders,
    .get_speed_mps    = io_get_speed_mps,
    .get_delta_counts = io_get_delta_counts,
    .get_yaw          = io_get_yaw,
    .millis           = io_millis,
};

const drive_io_t *board_io_get(void)
{
    return &s_board_io;
}
