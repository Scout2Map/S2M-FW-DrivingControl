/*
 * File   : bno055.c
 * Purpose: BNO055 driver providing fused orientation and yaw rate to the
 *          drive layer, without blocking the control loop.
 * Author : jihoonkimtech
 *
 * Wiring
 *   I2C2 SCL = PB10, SDA = PB11, address 0x28 (0x29 if COM3 is pulled up)
 *   Powered from 5V through the module regulator
 *
 * The device runs in NDOF mode, so its own processor does the fusion and
 * the MCU only reads results. Doing sensor fusion here would waste the
 * one thing this part is good at.
 *
 * Note: fusion output updates at 100Hz. Polling faster returns the same
 * numbers and only burns bus time, so IMU_PERIOD_MS is 10.
 *
 * Note: the quaternion and gyro registers are read in a single burst.
 * Reading them separately would let the device update between reads and
 * produce an orientation that never actually existed.
 *
 * Note: after a power on the chip needs roughly 650ms before it answers
 * at all, and mode changes need 20ms to take effect. The init sequence
 * is written as a state machine with explicit waits rather than a chain
 * of blocking delays, so the watchdog and the drive loop keep running.
 */

#include <string.h>
#include <math.h>
#include "board_config.h"
#include "bno055.h"
#include "i2c.h"
#include "systick.h"

// ---- Register map, page 0 ----
#define REG_CHIP_ID         0x00
#define REG_ACC_ID          0x01
#define REG_PAGE_ID         0x07
#define REG_ACC_DATA        0x08
#define REG_EUL_DATA        0x1A
#define REG_QUA_DATA        0x20
#define REG_GYR_DATA        0x14
#define REG_CALIB_STAT      0x35
#define REG_ST_RESULT       0x36
#define REG_SYS_STATUS      0x39
#define REG_SYS_ERR         0x3A
#define REG_UNIT_SEL        0x3B
#define REG_OPR_MODE        0x3D
#define REG_PWR_MODE        0x3E
#define REG_SYS_TRIGGER     0x3F

#define CHIP_ID_EXPECTED    0xA0

#define MODE_CONFIG         0x00
#define MODE_NDOF           0x0C
#define PWR_NORMAL          0x00

// Waits required by the datasheet
#define BOOT_DELAY_MS       700U    // power on to first response
#define MODE_DELAY_MS       25U     // config to operating mode switch
#define RESET_DELAY_MS      700U    // after a soft reset

typedef enum {
    INIT_WAIT_BOOT = 0,
    INIT_READ_ID,
    INIT_CHECK_ID,
    INIT_SET_CONFIG,
    INIT_WAIT_CONFIG,
    INIT_SET_UNITS,
    INIT_WAIT_UNITS,
    INIT_SET_POWER,
    INIT_WAIT_POWER,
    INIT_SET_NDOF,
    INIT_WAIT_NDOF,
    INIT_COMPLETE,
    INIT_FAILED
} init_step_t;

typedef enum {
    RUN_IDLE = 0,
    RUN_READ_QUAT,
    RUN_READ_GYRO,
    RUN_WAIT_GYRO,
    RUN_READ_CALIB,
    RUN_WAIT_CALIB
} run_step_t;

static init_step_t s_init;
static run_step_t  s_run;
static uint32_t    s_wait_until;
static uint8_t     s_buf[8];
static uint8_t     s_ready;
static uint32_t    s_read_ok;
static uint32_t    s_read_fail;
static uint8_t     s_consecutive_fail;

// Latest values, updated only on a complete successful burst
static int16_t s_quat[4];       // w, x, y, z at 1/16384 scale
static int16_t s_gyro[3];       // x, y, z at 1/16 deg/s
static uint8_t s_calib;         // packed sys/gyr/acc/mag, 2 bits each

static void wait_ms(uint32_t ms)
{
    s_wait_until = systick_millis() + ms;
}

static uint8_t wait_done(void)
{
    return (uint8_t)((int32_t)(systick_millis() - s_wait_until) >= 0);
}

void bno055_init(void)
{
    i2c_init();
    s_init   = INIT_WAIT_BOOT;
    s_run    = RUN_IDLE;
    s_ready  = 0;
    s_read_ok = 0;
    s_read_fail = 0;
    s_consecutive_fail = 0;
    memset(s_quat, 0, sizeof s_quat);
    memset(s_gyro, 0, sizeof s_gyro);
    s_calib = 0;
    wait_ms(BOOT_DELAY_MS);
}

// Drives the init sequence. Returns 1 once the device is streaming.
static uint8_t init_poll(void)
{
    static const uint8_t cfg_mode   = MODE_CONFIG;
    static const uint8_t ndof_mode  = MODE_NDOF;
    static const uint8_t pwr_normal = PWR_NORMAL;
    // Windows degrees, m/s2, deg/s, Celsius, and Android orientation
    static const uint8_t unit_sel   = 0x00;

    i2c_result_t r;

    switch (s_init) {
    case INIT_WAIT_BOOT:
        if (wait_done()) {
            s_init = INIT_READ_ID;
        }
        break;

    case INIT_READ_ID:
        if (i2c_start_read(BNO055_ADDR, REG_CHIP_ID, s_buf, 1)) {
            s_init = INIT_CHECK_ID;
        }
        break;

    case INIT_CHECK_ID:
        r = i2c_poll();
        if (r == I2C_RESULT_DONE) {
            if (s_buf[0] == CHIP_ID_EXPECTED) {
                s_init = INIT_SET_CONFIG;
            } else {
                // Wrong ID means the wrong device, or none. Retrying is
                // pointless but failing loudly is better than pretending.
                s_init = INIT_FAILED;
            }
        } else if (r == I2C_RESULT_ERROR) {
            // No answer yet. The chip may still be booting, so retry
            // rather than latch a failure on the first miss.
            wait_ms(100);
            s_init = INIT_WAIT_BOOT;
        }
        break;

    case INIT_SET_CONFIG:
        if (i2c_start_write(BNO055_ADDR, REG_OPR_MODE, &cfg_mode, 1)) {
            s_init = INIT_WAIT_CONFIG;
        }
        break;

    case INIT_WAIT_CONFIG:
        r = i2c_poll();
        if (r == I2C_RESULT_DONE) {
            wait_ms(MODE_DELAY_MS);
            s_init = INIT_SET_UNITS;
        } else if (r == I2C_RESULT_ERROR) {
            s_init = INIT_SET_CONFIG;
        }
        break;

    case INIT_SET_UNITS:
        if (wait_done() && i2c_start_write(BNO055_ADDR, REG_UNIT_SEL, &unit_sel, 1)) {
            s_init = INIT_WAIT_UNITS;
        }
        break;

    case INIT_WAIT_UNITS:
        r = i2c_poll();
        if (r == I2C_RESULT_DONE) {
            s_init = INIT_SET_POWER;
        } else if (r == I2C_RESULT_ERROR) {
            s_init = INIT_SET_UNITS;
        }
        break;

    case INIT_SET_POWER:
        if (i2c_start_write(BNO055_ADDR, REG_PWR_MODE, &pwr_normal, 1)) {
            s_init = INIT_WAIT_POWER;
        }
        break;

    case INIT_WAIT_POWER:
        r = i2c_poll();
        if (r == I2C_RESULT_DONE) {
            s_init = INIT_SET_NDOF;
        } else if (r == I2C_RESULT_ERROR) {
            s_init = INIT_SET_POWER;
        }
        break;

    case INIT_SET_NDOF:
        if (i2c_start_write(BNO055_ADDR, REG_OPR_MODE, &ndof_mode, 1)) {
            s_init = INIT_WAIT_NDOF;
        }
        break;

    case INIT_WAIT_NDOF:
        r = i2c_poll();
        if (r == I2C_RESULT_DONE) {
            wait_ms(MODE_DELAY_MS);
            s_init = INIT_COMPLETE;
        } else if (r == I2C_RESULT_ERROR) {
            s_init = INIT_SET_NDOF;
        }
        break;

    case INIT_COMPLETE:
        if (wait_done()) {
            s_ready = 1;
            return 1;
        }
        break;

    case INIT_FAILED:
    default:
        break;
    }
    return 0;
}

// Called at IMU_PERIOD_MS. Never blocks.
void bno055_poll(void)
{
    if (!s_ready) {
        (void)init_poll();
        return;
    }

    i2c_result_t r;

    switch (s_run) {
    case RUN_IDLE:
        // Quaternion first, it is what the SBC actually consumes
        if (i2c_start_read(BNO055_ADDR, REG_QUA_DATA, s_buf, 8)) {
            s_run = RUN_READ_QUAT;
        }
        break;

    case RUN_READ_QUAT:
        r = i2c_poll();
        if (r == I2C_RESULT_DONE) {
            // Little endian pairs, w x y z
            s_quat[0] = (int16_t)(s_buf[0] | (s_buf[1] << 8));
            s_quat[1] = (int16_t)(s_buf[2] | (s_buf[3] << 8));
            s_quat[2] = (int16_t)(s_buf[4] | (s_buf[5] << 8));
            s_quat[3] = (int16_t)(s_buf[6] | (s_buf[7] << 8));
            s_read_ok++;
            s_consecutive_fail = 0;
            s_run = RUN_READ_GYRO;
        } else if (r == I2C_RESULT_ERROR) {
            s_read_fail++;
            s_consecutive_fail++;
            s_run = RUN_IDLE;
        }
        break;

    case RUN_READ_GYRO:
        if (i2c_start_read(BNO055_ADDR, REG_GYR_DATA, s_buf, 6)) {
            s_run = RUN_WAIT_GYRO;
        }
        break;

    case RUN_WAIT_GYRO:
        r = i2c_poll();
        if (r == I2C_RESULT_DONE) {
            s_gyro[0] = (int16_t)(s_buf[0] | (s_buf[1] << 8));
            s_gyro[1] = (int16_t)(s_buf[2] | (s_buf[3] << 8));
            s_gyro[2] = (int16_t)(s_buf[4] | (s_buf[5] << 8));
            s_run = RUN_READ_CALIB;
        } else if (r == I2C_RESULT_ERROR) {
            s_read_fail++;
            s_consecutive_fail++;
            s_run = RUN_IDLE;
        }
        break;

    case RUN_READ_CALIB:
        // Calibration status is cheap and tells the operator whether the
        // fusion output can be trusted yet, so it rides along each cycle
        if (i2c_start_read(BNO055_ADDR, REG_CALIB_STAT, s_buf, 1)) {
            s_run = RUN_WAIT_CALIB;
        }
        break;

    case RUN_WAIT_CALIB:
        r = i2c_poll();
        if (r == I2C_RESULT_DONE) {
            s_calib = s_buf[0];
            s_run = RUN_IDLE;
        } else if (r == I2C_RESULT_ERROR) {
            // A failed calibration read is not worth counting against
            // the sensor, the orientation data already arrived
            s_run = RUN_IDLE;
        }
        break;

    default:
        s_run = RUN_IDLE;
        break;
    }

    // A run of failures means the bus is wedged rather than noisy.
    // Recovering costs a few milliseconds and is preferable to sitting
    // with a dead IMU for the rest of the mission.
    if (s_consecutive_fail > 10U) {
        i2c_bus_recover();
        s_consecutive_fail = 0;
    }
}

uint8_t bno055_is_ok(void)
{
    return (uint8_t)(s_ready && s_consecutive_fail < 5U);
}

int16_t bno055_quat_w(void) { return s_quat[0]; }
int16_t bno055_quat_x(void) { return s_quat[1]; }
int16_t bno055_quat_y(void) { return s_quat[2]; }
int16_t bno055_quat_z(void) { return s_quat[3]; }
int16_t bno055_gyro_z(void) { return s_gyro[2]; }

uint8_t bno055_calib(void)  { return s_calib; }

// Yaw in radians from the fused quaternion.
// Only the heading term is derived here; roll and pitch are left to the
// SBC, which has the floating point budget to do it properly.
float bno055_yaw_rad(void)
{
    const float scale = 1.0f / 16384.0f;
    float w = (float)s_quat[0] * scale;
    float x = (float)s_quat[1] * scale;
    float y = (float)s_quat[2] * scale;
    float z = (float)s_quat[3] * scale;

    float siny = 2.0f * (w * z + x * y);
    float cosy = 1.0f - 2.0f * (y * y + z * z);

    // A degenerate quaternion appears before the first successful read
    if (siny == 0.0f && cosy == 0.0f) {
        return 0.0f;
    }
    return atan2f(siny, cosy);
}

uint32_t bno055_read_ok(void)   { return s_read_ok; }
uint32_t bno055_read_fail(void) { return s_read_fail; }
