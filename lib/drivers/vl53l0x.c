/*
 * File   : vl53l0x.c
 * Purpose: VL53L0X time of flight range finder, as an alternative to the
 *          analog Sharp sensor on the same distance interface.
 * Author : jihoonkimtech
 *
 * Wiring
 *   I2C2 SCL = PB10, SDA = PB11, shared with the BNO055
 *   Address 0x29, which is the part default
 *   Powered from 5V through the breakout regulator
 *
 * The BNO055 must be moved to 0x28 by pulling its ADDR pin LOW, because
 * it also defaults to 0x29. Both devices then coexist on I2C2.
 *
 * Why this over the analog Sharp part: the output is a digital distance
 * in millimetres, so there is no response curve to calibrate and no
 * non-monotonic blind zone to latch around. It also reaches further and
 * costs no ADC channel.
 *
 * Note: the initialisation sequence is a long list of register writes.
 * Rather than blocking through it, the table below is walked one entry
 * per pump call by the same non-blocking state machine the BNO055 uses,
 * so a sensor that never answers cannot stall the control loop.
 *
 * Note: this driver is UNTESTED against hardware. The register sequence
 * follows the published minimal initialisation, but the part is complex
 * and ST's own API is far larger. Treat a first bring-up as debugging,
 * not as verification.
 */

#include <string.h>
#include "board_config.h"

#if DIST_SENSOR == DIST_SENSOR_VL53L0X

#include "vl53l0x.h"
#include "i2c.h"
#include "systick.h"

#define VL_ADDR                 0x29U

// ---- Registers ----
#define REG_SYSRANGE_START              0x00
#define REG_SYSTEM_SEQUENCE_CONFIG      0x01
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO 0x0A
#define REG_SYSTEM_INTERRUPT_CLEAR      0x0B
#define REG_RESULT_INTERRUPT_STATUS     0x13
#define REG_RESULT_RANGE_STATUS         0x14
#define REG_MSRC_CONFIG_CONTROL         0x60
#define REG_FINAL_RANGE_CFG_MIN_CR_RTN  0x44
#define REG_GPIO_HV_MUX_ACTIVE_HIGH     0x84
#define REG_IDENTIFICATION_MODEL_ID     0xC0

#define MODEL_ID_EXPECTED       0xEE

// Range result sits ten bytes past the status register
#define RANGE_OFFSET            10U

// The sensor reports these when nothing is in range
#define VL_OUT_OF_RANGE         8190U

typedef struct {
    uint8_t reg;
    uint8_t val;
} reg_write_t;

// Minimal initialisation. Several entries are undocumented addresses
// carried over from ST's reference driver; they are required and their
// meaning is not published.
static const reg_write_t s_init_seq[] = {
    // Use 2.8V mode on the IO pads
    { 0x89, 0x01 },
    // Unlock the private register space and read the stop variable,
    // which the start sequence later needs
    { 0x88, 0x00 },
    { 0x80, 0x01 },
    { 0xFF, 0x01 },
    { 0x00, 0x00 },
    // 0x91 is read here at runtime, see INIT_READ_STOP
    { 0x00, 0x01 },
    { 0xFF, 0x00 },
    { 0x80, 0x00 },

    // Disable the signal rate and range ignore limit checks
    { REG_MSRC_CONFIG_CONTROL, 0x12 },
    // Minimum count rate return limit, 0.25 MCPS in 9.7 fixed point
    { REG_FINAL_RANGE_CFG_MIN_CR_RTN, 0x00 },
    { 0x45, 0x20 },
    { REG_SYSTEM_SEQUENCE_CONFIG, 0xFF },

    // Interrupt on new sample ready, active low
    { REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04 },
    { REG_SYSTEM_INTERRUPT_CLEAR, 0x01 },

    // Enable the standard measurement sequence steps
    { REG_SYSTEM_SEQUENCE_CONFIG, 0xE8 },
};
#define INIT_SEQ_LEN (sizeof s_init_seq / sizeof s_init_seq[0])

typedef enum {
    ST_WAIT_BOOT = 0,
    ST_READ_ID,
    ST_CHECK_ID,
    ST_SEQ_WRITE,
    ST_SEQ_WAIT,
    ST_READ_STOP,
    ST_WAIT_STOP,
    ST_START_WRITE,
    ST_START_WAIT,
    ST_READY,
    ST_FAILED
} vl_state_t;

typedef enum {
    RUN_IDLE = 0,
    RUN_READ_STATUS,
    RUN_WAIT_STATUS,
    RUN_READ_RANGE,
    RUN_WAIT_RANGE,
    RUN_CLEAR_INT,
    RUN_WAIT_CLEAR
} vl_run_t;

static vl_state_t s_state;
static vl_run_t   s_run;
static uint8_t    s_seq_idx;
static uint8_t    s_buf[4];
static uint8_t    s_stop_var;
static uint8_t    s_model_id;
static uint16_t   s_range_mm;
static uint16_t   s_dist_mm;
static uint32_t   s_read_ok;
static uint32_t   s_read_fail;
static uint8_t    s_consecutive_fail;
static uint32_t   s_wait_until;

// Written to SYSRANGE_START to begin continuous back to back ranging
static uint8_t s_start_cmd = 0x02;

static void wait_ms(uint32_t ms) { s_wait_until = systick_millis() + ms; }
static uint8_t wait_done(void)
{
    return (uint8_t)((int32_t)(systick_millis() - s_wait_until) >= 0);
}

void vl53l0x_init(void)
{
    i2c_init();
    s_state   = ST_WAIT_BOOT;
    s_run     = RUN_IDLE;
    s_seq_idx = 0;
    s_stop_var = 0;
    s_model_id = 0xFF;
    s_range_mm = 0;
    s_dist_mm  = DIST_INVALID;
    s_read_ok = 0;
    s_read_fail = 0;
    s_consecutive_fail = 0;
    wait_ms(50);        // the part needs a moment after power on
}

static void init_pump(void)
{
    i2c_result_t r;

    switch (s_state) {
    case ST_WAIT_BOOT:
        if (wait_done()) s_state = ST_READ_ID;
        break;

    case ST_READ_ID:
        if (i2c_start_read(VL_ADDR, REG_IDENTIFICATION_MODEL_ID, s_buf, 1)) {
            s_state = ST_CHECK_ID;
        }
        break;

    case ST_CHECK_ID:
        r = i2c_poll();
        if (r == I2C_RESULT_DONE) {
            s_model_id = s_buf[0];
            if (s_buf[0] == MODEL_ID_EXPECTED) {
                s_seq_idx = 0;
                s_state = ST_SEQ_WRITE;
            } else {
                // Wrong ID means the wrong device or the wrong address.
                // Retrying will not change that, so stop and report.
                s_state = ST_FAILED;
            }
        } else if (r == I2C_RESULT_ERROR) {
            wait_ms(200);
            s_state = ST_WAIT_BOOT;
        }
        break;

    case ST_SEQ_WRITE:
        if (s_seq_idx >= INIT_SEQ_LEN) {
            s_state = ST_START_WRITE;
            break;
        }
        // The stop variable has to be read partway through the sequence
        if (s_seq_idx == 5U) {
            s_state = ST_READ_STOP;
            break;
        }
        if (i2c_start_write(VL_ADDR, s_init_seq[s_seq_idx].reg,
                            &s_init_seq[s_seq_idx].val, 1)) {
            s_state = ST_SEQ_WAIT;
        }
        break;

    case ST_SEQ_WAIT:
        r = i2c_poll();
        if (r == I2C_RESULT_DONE) {
            s_seq_idx++;
            s_state = ST_SEQ_WRITE;
        } else if (r == I2C_RESULT_ERROR) {
            s_read_fail++;
            s_state = ST_SEQ_WRITE;     // retry the same entry
        }
        break;

    case ST_READ_STOP:
        if (i2c_start_read(VL_ADDR, 0x91, s_buf, 1)) {
            s_state = ST_WAIT_STOP;
        }
        break;

    case ST_WAIT_STOP:
        r = i2c_poll();
        if (r == I2C_RESULT_DONE) {
            s_stop_var = s_buf[0];
            s_seq_idx++;
            s_state = ST_SEQ_WRITE;
        } else if (r == I2C_RESULT_ERROR) {
            s_state = ST_READ_STOP;
        }
        break;

    case ST_START_WRITE:
        // Continuous back to back ranging. The default timing budget is
        // about 33ms, which sits comfortably inside DIST_PERIOD_MS.
        if (i2c_start_write(VL_ADDR, REG_SYSRANGE_START, &s_start_cmd, 1)) {
            s_state = ST_START_WAIT;
        }
        break;

    case ST_START_WAIT:
        r = i2c_poll();
        if (r == I2C_RESULT_DONE) {
            s_state = ST_READY;
        } else if (r == I2C_RESULT_ERROR) {
            s_state = ST_START_WRITE;
        }
        break;

    default:
        break;
    }
}

static void run_pump(void)
{
    i2c_result_t r;

    switch (s_run) {
    case RUN_IDLE:
        break;

    case RUN_READ_STATUS:
        if (i2c_start_read(VL_ADDR, REG_RESULT_INTERRUPT_STATUS, s_buf, 1)) {
            s_run = RUN_WAIT_STATUS;
        }
        break;

    case RUN_WAIT_STATUS:
        r = i2c_poll();
        if (r == I2C_RESULT_DONE) {
            // Bits 2:0 hold the interrupt reason; zero means not ready
            s_run = (s_buf[0] & 0x07U) ? RUN_READ_RANGE : RUN_IDLE;
        } else if (r == I2C_RESULT_ERROR) {
            s_read_fail++;
            s_consecutive_fail++;
            s_run = RUN_IDLE;
        }
        break;

    case RUN_READ_RANGE:
        if (i2c_start_read(VL_ADDR,
                           REG_RESULT_RANGE_STATUS + RANGE_OFFSET, s_buf, 2)) {
            s_run = RUN_WAIT_RANGE;
        }
        break;

    case RUN_WAIT_RANGE:
        r = i2c_poll();
        if (r == I2C_RESULT_DONE) {
            // Big endian, unlike most of this device
            s_range_mm = (uint16_t)((s_buf[0] << 8) | s_buf[1]);

            if (s_range_mm >= VL_OUT_OF_RANGE) {
                s_dist_mm = DIST_INVALID;
            } else if (s_range_mm < DIST_MIN_MM) {
                // Below the specified minimum the reading is unreliable
                // rather than merely close, so report it as such instead
                // of passing a number the caller would trust
                s_dist_mm = DIST_TOO_CLOSE;
            } else if (s_range_mm > DIST_MAX_MM) {
                s_dist_mm = DIST_INVALID;
            } else {
                s_dist_mm = s_range_mm;
            }

            s_read_ok++;
            s_consecutive_fail = 0;
            s_run = RUN_CLEAR_INT;
        } else if (r == I2C_RESULT_ERROR) {
            s_read_fail++;
            s_consecutive_fail++;
            s_run = RUN_IDLE;
        }
        break;

    case RUN_CLEAR_INT: {
        static const uint8_t clear = 0x01;
        if (i2c_start_write(VL_ADDR, REG_SYSTEM_INTERRUPT_CLEAR, &clear, 1)) {
            s_run = RUN_WAIT_CLEAR;
        }
        break;
    }

    case RUN_WAIT_CLEAR:
        r = i2c_poll();
        if (r != I2C_RESULT_BUSY) {
            s_run = RUN_IDLE;
        }
        break;

    default:
        s_run = RUN_IDLE;
        break;
    }

    // A run of failures means the bus is wedged rather than noisy
    if (s_consecutive_fail > 10U) {
        i2c_bus_recover();
        s_consecutive_fail = 0;
    }
}

// Advances whatever transfer is in flight. Call every main loop pass.
void vl53l0x_pump(void)
{
    if (s_state != ST_READY) {
        init_pump();
        return;
    }
    run_pump();
}

// Starts a fresh read cycle. Call at DIST_PERIOD_MS.
void vl53l0x_poll(void)
{
    if (s_state == ST_READY && s_run == RUN_IDLE) {
        s_run = RUN_READ_STATUS;
    }
}

uint8_t  vl53l0x_is_ok(void)
{
    return (uint8_t)(s_state == ST_READY && s_consecutive_fail < 5U);
}

uint8_t  vl53l0x_model_id(void)  { return s_model_id; }
uint8_t  vl53l0x_state(void)     { return (uint8_t)s_state; }
uint16_t vl53l0x_raw_mm(void)    { return s_range_mm; }
uint32_t vl53l0x_read_ok(void)   { return s_read_ok; }
uint32_t vl53l0x_read_fail(void) { return s_read_fail; }

// Shared distance interface. Only one distance driver defines these.
uint16_t dist_get_mm(void)       { return s_dist_mm; }
uint8_t  dist_is_too_close(void) { return (uint8_t)(s_dist_mm == DIST_TOO_CLOSE); }
uint16_t dist_get_counts(void)   { return s_range_mm; }   // raw, for --dist
uint16_t dist_get_mv(void)       { return 0; }            // not analog

#endif // DIST_SENSOR == DIST_SENSOR_VL53L0X
