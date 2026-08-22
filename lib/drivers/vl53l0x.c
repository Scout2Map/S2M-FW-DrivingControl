/*
 * File   : vl53l0x.c
 * Purpose: VL53L0X time of flight range finder, an alternative to the
 *          analog Sharp sensor on the same distance interface.
 * Author : jihoonkimtech
 *
 * Wiring
 *   I2C2 SCL = PB10, SDA = PB11, shared with the BNO055
 *   Address 0x29, the part default
 *   VIN from 5V through the breakout regulator, GND common
 *   XSHUT and GPIO1 unused with a single sensor
 *
 * The BNO055 must move to 0x28 by tying its ADDR pin to GND, because it
 * also sits at 0x29. Both devices then share I2C2 without contention.
 *
 * Note: the initialisation is a long ordered sequence, so it is encoded
 * as an opcode table walked one entry per pump call by the same non
 * blocking machine the BNO055 uses. A sensor that never answers cannot
 * stall the 200Hz control loop.
 *
 * Note: several registers need read-modify-write rather than a plain
 * store. 0x89 bit 0 selects 2.8V pad mode and the surrounding bits are
 * device state; 0x60 disables two limit checks by setting bits. Writing
 * a literal to either clears configuration the part depends on, which
 * is why OP_SET_BITS and OP_CLR_BITS exist.
 *
 * Note: reference calibration is not optional. Without the VHV and
 * phase passes the part either ranges badly or not at all, so the two
 * calibration states below run before continuous mode starts.
 *
 * Note: the SPAD configuration and the reference tuning block from ST's
 * driver are NOT applied. The part operates on its defaults, which is
 * enough to range, but maximum range and ambient immunity will be below
 * the datasheet figures. If range proves short, that block is the next
 * thing to add.
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
#define REG_FINAL_RANGE_MIN_COUNT_HI    0x44
#define REG_FINAL_RANGE_MIN_COUNT_LO    0x45
#define REG_GPIO_HV_MUX_ACTIVE_HIGH     0x84
#define REG_STOP_VARIABLE               0x91
#define REG_IDENTIFICATION_MODEL_ID     0xC0

#define MODEL_ID_EXPECTED       0xEE

// The range result sits ten bytes past the status register
#define RANGE_OFFSET            10U
#define VL_OUT_OF_RANGE         8190U

// ---- Init program ----
typedef enum {
    OP_WRITE = 0,   // store val
    OP_SET_BITS,    // read, or in val, write back
    OP_CLR_BITS,    // read, mask out val, write back
    OP_READ_STOP,   // capture the stop variable
    OP_WRITE_STOP,  // write the captured stop variable back
    OP_END
} op_t;

typedef struct {
    uint8_t op;
    uint8_t reg;
    uint8_t val;
} init_op_t;

static const init_op_t s_prog[] = {
    // 2.8V pad mode, preserving the rest of the register
    { OP_SET_BITS, 0x89, 0x01 },

    // Unlock the private register space and capture the stop variable
    { OP_WRITE, 0x88, 0x00 },
    { OP_WRITE, 0x80, 0x01 },
    { OP_WRITE, 0xFF, 0x01 },
    { OP_WRITE, 0x00, 0x00 },
    { OP_READ_STOP, REG_STOP_VARIABLE, 0x00 },
    { OP_WRITE, 0x00, 0x01 },
    { OP_WRITE, 0xFF, 0x00 },
    { OP_WRITE, 0x80, 0x00 },

    // Drop the signal rate and pre range limit checks
    { OP_SET_BITS, REG_MSRC_CONFIG_CONTROL, 0x12 },

    // Final range minimum count rate, 0.25 MCPS in 9.7 fixed point
    { OP_WRITE, REG_FINAL_RANGE_MIN_COUNT_HI, 0x00 },
    { OP_WRITE, REG_FINAL_RANGE_MIN_COUNT_LO, 0x20 },

    { OP_WRITE, REG_SYSTEM_SEQUENCE_CONFIG, 0xFF },

    // Interrupt on sample ready, active low
    { OP_WRITE, REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04 },
    { OP_CLR_BITS, REG_GPIO_HV_MUX_ACTIVE_HIGH, 0x10 },
    { OP_WRITE, REG_SYSTEM_INTERRUPT_CLEAR, 0x01 },

    { OP_WRITE, REG_SYSTEM_SEQUENCE_CONFIG, 0xE8 },
    { OP_END, 0x00, 0x00 },
};

// Run before continuous mode. Each entry is one calibration pass: the
// sequence register selects which, and SYSRANGE_START launches it.
static const init_op_t s_start_prog[] = {
    { OP_WRITE, 0x80, 0x01 },
    { OP_WRITE, 0xFF, 0x01 },
    { OP_WRITE, 0x00, 0x00 },
    { OP_WRITE_STOP, REG_STOP_VARIABLE, 0x00 },
    { OP_WRITE, 0x00, 0x01 },
    { OP_WRITE, 0xFF, 0x00 },
    { OP_WRITE, 0x80, 0x00 },
    { OP_WRITE, REG_SYSRANGE_START, 0x02 },   // continuous back to back
    { OP_END, 0x00, 0x00 },
};

typedef enum {
    ST_WAIT_BOOT = 0,
    ST_READ_ID,
    ST_CHECK_ID,
    ST_PROG_STEP,
    ST_PROG_RD,       // read phase of a modify or a stop capture
    ST_PROG_WR,       // write phase
    ST_CAL_SEQ,       // select the calibration pass
    ST_CAL_SEQ_WAIT,
    ST_CAL_START,
    ST_CAL_START_WAIT,
    ST_CAL_POLL,
    ST_CAL_POLL_WAIT,
    ST_CAL_CLEAR,
    ST_CAL_CLEAR_WAIT,
    ST_CAL_STOP,
    ST_CAL_STOP_WAIT,
    ST_SEQ_RESTORE,
    ST_SEQ_RESTORE_WAIT,
    ST_START_STEP,
    ST_START_RD,
    ST_START_WR,
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
static uint8_t    s_pc;             // program counter into the tables
static uint8_t    s_cal_pass;       // 0 = VHV, 1 = phase
static uint8_t    s_buf[4];
static uint8_t    s_wr[2];
static uint8_t    s_stop_var;
static uint8_t    s_model_id;
static uint16_t   s_range_mm;
static uint16_t   s_dist_mm;
static uint32_t   s_read_ok;
static uint32_t   s_read_fail;
static uint8_t    s_consecutive_fail;
static uint32_t   s_wait_until;
static uint32_t   s_cal_guard;

// Calibration pass parameters: sequence config, then the start value
static const uint8_t s_cal_seq[2]   = { 0x01, 0x02 };
static const uint8_t s_cal_start[2] = { 0x41, 0x01 };

static void wait_ms(uint32_t ms) { s_wait_until = systick_millis() + ms; }
static uint8_t wait_done(void)
{
    return (uint8_t)((int32_t)(systick_millis() - s_wait_until) >= 0);
}

void vl53l0x_init(void)
{
    i2c_init();
    s_state    = ST_WAIT_BOOT;
    s_run      = RUN_IDLE;
    s_pc       = 0;
    s_cal_pass = 0;
    s_stop_var = 0;
    s_model_id = 0xFF;
    s_range_mm = 0;
    s_dist_mm  = DIST_INVALID;
    s_read_ok  = 0;
    s_read_fail = 0;
    s_consecutive_fail = 0;
    s_cal_guard = 0;
    wait_ms(50);        // the part needs a moment after power on
}

// Walks one opcode table. Returns 1 when the table has finished.
static uint8_t run_program(const init_op_t *prog,
                           vl_state_t st_step,
                           vl_state_t st_rd,
                           vl_state_t st_wr)
{
    i2c_result_t r;
    const init_op_t *op = &prog[s_pc];

    if (s_state == st_step) {
        switch (op->op) {
        case OP_END:
            return 1;

        case OP_WRITE:
            if (i2c_start_write(VL_ADDR, op->reg, &op->val, 1)) {
                s_state = st_wr;
            }
            break;

        case OP_WRITE_STOP:
            s_wr[0] = s_stop_var;
            if (i2c_start_write(VL_ADDR, op->reg, s_wr, 1)) {
                s_state = st_wr;
            }
            break;

        case OP_SET_BITS:
        case OP_CLR_BITS:
        case OP_READ_STOP:
            if (i2c_start_read(VL_ADDR, op->reg, s_buf, 1)) {
                s_state = st_rd;
            }
            break;

        default:
            s_pc++;
            break;
        }
        return 0;
    }

    if (s_state == st_rd) {
        r = i2c_poll();
        if (r == I2C_RESULT_DONE) {
            if (op->op == OP_READ_STOP) {
                s_stop_var = s_buf[0];
                s_pc++;
                s_state = st_step;
            } else {
                s_wr[0] = (op->op == OP_SET_BITS)
                        ? (uint8_t)(s_buf[0] | op->val)
                        : (uint8_t)(s_buf[0] & (uint8_t)~op->val);
                if (i2c_start_write(VL_ADDR, op->reg, s_wr, 1)) {
                    s_state = st_wr;
                }
            }
        } else if (r == I2C_RESULT_ERROR) {
            s_read_fail++;
            s_state = st_step;      // retry the same opcode
        }
        return 0;
    }

    if (s_state == st_wr) {
        r = i2c_poll();
        if (r == I2C_RESULT_DONE) {
            s_pc++;
            s_state = st_step;
        } else if (r == I2C_RESULT_ERROR) {
            s_read_fail++;
            s_state = st_step;
        }
        return 0;
    }

    return 0;
}

static void init_pump(void)
{
    i2c_result_t r;
    static const uint8_t seq_final = 0xE8;
    static const uint8_t zero = 0x00;
    static const uint8_t clear = 0x01;

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
                s_pc = 0;
                s_state = ST_PROG_STEP;
            } else {
                // Wrong ID means the wrong device or a clashing address,
                // neither of which retrying can fix
                s_state = ST_FAILED;
            }
        } else if (r == I2C_RESULT_ERROR) {
            wait_ms(200);
            s_state = ST_WAIT_BOOT;
        }
        break;

    case ST_PROG_STEP:
    case ST_PROG_RD:
    case ST_PROG_WR:
        if (run_program(s_prog, ST_PROG_STEP, ST_PROG_RD, ST_PROG_WR)) {
            s_cal_pass = 0;
            s_state = ST_CAL_SEQ;
        }
        break;

    // ---- Reference calibration, two passes ----
    case ST_CAL_SEQ:
        if (i2c_start_write(VL_ADDR, REG_SYSTEM_SEQUENCE_CONFIG,
                            &s_cal_seq[s_cal_pass], 1)) {
            s_state = ST_CAL_SEQ_WAIT;
        }
        break;

    case ST_CAL_SEQ_WAIT:
        r = i2c_poll();
        if (r == I2C_RESULT_DONE)      s_state = ST_CAL_START;
        else if (r == I2C_RESULT_ERROR) s_state = ST_CAL_SEQ;
        break;

    case ST_CAL_START:
        if (i2c_start_write(VL_ADDR, REG_SYSRANGE_START,
                            &s_cal_start[s_cal_pass], 1)) {
            s_cal_guard = systick_millis();
            s_state = ST_CAL_START_WAIT;
        }
        break;

    case ST_CAL_START_WAIT:
        r = i2c_poll();
        if (r == I2C_RESULT_DONE)      s_state = ST_CAL_POLL;
        else if (r == I2C_RESULT_ERROR) s_state = ST_CAL_START;
        break;

    case ST_CAL_POLL:
        if (i2c_start_read(VL_ADDR, REG_RESULT_INTERRUPT_STATUS, s_buf, 1)) {
            s_state = ST_CAL_POLL_WAIT;
        }
        break;

    case ST_CAL_POLL_WAIT:
        r = i2c_poll();
        if (r == I2C_RESULT_DONE) {
            if (s_buf[0] & 0x07U) {
                s_state = ST_CAL_CLEAR;
            } else if ((systick_millis() - s_cal_guard) > 500U) {
                // Calibration should finish in a few milliseconds. A
                // part that never raises the flag is not going to.
                s_state = ST_FAILED;
            } else {
                s_state = ST_CAL_POLL;
            }
        } else if (r == I2C_RESULT_ERROR) {
            s_state = ST_CAL_POLL;
        }
        break;

    case ST_CAL_CLEAR:
        if (i2c_start_write(VL_ADDR, REG_SYSTEM_INTERRUPT_CLEAR, &clear, 1)) {
            s_state = ST_CAL_CLEAR_WAIT;
        }
        break;

    case ST_CAL_CLEAR_WAIT:
        r = i2c_poll();
        if (r == I2C_RESULT_DONE)      s_state = ST_CAL_STOP;
        else if (r == I2C_RESULT_ERROR) s_state = ST_CAL_CLEAR;
        break;

    case ST_CAL_STOP:
        if (i2c_start_write(VL_ADDR, REG_SYSRANGE_START, &zero, 1)) {
            s_state = ST_CAL_STOP_WAIT;
        }
        break;

    case ST_CAL_STOP_WAIT:
        r = i2c_poll();
        if (r == I2C_RESULT_DONE) {
            if (++s_cal_pass < 2U) {
                s_state = ST_CAL_SEQ;       // second pass
            } else {
                s_state = ST_SEQ_RESTORE;
            }
        } else if (r == I2C_RESULT_ERROR) {
            s_state = ST_CAL_STOP;
        }
        break;

    case ST_SEQ_RESTORE:
        // Put the full measurement sequence back after calibration
        if (i2c_start_write(VL_ADDR, REG_SYSTEM_SEQUENCE_CONFIG,
                            &seq_final, 1)) {
            s_state = ST_SEQ_RESTORE_WAIT;
        }
        break;

    case ST_SEQ_RESTORE_WAIT:
        r = i2c_poll();
        if (r == I2C_RESULT_DONE) {
            s_pc = 0;
            s_state = ST_START_STEP;
        } else if (r == I2C_RESULT_ERROR) {
            s_state = ST_SEQ_RESTORE;
        }
        break;

    case ST_START_STEP:
    case ST_START_RD:
    case ST_START_WR:
        if (run_program(s_start_prog, ST_START_STEP,
                        ST_START_RD, ST_START_WR)) {
            s_state = ST_READY;
        }
        break;

    default:
        break;
    }
}

static void run_pump(void)
{
    i2c_result_t r;
    static const uint8_t clear = 0x01;

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
            // Bits 2:0 hold the interrupt reason, zero means not ready
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
            // Big endian here, unlike most of this device
            s_range_mm = (uint16_t)((s_buf[0] << 8) | s_buf[1]);

            if (s_range_mm >= VL_OUT_OF_RANGE || s_range_mm > DIST_MAX_MM) {
                s_dist_mm = DIST_INVALID;
            } else if (s_range_mm < DIST_MIN_MM) {
                // Below the specified minimum the number is unreliable
                // rather than merely small, so it is reported as such
                s_dist_mm = DIST_TOO_CLOSE;
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

    case RUN_CLEAR_INT:
        if (i2c_start_write(VL_ADDR, REG_SYSTEM_INTERRUPT_CLEAR, &clear, 1)) {
            s_run = RUN_WAIT_CLEAR;
        }
        break;

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

void vl53l0x_pump(void)
{
    if (s_state != ST_READY) {
        init_pump();
        return;
    }
    run_pump();
}

void vl53l0x_poll(void)
{
    if (s_state == ST_READY && s_run == RUN_IDLE) {
        s_run = RUN_READ_STATUS;
    }
}

uint8_t vl53l0x_is_ok(void)
{
    return (uint8_t)(s_state == ST_READY && s_consecutive_fail < 5U);
}

uint8_t  vl53l0x_model_id(void)  { return s_model_id; }

// The raw state index is useful for diagnosis but must not be something
// the host has to interpret: inserting a state would silently shift it.
// Ready and failed are reported as their own flags instead.
uint8_t  vl53l0x_state(void)     { return (uint8_t)s_state; }
uint8_t  vl53l0x_is_ready(void)  { return (uint8_t)(s_state == ST_READY); }
uint8_t  vl53l0x_is_failed(void) { return (uint8_t)(s_state == ST_FAILED); }
uint16_t vl53l0x_raw_mm(void)    { return s_range_mm; }
uint32_t vl53l0x_read_ok(void)   { return s_read_ok; }
uint32_t vl53l0x_read_fail(void) { return s_read_fail; }

// Shared distance interface. Only one distance driver defines these.
uint16_t dist_get_mm(void)       { return s_dist_mm; }
uint8_t  dist_is_too_close(void) { return (uint8_t)(s_dist_mm == DIST_TOO_CLOSE); }
uint16_t dist_get_counts(void)   { return s_range_mm; }   // raw mm
uint16_t dist_get_mv(void)       { return 0; }            // not analog

#endif // DIST_SENSOR == DIST_SENSOR_VL53L0X
