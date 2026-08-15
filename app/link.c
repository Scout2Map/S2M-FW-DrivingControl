/*
 * File   : link.c
 * Purpose: Binds the USB CDC transport to the protocol framing and the
 *          drive control layer. Parses inbound commands and emits
 *          periodic telemetry.
 * Author : jihoonkimtech
 *
 * This is the only place that knows both the wire format and the local
 * control API, keeping protocol details out of lib/control and hardware
 * details out of the framing code.
 *
 * Note: telemetry is dropped rather than queued when the previous packet
 * is still in flight. A stale reading delivered late is worse than a
 * missing one, since the host timestamps on arrival for anything it
 * cannot correlate, and the next frame is only 20ms away.
 *
 * Note: commands are not acknowledged. The host republishes cmd_vel
 * continuously, so a dropped command corrects itself within one period,
 * and the command timeout in drive.c covers a genuine link loss.
 */

#include <string.h>
#include "board_config.h"
#include "link.h"
#include "usb_cdc.h"
#include "framing.h"
#include "drive.h"
#include "motor.h"
#include "encoder.h"
#include "bno055.h"
#include "i2c.h"
#include "systick.h"

// Firmware version reported in MSG_BOOT_INFO
#define FW_MAJOR    0
#define FW_MINOR    2
#define FW_PATCH    0

static frame_decoder_t s_dec;
static uint8_t         s_txbuf[PROTO_MAX_FRAME];
static uint8_t         s_boot_sent;
static uint8_t         s_estop_latched;
static uint32_t        s_frames_tx;
static uint32_t        s_frames_dropped;

// Supplied by the IR distance driver. 0xFFFF means no valid reading.
uint16_t dist_get_mm(void) __attribute__((weak));
uint16_t dist_get_mm(void) { return 0xFFFFU; }

// Supplied by the battery monitor once the ADC driver lands.
uint16_t batt_get_mv(void) __attribute__((weak));
uint16_t batt_get_mv(void) { return 0; }

void link_init(void)
{
    frame_decoder_init(&s_dec);
    usb_cdc_init();
    s_boot_sent      = 0;
    s_estop_latched  = 0;
    s_frames_tx      = 0;
    s_frames_dropped = 0;
}

static void send_frame(uint8_t type, const void *payload, uint8_t len)
{
    uint16_t n = frame_encode(s_txbuf, type, payload, len);
    if (usb_cdc_write(s_txbuf, n)) {
        s_frames_tx++;
    } else {
        s_frames_dropped++;
    }
}

static void send_boot_info(void)
{
    boot_info_t bi = {
        .proto_version        = PROTO_VERSION,
        .fw_major             = FW_MAJOR,
        .fw_minor             = FW_MINOR,
        .fw_patch             = FW_PATCH,
        // The host must not have to hardcode the odometry scale; a motor
        // swap changes this and the bridge should follow automatically
        .counts_per_wheel_rev = COUNTS_PER_WHEEL_REV,
        .wheel_base_mm        = (uint16_t)WHEEL_BASE_MM,
    };
    send_frame(MSG_BOOT_INFO, &bi, sizeof bi);
}

static void handle_command(uint8_t type, const uint8_t *payload, uint8_t len)
{
    switch (type) {
    case MSG_CMD_VELOCITY:
        if (len != sizeof(cmd_velocity_t) || s_estop_latched) {
            break;
        }
        {
            cmd_velocity_t c;
            memcpy(&c, payload, sizeof c);
            drive_command((float)c.linear_mmps    * 0.001f,
                          (float)c.angular_mradps * 0.001f);
        }
        break;

    case MSG_CMD_WHEEL_RAW:
        if (len != sizeof(cmd_wheel_raw_t) || s_estop_latched) {
            break;
        }
        {
            cmd_wheel_raw_t c;
            memcpy(&c, payload, sizeof c);
            // Bypasses the velocity loop on purpose. During bring-up this
            // separates a wiring or gearing fault from a tuning fault.
            drive_set_raw(c.left_permille, c.right_permille);
        }
        break;

    case MSG_CMD_ESTOP:
        // Latched deliberately. An emergency stop that a subsequent
        // velocity command could silently undo is not an emergency stop.
        s_estop_latched = 1;
        motor_estop();
        drive_stop();
        break;

    case MSG_CMD_CLEAR_FAULT:
        s_estop_latched = 0;
        motor_clear_fault();
        motor_enable();
        break;

    case MSG_CMD_RESET_ODOM:
        drive_reset_odom();
        encoder_reset();
        break;

    case MSG_CMD_PING:
        send_frame(MSG_PONG, 0, 0);
        break;

    case MSG_CMD_I2C_SCAN: {
        // Blocking, but only ever triggered by hand during bring-up.
        // The drive loop pauses for a few milliseconds, which is why
        // the motors are stopped first.
        drive_stop();
        i2c_scan_t sc;
        sc.count = i2c_scan(sc.bitmap);
        send_frame(MSG_I2C_SCAN, &sc, sizeof sc);
        break;
    }

    case MSG_CMD_DIAG: {
        diag_t d;
        d.imu_init_step  = bno055_init_step();
        d.imu_chip_id    = bno055_last_id();
        d.imu_calib      = bno055_calib();
        d.reserved       = 0;
        d.imu_read_ok    = bno055_read_ok();
        d.imu_read_fail  = bno055_read_fail();
        d.i2c_errors     = (uint16_t)i2c_error_count();
        d.i2c_recoveries = (uint16_t)i2c_recovery_count();
        send_frame(MSG_DIAG, &d, sizeof d);
        break;
    }

    default:
        break;
    }
}

void link_poll_rx(void)
{
    uint8_t  buf[64];
    uint16_t n = usb_cdc_read(buf, sizeof buf);

    for (uint16_t i = 0; i < n; i++) {
        if (frame_decode_byte(&s_dec, buf[i])) {
            handle_command(s_dec.type, s_dec.payload, s_dec.len);
        }
    }
}

void link_send_telemetry(void)
{
    if (!usb_cdc_ready()) {
        // No host has the port open. Sending would only fill a buffer
        // that nobody drains, and the boot frame must be re-sent when a
        // host does attach.
        s_boot_sent = 0;
        return;
    }

    if (!s_boot_sent) {
        send_boot_info();
        s_boot_sent = 1;
        return;
    }

    telemetry_t t;
    float x, y, th;

    // Stamped here, at sample time, so the host can estimate the clock
    // offset rather than guessing from arrival time
    t.timestamp_ms = systick_millis();

    t.enc_left  = encoder_get_total(ENC_LEFT);
    t.enc_right = encoder_get_total(ENC_RIGHT);

    t.speed_left_mmps  = (int16_t)(encoder_get_speed_mps(ENC_LEFT)  * 1000.0f);
    t.speed_right_mmps = (int16_t)(encoder_get_speed_mps(ENC_RIGHT) * 1000.0f);

    drive_get_odom(&x, &y, &th);
    t.odom_x_mm       = (int32_t)(x * 1000.0f);
    t.odom_y_mm       = (int32_t)(y * 1000.0f);
    t.odom_theta_mrad = (int32_t)(th * 1000.0f);

    // Forwarded at the device native scaling, the SBC converts
    t.quat_w = bno055_quat_w();
    t.quat_x = bno055_quat_x();
    t.quat_y = bno055_quat_y();
    t.quat_z = bno055_quat_z();
    t.gyro_z = bno055_gyro_z();

    t.distance_mm = dist_get_mm();
    t.battery_mv  = batt_get_mv();

    t.duty_left  = drive_get_duty(0);
    t.duty_right = drive_get_duty(1);

    uint16_t st = 0;
    if (motor_is_enabled())   st |= STATUS_MOTOR_ENABLED;
    if (drive_is_openloop())  st |= STATUS_OPENLOOP;
    if (motor_get_fault())    st |= STATUS_FAULT_STALL;
    if (drive_cmd_expired())  st |= STATUS_CMD_TIMEOUT;
    if (s_estop_latched)      st |= STATUS_ESTOP_LATCHED;
    if (bno055_is_ok())       st |= STATUS_IMU_OK;
    // Bits 7:6 of the calibration byte are the system field; 3 means the
    // fusion result is trustworthy in absolute terms
    if ((bno055_calib() >> 6) == 3U) st |= STATUS_IMU_CALIBRATED;
    t.status = st;

    send_frame(MSG_TELEMETRY, &t, sizeof t);
}

uint32_t link_frames_tx(void)      { return s_frames_tx; }
uint32_t link_frames_dropped(void) { return s_frames_dropped; }
uint32_t link_crc_errors(void)     { return s_dec.crc_errors; }
