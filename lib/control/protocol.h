/*
 * File   : protocol.h
 * Purpose: Binary framing shared by the STM32 drive MCU and the RPi5
 *          bridge node. This header is the single source of truth for
 *          the wire format; PROTOCOL.md in scout2map-bridge mirrors it.
 * Author : jihoonkimtech
 *
 * Frame layout
 *   [0]     0xAA          sync high
 *   [1]     0x55          sync low
 *   [2]     TYPE          message type
 *   [3]     LEN           payload length, excludes header and CRC
 *   [4..]   PAYLOAD       LEN bytes, little endian
 *   [..+2]  CRC16         CCITT over TYPE, LEN and PAYLOAD
 *
 * Every frame fits inside 64 bytes so a bulk transfer never splits,
 * which keeps the host side reader from having to reassemble.
 *
 * Note: all multi byte fields are little endian, matching both the
 * Cortex-M3 and the ARM cores on the RPi5, so neither side byte swaps.
 *
 * Note: physical quantities travel as scaled integers rather than
 * floats. The MCU has no FPU, and fixed point keeps the frame compact
 * and the host parsing unambiguous.
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define PROTO_VERSION       1

#define PROTO_SYNC0         0xAAU
#define PROTO_SYNC1         0x55U
#define PROTO_HEADER_LEN    4U      // sync0, sync1, type, len
#define PROTO_CRC_LEN       2U
#define PROTO_MAX_PAYLOAD   56U
#define PROTO_MAX_FRAME     (PROTO_HEADER_LEN + PROTO_MAX_PAYLOAD + PROTO_CRC_LEN)

// ---- Host to MCU ----
#define MSG_CMD_VELOCITY    0x01U   // differential drive command
#define MSG_CMD_WHEEL_RAW   0x02U   // direct duty, bypasses the PID
#define MSG_CMD_ESTOP       0x03U   // immediate coast, latches a fault
#define MSG_CMD_RESET_ODOM  0x04U   // zero the integrated pose
#define MSG_CMD_CLEAR_FAULT 0x05U   // release a latched motor fault
#define MSG_CMD_PING        0x06U   // liveness probe, answered with PONG
#define MSG_CMD_DIAG        0x07U   // request a diagnostics frame
#define MSG_CMD_I2C_SCAN    0x08U   // probe every address on the bus

// ---- MCU to host ----
#define MSG_TELEMETRY       0x81U   // periodic state dump
#define MSG_PONG            0x86U   // reply to PING
#define MSG_BOOT_INFO       0x87U   // sent once after enumeration
#define MSG_DIAG            0x88U   // bring-up diagnostics, on request
#define MSG_I2C_SCAN        0x89U   // scan result

// ---- Status flags in telemetry ----
#define STATUS_MOTOR_ENABLED    (1U << 0)
#define STATUS_OPENLOOP         (1U << 1)   // encoder feedback unavailable
#define STATUS_FAULT_STALL      (1U << 2)
#define STATUS_CMD_TIMEOUT      (1U << 3)   // host went quiet
#define STATUS_ESTOP_LATCHED    (1U << 4)
#define STATUS_IMU_OK           (1U << 5)
#define STATUS_BATT_WARN        (1U << 6)
#define STATUS_BATT_CRITICAL    (1U << 7)
// Set once the BNO055 reports its fusion subsystem fully calibrated.
// Orientation before that point is usable for relative motion but the
// absolute heading may drift, so the SBC should weight it accordingly.
#define STATUS_IMU_CALIBRATED   (1U << 8)

// ============================================================
// Payload layouts
//
// Scaling is fixed and must match the bridge node exactly.
//   linear velocity   mm/s        int16   +-32.7 m/s
//   angular velocity  mrad/s      int16   +-32.7 rad/s
//   duty              permille    int16   -1000..1000
//   position          mm          int32   +-2147 km
//   heading           mrad        int32
//   quaternion        1/16384     int16   BNO055 native scaling
//   gyro              1/16 deg/s  int16   BNO055 native scaling
//   accel             1/100 m/s2  int16   BNO055 native scaling
//   distance          mm          uint16
//   battery           mV          uint16
//   timestamp         ms          uint32  wraps every 49.7 days
// ============================================================

// MSG_CMD_VELOCITY, 4 bytes
typedef struct __attribute__((packed)) {
    int16_t linear_mmps;        // forward positive
    int16_t angular_mradps;     // counterclockwise positive
} cmd_velocity_t;

// MSG_CMD_WHEEL_RAW, 4 bytes
// Bypasses the velocity loop entirely. Intended for bring-up: it tells
// a motor wiring problem apart from a control tuning problem.
typedef struct __attribute__((packed)) {
    int16_t left_permille;
    int16_t right_permille;
} cmd_wheel_raw_t;

// MSG_TELEMETRY, 44 bytes
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;      // MCU uptime, stamped at sample time
    int32_t  enc_left;          // accumulated counts since boot
    int32_t  enc_right;
    int16_t  speed_left_mmps;   // measured wheel surface speed
    int16_t  speed_right_mmps;
    int32_t  odom_x_mm;         // integrated pose, for slip detection
    int32_t  odom_y_mm;
    int32_t  odom_theta_mrad;
    int16_t  quat_w;            // BNO055 fusion output, 0 until wired
    int16_t  quat_x;
    int16_t  quat_y;
    int16_t  quat_z;
    int16_t  gyro_z;            // yaw rate, the term slip detection needs
    uint16_t distance_mm;       // 2D120X, 0xFFFF when out of range
    uint16_t battery_mv;        // reserved, reads 0 until the ADC lands
    int16_t  duty_left;         // applied duty, for stall diagnosis
    int16_t  duty_right;
    uint16_t status;            // STATUS_* bitfield
} telemetry_t;

// MSG_DIAG, 16 bytes
// Everything needed to tell an absent sensor from a stuck bus from a
// wrong address, without attaching a debugger.
typedef struct __attribute__((packed)) {
    uint8_t  imu_init_step;     // where the init state machine stopped
    uint8_t  imu_chip_id;       // 0xA0 when a BNO055 answered
    uint8_t  imu_calib;         // packed sys/gyr/acc/mag
    uint8_t  reserved;
    uint32_t imu_read_ok;
    uint32_t imu_read_fail;
    uint16_t i2c_errors;
    uint16_t i2c_recoveries;
} diag_t;

// MSG_I2C_SCAN, 17 bytes
// A bitmap rather than a list, so the payload size is fixed regardless
// of how many devices answer.
typedef struct __attribute__((packed)) {
    uint8_t count;              // devices that acknowledged
    uint8_t lines;              // bit0 SCL idle high, bit1 SDA idle high
    uint8_t bitmap[16];         // one bit per 7 bit address, LSB first
} i2c_scan_t;

// MSG_BOOT_INFO, 8 bytes
// Lets the bridge verify it is talking to a firmware it understands
// before it starts issuing commands.
typedef struct __attribute__((packed)) {
    uint8_t  proto_version;
    uint8_t  fw_major;
    uint8_t  fw_minor;
    uint8_t  fw_patch;
    uint16_t counts_per_wheel_rev;  // so the host never guesses odometry scale
    uint16_t wheel_base_mm;
} boot_info_t;

// CRC16-CCITT, polynomial 0x1021, initial value 0xFFFF.
// Shared by both ends; the host implementation must match bit for bit.
uint16_t proto_crc16(const uint8_t *data, uint16_t len);

#endif
