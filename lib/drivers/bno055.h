/*
 * File   : bno055.h
 * Purpose: BNO055 fused orientation interface.
 * Author : jihoonkimtech
 */

#ifndef BNO055_H
#define BNO055_H

#include <stdint.h>

void     bno055_init(void);
// Starts a read cycle. Call at IMU_PERIOD_MS.
void     bno055_poll(void);
// Advances the transfer in flight. Call every main loop pass; a single
// I2C phase must complete well inside I2C_TIMEOUT_MS.
void     bno055_pump(void);
uint8_t  bno055_is_ok(void);

// Raw fusion output at the device native scaling, forwarded to the SBC
// without conversion so no precision is lost on the way
int16_t  bno055_quat_w(void);    // 1/16384
int16_t  bno055_quat_x(void);
int16_t  bno055_quat_y(void);
int16_t  bno055_quat_z(void);
int16_t  bno055_gyro_z(void);    // 1/16 deg/s, yaw rate

// 1/100 m/s2, gravity included. Used for bump and vibration events and
// for identifying how the module is mounted.
int16_t  bno055_accel_x(void);
int16_t  bno055_accel_y(void);
int16_t  bno055_accel_z(void);

// Packed calibration status: bits 7:6 sys, 5:4 gyr, 3:2 acc, 1:0 mag
// Each field reads 3 when that subsystem is fully calibrated
uint8_t  bno055_calib(void);

float    bno055_yaw_rad(void);

uint32_t bno055_read_ok(void);
uint32_t bno055_read_fail(void);

// Diagnostics for bring-up. init_step follows the enum in bno055.c;
// last_id holds whatever the chip ID register returned, 0xFF if never read.
uint8_t  bno055_init_step(void);
uint8_t  bno055_last_id(void);

#endif
