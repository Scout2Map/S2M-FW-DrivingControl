/*
 * File   : drive.h
 * Purpose: Hardware independent drive control interface. All hardware
 *          access is injected through drive_io_t at init time.
 * Author : jihoonkimtech
 */

#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>

// ============================================================
// Hardware independent drive control
//
// This module never talks to a peripheral register. All access to
// motors, encoders, the IMU and the clock arrives through the
// drive_io_t function table supplied at init.
//
// That indirection lets the whole file compile and run on a host
// PC against stub implementations, so the kinematics and the PID
// can be unit tested without the board being present.
// ============================================================

typedef struct {
    // Apply duty in permille, idx 0 = left, 1 = right
    void     (*set_duty)(int idx, int16_t permille);
    // Push the applied duty out to the hardware, called once per loop
    void     (*commit)(void);
    // Latch fresh encoder samples, called once per loop before reads
    void     (*sample_encoders)(void);
    // Wheel surface speed in m/s from the last sample
    float    (*get_speed_mps)(int idx);
    // Encoder counts accumulated during the last sample
    int32_t  (*get_delta_counts)(int idx);
    // Fused yaw in radians, may return a constant when no IMU is present
    float    (*get_yaw)(void);
    // Milliseconds since boot
    uint32_t (*millis)(void);
} drive_io_t;

void    drive_init(const drive_io_t *io);
void    drive_command(float v_mps, float w_radps);
void    drive_update(void);
void    drive_stop(void);
void    drive_get_odom(float *x, float *y, float *th);
void    drive_reset_odom(void);
uint8_t drive_is_openloop(void);

#endif
