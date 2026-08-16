/*
 * File   : vl53l0x.h
 * Purpose: VL53L0X time of flight range finder interface.
 * Author : jihoonkimtech
 */

#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdint.h>
#include "dist.h"

void     vl53l0x_init(void);
void     vl53l0x_poll(void);     // start a cycle, call at DIST_PERIOD_MS
void     vl53l0x_pump(void);     // advance a transfer, call every loop
uint8_t  vl53l0x_is_ok(void);

// Diagnostics. model_id reads 0xEE on a healthy part; state follows the
// init enum in vl53l0x.c.
uint8_t  vl53l0x_model_id(void);
uint8_t  vl53l0x_state(void);
uint16_t vl53l0x_raw_mm(void);
uint32_t vl53l0x_read_ok(void);
uint32_t vl53l0x_read_fail(void);

#endif
