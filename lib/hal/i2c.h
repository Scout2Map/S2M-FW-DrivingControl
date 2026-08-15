/*
 * File   : i2c.h
 * Purpose: Non-blocking I2C2 master interface for the BNO055.
 * Author : jihoonkimtech
 */

#ifndef I2C_H
#define I2C_H

#include <stdint.h>

typedef enum {
    I2C_RESULT_IDLE = 0,
    I2C_RESULT_BUSY,
    I2C_RESULT_DONE,
    I2C_RESULT_ERROR
} i2c_result_t;

void         i2c_init(void);
uint8_t      i2c_is_busy(void);
uint8_t      i2c_start_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
uint8_t      i2c_start_write(uint8_t addr, uint8_t reg, const uint8_t *buf, uint8_t len);

// Advances the transfer. Call every loop while a transfer is pending.
i2c_result_t i2c_poll(void);

// Manually clocks the bus to free a slave holding SDA low.
uint8_t      i2c_bus_recover(void);

// Bring-up helpers. Blocking, and not for use on the control path.
uint8_t      i2c_probe(uint8_t addr);
uint8_t      i2c_scan(uint8_t *bitmap16);   // 16 bytes, one bit per address

uint32_t     i2c_error_count(void);
uint32_t     i2c_recovery_count(void);

#endif
