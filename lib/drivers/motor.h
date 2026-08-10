/*
 * File   : motor.h
 * Purpose: BTS7960 motor driver interface. Duty is signed permille.
 * Author : jihoonkimtech
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT,
    MOTOR_COUNT
} motor_id_t;

#define MOTOR_FAULT_STALL   (1U << 0)

void    motor_init(void);
void    motor_enable(void);
void    motor_estop(void);
void    motor_set(motor_id_t id, int16_t duty_permille);
void    motor_update(void);
uint8_t motor_get_fault(void);
void    motor_clear_fault(void);

#endif
