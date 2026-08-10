/*
 * File   : encoder.h
 * Purpose: Quadrature encoder interface for both drive wheels.
 * Author : jihoonkimtech
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef enum {
    ENC_LEFT = 0,
    ENC_RIGHT,
    ENC_COUNT
} enc_id_t;

void     encoder_init(void);
void     encoder_update(void);
int32_t  encoder_get_total(enc_id_t id);
int32_t  encoder_get_delta(enc_id_t id);
float    encoder_get_speed_mps(enc_id_t id);
uint16_t encoder_get_raw(enc_id_t id);
void     encoder_reset(void);

#endif
