/*
 * File   : dist.h
 * Purpose: Distance sensor interface, shared by whichever driver is
 *          selected in board_config.h.
 * Author : jihoonkimtech
 *
 * Exactly one driver defines these at a time: adc.c for the analog
 * Sharp part, vl53l0x.c for the time of flight part. Callers include
 * this header and never learn which one is present.
 */

#ifndef DIST_H
#define DIST_H

#include <stdint.h>

// Sentinels sit outside any real measurement, so a caller never needs a
// separate validity flag alongside the distance.
#define DIST_INVALID    0xFFFFU     // nothing within range
#define DIST_TOO_CLOSE  0xFFFEU     // inside the minimum, distance unknown

uint16_t dist_get_mm(void);
uint8_t  dist_is_too_close(void);

// Raw channel, for bring-up. Analog builds report ADC counts and
// millivolts; the time of flight build reports millimetres and zero.
uint16_t dist_get_counts(void);
uint16_t dist_get_mv(void);

#endif
