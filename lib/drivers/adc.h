/*
 * File   : adc.h
 * Purpose: IR range finder and battery voltage interface.
 * Author : jihoonkimtech
 */

#ifndef ADC_H
#define ADC_H

#include <stdint.h>

// Distance sentinels. Both are outside any real measurement so the
// consumer never has to carry a separate validity flag.
#define DIST_INVALID    0xFFFFU     // nothing within range
#define DIST_TOO_CLOSE  0xFFFEU     // inside the minimum range, distance unknown

// OK, WARN and CRITICAL are advisory: the SBC decides what to do.
// DEAD is not advisory; the firmware cuts drive to stop the pack being
// destroyed, because no mission policy can make that safe.
typedef enum {
    BATT_UNKNOWN = 0,
    BATT_OK,
    BATT_WARN,
    BATT_CRITICAL,
    BATT_DEAD
} batt_state_t;

void         adc_init(void);
void         adc_poll(void);        // call at ADC_PERIOD_MS

uint16_t     dist_get_mm(void);
uint8_t      dist_is_too_close(void);

uint16_t     batt_get_mv(void);     // pack voltage, filtered
batt_state_t batt_get_state(void);

#endif
