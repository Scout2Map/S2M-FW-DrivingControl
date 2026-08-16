/*
 * File   : adc.h
 * Purpose: IR range finder and battery voltage interface.
 * Author : jihoonkimtech
 */

#ifndef ADC_H
#define ADC_H

#include <stdint.h>
#include "dist.h"    // distance interface, shared with the ToF driver

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


uint16_t     batt_get_mv(void);     // pack voltage, filtered
// Raw ADC counts behind the last battery sample. Only used to
// recalibrate BATT_UV_PER_COUNT against a meter.
uint16_t     batt_get_counts(void);
batt_state_t batt_get_state(void);

#endif
