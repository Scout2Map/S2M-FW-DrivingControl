/*
 * File   : dist_none.c
 * Purpose: Null distance driver, used when no range finder is fitted.
 * Author : jihoonkimtech
 *
 * Reporting DIST_INVALID is the honest answer for a sensor that is not
 * there, and it is the same answer the real drivers give for a target
 * out of range. Consumers therefore need no special case, and a robot
 * built without the sensor behaves like one whose sensor sees nothing.
 */

#include "board_config.h"

#if DIST_SENSOR == DIST_SENSOR_NONE

#include "dist.h"

uint16_t dist_get_mm(void)       { return DIST_INVALID; }
uint8_t  dist_is_too_close(void) { return 0; }
uint16_t dist_get_counts(void)   { return 0; }
uint16_t dist_get_mv(void)       { return 0; }

#endif
