/*
 * File   : test_adc.c
 * Purpose: Host tests for the range finder curve and the battery state
 *          machine. Both are pure arithmetic and both are safety
 *          relevant, so neither should be verified only on hardware.
 * Author : jihoonkimtech
 *
 * The logic is transcribed from adc.c because that file touches ADC
 * registers directly. Any change there must be mirrored here.
 *
 * Note: the non-monotonic region of the GP2D120X is the reason this
 * file exists. A reading of 2.0V is either 5cm or about 2cm, and
 * resolving it as 5cm would let the robot drive into an obstacle while
 * reporting clear space.
 */

#include <stdio.h>
#include <stdint.h>
#include "board_config.h"

#define DIST_INVALID    0xFFFFU
#define DIST_TOO_CLOSE  0xFFFEU
#define DIST_AMBIGUOUS_MV   2350U
#define DIST_FARTHEST_MV    380U

typedef struct { uint16_t mv, mm; } pt_t;
static const pt_t curve[] = {
    {2300,40},{2050,50},{1750,60},{1550,70},{1400,80},
    {1150,100},{970,120},{800,150},{620,200},{500,250},{420,300},
};
#define N (sizeof curve / sizeof curve[0])

static int fails;
static void check(const char *n, int c)
{
    printf("%-52s %s\n", n, c ? "PASS" : "FAIL");
    if (!c) fails++;
}

static uint16_t mv_to_mm(uint16_t mv)
{
    if (mv >= curve[0].mv) return curve[0].mm;
    if (mv <= curve[N-1].mv) return curve[N-1].mm;
    for (unsigned i = 1; i < N; i++) {
        if (mv >= curve[i].mv) {
            uint16_t vh = curve[i-1].mv, vl = curve[i].mv;
            uint16_t dh = curve[i-1].mm, dl = curve[i].mm;
            return (uint16_t)(dh + ((dl - dh) * (uint32_t)(vh - mv)) / (vh - vl));
        }
    }
    return DIST_INVALID;
}

// Mirrors dist_sample() minus the hardware
static uint8_t g_latch;
static uint16_t dist_update(uint16_t mv)
{
    if (mv >= DIST_AMBIGUOUS_MV) { g_latch = 1; return DIST_TOO_CLOSE; }
    if (g_latch) {
        if (mv < DIST_RELEASE_MV) g_latch = 0;
        else return DIST_TOO_CLOSE;
    }
    return (mv <= DIST_FARTHEST_MV) ? DIST_INVALID : mv_to_mm(mv);
}

int main(void)
{
    printf("\n=== Scout2Map ADC logic ===\n\n");

    // --- curve is monotonic in the region it covers ---
    {
        int mono = 1;
        uint16_t prev = 0;
        for (uint16_t mv = 420; mv <= 2300; mv += 10) {
            uint16_t d = mv_to_mm(mv);
            if (prev && d > prev) mono = 0;   // higher voltage, nearer
            prev = d;
        }
        check("distance falls monotonically as voltage rises", mono);
    }

    // --- table points reproduce themselves ---
    {
        int ok = 1;
        for (unsigned i = 0; i < N; i++) {
            uint16_t d = mv_to_mm(curve[i].mv);
            if (d != curve[i].mm) { ok = 0;
                printf("   %u mV gave %u mm, table says %u\n",
                       curve[i].mv, d, curve[i].mm); }
        }
        check("every table point maps back to itself", ok);
    }

    // --- interpolation lands between neighbours ---
    {
        uint16_t d = mv_to_mm(1900);      // between 2050 (50mm) and 1750 (60mm)
        printf("   1900 mV -> %u mm, expected between 50 and 60\n", d);
        check("interpolation stays between its neighbours", d > 50 && d < 60);
    }

    // --- the non-monotonic region is refused, not guessed ---
    g_latch = 0;
    check("a reading past the curve peak reports too close",
          dist_update(2500) == DIST_TOO_CLOSE);

    // --- and stays latched while the reading is still ambiguous ---
    check("the alarm holds while the reading remains near the peak",
          dist_update(2200) == DIST_TOO_CLOSE);

    // --- released only once clearly outside the region ---
    {
        uint16_t d = dist_update(1500);
        printf("   after backing off to 1500 mV -> %u mm\n", d);
        check("alarm clears once the target is unambiguously far",
              d != DIST_TOO_CLOSE && d > 60 && d < 90);
    }

    // --- out of range reads as invalid, not as maximum distance ---
    check("a target beyond range reports invalid",
          dist_update(300) == DIST_INVALID);

    // --- battery thresholds are ordered and physical ---
    printf("\n   warn %u mV, critical %u mV, hysteresis %u mV\n",
           BATT_WARN_MV, BATT_CRITICAL_MV, BATT_HYST_MV);
    check("critical sits below warn", BATT_CRITICAL_MV < BATT_WARN_MV);
    check("hysteresis cannot span the two thresholds",
          BATT_WARN_MV - BATT_CRITICAL_MV > BATT_HYST_MV);
    check("critical stays above the cell damage point",
          BATT_CRITICAL_MV >= 9000U);
    check("warn sits below a nominal pack", BATT_WARN_MV < 11100U);

    // --- divider cannot overrange the ADC ---
    {
        uint32_t full_mv = 12900U * BATT_DIVIDER_RATIO / 1000U;
        printf("   12.9 V charged pack presents %u mV at the pin\n",
               (unsigned)full_mv);
        check("a fully charged pack stays inside the 3.3 V reference",
              full_mv < 3300U);
    }

    printf("\n%s (%d failure%s)\n\n",
           fails ? "SOME TESTS FAILED" : "ALL TESTS PASSED",
           fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
