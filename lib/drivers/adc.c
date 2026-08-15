/*
 * File   : adc.c
 * Purpose: ADC1 sampling for the GP2D120X infrared range finder and the
 *          battery voltage divider.
 * Author : jihoonkimtech
 *
 * Pin map
 *   PA4  ADC12_IN4  GP2D120X analog output, sensor runs from 5V
 *   PA5  ADC12_IN5  battery divider, 30k over 7.5k, ratio 0.200
 *
 * The GP2D120X puts out at most about 2.5V, and the divider caps at
 * 2.6V on a fully charged 3S pack, so neither input can exceed the
 * 3.3V reference.
 *
 * Note: a conversion at 12MHz with 239.5 cycles of sampling takes about
 * 21us. That is 0.4 percent of one control loop, so this driver blocks
 * for the conversion rather than carrying a state machine. The long
 * sampling window is what lets the 6k source impedance of the divider
 * settle; shortening it to save the 21us would corrupt the reading.
 *
 * Note: the GP2D120X response is NOT monotonic. Below roughly 4cm the
 * output voltage falls again, so a single reading of 2.0V could mean
 * 5cm or 2cm. Treating that ambiguity as a valid distance would let the
 * robot drive into an obstacle while reporting clear space, which is
 * why dist_get_mm() reports an explicit too-close code instead of
 * guessing, and latches it until the reading is unambiguously clear.
 */

#include "stm32f1xx.h"
#include "board_config.h"
#include "adc.h"

#define ADC_MAX             4095U

// GP2D120X transfer curve, from the datasheet. Voltage falls as the
// target recedes, so the table is ordered by descending voltage.
typedef struct {
    uint16_t mv;
    uint16_t mm;
} dist_point_t;

static const dist_point_t s_curve[] = {
    { 2300,  40 },
    { 2050,  50 },
    { 1750,  60 },
    { 1550,  70 },
    { 1400,  80 },
    { 1150, 100 },
    {  970, 120 },
    {  800, 150 },
    {  620, 200 },
    {  500, 250 },
    {  420, 300 },
};
#define CURVE_LEN  (sizeof s_curve / sizeof s_curve[0])

// Above this the reading is on the ambiguous side of the curve peak
#define DIST_AMBIGUOUS_MV   2350U
// Below this the target is beyond the sensor range
#define DIST_FARTHEST_MV    380U

static uint16_t s_dist_raw[DIST_MEDIAN_N];
static uint8_t  s_dist_idx;
static uint16_t s_dist_mm;
static uint8_t  s_dist_too_close;

static uint32_t s_batt_filt;        // IIR accumulator, scaled by 16
static uint16_t s_batt_mv;
static batt_state_t s_batt_state;
static uint8_t  s_batt_primed;

static void gpio_analog(uint32_t pin)
{
    // Analog input: CNF=00 MODE=00, the whole nibble is zero
    volatile uint32_t *cr = (pin < 8U) ? &GPIOA->CRL : &GPIOA->CRH;
    *cr &= ~(0xFU << ((pin % 8U) * 4U));
}

void adc_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN;

    gpio_analog(DIST_ADC_PIN);
    gpio_analog(BATT_ADC_PIN);

    // Longest sampling window on both channels. The battery divider
    // presents 6k of source impedance and needs the time to settle.
    ADC1->SMPR2 = (7U << (DIST_ADC_CHANNEL * 3U))
                | (7U << (BATT_ADC_CHANNEL * 3U));

    ADC1->SQR1 = 0;                     // one conversion per sequence
    ADC1->CR1  = 0;
    ADC1->CR2  = ADC_CR2_ADON;          // first write only wakes it up

    // The reference manual requires a settling delay before calibration
    for (volatile uint32_t d = 0; d < 10000U; d++) { }

    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while (ADC1->CR2 & ADC_CR2_RSTCAL) { }
    ADC1->CR2 |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL) { }

    for (uint8_t i = 0; i < DIST_MEDIAN_N; i++) {
        s_dist_raw[i] = 0;
    }
    s_dist_idx       = 0;
    s_dist_mm        = DIST_INVALID;
    s_dist_too_close = 0;
    s_batt_filt      = 0;
    s_batt_mv        = 0;
    s_batt_state     = BATT_UNKNOWN;
    s_batt_primed    = 0;
}

static uint16_t adc_convert(uint8_t channel)
{
    ADC1->SQR3 = channel;
    ADC1->CR2 |= ADC_CR2_ADON;          // second write starts a conversion

    uint32_t guard = 0;
    while (!(ADC1->SR & ADC_SR_EOC)) {
        // A conversion cannot legitimately take this long; bail out
        // rather than stall the control loop on a peripheral fault
        if (++guard > 100000U) {
            return 0xFFFFU;
        }
    }
    return (uint16_t)(ADC1->DR & ADC_MAX);
}

static uint16_t counts_to_mv(uint16_t counts)
{
    return (uint16_t)(((uint32_t)counts * 3300U) / ADC_MAX);
}

// Median of the sample ring. Cheap insertion sort on a copy; the ring
// is five entries, so the cost is irrelevant next to the conversion.
static uint16_t median_of(const uint16_t *src, uint8_t n)
{
    uint16_t tmp[DIST_MEDIAN_N];
    for (uint8_t i = 0; i < n; i++) {
        tmp[i] = src[i];
    }
    for (uint8_t i = 1; i < n; i++) {
        uint16_t v = tmp[i];
        int8_t j = (int8_t)(i - 1);
        while (j >= 0 && tmp[j] > v) {
            tmp[j + 1] = tmp[j];
            j--;
        }
        tmp[j + 1] = v;
    }
    return tmp[n / 2];
}

// Linear interpolation across the datasheet curve.
static uint16_t mv_to_mm(uint16_t mv)
{
    if (mv >= s_curve[0].mv) {
        return s_curve[0].mm;
    }
    if (mv <= s_curve[CURVE_LEN - 1].mv) {
        return s_curve[CURVE_LEN - 1].mm;
    }
    for (uint8_t i = 1; i < CURVE_LEN; i++) {
        if (mv >= s_curve[i].mv) {
            uint16_t v_hi = s_curve[i - 1].mv, v_lo = s_curve[i].mv;
            uint16_t d_hi = s_curve[i - 1].mm, d_lo = s_curve[i].mm;
            uint32_t span = (uint32_t)(v_hi - v_lo);
            uint32_t frac = (uint32_t)(v_hi - mv);
            return (uint16_t)(d_hi + ((d_lo - d_hi) * frac) / span);
        }
    }
    return DIST_INVALID;
}

static void dist_sample(void)
{
    uint16_t counts = adc_convert(DIST_ADC_CHANNEL);
    if (counts == 0xFFFFU) {
        return;
    }

    s_dist_raw[s_dist_idx] = counts;
    s_dist_idx = (uint8_t)((s_dist_idx + 1U) % DIST_MEDIAN_N);

    uint16_t mv = counts_to_mv(median_of(s_dist_raw, DIST_MEDIAN_N));

    if (mv >= DIST_AMBIGUOUS_MV) {
        // Past the curve peak. The target is at or inside the minimum
        // range and the reading no longer identifies a distance.
        s_dist_too_close = 1;
        s_dist_mm = DIST_TOO_CLOSE;
        return;
    }

    if (s_dist_too_close) {
        // Latched. Only release once the target is clearly outside the
        // ambiguous region, otherwise the alarm would chatter as the
        // reading crosses back and forth over the peak.
        if (mv < DIST_RELEASE_MV) {
            s_dist_too_close = 0;
        } else {
            s_dist_mm = DIST_TOO_CLOSE;
            return;
        }
    }

    s_dist_mm = (mv <= DIST_FARTHEST_MV) ? DIST_INVALID : mv_to_mm(mv);
}

static void batt_sample(void)
{
    uint16_t counts = adc_convert(BATT_ADC_CHANNEL);
    if (counts == 0xFFFFU) {
        return;
    }

    // Divider ratio 7.5 / (30 + 7.5) = 0.2, so the pack voltage is five
    // times the pin voltage
    uint32_t mv = ((uint32_t)counts_to_mv(counts) * 1000U) / BATT_DIVIDER_RATIO;

    if (!s_batt_primed) {
        // Seed the filter with the first reading, otherwise it ramps up
        // from zero and trips the critical threshold on every boot
        s_batt_filt   = mv << 4;
        s_batt_primed = 1;
    } else {
        // First order IIR, coefficient 1/16. Motor current draw makes
        // the rail sag transiently and an unfiltered reading would
        // declare a critical fault every time the robot accelerates.
        s_batt_filt += mv - (s_batt_filt >> 4);
    }
    s_batt_mv = (uint16_t)(s_batt_filt >> 4);

    // Hysteresis so a pack sitting near a threshold does not oscillate
    switch (s_batt_state) {
    case BATT_OK:
        if (s_batt_mv < BATT_WARN_MV)      s_batt_state = BATT_WARN;
        break;
    case BATT_WARN:
        if (s_batt_mv < BATT_CRITICAL_MV)  s_batt_state = BATT_CRITICAL;
        else if (s_batt_mv > BATT_WARN_MV + BATT_HYST_MV)
                                           s_batt_state = BATT_OK;
        break;
    case BATT_CRITICAL:
        if (s_batt_mv < BATT_DEAD_MV)      s_batt_state = BATT_DEAD;
        else if (s_batt_mv > BATT_CRITICAL_MV + BATT_HYST_MV)
                                           s_batt_state = BATT_WARN;
        break;
    case BATT_DEAD:
        // Recovers only on a genuine reconnect or a fresh pack, well
        // clear of the damage point. A sagging rail must not be able to
        // flap in and out of the protection state.
        if (s_batt_mv > BATT_DEAD_MV + BATT_HYST_MV * 2U)
                                           s_batt_state = BATT_CRITICAL;
        break;
    default:
        // First classification after priming
        if (s_batt_mv < BATT_DEAD_MV)          s_batt_state = BATT_DEAD;
        else if (s_batt_mv < BATT_CRITICAL_MV) s_batt_state = BATT_CRITICAL;
        else if (s_batt_mv < BATT_WARN_MV)     s_batt_state = BATT_WARN;
        else                                   s_batt_state = BATT_OK;
        break;
    }
}

// Called at ADC_PERIOD_MS.
void adc_poll(void)
{
    static uint8_t divider;

    dist_sample();

    // The pack cannot change meaningfully in 40ms, so sampling it every
    // cycle would only add conversions and filter lag
    if (++divider >= BATT_SAMPLE_DIVIDER) {
        divider = 0;
        batt_sample();
    }
}

uint16_t     dist_get_mm(void)    { return s_dist_mm; }
uint8_t      dist_is_too_close(void) { return s_dist_too_close; }
uint16_t     batt_get_mv(void)    { return s_batt_mv; }
batt_state_t batt_get_state(void) { return s_batt_state; }
