/*
 * File   : encoder.c
 * Purpose: Quadrature decoding for the JGB37-520 hall encoders using
 *          the STM32 timer encoder mode, so counting costs no CPU time
 *          and needs no interrupts.
 * Author : jihoonkimtech
 *
 * Pin map
 *   TIM1 : PA8(CH1)=yellow A, PA9(CH2)=green B  -> left wheel
 *   TIM4 : PB6(CH1)=yellow A, PB7(CH2)=green B  -> right wheel
 *   Encoder supply is blue(+) / black(-) at 3.3V, motor leads are
 *   red(+) / white(-) and go to the BTS7960, never to the MCU.
 *
 * The 16 bit counters wrap silently. Taking the difference as int16_t
 * makes the wrap arithmetic resolve itself, provided the poll rate
 * stays above half a counter period. At 0.26 m/s on a 5ms loop that is
 * roughly 40 counts per sample against a 32768 margin.
 *
 * Note: the input filter is set to maximum. Brush noise coupled onto
 * the encoder leads registers as phantom counts at weaker settings,
 * which shows up as odometry drift that is hard to trace later.
 */

#include "stm32f1xx.h"
#include "board_config.h"
#include "encoder.h"

// ============================================================
// Quadrature decoding on TIM1 (left) and TIM4 (right)
// The timer counts in hardware, so the CPU cost is one register
// read per loop and no interrupts are required
//
// The 16 bit counters wrap silently. Taking the difference as
// int16_t makes the wrap arithmetic resolve itself, as long as
// we poll faster than half a counter period. At 0.26 m/s and a
// 5ms loop that is roughly 40 counts, far below 32768.
// ============================================================

typedef struct {
    TIM_TypeDef *tim;
    uint16_t     prev_raw;   // counter value at the previous loop
    int32_t      total;      // accumulated counts since boot
    int32_t      delta;      // counts during the last loop
    int8_t       invert;     // mirrored motor mounting
} enc_ch_t;

static enc_ch_t s_enc[ENC_COUNT];

static void gpio_input_pullup(GPIO_TypeDef *port, uint32_t pin)
{
    // CNF=10 input with pull, MODE=00 -> nibble 0x8
    // Hall outputs are frequently open drain, the pull up makes them readable
    volatile uint32_t *cr = (pin < 8U) ? &port->CRL : &port->CRH;
    uint32_t shift = (pin % 8U) * 4U;
    *cr &= ~(0xFU << shift);
    *cr |= (0x8U << shift);
    port->ODR |= (1U << pin);   // ODR picks pull up rather than pull down
}

static void timer_encoder_init(TIM_TypeDef *tim)
{
    tim->CR1 = 0;
    tim->PSC = 0;
    tim->ARR = 0xFFFFU;         // free running across the full 16 bit range

    // Encoder mode 3 counts both edges of both channels, giving x4
    tim->SMCR &= ~TIM_SMCR_SMS;
    tim->SMCR |= (3U << TIM_SMCR_SMS_Pos);

    // Map CC1 to TI1 and CC2 to TI2, both configured as inputs
    tim->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    tim->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;

    // Maximum input filter, motor leads carry a lot of brush noise
    // A weaker filter lets that noise register as phantom counts
    tim->CCMR1 &= ~(TIM_CCMR1_IC1F | TIM_CCMR1_IC2F);
    tim->CCMR1 |= (0xFU << TIM_CCMR1_IC1F_Pos) | (0xFU << TIM_CCMR1_IC2F_Pos);

    // Both inputs non inverted, direction comes from the phase relationship
    tim->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);

    tim->CNT = 0;
    tim->EGR = TIM_EGR_UG;
    tim->CR1 |= TIM_CR1_CEN;
}

void encoder_init(void)
{
    // TIM1 sits on APB2, TIM4 on APB1
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    gpio_input_pullup(GPIOA, ENC_L_A_PIN);
    gpio_input_pullup(GPIOA, ENC_L_B_PIN);
    gpio_input_pullup(GPIOB, ENC_R_A_PIN);
    gpio_input_pullup(GPIOB, ENC_R_B_PIN);

    timer_encoder_init(TIM1);
    timer_encoder_init(TIM4);

    s_enc[ENC_LEFT].tim     = TIM1;
    s_enc[ENC_LEFT].invert  = ENC_L_INVERT ? -1 : 1;
    s_enc[ENC_RIGHT].tim    = TIM4;
    s_enc[ENC_RIGHT].invert = ENC_R_INVERT ? -1 : 1;

    for (int i = 0; i < ENC_COUNT; i++) {
        s_enc[i].prev_raw = (uint16_t)s_enc[i].tim->CNT;
        s_enc[i].total    = 0;
        s_enc[i].delta    = 0;
    }
}

// Must be called once per control loop at a fixed period
void encoder_update(void)
{
    for (int i = 0; i < ENC_COUNT; i++) {
        uint16_t raw = (uint16_t)s_enc[i].tim->CNT;
        // The int16_t cast resolves counter wrap in both directions
        int16_t  d   = (int16_t)(raw - s_enc[i].prev_raw);
        s_enc[i].prev_raw = raw;
        s_enc[i].delta    = (int32_t)d * s_enc[i].invert;
        s_enc[i].total   += s_enc[i].delta;
    }
}

int32_t encoder_get_total(enc_id_t id)
{
    return (id < ENC_COUNT) ? s_enc[id].total : 0;
}

int32_t encoder_get_delta(enc_id_t id)
{
    return (id < ENC_COUNT) ? s_enc[id].delta : 0;
}

// Wheel surface speed in m/s from the counts of the last loop
float encoder_get_speed_mps(enc_id_t id)
{
    if (id >= ENC_COUNT) {
        return 0.0f;
    }
    float mm = (float)s_enc[id].delta * MM_PER_COUNT;
    return (mm * 0.001f) / LOOP_PERIOD_S;
}

// Raw counter, used for bench calibration of COUNTS_PER_WHEEL_REV
uint16_t encoder_get_raw(enc_id_t id)
{
    return (id < ENC_COUNT) ? (uint16_t)s_enc[id].tim->CNT : 0;
}

void encoder_reset(void)
{
    for (int i = 0; i < ENC_COUNT; i++) {
        s_enc[i].tim->CNT = 0;
        s_enc[i].prev_raw = 0;
        s_enc[i].total    = 0;
        s_enc[i].delta    = 0;
    }
}
