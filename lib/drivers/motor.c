/*
 * File   : motor.c
 * Purpose: BTS7960 half bridge pair driver for skid steer propulsion.
 *          Owns PWM generation, shoot-through prevention, slew rate
 *          limiting, direction change deadtime and the stall guard.
 * Author : jihoonkimtech
 *
 * Pin map
 *   TIM2 : PA0(CH1), PA1(CH2) -> left  BTS7960, enable on PB0
 *   TIM3 : PA6(CH1), PA7(CH2) -> right BTS7960, enable on PB1
 *
 * Duty is carried as a signed permille integer throughout. No float
 * appears in this file, which keeps the write path cheap and exact.
 *
 * Note: driving RPWM and LPWM high at once shorts the half bridge
 * through the load. pwm_write() is the single choke point that makes
 * that impossible, so never write CCR1/CCR2 from anywhere else.
 *
 * Note: the enable pins are the emergency stop path. Pulling them low
 * coasts the bridges immediately, which a PWM value of zero does not.
 */

#include "stm32f1xx.h"
#include "board_config.h"
#include "motor.h"
#include "systick.h"

// ============================================================
// BTS7960 pair driver
// Each board exposes RPWM and LPWM plus tied enable pins
// Only one of RPWM / LPWM may be non zero at any instant
// Driving both high shorts the half bridge through the load
// ============================================================

typedef struct {
    TIM_TypeDef *tim;       // timer owning both PWM channels
    int8_t       invert;    // mirrored mounting, see MOTx_INVERT
    int16_t      target;    // requested duty, permille, signed
    int16_t      applied;   // duty actually written, after slew limiting
    int8_t       last_sign; // previous direction, for deadtime detection
    uint32_t     dead_until;// timestamp until which output stays at zero
    uint32_t     stall_ms;  // accumulated time at high duty
} motor_ch_t;

static motor_ch_t s_mot[MOTOR_COUNT];
static uint8_t    s_enabled;
static uint8_t    s_fault;

static void pwm_write(TIM_TypeDef *tim, uint16_t ccr_fwd, uint16_t ccr_rev)
{
    // CH2 is treated as forward, CH1 as reverse
    // Swap these two lines if the robot drives backwards on a positive command
    tim->CCR2 = ccr_fwd;
    tim->CCR1 = ccr_rev;
}

static void gpio_af_pushpull(GPIO_TypeDef *port, uint32_t pin)
{
    // 50MHz alternate function push pull, CNF=10 MODE=11 -> 0xB
    volatile uint32_t *cr = (pin < 8U) ? &port->CRL : &port->CRH;
    uint32_t shift = (pin % 8U) * 4U;
    *cr &= ~(0xFU << shift);
    *cr |= (0xBU << shift);
}

static void gpio_output_pushpull(GPIO_TypeDef *port, uint32_t pin)
{
    // 2MHz general purpose push pull, CNF=00 MODE=10 -> 0x2
    volatile uint32_t *cr = (pin < 8U) ? &port->CRL : &port->CRH;
    uint32_t shift = (pin % 8U) * 4U;
    *cr &= ~(0xFU << shift);
    *cr |= (0x2U << shift);
}

static void timer_pwm_init(TIM_TypeDef *tim)
{
    tim->CR1  = 0;
    tim->PSC  = 0;                  // timer clock is already 72MHz
    tim->ARR  = PWM_ARR;            // 20kHz
    tim->CCR1 = 0;
    tim->CCR2 = 0;

    // PWM mode 1 with preload on both channels
    tim->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE
               | (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;

    tim->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    tim->CR1 |= TIM_CR1_ARPE;
    tim->EGR  = TIM_EGR_UG;         // load the shadow registers now
    tim->CR1 |= TIM_CR1_CEN;
}

void motor_init(void)
{
    // Enable pins first and hold them low so the bridges stay off
    gpio_output_pushpull(MOTL_EN_PORT, MOTL_EN_PIN);
    gpio_output_pushpull(MOTR_EN_PORT, MOTR_EN_PIN);
    MOTL_EN_PORT->BSRR = (1U << (MOTL_EN_PIN + 16));
    MOTR_EN_PORT->BSRR = (1U << (MOTR_EN_PIN + 16));

    // PWM pins to alternate function
    gpio_af_pushpull(GPIOA, MOTL_LPWM_PIN);
    gpio_af_pushpull(GPIOA, MOTL_RPWM_PIN);
    gpio_af_pushpull(GPIOA, MOTR_LPWM_PIN);
    gpio_af_pushpull(GPIOA, MOTR_RPWM_PIN);

    timer_pwm_init(TIM2);
    timer_pwm_init(TIM3);

    for (int i = 0; i < MOTOR_COUNT; i++) {
        s_mot[i].tim        = (i == MOTOR_LEFT) ? TIM2 : TIM3;
        s_mot[i].invert     = (i == MOTOR_LEFT)
                              ? (MOTL_INVERT ? -1 : 1)
                              : (MOTR_INVERT ? -1 : 1);
        s_mot[i].target     = 0;
        s_mot[i].applied    = 0;
        s_mot[i].last_sign  = 0;
        s_mot[i].dead_until = 0;
        s_mot[i].stall_ms   = 0;
    }

    s_enabled = 0;
    s_fault   = 0;
}

void motor_enable(void)
{
    if (s_fault) {
        return;   // a latched fault must be cleared explicitly
    }
    MOTL_EN_PORT->BSRR = (1U << MOTL_EN_PIN);
    MOTR_EN_PORT->BSRR = (1U << MOTR_EN_PIN);
    s_enabled = 1;
}

void motor_estop(void)
{
    // Pull enable low first, this coasts the bridges immediately
    MOTL_EN_PORT->BSRR = (1U << (MOTL_EN_PIN + 16));
    MOTR_EN_PORT->BSRR = (1U << (MOTR_EN_PIN + 16));
    s_enabled = 0;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        s_mot[i].target    = 0;
        s_mot[i].applied   = 0;
        s_mot[i].stall_ms  = 0;
        // Clearing the direction memory means the first command after a
        // stop never has to pay for a reversal it did not request
        s_mot[i].last_sign = 0;
        s_mot[i].dead_until = 0;
        pwm_write(s_mot[i].tim, 0, 0);
    }
}

// Applies the mounting inversion here, at the single entry point, so
// the slew limiter, the direction deadtime and the stall guard all
// operate on the sign the hardware will really see. Inverting further
// down would let those three disagree with each other.
void motor_set(motor_id_t id, int16_t duty_permille)
{
    if (id >= MOTOR_COUNT) {
        return;
    }
    if (duty_permille >  DUTY_MAX) duty_permille =  DUTY_MAX;
    if (duty_permille < -DUTY_MAX) duty_permille = -DUTY_MAX;
    s_mot[id].target = (int16_t)(duty_permille * s_mot[id].invert);
}

uint8_t motor_is_enabled(void)
{
    return s_enabled;
}

uint8_t motor_get_fault(void)
{
    return s_fault;
}

void motor_clear_fault(void)
{
    s_fault = 0;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        s_mot[i].stall_ms = 0;
    }
}

// Called every LOOP_PERIOD_MS from the main scheduler
void motor_update(void)
{
    uint32_t now = systick_millis();

    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_ch_t *m = &s_mot[i];

        // Deadtime is a state, not a per-loop test.
        //
        // An earlier version re-evaluated the reversal condition every
        // loop. Slew would step applied off zero toward the new sign,
        // that tripped the check, applied was forced back to zero and
        // last_sign never updated because sign had been zeroed. The
        // wheel then sat still forever on any direction change. Holding
        // an explicit window and committing the new direction when it
        // expires makes the transition happen exactly once.
        if (now < m->dead_until) {
            m->applied = 0;
            pwm_write(m->tim, 0, 0);
            continue;
        }

        int8_t want = (m->target > 0) ? 1 : ((m->target < 0) ? -1 : 0);

        // Entering a reversal: coast briefly before the bridge is asked
        // to conduct the other way
        if (want != 0 && m->last_sign != 0 && want != m->last_sign) {
            m->dead_until = now + DIR_CHANGE_DEADTIME_MS;
            m->applied    = 0;
            m->last_sign  = 0;      // committed, the next pass may proceed
            pwm_write(m->tim, 0, 0);
            continue;
        }

        // Slew limit so the gearbox and the battery rail are not shocked
        int16_t delta = m->target - m->applied;
        if (delta >  DUTY_SLEW_PER_LOOP) delta =  DUTY_SLEW_PER_LOOP;
        if (delta < -DUTY_SLEW_PER_LOOP) delta = -DUTY_SLEW_PER_LOOP;
        m->applied += delta;

        int8_t sign = (m->applied > 0) ? 1 : ((m->applied < 0) ? -1 : 0);
        if (sign != 0) {
            m->last_sign = sign;
        }

        if (!s_enabled) {
            pwm_write(m->tim, 0, 0);
            continue;
        }

        // Convert permille to compare value, exactly one side is non zero
        uint32_t mag = (uint32_t)((m->applied < 0) ? -m->applied : m->applied);
        uint32_t ccr = (mag * (PWM_ARR + 1U)) / 1000U;

        if (m->applied >= 0) {
            pwm_write(m->tim, (uint16_t)ccr, 0);
        } else {
            pwm_write(m->tim, 0, (uint16_t)ccr);
        }

        // Stall guard, the only protection available without encoders
        // Sustained high duty most likely means a jammed wheel
        if (mag > STALL_DUTY_THRES) {
            m->stall_ms += LOOP_PERIOD_MS;
            if (m->stall_ms > STALL_TIMEOUT_MS) {
                s_fault |= MOTOR_FAULT_STALL;
                motor_estop();
                return;
            }
        } else {
            m->stall_ms = 0;
        }
    }
}
