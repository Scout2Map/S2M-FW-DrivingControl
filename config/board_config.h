/*
 * File   : board_config.h
 * Purpose: Single source of truth for the pin map, clock tree, safety
 *          limits and control tuning constants. Every layer may include
 *          this header; it contains macros only and no hardware access.
 * Author : jihoonkimtech
 *
 * Pin map
 *   TIM2 : PA0(CH1)=L_LPWM, PA1(CH2)=L_RPWM -> BTS7960 #1   [20kHz]
 *   TIM3 : PA6(CH1)=R_LPWM, PA7(CH2)=R_RPWM -> BTS7960 #2   [20kHz]
 *   GPIO : PB0 = L_EN, PB1 = R_EN           -> BTS7960 enable / e-stop
 *   TIM1 : PA8(CH1)=A, PA9(CH2)=B  -> left encoder   [quadrature x4]
 *   TIM4 : PB6(CH1)=A, PB7(CH2)=B  -> right encoder  [quadrature x4]
 *   ADC1 : PA4 (IN4)               -> 2D120X IR distance      [5V sensor]
 *   I2C2 : SCL=PB10, SDA=PB11      -> BNO055(0x28)            [3.3V]
 *   USB  : PA11=DM, PA12=DP        -> RPi5, VBUS trace cut
 *   SWD  : PA13=SWDIO, PA14=SWCLK
 *
 * Note: encoders must sit on a timer CH1+CH2 pair. PB0/PB1 are TIM3
 * CH3/CH4 and cannot decode quadrature, and PB2 is BOOT1. That is why
 * the enable pins keep PB0/PB1 and the IMU moved off PB6/PB7 to I2C2.
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

// ============================================================
// Scout2Map UGV - STM32F103C8T6 drive controller
// Bare metal, CMSIS register level, no HAL
// ============================================================

// ---- Feature switches ----
// Encoder motors installed, closed loop velocity control active
#define ENCODER_AVAILABLE       1

// Heading assist is only used while running open loop
#define HEADING_ASSIST_ENABLE   1

// ---- Clock tree ----
// HSE 8MHz crystal, PLL x9 -> 72MHz sysclk
// APB1 = 36MHz but timer clock doubles back to 72MHz
// USB prescaler /1.5 -> 48MHz exactly
#define HSE_FREQ_HZ             8000000UL
#define SYSCLK_FREQ_HZ          72000000UL
#define APB1_TIMER_CLK_HZ       72000000UL

// ---- Motor PWM ----
// 20kHz stays above audible range and under the BTS7960 25kHz limit
#define PWM_FREQ_HZ             20000UL
#define PWM_ARR                 ((APB1_TIMER_CLK_HZ / PWM_FREQ_HZ) - 1U)   // 3599

// ---- Control loop timing ----
#define LOOP_PERIOD_MS          5U      // 200Hz motor control
#define LOOP_PERIOD_S           0.005f
#define IMU_PERIOD_MS           10U     // BNO055 fusion output caps at 100Hz
#define ADC_PERIOD_MS           40U     // GP2D120 refreshes about every 38ms
#define TELEM_PERIOD_MS         20U     // 50Hz telemetry to RPi5

// ---- Safety limits ----
#define CMD_TIMEOUT_MS          300U    // no command from RPi5 -> stop
#define DUTY_SLEW_PER_LOOP      20      // max duty change per 5ms loop, permille
#define DIR_CHANGE_DEADTIME_MS  2U      // pause before reversing a half bridge
#define STALL_DUTY_THRES        700     // permille, sustained high duty
#define STALL_TIMEOUT_MS        3000U   // then assume stuck and coast
#define STALL_MIN_COUNTS        2       // counts expected per loop while driving

// Duty is expressed in permille (-1000 .. +1000), integers only in the ISR path
#define DUTY_MAX                1000
#define DUTY_MIN_MOVE           250     // open loop only, below this nothing moves

// ============================================================
// Pin map
// ============================================================

// ---- Left motor driver (BTS7960 #1) ----
#define MOTL_EN_PORT            GPIOB
#define MOTL_EN_PIN             0U      // PB0, emergency stop path
#define MOTL_LPWM_PIN           0U      // PA0, TIM2_CH1
#define MOTL_RPWM_PIN           1U      // PA1, TIM2_CH2

// ---- Right motor driver (BTS7960 #2) ----
#define MOTR_EN_PORT            GPIOB
#define MOTR_EN_PIN             1U      // PB1, emergency stop path
#define MOTR_LPWM_PIN           6U      // PA6, TIM3_CH1
#define MOTR_RPWM_PIN           7U      // PA7, TIM3_CH2

// ---- Encoders ----
// Hardware quadrature only works on a timer CH1 + CH2 pair
// Left  -> TIM1, PA8 (yellow, A) / PA9 (green, B)
// Right -> TIM4, PB6 (yellow, A) / PB7 (green, B)
#define ENC_L_A_PIN             8U
#define ENC_L_B_PIN             9U
#define ENC_R_A_PIN             6U
#define ENC_R_B_PIN             7U

// Flip if a wheel counts down while being driven forward
#define ENC_L_INVERT            0
#define ENC_R_INVERT            1       // right motor is mounted mirrored

// ---- IR distance sensor 2D120X ----
#define DIST_ADC_PIN            4U      // PA4, ADC12_IN4
#define DIST_ADC_CHANNEL        4U

// ---- IMU BNO055 on I2C2 ----
// Moved off PB6/PB7 because TIM4 now owns those pins
#define IMU_I2C_SCL_PIN         10U     // PB10
#define IMU_I2C_SDA_PIN         11U     // PB11
#define BNO055_ADDR             0x28U   // 0x29 if the COM3 pin is pulled high

// ---- Status LED ----
#define LED_PORT                GPIOC
#define LED_PIN                 13U     // onboard LED, active low

// ---- USB ----
// PA11 = DM, PA12 = DP, driven by the USB peripheral
// VBUS trace must be cut, the board is powered from the UBEC 5V rail

// ============================================================
// Mechanical constants
// ============================================================

#define WHEEL_DIAMETER_MM       66.0f
#define WHEEL_CIRCUM_MM         (3.14159265f * WHEEL_DIAMETER_MM)
#define WHEEL_BASE_MM           240.0f  // track width, matches the URDF

// Encoder resolution
// Yellow wire emits 11 pulses per motor revolution
// Quadrature counts both edges of both channels, hence x4
// GEAR_RATIO is a placeholder for the 76RPM variant, MEASURE IT
// Procedure: hand turn one wheel exactly 10 revolutions, read the raw
// counter delta, divide by 10, write the result into COUNTS_PER_WHEEL_REV
#define ENC_PPR                 11
#define GEAR_RATIO              150     // TODO verify by measurement
#define COUNTS_PER_WHEEL_REV    (ENC_PPR * 4 * GEAR_RATIO)
#define MM_PER_COUNT            (WHEEL_CIRCUM_MM / (float)COUNTS_PER_WHEEL_REV)

// ---- Velocity PID ----
// Executes in the main loop only, never inside an interrupt handler
// Tune P first, then add I, D usually stays at zero on a geared drivetrain
#define PID_KP                  180.0f
#define PID_KI                  90.0f
#define PID_USE_D               0        // preprocessor cannot compare floats
#define PID_KD                  0.0f
#define PID_I_LIMIT             400.0f  // anti windup clamp, permille
#define PID_FF                  3800.0f // feedforward, reuse the open loop curve

// 76rpm -> 1.267 rev/s -> 0.262 m/s with 66mm wheels
#define MAX_WHEEL_SPEED_MPS     0.26f

// ---- Open loop fallback ----
// Still used when ENCODER_AVAILABLE is 0 or an encoder fault is latched
#define DUTY_PER_MPS            3800.0f
#define DUTY_PER_RADPS          400.0f
#define HEADING_KP              300.0f   // proportional only, open loop hold

#endif // BOARD_CONFIG_H
