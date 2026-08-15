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
// Bench calibration build. Motors stay disabled and the firmware only
// reports encoder counts over USART2 and the LED. Set back to 0 before
// putting the robot on the floor.
//
// Encoder resolution was confirmed on 2026-08-13, so this is now off by
// default. Set to 1 again if a motor is replaced or rewired.
#define CALIB_MODE              0

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

// ---- Watchdog ----
// IWDG runs on the 40kHz LSI. Prescaler 32 with reload 500 lands near
// 400ms. Any blocking routine longer than this must refresh the key
// register itself or the board resets mid-operation.
#define IWDG_PRESCALER          3U
#define IWDG_RELOAD             500U
#define IWDG_TIMEOUT_MS         400U
#define IWDG_REFRESH_KEY        0xAAAAU
#define IWDG_UNLOCK_KEY         0x5555U
#define IWDG_START_KEY          0xCCCCU

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

// ---- Motor direction ----
// The two drive motors face opposite ways on the chassis, so one side
// must be inverted for a positive command to move the robot forward.
// Handled here rather than by swapping wires: the perfboard stays
// consistent with the schematic, and a motor swap needs one edit.
//
// Measured on hardware 2026-08-14 with the chassis lifted.
#define MOTL_INVERT             0       // verified, drives forward
#define MOTR_INVERT             1       // verified, mounted mirrored

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

// Measured on hardware 2026-08-14 with --raw and the chassis lifted,
// then re-checked by turning each wheel forward by hand.
//   left  : drove forward, counted down      -> invert
//   right : counted down when turned forward -> do not invert, because
//           MOTR_INVERT already corrects the drive direction
#define ENC_L_INVERT            1       // verified
#define ENC_R_INVERT            0       // verified

// ---- IR distance sensor 2D120X ----
#define DIST_ADC_PIN            4U      // PA4, ADC12_IN4
#define DIST_ADC_CHANNEL        4U

// ---- IMU BNO055 on I2C2 ----
// Moved off PB6/PB7 because TIM4 now owns those pins
#define IMU_I2C_SCL_PIN         10U     // PB10
#define IMU_I2C_SDA_PIN         11U     // PB11
#define BNO055_ADDR             0x28U   // 0x29 if the COM3 pin is pulled high

// ---- Status LED ----
// This core board is NOT a stock Blue Pill. Its schematic wires the
// user LED D2 through R6 to PB12; a standard Blue Pill would use PC13.
// Verify against the schematic before assuming either on a new board.
#define LED_PORT                GPIOB
#define LED_PIN                 12U
#define LED_ACTIVE_LOW          1       // cathode side driven by the pin

// ---- USB ----
// PA11 = DM, PA12 = DP, driven by the USB peripheral
// VBUS trace must be cut, the board is powered from the UBEC 5V rail

// ============================================================
// Mechanical constants
// ============================================================

#define WHEEL_DIAMETER_MM       66.0f
#define WHEEL_CIRCUM_MM         (3.14159265f * WHEEL_DIAMETER_MM)
#define WHEEL_BASE_MM           240.0f  // track width, matches the URDF

// Encoder resolution, VERIFIED ON HARDWARE 2026-08-13
//
// Bench measurement: one wheel hand turned exactly 10 revolutions
// tripped the calibration threshold at 57640 counts, confirming both
// terms below. The 13 PPR hypothesis would have tripped near 8.5
// revolutions and did not.
//
//   ENC_PPR    11   yellow wire, pulses per MOTOR revolution
//   GEAR_RATIO 131  matches the JGB37-520 12V datasheet column
//                   where no-load speed reads 76 RPM
//   11 x 4 (quadrature) x 131 = 5764 counts per WHEEL revolution
//
// A wrong value here scales every distance the robot reports and shows
// up as loop closure drift in SLAM, so do not change these without
// repeating the bench measurement.
#define ENC_PPR                 11      // verified
#define GEAR_RATIO              131     // verified
#define COUNTS_PER_WHEEL_REV    (ENC_PPR * 4 * GEAR_RATIO)   // 5764
#define MM_PER_COUNT            (WHEEL_CIRCUM_MM / (float)COUNTS_PER_WHEEL_REV)

// ---- Velocity PID ----
// Executes in the main loop only, never inside an interrupt handler.
//
// Tuned against the unloaded step response captured 2026-08-14.
// That run settled at 172 mm/s against a 150 mm/s target, a 15% steady
// error the loop never removed, because the original gains were far
// too small: a 22 mm/s error asked for 4 permille of correction and
// the integrator needed 48 seconds to cover the gap.
//
// Sizing rule used here: a 10% speed error should command roughly a
// 10% duty correction. Full duty corresponds to MAX_WHEEL_SPEED_MPS,
// so KP is on the order of DUTY_MAX / MAX_WHEEL_SPEED_MPS.
//
// These sit below what that rule alone suggests. The capture showed a
// monotonic rise with no overshoot, so the real plant is slower than a
// first order fit predicts and there is headroom to raise them further
// if tracking proves sluggish once the chassis is loaded.
//
// Retune on the ground, not on a stand. Unloaded, the feedforward runs
// about 16% high and the integrator spends seconds cancelling it, which
// looks like a tuning fault but is only an artefact of no load.
#define PID_KP                  3000.0f
#define PID_KI                  3000.0f
#define PID_USE_D               0        // preprocessor cannot compare floats
#define PID_KD                  0.0f

// Must be large enough to absorb the feedforward error on its own.
// Unloaded, the rated slope overshoots by roughly 100 permille at mid
// speed, and the integrator has to cover that before it can trim.
#define PID_I_LIMIT             600.0f

// Feedforward slope, permille per m/s. Derived from the RATED speed so
// full duty maps to MAX_WHEEL_SPEED_MPS.
//
// Measured unloaded the slope is nearer 4300, but the wheels were off
// the ground. Under load the same duty yields less speed, so the rated
// figure is the better default: it undershoots slightly on the bench,
// which the PID trims, and lands close once the chassis carries itself.
#define PID_FF                  5000.0f

// 76rpm is the NO-LOAD figure. Rated speed under load is 58rpm, which
// is what the robot actually sustains once it carries its own mass.
//   58rpm -> 0.967 rev/s -> 0.200 m/s with 66mm wheels
// Sizing the ceiling off the no-load number would let the controller
// command a speed the drivetrain cannot reach, leaving the integrator
// permanently saturated.
#define MAX_WHEEL_SPEED_MPS     0.20f

// Angular ceiling. Spinning in place at the wheel limit gives
//   w = 2 * v / track = 2 * 0.20 / 0.24 = 1.67 rad/s, about 96 deg/s
// A full RPLiDAR C1 revolution takes 100ms, so that rate smears one
// scan by roughly 9.6 degrees and degrades scan matching. Cap the
// commanded rate well below the mechanical limit while mapping.
#define MAX_ANGULAR_RATE        0.80f   // rad/s, about 46 deg/s
#define MAX_ANGULAR_RATE_MECH   1.67f   // rad/s, what the wheels allow

// ---- Open loop fallback ----
// Still used when ENCODER_AVAILABLE is 0 or an encoder fault is latched
#define DUTY_PER_MPS            5000.0f
#define DUTY_PER_RADPS          400.0f
#define HEADING_KP              300.0f   // proportional only, open loop hold

#endif // BOARD_CONFIG_H
