/*
 * File   : i2c.c
 * Purpose: Non-blocking I2C2 master for the BNO055, with the bus recovery
 *          the STM32F1 peripheral needs in practice.
 * Author : jihoonkimtech
 *
 * Pin map
 *   I2C2 SCL = PB10, SDA = PB11, both open drain with external pull ups
 *
 * The transfer runs as a state machine driven from the main loop. A
 * blocking driver would stall the 200Hz control loop for the duration
 * of a nine byte read, which at 100kHz is close to a millisecond.
 *
 * Note: the STM32F1 I2C peripheral is known to latch up. If a transfer
 * is interrupted mid byte the slave can hold SDA low forever, and no
 * amount of poking the peripheral clears it. The only fix is to drop
 * the pins to GPIO, clock SCL manually until the slave releases SDA,
 * issue a STOP by hand, then software reset the peripheral. That is
 * what i2c_bus_recover() does, and it is not optional on this part.
 *
 * Note: BNO055 stretches the clock. The bus runs at 100kHz rather than
 * 400kHz because the combination of clock stretching and the F1 errata
 * is far more reliable there, and 100kHz is ample for 100Hz fusion data.
 */

#include "stm32f1xx.h"
#include "board_config.h"
#include "i2c.h"
#include "systick.h"

#define I2C_BUS             I2C2
#define I2C_TIMEOUT_MS      5U      // any single phase taking longer is a fault

typedef enum {
    ST_IDLE = 0,
    ST_START,
    ST_ADDR_W,
    ST_REG,
    ST_WRITE_DATA,
    ST_RESTART,
    ST_ADDR_R,
    ST_READ_DATA,
    ST_DONE,
    ST_ERROR
} i2c_state_t;

static volatile i2c_state_t s_state;
static uint8_t   s_addr;
static uint8_t   s_reg;
static uint8_t  *s_buf;
static uint8_t   s_len;
static uint8_t   s_idx;
static uint8_t   s_is_read;
static uint32_t  s_phase_start;
static uint32_t  s_errors;
static uint32_t  s_recoveries;

static void gpio_od_af(uint32_t pin)
{
    // Alternate function open drain, 50MHz: CNF=11 MODE=11 -> 0xF
    volatile uint32_t *cr = (pin < 8U) ? &GPIOB->CRL : &GPIOB->CRH;
    uint32_t shift = (pin % 8U) * 4U;
    *cr &= ~(0xFU << shift);
    *cr |= (0xFU << shift);
}

static void gpio_od_out(uint32_t pin)
{
    // General purpose open drain, 50MHz: CNF=01 MODE=11 -> 0x7
    volatile uint32_t *cr = (pin < 8U) ? &GPIOB->CRL : &GPIOB->CRH;
    uint32_t shift = (pin % 8U) * 4U;
    *cr &= ~(0xFU << shift);
    *cr |= (0x7U << shift);
}

static void i2c_peripheral_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

    I2C_BUS->CR1 = I2C_CR1_SWRST;       // hold in reset
    I2C_BUS->CR1 = 0;                   // release

    // APB1 is 36MHz
    I2C_BUS->CR2   = 36U;
    // Standard mode 100kHz: CCR = APB1 / (2 * 100k) = 180
    I2C_BUS->CCR   = 180U;
    // Max rise time in standard mode is 1000ns, so TRISE = APB1MHz + 1
    I2C_BUS->TRISE = 37U;
    I2C_BUS->CR1  |= I2C_CR1_PE;
}

// Clocks the bus manually until a stuck slave releases SDA.
// Returns 1 if the bus came back, 0 if the slave never let go.
uint8_t i2c_bus_recover(void)
{
    s_recoveries++;

    I2C_BUS->CR1 &= ~I2C_CR1_PE;

    gpio_od_out(IMU_I2C_SCL_PIN);
    gpio_od_out(IMU_I2C_SDA_PIN);
    GPIOB->BSRR = (1U << IMU_I2C_SCL_PIN) | (1U << IMU_I2C_SDA_PIN);

    // Nine edges is one byte plus the ACK slot, enough for a slave to
    // finish whatever transfer it thinks is still running
    for (int i = 0; i < 9; i++) {
        GPIOB->BSRR = (1U << (IMU_I2C_SCL_PIN + 16U));
        systick_delay_us(5);
        GPIOB->BSRR = (1U << IMU_I2C_SCL_PIN);
        systick_delay_us(5);
        if (GPIOB->IDR & (1U << IMU_I2C_SDA_PIN)) {
            break;                      // slave released the line
        }
    }

    // Manual STOP: SDA rises while SCL is high
    GPIOB->BSRR = (1U << (IMU_I2C_SDA_PIN + 16U));
    systick_delay_us(5);
    GPIOB->BSRR = (1U << IMU_I2C_SCL_PIN);
    systick_delay_us(5);
    GPIOB->BSRR = (1U << IMU_I2C_SDA_PIN);
    systick_delay_us(5);

    uint8_t freed = (GPIOB->IDR & (1U << IMU_I2C_SDA_PIN)) ? 1U : 0U;

    gpio_od_af(IMU_I2C_SCL_PIN);
    gpio_od_af(IMU_I2C_SDA_PIN);
    i2c_peripheral_init();

    return freed;
}

void i2c_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    gpio_od_af(IMU_I2C_SCL_PIN);
    gpio_od_af(IMU_I2C_SDA_PIN);
    i2c_peripheral_init();

    s_state      = ST_IDLE;
    s_errors     = 0;
    s_recoveries = 0;

    // A slave left mid transfer by a previous reset would hold SDA low
    if (!(GPIOB->IDR & (1U << IMU_I2C_SDA_PIN))) {
        i2c_bus_recover();
    }
}

uint8_t i2c_is_busy(void)
{
    return (uint8_t)(s_state != ST_IDLE && s_state != ST_DONE
                     && s_state != ST_ERROR);
}

uint8_t i2c_start_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (i2c_is_busy() || len == 0U) {
        return 0;
    }
    s_addr    = addr;
    s_reg     = reg;
    s_buf     = buf;
    s_len     = len;
    s_idx     = 0;
    s_is_read = 1;
    s_state   = ST_START;
    s_phase_start = systick_millis();
    return 1;
}

uint8_t i2c_start_write(uint8_t addr, uint8_t reg, const uint8_t *buf, uint8_t len)
{
    if (i2c_is_busy()) {
        return 0;
    }
    s_addr    = addr;
    s_reg     = reg;
    s_buf     = (uint8_t *)buf;
    s_len     = len;
    s_idx     = 0;
    s_is_read = 0;
    s_state   = ST_START;
    s_phase_start = systick_millis();
    return 1;
}

i2c_result_t i2c_poll(void)
{
    uint32_t sr1 = I2C_BUS->SR1;

    // A NACK means the device did not answer. Clear it and give up on
    // this transfer rather than spinning until the timeout.
    if (sr1 & I2C_SR1_AF) {
        I2C_BUS->SR1 &= ~I2C_SR1_AF;
        I2C_BUS->CR1 |= I2C_CR1_STOP;
        s_state = ST_ERROR;
        s_errors++;
        return I2C_RESULT_ERROR;
    }

    // Any phase overrunning its budget means the bus is wedged
    if (i2c_is_busy() && (systick_millis() - s_phase_start) > I2C_TIMEOUT_MS) {
        i2c_bus_recover();
        s_state = ST_ERROR;
        s_errors++;
        return I2C_RESULT_ERROR;
    }

    switch (s_state) {
    case ST_IDLE:
        return I2C_RESULT_IDLE;

    case ST_START:
        I2C_BUS->CR1 |= I2C_CR1_ACK | I2C_CR1_START;
        s_state = ST_ADDR_W;
        s_phase_start = systick_millis();
        break;

    case ST_ADDR_W:
        if (sr1 & I2C_SR1_SB) {
            I2C_BUS->DR = (uint8_t)(s_addr << 1);   // write direction
            s_state = ST_REG;
            s_phase_start = systick_millis();
        }
        break;

    case ST_REG:
        if (sr1 & I2C_SR1_ADDR) {
            (void)I2C_BUS->SR2;                     // clearing sequence
            I2C_BUS->DR = s_reg;
            s_state = s_is_read ? ST_RESTART : ST_WRITE_DATA;
            s_phase_start = systick_millis();
        }
        break;

    case ST_WRITE_DATA:
        if (sr1 & I2C_SR1_TXE) {
            if (s_idx < s_len) {
                I2C_BUS->DR = s_buf[s_idx++];
                s_phase_start = systick_millis();
            } else if (sr1 & I2C_SR1_BTF) {
                I2C_BUS->CR1 |= I2C_CR1_STOP;
                s_state = ST_DONE;
            }
        }
        break;

    case ST_RESTART:
        if (sr1 & I2C_SR1_BTF) {
            I2C_BUS->CR1 |= I2C_CR1_START;          // repeated start
            s_state = ST_ADDR_R;
            s_phase_start = systick_millis();
        }
        break;

    case ST_ADDR_R:
        if (sr1 & I2C_SR1_SB) {
            I2C_BUS->DR = (uint8_t)((s_addr << 1) | 1U);   // read direction

            // The last byte must be NACKed, and for a single byte read
            // that decision has to be made before ADDR is cleared
            if (s_len == 1U) {
                I2C_BUS->CR1 &= ~I2C_CR1_ACK;
            }
            s_state = ST_READ_DATA;
            s_phase_start = systick_millis();
        }
        break;

    case ST_READ_DATA:
        if (sr1 & I2C_SR1_ADDR) {
            (void)I2C_BUS->SR2;
            if (s_len == 1U) {
                I2C_BUS->CR1 |= I2C_CR1_STOP;
            }
            s_phase_start = systick_millis();
        } else if (sr1 & I2C_SR1_RXNE) {
            s_buf[s_idx++] = (uint8_t)I2C_BUS->DR;
            s_phase_start = systick_millis();

            if (s_idx == s_len - 1U) {
                // NACK the final byte so the slave stops driving
                I2C_BUS->CR1 &= ~I2C_CR1_ACK;
                I2C_BUS->CR1 |= I2C_CR1_STOP;
            }
            if (s_idx >= s_len) {
                s_state = ST_DONE;
            }
        }
        break;

    case ST_DONE:
        s_state = ST_IDLE;
        return I2C_RESULT_DONE;

    case ST_ERROR:
        s_state = ST_IDLE;
        return I2C_RESULT_ERROR;

    default:
        s_state = ST_IDLE;
        break;
    }

    return i2c_is_busy() ? I2C_RESULT_BUSY : I2C_RESULT_IDLE;
}

uint32_t i2c_error_count(void)    { return s_errors; }
uint32_t i2c_recovery_count(void) { return s_recoveries; }

// ============================================================
// Bus scan
//
// Probes every 7 bit address by sending the address byte and watching
// for an ACK. Blocking on purpose: it is a bring-up tool, not part of
// the control path, and a scan of 112 addresses at 100kHz takes only a
// few milliseconds.
//
// Note: probing with a write bit is the safe direction. A read probe
// would make some devices start streaming data, which then has to be
// clocked out before the bus returns to idle.
// ============================================================

// Guard sized against the actual bus, not an arbitrary large number.
// One address byte at 100kHz takes about 110us; 20000 iterations of a
// register poll is roughly 1.4ms at 72MHz, which is ample headroom.
//
// The original 100000 was catastrophic here: with nothing on the bus
// every probe ran all three guards to completion, so a full scan took
// 2.3 seconds. The main loop stalled long enough to break USB and the
// watchdog fired mid scan, which looked like the console silently
// returning nothing at all.
#define PROBE_GUARD     20000U

static void probe_wait_tick(void)
{
    // Fed on every guard iteration, not just the outer loop. A scan of
    // an empty bus spends all its time inside these waits.
    IWDG->KR = IWDG_REFRESH_KEY;
}

uint8_t i2c_probe(uint8_t addr)
{
    uint32_t guard;

    // Wait for the bus to be free, but never forever
    guard = 0;
    while (I2C_BUS->SR2 & I2C_SR2_BUSY) {
        probe_wait_tick();
        if (++guard > PROBE_GUARD) {
            i2c_bus_recover();
            s_errors++;
            return 0;
        }
    }

    I2C_BUS->CR1 |= I2C_CR1_START;

    guard = 0;
    while (!(I2C_BUS->SR1 & I2C_SR1_SB)) {
        probe_wait_tick();
        if (++guard > PROBE_GUARD) {
            I2C_BUS->CR1 |= I2C_CR1_STOP;
            s_errors++;             // distinguishes a dead bus from a NACK
            return 0;
        }
    }

    I2C_BUS->DR = (uint8_t)(addr << 1);     // write direction

    // Either ADDR (acked) or AF (nacked) will come up
    guard = 0;
    uint8_t found = 0;
    while (1) {
        uint32_t sr1 = I2C_BUS->SR1;
        if (sr1 & I2C_SR1_ADDR) {
            (void)I2C_BUS->SR2;             // clearing sequence
            found = 1;
            break;
        }
        if (sr1 & I2C_SR1_AF) {
            I2C_BUS->SR1 &= ~I2C_SR1_AF;
            break;
        }
        probe_wait_tick();
        if (++guard > PROBE_GUARD) {
            s_errors++;
            break;
        }
    }

    I2C_BUS->CR1 |= I2C_CR1_STOP;

    // Let the STOP finish before the next probe starts
    guard = 0;
    while ((I2C_BUS->CR1 & I2C_CR1_STOP) && ++guard < PROBE_GUARD) {
        probe_wait_tick();
    }

    return found;
}

// Fills a bitmap of responding addresses, one bit per address 0..127.
// Returns how many devices answered.
// Returns 1 if the bus lines are in a state where a scan can succeed.
// Both must idle high; a line stuck low means no pull up, no power on
// the sensor board, or a short.
uint8_t i2c_lines_idle(void)
{
    return (uint8_t)((GPIOB->IDR & (1U << IMU_I2C_SCL_PIN)) &&
                     (GPIOB->IDR & (1U << IMU_I2C_SDA_PIN)));
}

uint8_t i2c_scan(uint8_t *bitmap16, uint8_t *lines_out)
{
    uint8_t count = 0;
    uint8_t consecutive_timeouts = 0;

    for (int i = 0; i < 16; i++) {
        bitmap16[i] = 0;
    }

    // Sample the idle line state only once the peripheral has released
    // the bus. Any other moment can catch a transfer in progress, where
    // a low line is normal traffic rather than a fault.
    uint32_t guard = 0;
    while ((I2C_BUS->SR2 & I2C_SR2_BUSY) && ++guard < PROBE_GUARD) {
        IWDG->KR = IWDG_REFRESH_KEY;
    }
    if (lines_out) {
        *lines_out = i2c_lines_idle() ? 0x03U : 0x00U;
    }

    // 0x00 to 0x07 and 0x78 to 0x7F are reserved by the specification
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        IWDG->KR = IWDG_REFRESH_KEY;

        uint32_t before = s_errors;
        if (i2c_probe(a)) {
            bitmap16[a >> 3] |= (uint8_t)(1U << (a & 7U));
            count++;
            consecutive_timeouts = 0;
            continue;
        }

        // A NACK is a normal negative answer and costs microseconds.
        // A timeout means the peripheral never even reached the address
        // phase, which no amount of further probing will change.
        if (s_errors != before) {
            consecutive_timeouts++;
        } else {
            consecutive_timeouts = 0;
        }

        // Eight dead probes in a row means the bus is not usable. Going
        // on would burn a full second and stall USB for long enough to
        // drop the host connection, which is how this failure first
        // presented: the console returned nothing at all.
        if (consecutive_timeouts >= 8U) {
            break;
        }
    }
    return count;
}
