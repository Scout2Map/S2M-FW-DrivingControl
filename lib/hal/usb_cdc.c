/*
 * File   : usb_cdc.c
 * Purpose: Bare metal USB CDC-ACM device for the STM32F103, providing a
 *          /dev/ttyACM* serial link to the RPi5 without any USB stack.
 * Author : jihoonkimtech
 *
 * Endpoints
 *   EP0  control, 64 byte packets
 *   EP1  bulk IN,  64 bytes, device to host
 *   EP2  bulk OUT, 64 bytes, host to device
 *   EP3  interrupt IN, notification, never actually used
 *
 * PMA budget, 512 bytes total on this part
 *   0x000..0x03F  buffer descriptor table, 8 endpoints x 8 bytes
 *   0x040..0x07F  EP0 RX
 *   0x080..0x0BF  EP0 TX
 *   0x0C0..0x0FF  EP1 TX
 *   0x100..0x13F  EP2 RX
 *   0x140..0x14F  EP3 TX
 *
 * Note: PMA is addressed as 16 bit words on a 32 bit stride from the
 * CPU side. Writing it like a flat byte array silently corrupts every
 * other halfword, which is the classic first bug when driving this
 * peripheral directly.
 *
 * Note: endpoint registers mix toggle-only bits (STAT_RX, STAT_TX,
 * DTOG) with write-zero-to-clear bits (CTR_RX, CTR_TX). A naive
 * read-modify-write clears the CTR flags and scrambles STAT. Every
 * access here goes through ep_set_stat_rx/tx or ep_clear_ctr_rx/tx,
 * which force the clear-only bits high and XOR the toggle bits.
 *
 * Note: many clone boards fit a 10k pull up on PA12 instead of the
 * required 1.5k, and never enumerate. This board was verified to carry
 * the correct 1.5k. If a future board fails to appear on the host,
 * check R7 before suspecting the firmware.
 */

#include <string.h>
#include "stm32f1xx.h"
#include "board_config.h"
#include "usb_cdc.h"

// ---- PMA access ----
// The packet memory sits at USB_PMAADDR and is seen by the CPU as
// 16 bit values spaced 4 bytes apart.
#define PMA_BASE        (USB_PMAADDR)
#define PMA(offset)     (*(volatile uint16_t *)(PMA_BASE + ((offset) * 2U)))

#define BTABLE_ADDR     0x000U
#define EP0_RX_ADDR     0x040U
#define EP0_TX_ADDR     0x080U
#define EP1_TX_ADDR     0x0C0U
#define EP2_RX_ADDR     0x100U
#define EP3_TX_ADDR     0x140U

#define EP_MAX_PACKET   64U

// Buffer descriptor table entries, indices into the PMA word space
#define BDT_ADDR_TX(ep) PMA(BTABLE_ADDR / 2U + (ep) * 8U + 0U)
#define BDT_COUNT_TX(ep) PMA(BTABLE_ADDR / 2U + (ep) * 8U + 2U)
#define BDT_ADDR_RX(ep) PMA(BTABLE_ADDR / 2U + (ep) * 8U + 4U)
#define BDT_COUNT_RX(ep) PMA(BTABLE_ADDR / 2U + (ep) * 8U + 6U)

// Endpoint register bits that must be preserved rather than rewritten
#define EP_NONTOGGLE    (USB_EP0R_CTR_RX | USB_EP0R_CTR_TX | \
                         USB_EP0R_EP_TYPE | USB_EP0R_EP_KIND | USB_EP0R_EA)
#define EP_TOGGLE       (USB_EP0R_STAT_RX | USB_EP0R_STAT_TX | \
                         USB_EP0R_DTOG_RX | USB_EP0R_DTOG_TX)

#define STAT_DISABLED   0U
#define STAT_STALL      1U
#define STAT_NAK        2U
#define STAT_VALID      3U

static volatile uint16_t *const s_epr[4] = {
    &USB->EP0R, &USB->EP1R, &USB->EP2R, &USB->EP3R
};

// ---- Driver state ----
static uint8_t  s_configured;
static uint8_t  s_tx_busy;
static uint8_t  s_dtr;              // host opened the port
static uint8_t  s_rx_buf[USB_CDC_RX_BUF];
static volatile uint16_t s_rx_head, s_rx_tail;

// Line coding is accepted and echoed but has no effect, the link is
// not a real UART. Hosts still expect the requests to succeed.
static uint8_t s_line_coding[7] = { 0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08 };

// Pending control transfer state
static const uint8_t *s_ctrl_data;
static uint16_t       s_ctrl_len;
static uint8_t        s_pending_addr;

// ============================================================
// Descriptors
// ============================================================

static const uint8_t dev_desc[] = {
    18, 0x01,
    0x00, 0x02,             // USB 2.0
    0x02, 0x00, 0x00,       // class CDC, defined at interface level
    EP_MAX_PACKET,
    0x83, 0x04,             // VID 0x0483, ST
    0x40, 0x57,             // PID 0x5740, virtual COM port
    0x00, 0x02,             // device release 2.00
    1, 2, 3,                // manufacturer, product, serial
    1                       // one configuration
};

static const uint8_t cfg_desc[] = {
    // Configuration
    9, 0x02, 67, 0x00, 2, 1, 0, 0x80, 250,
    // Interface 0, communication class
    9, 0x04, 0, 0, 1, 0x02, 0x02, 0x01, 0,
    // CDC header
    5, 0x24, 0x00, 0x10, 0x01,
    // Call management, no data interface used
    5, 0x24, 0x01, 0x00, 1,
    // Abstract control management
    4, 0x24, 0x02, 0x02,
    // Union, controlling interface 0, subordinate 1
    5, 0x24, 0x06, 0, 1,
    // Notification endpoint, present because the class requires it
    7, 0x05, 0x83, 0x03, 8, 0x00, 0xFF,
    // Interface 1, data class
    9, 0x04, 1, 0, 2, 0x0A, 0x00, 0x00, 0,
    // Bulk OUT, host to device
    7, 0x05, 0x02, 0x02, EP_MAX_PACKET, 0x00, 0,
    // Bulk IN, device to host
    7, 0x05, 0x81, 0x02, EP_MAX_PACKET, 0x00, 0
};

static const uint8_t str_lang[]  = { 4, 0x03, 0x09, 0x04 };
static const uint8_t str_manuf[] = {
    22, 0x03, 'S',0,'c',0,'o',0,'u',0,'t',0,'2',0,'M',0,'a',0,'p',0,' ',0
};
static const uint8_t str_prod[]  = {
    28, 0x03, 'D',0,'r',0,'i',0,'v',0,'e',0,' ',0,'C',0,'o',0,'n',0,
    't',0,'r',0,'o',0,'l',0
};
static const uint8_t str_serial[] = {
    18, 0x03, 'S',0,'2',0,'M',0,'-',0,'D',0,'R',0,'V',0,'1',0
};

// ============================================================
// Low level helpers
// ============================================================

static void ep_set_stat_rx(uint8_t ep, uint16_t stat)
{
    uint16_t v = *s_epr[ep];
    v = (v & (EP_NONTOGGLE | USB_EP0R_STAT_RX))
        | USB_EP0R_CTR_RX | USB_EP0R_CTR_TX;
    v ^= (stat << USB_EP0R_STAT_RX_Pos);
    *s_epr[ep] = v;
}

static void ep_set_stat_tx(uint8_t ep, uint16_t stat)
{
    uint16_t v = *s_epr[ep];
    v = (v & (EP_NONTOGGLE | USB_EP0R_STAT_TX))
        | USB_EP0R_CTR_RX | USB_EP0R_CTR_TX;
    v ^= (stat << USB_EP0R_STAT_TX_Pos);
    *s_epr[ep] = v;
}

static void ep_clear_ctr_rx(uint8_t ep)
{
    // Write 0 to CTR_RX, 1 to everything else that must survive
    *s_epr[ep] = (*s_epr[ep] & EP_NONTOGGLE & ~USB_EP0R_CTR_RX)
                 | USB_EP0R_CTR_TX;
}

static void ep_clear_ctr_tx(uint8_t ep)
{
    *s_epr[ep] = (*s_epr[ep] & EP_NONTOGGLE & ~USB_EP0R_CTR_TX)
                 | USB_EP0R_CTR_RX;
}

// Copies into packet memory. The source may be odd length, the trailing
// byte is written as a halfword with a zero upper half.
static void pma_write(uint16_t pma_offset, const uint8_t *src, uint16_t len)
{
    volatile uint16_t *dst = &PMA(pma_offset / 2U);
    for (uint16_t i = 0; i < len / 2U; i++) {
        dst[i * 2U] = (uint16_t)(src[i * 2U] | (src[i * 2U + 1U] << 8));
    }
    if (len & 1U) {
        dst[(len / 2U) * 2U] = src[len - 1U];
    }
}

static void pma_read(uint16_t pma_offset, uint8_t *dst, uint16_t len)
{
    volatile uint16_t *src = &PMA(pma_offset / 2U);
    for (uint16_t i = 0; i < len / 2U; i++) {
        uint16_t w = src[i * 2U];
        dst[i * 2U]      = (uint8_t)(w & 0xFFU);
        dst[i * 2U + 1U] = (uint8_t)(w >> 8);
    }
    if (len & 1U) {
        dst[len - 1U] = (uint8_t)(src[(len / 2U) * 2U] & 0xFFU);
    }
}

// ============================================================
// Control transfers
// ============================================================

static void ep0_send(const uint8_t *data, uint16_t len)
{
    uint16_t chunk = (len > EP_MAX_PACKET) ? EP_MAX_PACKET : len;
    pma_write(EP0_TX_ADDR, data, chunk);
    BDT_COUNT_TX(0) = chunk;

    // Remember the tail so the next IN token continues the transfer
    s_ctrl_data = data + chunk;
    s_ctrl_len  = len - chunk;

    ep_set_stat_tx(0, STAT_VALID);
}

static void ep0_send_zlp(void)
{
    BDT_COUNT_TX(0) = 0;
    s_ctrl_len = 0;
    ep_set_stat_tx(0, STAT_VALID);
}

static void ep0_stall(void)
{
    ep_set_stat_tx(0, STAT_STALL);
    ep_set_stat_rx(0, STAT_STALL);
}

static void handle_setup(void)
{
    uint8_t  buf[8];
    uint16_t n = BDT_COUNT_RX(0) & 0x3FFU;
    if (n > 8U) n = 8U;
    pma_read(EP0_RX_ADDR, buf, n);

    uint8_t  bmRequestType = buf[0];
    uint8_t  bRequest      = buf[1];
    uint16_t wValue        = (uint16_t)(buf[2] | (buf[3] << 8));
    uint16_t wLength       = (uint16_t)(buf[6] | (buf[7] << 8));

    s_ctrl_data = 0;
    s_ctrl_len  = 0;

    // ---- Standard device requests ----
    if ((bmRequestType & 0x60U) == 0x00U) {
        switch (bRequest) {
        case 0x05:  // SET_ADDRESS
            // The address may only take effect after the status stage
            s_pending_addr = (uint8_t)(wValue & 0x7FU);
            ep0_send_zlp();
            return;

        case 0x06: {  // GET_DESCRIPTOR
            const uint8_t *d = 0;
            uint16_t       l = 0;
            switch (wValue >> 8) {
            case 0x01: d = dev_desc; l = sizeof dev_desc; break;
            case 0x02: d = cfg_desc; l = sizeof cfg_desc; break;
            case 0x03:
                switch (wValue & 0xFFU) {
                case 0: d = str_lang;   l = sizeof str_lang;   break;
                case 1: d = str_manuf;  l = sizeof str_manuf;  break;
                case 2: d = str_prod;   l = sizeof str_prod;   break;
                case 3: d = str_serial; l = sizeof str_serial; break;
                default: break;
                }
                break;
            default: break;
            }
            if (d == 0) {
                ep0_stall();
                return;
            }
            if (l > wLength) l = wLength;
            ep0_send(d, l);
            return;
        }

        case 0x09:  // SET_CONFIGURATION
            s_configured = (wValue != 0U);
            ep0_send_zlp();
            return;

        case 0x08: {  // GET_CONFIGURATION
            static uint8_t cfg;
            cfg = s_configured;
            ep0_send(&cfg, 1);
            return;
        }

        case 0x00: {  // GET_STATUS
            static const uint8_t st[2] = { 0, 0 };
            ep0_send(st, 2);
            return;
        }

        default:
            break;
        }
    }

    // ---- CDC class requests ----
    if ((bmRequestType & 0x60U) == 0x20U) {
        switch (bRequest) {
        case 0x20:  // SET_LINE_CODING, data stage follows and is ignored
            ep0_send_zlp();
            return;
        case 0x21:  // GET_LINE_CODING
            ep0_send(s_line_coding, sizeof s_line_coding);
            return;
        case 0x22:  // SET_CONTROL_LINE_STATE
            // Bit 0 is DTR. Tracking it lets the firmware tell an open
            // port from a merely enumerated one, so telemetry is not
            // written into a buffer nobody is draining.
            s_dtr = (uint8_t)(wValue & 0x01U);
            ep0_send_zlp();
            return;
        default:
            break;
        }
    }

    ep0_stall();
}

static void handle_ep0(void)
{
    uint16_t epr = USB->EP0R;

    if (epr & USB_EP0R_CTR_RX) {
        uint8_t setup = (epr & USB_EP0R_SETUP) ? 1U : 0U;
        ep_clear_ctr_rx(0);
        if (setup) {
            handle_setup();
        } else {
            // OUT data stage, acknowledged and discarded
            ep0_send_zlp();
        }
        BDT_COUNT_RX(0) = (1U << 15) | (1U << 10);  // 64 byte block
        ep_set_stat_rx(0, STAT_VALID);
    }

    if (epr & USB_EP0R_CTR_TX) {
        ep_clear_ctr_tx(0);

        // Applying the address any earlier would break the status stage
        if (s_pending_addr) {
            USB->DADDR = (uint16_t)(USB_DADDR_EF | s_pending_addr);
            s_pending_addr = 0;
        }

        if (s_ctrl_len > 0U && s_ctrl_data != 0) {
            ep0_send(s_ctrl_data, s_ctrl_len);
        }
    }
}

// ============================================================
// Bulk endpoints
// ============================================================

static void handle_ep1_tx(void)
{
    ep_clear_ctr_tx(1);
    s_tx_busy = 0;
}

static void handle_ep2_rx(void)
{
    uint16_t n = BDT_COUNT_RX(2) & 0x3FFU;
    uint8_t  tmp[EP_MAX_PACKET];

    if (n > EP_MAX_PACKET) n = EP_MAX_PACKET;
    pma_read(EP2_RX_ADDR, tmp, n);
    ep_clear_ctr_rx(2);

    for (uint16_t i = 0; i < n; i++) {
        uint16_t next = (uint16_t)((s_rx_head + 1U) % USB_CDC_RX_BUF);
        if (next == s_rx_tail) {
            // Ring full. Dropping here is preferable to blocking inside
            // an ISR; the host will retransmit nothing, but commands are
            // republished continuously so a lost one self corrects.
            break;
        }
        s_rx_buf[s_rx_head] = tmp[i];
        s_rx_head = next;
    }

    ep_set_stat_rx(2, STAT_VALID);
}

// ============================================================
// Interrupt entry
// ============================================================

void USB_LP_CAN1_RX0_IRQHandler(void)
{
    uint16_t istr = USB->ISTR;

    if (istr & USB_ISTR_RESET) {
        USB->ISTR = (uint16_t)~USB_ISTR_RESET;
        usb_cdc_reset_endpoints();
        return;
    }

    if (istr & USB_ISTR_CTR) {
        uint8_t ep = (uint8_t)(istr & USB_ISTR_EP_ID);
        switch (ep) {
        case 0: handle_ep0();    break;
        case 1: handle_ep1_tx(); break;
        case 2: handle_ep2_rx(); break;
        default:
            // Notification endpoint, acknowledge and ignore
            ep_clear_ctr_tx(3);
            break;
        }
    }

    // Suspend and wakeup are acknowledged but not acted on. The robot is
    // bus powered from its own regulator, so low power states carry no
    // benefit and add failure modes.
    if (istr & USB_ISTR_SUSP) {
        USB->ISTR = (uint16_t)~USB_ISTR_SUSP;
    }
    if (istr & USB_ISTR_WKUP) {
        USB->ISTR = (uint16_t)~USB_ISTR_WKUP;
    }
    if (istr & USB_ISTR_ERR) {
        USB->ISTR = (uint16_t)~USB_ISTR_ERR;
    }
}

void usb_cdc_reset_endpoints(void)
{
    USB->BTABLE = BTABLE_ADDR;

    // EP0, control
    BDT_ADDR_TX(0)  = EP0_TX_ADDR;
    BDT_COUNT_TX(0) = 0;
    BDT_ADDR_RX(0)  = EP0_RX_ADDR;
    BDT_COUNT_RX(0) = (1U << 15) | (1U << 10);   // BL_SIZE=1, 2 blocks of 32
    USB->EP0R = 0x0200U | 0U;                    // CONTROL, address 0
    ep_set_stat_rx(0, STAT_VALID);
    ep_set_stat_tx(0, STAT_NAK);

    // EP1, bulk IN
    BDT_ADDR_TX(1)  = EP1_TX_ADDR;
    BDT_COUNT_TX(1) = 0;
    USB->EP1R = 0x0000U | 1U;                    // BULK, address 1
    ep_set_stat_tx(1, STAT_NAK);
    ep_set_stat_rx(1, STAT_DISABLED);

    // EP2, bulk OUT
    BDT_ADDR_RX(2)  = EP2_RX_ADDR;
    BDT_COUNT_RX(2) = (1U << 15) | (1U << 10);
    USB->EP2R = 0x0000U | 2U;                    // BULK, address 2
    ep_set_stat_rx(2, STAT_VALID);
    ep_set_stat_tx(2, STAT_DISABLED);

    // EP3, interrupt IN for notifications, never written
    BDT_ADDR_TX(3)  = EP3_TX_ADDR;
    BDT_COUNT_TX(3) = 0;
    USB->EP3R = 0x0600U | 3U;                    // INTERRUPT, address 3
    ep_set_stat_tx(3, STAT_NAK);
    ep_set_stat_rx(3, STAT_DISABLED);

    USB->DADDR = USB_DADDR_EF;                   // enabled, address 0

    s_configured = 0;
    s_tx_busy    = 0;
    s_rx_head    = 0;
    s_rx_tail    = 0;
}

void usb_cdc_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USBEN;

    // Force a bus reset so a host that already enumerated this device
    // re-runs enumeration after a firmware reset. Without this the port
    // often stays stale until the cable is unplugged.
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    GPIOA->CRH &= ~(0xFU << 16);       // PA12 as output
    GPIOA->CRH |=  (0x2U << 16);
    GPIOA->BSRR = (1U << (12U + 16U)); // drive D+ low
    for (volatile uint32_t d = 0; d < 200000U; d++) {
        IWDG->KR = IWDG_REFRESH_KEY;
    }
    GPIOA->CRH &= ~(0xFU << 16);       // release back to the peripheral

    USB->CNTR = USB_CNTR_FRES;         // hold in reset
    for (volatile uint32_t d = 0; d < 1000U; d++) { }
    USB->CNTR = 0;                     // release
    USB->ISTR = 0;

    usb_cdc_reset_endpoints();

    USB->CNTR = USB_CNTR_RESETM | USB_CNTR_CTRM | USB_CNTR_SUSPM
              | USB_CNTR_WKUPM | USB_CNTR_ERRM;

    // Above SysTick so a control transfer is never delayed by the
    // scheduler, below nothing else since no other interrupt is used
    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 1);
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}

uint8_t usb_cdc_ready(void)
{
    return (uint8_t)(s_configured && s_dtr);
}

uint8_t usb_cdc_tx_busy(void)
{
    return s_tx_busy;
}

// Queues one packet. Returns 0 when the previous one is still in flight,
// which the caller should treat as backpressure rather than an error.
uint8_t usb_cdc_write(const uint8_t *data, uint16_t len)
{
    if (!s_configured || s_tx_busy) {
        return 0;
    }
    if (len > EP_MAX_PACKET) {
        len = EP_MAX_PACKET;
    }

    s_tx_busy = 1;
    pma_write(EP1_TX_ADDR, data, len);
    BDT_COUNT_TX(1) = len;
    ep_set_stat_tx(1, STAT_VALID);
    return 1;
}

uint16_t usb_cdc_read(uint8_t *dst, uint16_t max)
{
    uint16_t n = 0;
    while (n < max && s_rx_tail != s_rx_head) {
        dst[n++]  = s_rx_buf[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) % USB_CDC_RX_BUF);
    }
    return n;
}

uint16_t usb_cdc_available(void)
{
    return (uint16_t)((s_rx_head + USB_CDC_RX_BUF - s_rx_tail)
                      % USB_CDC_RX_BUF);
}
