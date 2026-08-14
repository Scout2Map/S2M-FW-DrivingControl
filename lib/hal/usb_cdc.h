/*
 * File   : usb_cdc.h
 * Purpose: USB CDC-ACM device interface. Presents as /dev/ttyACM* on the
 *          RPi5 with no host side driver needed.
 * Author : jihoonkimtech
 */

#ifndef USB_CDC_H
#define USB_CDC_H

#include <stdint.h>

#define USB_CDC_RX_BUF  256U

void     usb_cdc_init(void);
void     usb_cdc_reset_endpoints(void);

// True once the host has both configured the device and asserted DTR,
// meaning a program actually has the port open.
uint8_t  usb_cdc_ready(void);

uint8_t  usb_cdc_tx_busy(void);

// Queues up to 64 bytes. Returns 0 if the previous packet is still in
// flight; treat that as backpressure, not an error.
uint8_t  usb_cdc_write(const uint8_t *data, uint16_t len);

uint16_t usb_cdc_read(uint8_t *dst, uint16_t max);
uint16_t usb_cdc_available(void);

#endif
