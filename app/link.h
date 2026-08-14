/*
 * File   : link.h
 * Purpose: USB command and telemetry link interface.
 * Author : jihoonkimtech
 */

#ifndef LINK_H
#define LINK_H

#include <stdint.h>

void     link_init(void);
void     link_poll_rx(void);        // call often, drains the USB ring
void     link_send_telemetry(void); // call at TELEM_PERIOD_MS

// Diagnostics, useful when a cable or a host is misbehaving
uint32_t link_frames_tx(void);
uint32_t link_frames_dropped(void);
uint32_t link_crc_errors(void);

#endif
