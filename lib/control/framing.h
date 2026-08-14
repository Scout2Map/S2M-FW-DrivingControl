/*
 * File   : framing.h
 * Purpose: Frame encoder and incremental decoder for the binary link.
 * Author : jihoonkimtech
 */

#ifndef FRAMING_H
#define FRAMING_H

#include <stdint.h>
#include "protocol.h"

// Builds a complete frame into out, returns the total byte count.
// out must hold at least PROTO_MAX_FRAME bytes.
uint16_t frame_encode(uint8_t *out, uint8_t type,
                      const void *payload, uint8_t len);

typedef enum {
    FRAME_HUNT_SYNC0 = 0,
    FRAME_HUNT_SYNC1,
    FRAME_READ_TYPE,
    FRAME_READ_LEN,
    FRAME_READ_PAYLOAD,
    FRAME_READ_CRC_HI,
    FRAME_READ_CRC_LO
} frame_state_t;

typedef struct {
    frame_state_t state;
    uint8_t       type;
    uint8_t       len;
    uint8_t       idx;
    uint8_t       payload[PROTO_MAX_PAYLOAD];
    uint16_t      crc_rx;
    // Diagnostics, surfaced so a flaky cable is visible rather than silent
    uint32_t      frames_ok;
    uint32_t      crc_errors;
    uint32_t      resyncs;
} frame_decoder_t;

void    frame_decoder_init(frame_decoder_t *d);

// Feeds one byte. Returns 1 when a valid frame is complete, in which
// case type, len and payload hold it until the next call.
uint8_t frame_decode_byte(frame_decoder_t *d, uint8_t b);

#endif
