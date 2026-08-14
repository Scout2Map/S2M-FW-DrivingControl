/*
 * File   : framing.c
 * Purpose: Frame encoder and byte-at-a-time decoder for the binary link.
 *          Hardware independent, unit tested on the host.
 * Author : jihoonkimtech
 *
 * Note: on a CRC mismatch the decoder returns to hunting for the sync
 * pattern instead of assuming the next byte starts a frame. A corrupted
 * length byte would otherwise consume real frame data and leave the
 * stream permanently misaligned.
 */

#include <string.h>
#include "framing.h"

// Only these appear on the wire. Anything else means the sync pattern
// was noise rather than a frame boundary.
static uint8_t frame_type_known(uint8_t type)
{
    switch (type) {
    case MSG_CMD_VELOCITY:
    case MSG_CMD_WHEEL_RAW:
    case MSG_CMD_ESTOP:
    case MSG_CMD_RESET_ODOM:
    case MSG_CMD_CLEAR_FAULT:
    case MSG_CMD_PING:
    case MSG_TELEMETRY:
    case MSG_PONG:
    case MSG_BOOT_INFO:
        return 1;
    default:
        return 0;
    }
}

uint16_t frame_encode(uint8_t *out, uint8_t type,
                      const void *payload, uint8_t len)
{
    if (len > PROTO_MAX_PAYLOAD) {
        return 0;
    }

    out[0] = PROTO_SYNC0;
    out[1] = PROTO_SYNC1;
    out[2] = type;
    out[3] = len;

    if (payload != 0 && len > 0) {
        memcpy(&out[PROTO_HEADER_LEN], payload, len);
    }

    // CRC covers type, len and payload, but not the sync bytes.
    // Including sync would add nothing since they are constant.
    uint16_t crc = proto_crc16(&out[2], (uint16_t)(len + 2U));

    out[PROTO_HEADER_LEN + len]      = (uint8_t)(crc >> 8);
    out[PROTO_HEADER_LEN + len + 1U] = (uint8_t)(crc & 0xFFU);

    return (uint16_t)(PROTO_HEADER_LEN + len + PROTO_CRC_LEN);
}

void frame_decoder_init(frame_decoder_t *d)
{
    memset(d, 0, sizeof(*d));
    d->state = FRAME_HUNT_SYNC0;
}

uint8_t frame_decode_byte(frame_decoder_t *d, uint8_t b)
{
    switch (d->state) {
    case FRAME_HUNT_SYNC0:
        if (b == PROTO_SYNC0) {
            d->state = FRAME_HUNT_SYNC1;
        }
        break;

    case FRAME_HUNT_SYNC1:
        if (b == PROTO_SYNC1) {
            d->state = FRAME_READ_TYPE;
        } else if (b == PROTO_SYNC0) {
            // Stay here, a repeated 0xAA could still precede 0x55
        } else {
            d->state = FRAME_HUNT_SYNC0;
        }
        break;

    case FRAME_READ_TYPE:
        // Reject unknown types immediately. Random noise containing the
        // sync pattern would otherwise open a phantom frame that then
        // swallows the next genuine one, costing a real frame rather
        // than just the noise. Validating here keeps the loss local.
        if (!frame_type_known(b)) {
            d->resyncs++;
            // The rejected byte may itself begin a real sync pattern
            d->state = (b == PROTO_SYNC0) ? FRAME_HUNT_SYNC1
                                          : FRAME_HUNT_SYNC0;
            break;
        }
        d->type  = b;
        d->state = FRAME_READ_LEN;
        break;

    case FRAME_READ_LEN:
        if (b > PROTO_MAX_PAYLOAD) {
            // Impossible length, the sync pattern was noise
            d->resyncs++;
            d->state = (b == PROTO_SYNC0) ? FRAME_HUNT_SYNC1
                                          : FRAME_HUNT_SYNC0;
            break;
        }
        d->len   = b;
        d->idx   = 0;
        d->state = (b == 0U) ? FRAME_READ_CRC_HI : FRAME_READ_PAYLOAD;
        break;

    case FRAME_READ_PAYLOAD:
        d->payload[d->idx++] = b;
        if (d->idx >= d->len) {
            d->state = FRAME_READ_CRC_HI;
        }
        break;

    case FRAME_READ_CRC_HI:
        d->crc_rx = (uint16_t)b << 8;
        d->state  = FRAME_READ_CRC_LO;
        break;

    case FRAME_READ_CRC_LO: {
        d->crc_rx |= b;
        d->state   = FRAME_HUNT_SYNC0;

        // Rebuild the covered span and use the shared CRC routine.
        // Duplicating the polynomial loop here would risk the two copies
        // drifting apart, which is exactly the kind of bug that only
        // shows up as intermittent frame loss on real hardware.
        uint8_t buf[PROTO_MAX_PAYLOAD + 2U];
        buf[0] = d->type;
        buf[1] = d->len;
        memcpy(&buf[2], d->payload, d->len);

        uint16_t crc = proto_crc16(buf, (uint16_t)(d->len + 2U));

        if (crc == d->crc_rx) {
            d->frames_ok++;
            return 1;
        }
        d->crc_errors++;
        break;
    }

    default:
        d->state = FRAME_HUNT_SYNC0;
        break;
    }

    return 0;
}
