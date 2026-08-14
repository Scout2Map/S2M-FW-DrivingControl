/*
 * File   : protocol.c
 * Purpose: CRC and frame encode/decode for the binary link. Contains no
 *          hardware access so it compiles and runs on the host.
 * Author : jihoonkimtech
 *
 * The decoder is a byte at a time state machine. USB CDC delivers data
 * in arbitrary chunks, so it must tolerate a frame arriving split
 * across packets or several frames arriving in one.
 *
 * Note: resynchronisation matters more than raw throughput here. After
 * a bad CRC the decoder rewinds to hunting for the sync pattern rather
 * than discarding a fixed number of bytes, so a corrupted length field
 * cannot desynchronise the stream permanently.
 */

#include <string.h>
#include "protocol.h"

// CRC16-CCITT, computed bitwise. A table would cost 512 bytes of flash
// to save time we do not need at 50 frames per second.
uint16_t proto_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x8000U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}
