/*
 * File   : test_framing.c
 * Purpose: Host tests for the wire format. Verifies CRC against known
 *          vectors, round trips every message type, and exercises the
 *          decoder against the failure modes a real serial link produces.
 * Author : jihoonkimtech
 *
 * Note: the split delivery and garbage resync cases matter most. USB CDC
 * hands over arbitrary chunk boundaries, and a decoder that only works
 * on whole frames will appear fine on the bench and fail in the field.
 */

#include <stdio.h>
#include <string.h>
#include "framing.h"
#include "board_config.h"

static int fails;

static void check(const char *name, int cond)
{
    printf("%-50s %s\n", name, cond ? "PASS" : "FAIL");
    if (!cond) fails++;
}

static uint8_t feed_all(frame_decoder_t *d, const uint8_t *buf, uint16_t n)
{
    uint8_t got = 0;
    for (uint16_t i = 0; i < n; i++) {
        if (frame_decode_byte(d, buf[i])) got++;
    }
    return got;
}

int main(void)
{
    uint8_t buf[PROTO_MAX_FRAME];
    uint8_t buf2[PROTO_MAX_FRAME];
    frame_decoder_t dec;

    printf("\n=== Scout2Map protocol, host tests ===\n\n");

    // --- CRC against the canonical CCITT vector ---
    // "123456789" under CRC16-CCITT-FALSE is 0x29B1
    check("CRC16 matches the standard check vector",
          proto_crc16((const uint8_t *)"123456789", 9) == 0x29B1);

    // --- velocity command round trip ---
    cmd_velocity_t cmd = { .linear_mmps = 200, .angular_mradps = -800 };
    uint16_t n = frame_encode(buf, MSG_CMD_VELOCITY, &cmd, sizeof cmd);
    check("velocity frame is 10 bytes", n == 10);

    frame_decoder_init(&dec);
    check("velocity frame decodes", feed_all(&dec, buf, n) == 1);
    check("decoded type is preserved", dec.type == MSG_CMD_VELOCITY);
    {
        cmd_velocity_t out;
        memcpy(&out, dec.payload, sizeof out);
        check("signed fields survive the round trip",
              out.linear_mmps == 200 && out.angular_mradps == -800);
    }

    // --- telemetry round trip, the largest frame ---
    telemetry_t tel;
    memset(&tel, 0, sizeof tel);
    tel.timestamp_ms  = 123456;
    tel.enc_left      = -5764;
    tel.enc_right     =  5764;
    tel.odom_x_mm     =  2073;
    tel.odom_theta_mrad = -3141;
    tel.status        = STATUS_MOTOR_ENABLED | STATUS_IMU_OK;

    n = frame_encode(buf, MSG_TELEMETRY, &tel, sizeof tel);
    printf("   telemetry payload %u bytes, frame %u bytes\n",
           (unsigned)sizeof(telemetry_t), n);
    check("telemetry frame fits one 64 byte bulk transfer", n <= 64);

    frame_decoder_init(&dec);
    check("telemetry frame decodes", feed_all(&dec, buf, n) == 1);
    {
        telemetry_t out;
        memcpy(&out, dec.payload, sizeof out);
        check("negative 32 bit fields survive",
              out.enc_left == -5764 && out.odom_theta_mrad == -3141);
        check("status bitfield survives", out.status == tel.status);
    }

    // --- split delivery, one byte at a time across a boundary ---
    frame_decoder_init(&dec);
    feed_all(&dec, buf, 5);
    check("partial frame yields nothing yet", dec.frames_ok == 0);
    feed_all(&dec, buf + 5, n - 5);
    check("frame completes across a split", dec.frames_ok == 1);

    // --- back to back frames in one chunk ---
    n = frame_encode(buf, MSG_CMD_VELOCITY, &cmd, sizeof cmd);
    memcpy(buf2, buf, n);
    memcpy(buf2 + n, buf, n);
    frame_decoder_init(&dec);
    check("two frames in one chunk both decode",
          feed_all(&dec, buf2, (uint16_t)(n * 2)) == 2);

    // --- corrupted payload is rejected ---
    frame_decoder_init(&dec);
    memcpy(buf2, buf, n);
    buf2[5] ^= 0xFF;
    feed_all(&dec, buf2, n);
    check("corrupted payload fails CRC", dec.frames_ok == 0 && dec.crc_errors == 1);

    // --- recovery after corruption ---
    check("decoder recovers on the next good frame",
          feed_all(&dec, buf, n) == 1);

    // --- garbage before a real frame ---
    frame_decoder_init(&dec);
    uint8_t noise[] = { 0x00, 0xAA, 0xAA, 0x12, 0xFF, 0xAA, 0x55, 0x99 };
    feed_all(&dec, noise, sizeof noise);
    check("noise does not produce a phantom frame", dec.frames_ok == 0);
    check("decoder resynchronises after noise", feed_all(&dec, buf, n) == 1);

    // --- impossible length is rejected without consuming the stream ---
    frame_decoder_init(&dec);
    uint8_t badlen[] = { PROTO_SYNC0, PROTO_SYNC1, MSG_TELEMETRY, 0xFF };
    feed_all(&dec, badlen, sizeof badlen);
    check("oversized length triggers a resync", dec.resyncs == 1);
    check("stream still usable after a bad length",
          feed_all(&dec, buf, n) == 1);

    // --- zero length frame ---
    n = frame_encode(buf, MSG_CMD_ESTOP, 0, 0);
    frame_decoder_init(&dec);
    check("zero length frame decodes", feed_all(&dec, buf, n) == 1);
    check("zero length frame keeps its type", dec.type == MSG_CMD_ESTOP);

    // --- boot info carries the odometry scale ---
    boot_info_t bi = {
        .proto_version = PROTO_VERSION,
        .fw_major = 0, .fw_minor = 2, .fw_patch = 0,
        .counts_per_wheel_rev = COUNTS_PER_WHEEL_REV,
        .wheel_base_mm = (uint16_t)WHEEL_BASE_MM,
    };
    n = frame_encode(buf, MSG_BOOT_INFO, &bi, sizeof bi);
    frame_decoder_init(&dec);
    feed_all(&dec, buf, n);
    {
        boot_info_t out;
        memcpy(&out, dec.payload, sizeof out);
        printf("   boot info advertises %u counts/rev, %u mm base\n",
               out.counts_per_wheel_rev, out.wheel_base_mm);
        check("boot info advertises the verified resolution",
              out.counts_per_wheel_rev == 5764);
    }

    printf("\n%s (%d failure%s)\n\n",
           fails ? "SOME TESTS FAILED" : "ALL TESTS PASSED",
           fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
