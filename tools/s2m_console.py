#!/usr/bin/env python3
"""
File   : s2m_console.py
Purpose: Host side console for the Scout2Map drive MCU. Decodes telemetry
         and issues commands over the USB CDC link, so the firmware can be
         exercised before the ROS2 bridge node exists.
Author : jihoonkimtech

Usage
    ./s2m_console.py                    live telemetry view
    ./s2m_console.py --raw 300 300      spin both wheels at 30 percent
    ./s2m_console.py --vel 100 0        drive at 100 mm/s straight
    ./s2m_console.py --step 150         step response capture for PID work
    ./s2m_console.py --imu              add heading and yaw rate columns
    ./s2m_console.py --diag             IMU and I2C bring-up diagnostics
    ./s2m_console.py --scan             list every device on the I2C bus
    ./s2m_console.py --estop            latch an emergency stop

Note: the frame layout here mirrors lib/control/protocol.h. Change one
and the other must follow, which is why the struct format strings are
kept adjacent to the field names rather than inlined.
"""

import argparse
import math
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing: pip install pyserial")

SYNC0, SYNC1 = 0xAA, 0x55
HEADER_LEN, CRC_LEN = 4, 2

MSG_CMD_VELOCITY    = 0x01
MSG_CMD_WHEEL_RAW   = 0x02
MSG_CMD_ESTOP       = 0x03
MSG_CMD_RESET_ODOM  = 0x04
MSG_CMD_CLEAR_FAULT = 0x05
MSG_CMD_PING        = 0x06
MSG_CMD_DIAG        = 0x07
MSG_CMD_I2C_SCAN    = 0x08
MSG_TELEMETRY       = 0x81
MSG_PONG            = 0x86
MSG_BOOT_INFO       = 0x87
MSG_DIAG            = 0x88
MSG_I2C_SCAN        = 0x89

STATUS_BITS = [
    (1 << 0, "ENABLED"),
    (1 << 1, "OPENLOOP"),
    (1 << 2, "STALL"),
    (1 << 3, "CMD_TIMEOUT"),
    (1 << 4, "ESTOP"),
    (1 << 5, "IMU_OK"),
    (1 << 6, "BATT_WARN"),
    (1 << 7, "BATT_CRIT"),
    (1 << 8, "IMU_CAL"),
]

TELEMETRY_FMT = "<IiihhiiihhhhhHHhhHBB"
BOOT_INFO_FMT = "<BBBBHH"
DIAG_FMT      = "<BBBBIIHH"

# Addresses worth naming when they turn up on this robot's bus
KNOWN_I2C = {
    0x28: "BNO055 (default)",
    0x29: "BNO055 (ADR high)",
    0x23: "BH1750",
    0x38: "AHT21",
    0x53: "ENS160",
    0x5C: "BH1750 (alt)",
    0x68: "MPU6050 / DS3231",
    0x76: "BMP280",
    0x77: "BMP280 (alt)",
}


def crc16(data: bytes) -> int:
    """CRC16-CCITT, must match proto_crc16 in the firmware bit for bit."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(msg_type: int, payload: bytes = b"") -> bytes:
    body = bytes([msg_type, len(payload)]) + payload
    return bytes([SYNC0, SYNC1]) + body + struct.pack(">H", crc16(body))


class Decoder:
    """Mirrors the firmware state machine, including type validation."""

    KNOWN = {MSG_CMD_VELOCITY, MSG_CMD_WHEEL_RAW, MSG_CMD_ESTOP,
             MSG_CMD_RESET_ODOM, MSG_CMD_CLEAR_FAULT, MSG_CMD_PING,
             MSG_TELEMETRY, MSG_PONG, MSG_BOOT_INFO, MSG_CMD_DIAG,
             MSG_DIAG, MSG_CMD_I2C_SCAN, MSG_I2C_SCAN}

    def __init__(self):
        self.buf = bytearray()
        self.ok = 0
        self.crc_err = 0

    def feed(self, data: bytes):
        self.buf.extend(data)
        out = []
        while True:
            start = self.buf.find(bytes([SYNC0, SYNC1]))
            if start < 0:
                # Keep one byte in case a sync pattern straddles the boundary
                del self.buf[:max(0, len(self.buf) - 1)]
                return out
            del self.buf[:start]
            if len(self.buf) < HEADER_LEN:
                return out

            mtype, mlen = self.buf[2], self.buf[3]
            if mtype not in self.KNOWN or mlen > 56:
                del self.buf[:2]
                continue

            total = HEADER_LEN + mlen + CRC_LEN
            if len(self.buf) < total:
                return out

            body = bytes(self.buf[2:HEADER_LEN + mlen])
            rx = struct.unpack(">H", bytes(self.buf[HEADER_LEN + mlen:total]))[0]
            if crc16(body) == rx:
                out.append((mtype, bytes(self.buf[HEADER_LEN:HEADER_LEN + mlen])))
                self.ok += 1
                del self.buf[:total]
            else:
                self.crc_err += 1
                del self.buf[:2]


def parse_telemetry(p: bytes) -> dict:
    f = struct.unpack(TELEMETRY_FMT, p)
    return dict(zip((
        "t_ms", "enc_l", "enc_r", "spd_l", "spd_r",
        "x_mm", "y_mm", "th_mrad",
        "qw", "qx", "qy", "qz", "gyro_z",
        "dist_mm", "batt_mv", "duty_l", "duty_r", "status",
        "imu_calib", "reserved"), f))


def status_text(s: int) -> str:
    names = [n for bit, n in STATUS_BITS if s & bit]
    return "|".join(names) if names else "-"


def open_port(dev: str) -> serial.Serial:
    return serial.Serial(dev, 115200, timeout=0.05)


def cmd_velocity(mmps: int, mradps: int) -> bytes:
    return encode(MSG_CMD_VELOCITY, struct.pack("<hh", mmps, mradps))


def cmd_raw(left: int, right: int) -> bytes:
    return encode(MSG_CMD_WHEEL_RAW, struct.pack("<hh", left, right))


def quat_to_yaw_deg(t: dict) -> float:
    """Heading from the BNO055 quaternion, which arrives at 1/16384."""
    s = 1.0 / 16384.0
    w, x, y, z = t["qw"]*s, t["qx"]*s, t["qy"]*s, t["qz"]*s
    if w == 0 and x == 0 and y == 0 and z == 0:
        return 0.0
    siny = 2.0*(w*z + x*y)
    cosy = 1.0 - 2.0*(y*y + z*z)
    return math.degrees(math.atan2(siny, cosy))


def run_monitor(ser, sender=None, duration=None, csv=False, show_imu=False):
    """Streams telemetry. sender is called every 100ms to republish a
    command, which the firmware requires or its timeout trips."""
    dec = Decoder()
    start = time.time()
    last_cmd = 0.0
    printed_header = False

    while duration is None or time.time() - start < duration:
        now = time.time()
        if sender and now - last_cmd > 0.1:
            ser.write(sender())
            last_cmd = now

        data = ser.read(256)
        if not data:
            continue

        for mtype, payload in dec.feed(data):
            if mtype == MSG_BOOT_INFO:
                v = struct.unpack(BOOT_INFO_FMT, payload)
                print(f"[boot] proto v{v[0]}  fw {v[1]}.{v[2]}.{v[3]}  "
                      f"{v[4]} counts/rev  {v[5]} mm base")
            elif mtype == MSG_PONG:
                print("[pong]")
            elif mtype == MSG_I2C_SCAN:
                count = payload[0]
                lines = payload[1]
                bits = payload[2:18]
                found = [a for a in range(128)
                         if bits[a >> 3] & (1 << (a & 7))]
                print(f"[i2c scan] {count} device(s) responded")
                if lines != 0x03:
                    print("  BUS LINES NOT IDLE HIGH -- no scan is possible.")
                    print("  A line held low means one of:")
                    print("    - no pull up resistors on SCL/SDA")
                    print("    - the sensor board has no power")
                    print("    - SCL or SDA shorted to ground")
                elif not found:
                    print("  lines are healthy but nothing answered.")
                    print("  Check the sensor is powered and shares a ground.")
                for a in found:
                    name = KNOWN_I2C.get(a, "")
                    print(f"  0x{a:02X}" + (f"  {name}" if name else ""))
                if 0x29 in found and 0x28 not in found:
                    print("  -> set BNO055_ADDR to 0x29 in board_config.h")
            elif mtype == MSG_DIAG:
                d = struct.unpack(DIAG_FMT, payload)
                steps = ["WAIT_BOOT", "READ_ID", "CHECK_ID", "SET_CONFIG",
                         "WAIT_CONFIG", "SET_UNITS", "WAIT_UNITS",
                         "SET_POWER", "WAIT_POWER", "SET_NDOF",
                         "WAIT_NDOF", "COMPLETE", "FAILED"]
                step = steps[d[0]] if d[0] < len(steps) else f"?{d[0]}"
                cal = d[2]
                print(f"[diag] imu init stage : {step}")
                print(f"       chip id        : 0x{d[1]:02X} "
                      f"({'BNO055 ok' if d[1] == 0xA0 else 'expected 0xA0'})")
                print(f"       calib sys/g/a/m: {cal>>6}/{(cal>>4)&3}/"
                      f"{(cal>>2)&3}/{cal&3}")
                print(f"       reads ok/fail  : {d[4]} / {d[5]}")
                print(f"       i2c err/recover: {d[6]} / {d[7]}")
            elif mtype == MSG_TELEMETRY:
                t = parse_telemetry(payload)
                if csv:
                    if not printed_header:
                        print(",".join(t.keys()))
                        printed_header = True
                    print(",".join(str(v) for v in t.values()))
                else:
                    line = (f"t={t['t_ms']:>8}  "
                            f"enc L{t['enc_l']:>9} R{t['enc_r']:>9}  "
                            f"spd L{t['spd_l']:>5} R{t['spd_r']:>5} mm/s  "
                            f"duty L{t['duty_l']:>5} R{t['duty_r']:>5}  "
                            f"odom {t['x_mm']:>6},{t['y_mm']:>6} "
                            f"{t['th_mrad']:>6}mrad")
                    if show_imu:
                        yaw = quat_to_yaw_deg(t)
                        line += (f"  yaw {yaw:>7.1f}deg"
                                 f"  gz {t['gyro_z']/16.0:>7.1f}deg/s")
                    line += f"  [{status_text(t['status'])}]"
                    print(line)

    if dec.crc_err:
        print(f"\n{dec.ok} frames ok, {dec.crc_err} CRC errors", file=sys.stderr)


def run_step(ser, target_mmps: int, seconds: float):
    """Captures a step response as CSV, for PID tuning."""
    print("# step response, paste into a plot")
    print("t_ms,target_mmps,spd_l,spd_r,duty_l,duty_r")

    dec = Decoder()
    t0 = time.time()
    last = 0.0
    while time.time() - t0 < seconds:
        now = time.time()
        # Zero for the first 0.5s establishes the baseline, then step
        target = 0 if now - t0 < 0.5 else target_mmps
        if now - last > 0.05:
            ser.write(cmd_velocity(target, 0))
            last = now

        for mtype, payload in dec.feed(ser.read(256)):
            if mtype == MSG_TELEMETRY:
                t = parse_telemetry(payload)
                print(f"{t['t_ms']},{target},{t['spd_l']},{t['spd_r']},"
                      f"{t['duty_l']},{t['duty_r']}")

    ser.write(cmd_velocity(0, 0))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default="/dev/ttyACM0")
    ap.add_argument("--raw", nargs=2, type=int, metavar=("L", "R"),
                    help="direct duty in permille, bypasses the PID")
    ap.add_argument("--vel", nargs=2, type=int, metavar=("MMPS", "MRADPS"),
                    help="velocity command through the PID")
    ap.add_argument("--step", type=int, metavar="MMPS",
                    help="capture a step response as CSV")
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--estop", action="store_true")
    ap.add_argument("--clear", action="store_true", help="clear a latched fault")
    ap.add_argument("--reset-odom", action="store_true")
    ap.add_argument("--ping", action="store_true")
    ap.add_argument("--diag", action="store_true",
                    help="dump IMU and I2C bring-up diagnostics")
    ap.add_argument("--scan", action="store_true",
                    help="probe every address on the I2C bus")
    ap.add_argument("--csv", action="store_true")
    ap.add_argument("--imu", action="store_true",
                    help="show heading and yaw rate from the BNO055")
    args = ap.parse_args()

    try:
        ser = open_port(args.port)
    except serial.SerialException as e:
        sys.exit(f"cannot open {args.port}: {e}")

    try:
        if args.estop:
            ser.write(encode(MSG_CMD_ESTOP))
            print("estop latched, use --clear to release")
        elif args.clear:
            ser.write(encode(MSG_CMD_CLEAR_FAULT))
            print("fault cleared")
        elif args.reset_odom:
            ser.write(encode(MSG_CMD_RESET_ODOM))
            print("odometry zeroed")
        elif args.ping:
            ser.write(encode(MSG_CMD_PING))
            run_monitor(ser, duration=1.0)
        elif args.diag:
            ser.write(encode(MSG_CMD_DIAG))
            run_monitor(ser, duration=1.5)
        elif args.scan:
            ser.write(encode(MSG_CMD_I2C_SCAN))
            run_monitor(ser, duration=2.0)
        elif args.step is not None:
            run_step(ser, args.step, args.seconds)
        elif args.raw:
            l, r = args.raw
            run_monitor(ser, sender=lambda: cmd_raw(l, r),
                        duration=args.seconds, csv=args.csv,
                        show_imu=args.imu)
            ser.write(cmd_raw(0, 0))
        elif args.vel:
            v, w = args.vel
            run_monitor(ser, sender=lambda: cmd_velocity(v, w),
                        duration=args.seconds, csv=args.csv,
                        show_imu=args.imu)
            ser.write(cmd_velocity(0, 0))
        else:
            run_monitor(ser, csv=args.csv, show_imu=args.imu)
    except KeyboardInterrupt:
        # Always leave the robot stopped, whatever happened
        ser.write(cmd_velocity(0, 0))
        print("\nstopped")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
