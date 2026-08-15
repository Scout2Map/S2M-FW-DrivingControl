#!/usr/bin/env python3
"""
File   : s2m_imu_view.py
Purpose: Live terminal view of the BNO055 for calibration and sanity
         checking. Draws heading, attitude and calibration progress, and
         measures the gyro zero bias while the robot sits still.
Author : jihoonkimtech

Usage
    ./s2m_imu_view.py                   live view
    ./s2m_imu_view.py --bias            measure the gyro zero offset
    ./s2m_imu_view.py --log imu.csv     record while viewing

Note: uses plain ANSI escapes rather than curses so the output stays
readable when piped or when the terminal is small.

Note: the calibration bars are the point of this tool. The BNO055 will
happily report an orientation long before its magnetometer is calibrated,
and that orientation drifts. Watching the bars fill is the only reliable
way to know the heading can be trusted.
"""

import argparse
import math
import struct
import sys
import time
from collections import deque

try:
    import serial
except ImportError:
    sys.exit("pyserial missing: sudo apt install python3-serial")

SYNC0, SYNC1 = 0xAA, 0x55
HEADER_LEN, CRC_LEN = 4, 2

MSG_CMD_PING  = 0x06
MSG_TELEMETRY = 0x81
MSG_PONG      = 0x86
MSG_BOOT_INFO = 0x87
MSG_DIAG      = 0x88
MSG_I2C_SCAN  = 0x89

TELEMETRY_FMT = "<IiihhiiihhhhhHHhhHBB"

KNOWN_TYPES = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
               MSG_TELEMETRY, MSG_PONG, MSG_BOOT_INFO, MSG_DIAG, MSG_I2C_SCAN}

CSI = "\033["
CLEAR = f"{CSI}2J{CSI}H"
HIDE = f"{CSI}?25l"
SHOW = f"{CSI}?25h"


def crc16(data: bytes) -> int:
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
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)
        out = []
        while True:
            start = self.buf.find(bytes([SYNC0, SYNC1]))
            if start < 0:
                del self.buf[:max(0, len(self.buf) - 1)]
                return out
            del self.buf[:start]
            if len(self.buf) < HEADER_LEN:
                return out
            mtype, mlen = self.buf[2], self.buf[3]
            if mtype not in KNOWN_TYPES or mlen > 56:
                del self.buf[:2]
                continue
            total = HEADER_LEN + mlen + CRC_LEN
            if len(self.buf) < total:
                return out
            body = bytes(self.buf[2:HEADER_LEN + mlen])
            rx = struct.unpack(">H", bytes(self.buf[HEADER_LEN + mlen:total]))[0]
            if crc16(body) == rx:
                out.append((mtype, bytes(self.buf[HEADER_LEN:HEADER_LEN + mlen])))
                del self.buf[:total]
            else:
                del self.buf[:2]


def parse_telemetry(p: bytes) -> dict:
    f = struct.unpack(TELEMETRY_FMT, p)
    return dict(zip((
        "t_ms", "enc_l", "enc_r", "spd_l", "spd_r",
        "x_mm", "y_mm", "th_mrad",
        "qw", "qx", "qy", "qz", "gyro_z",
        "dist_mm", "batt_mv", "duty_l", "duty_r", "status",
        "imu_calib", "reserved"), f))


def quat_to_euler(t: dict):
    """Returns roll, pitch, yaw in degrees. Zero when the quaternion is
    still all zeros, which is what a pre-init read looks like."""
    s = 1.0 / 16384.0
    w, x, y, z = t["qw"]*s, t["qx"]*s, t["qy"]*s, t["qz"]*s
    if w == 0 and x == 0 and y == 0 and z == 0:
        return 0.0, 0.0, 0.0

    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    # Clamped because rounding can push the argument outside asin's domain
    sp = max(-1.0, min(1.0, 2*(w*y - z*x)))
    pitch = math.asin(sp)
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def compass(yaw_deg: float, width: int = 60) -> str:
    """A strip showing where the heading sits against the cardinals."""
    y = yaw_deg % 360.0
    marks = {0: "N", 45: "NE", 90: "E", 135: "SE",
             180: "S", 225: "SW", 270: "W", 315: "NW"}
    row = [" "] * width
    for deg, label in marks.items():
        # Centre the strip on the current heading, +-180 across the width
        rel = ((deg - y + 180.0) % 360.0) - 180.0
        pos = int((rel + 180.0) / 360.0 * (width - 1))
        for i, ch in enumerate(label):
            if 0 <= pos + i < width:
                row[pos + i] = ch
    line = "".join(row)
    # The strip is centred on the current heading, so the marker belongs
    # at the midpoint of the same span the labels were placed across
    centre = (width - 1) // 2
    return line + "\n" + " " * centre + "^"


def bar(value: int, maximum: int = 3, width: int = 12) -> str:
    filled = int(round(value / maximum * width))
    return "[" + "#" * filled + "." * (width - filled) + "]"


def tilt_gauge(roll: float, pitch: float, width: int = 21) -> list:
    """Small cross hair showing tilt, saturating at 45 degrees."""
    half = width // 2
    rows = []
    height = 7
    cx = max(0, min(width - 1, half + int(roll / 45.0 * half)))
    cy = max(0, min(height - 1, height // 2 + int(pitch / 45.0 * (height // 2))))
    for y in range(height):
        line = []
        for x in range(width):
            at_centre = (x == half and y == height // 2)
            if x == cx and y == cy:
                # Show level explicitly rather than hiding the reference
                line.append("*" if at_centre else "O")
            elif at_centre:
                line.append("+")
            elif y == height // 2:
                line.append("-")
            elif x == half:
                line.append("|")
            else:
                line.append(" ")
        rows.append("".join(line))
    return rows


def status_flags(s: int) -> list:
    names = [(0, "ENABLED"), (1, "OPENLOOP"), (2, "STALL"), (3, "CMD_TIMEOUT"),
             (4, "ESTOP"), (5, "IMU_OK"), (6, "BATT_WARN"), (7, "BATT_CRIT"),
             (8, "IMU_CAL")]
    return [n for bit, n in names if s & (1 << bit)]


def render(t: dict, gz_hist: deque, bias: float, frames: int):
    roll, pitch, yaw = quat_to_euler(t)
    cal = t["imu_calib"]
    sysc, gyr, acc, mag = cal >> 6, (cal >> 4) & 3, (cal >> 2) & 3, cal & 3
    gz = t["gyro_z"] / 16.0

    out = [CLEAR]
    out.append("  Scout2Map  BNO055 live view          "
               f"frames {frames}   t={t['t_ms']/1000:.1f}s")
    out.append("  " + "-" * 66)
    out.append("")

    out.append(f"  heading   {yaw:8.2f} deg")
    out.append("  " + compass(yaw).replace("\n", "\n  "))
    out.append("")

    gauge = tilt_gauge(roll, pitch)
    out.append(f"  roll {roll:7.2f}      pitch {pitch:7.2f}")
    for row in gauge:
        out.append("     " + row)
    out.append("")

    out.append("  calibration        0 = none, 3 = full")
    out.append(f"    system   {bar(sysc)}  {sysc}")
    out.append(f"    gyro     {bar(gyr)}  {gyr}")
    out.append(f"    accel    {bar(acc)}  {acc}")
    out.append(f"    magneto  {bar(mag)}  {mag}")
    out.append("")

    corrected = gz - bias
    out.append(f"  yaw rate  {gz:8.2f} deg/s"
               + (f"   corrected {corrected:7.2f}" if bias else ""))
    if gz_hist:
        lo, hi = min(gz_hist), max(gz_hist)
        out.append(f"  recent    min {lo:7.2f}  max {hi:7.2f}  "
                   f"spread {hi-lo:6.2f}")
    out.append("")

    flags = status_flags(t["status"])
    out.append("  status    " + (" ".join(flags) if flags else "-"))
    out.append("")

    if mag < 3:
        out.append("  magnetometer not calibrated: move the chassis in a")
        out.append("  figure of eight until the bar fills. Heading drifts")
        out.append("  until then.")
    elif gyr < 3:
        out.append("  gyro not calibrated: hold the chassis still.")
    elif acc < 3:
        out.append("  accelerometer not calibrated: rest the chassis on")
        out.append("  each of its six faces in turn.")
    else:
        out.append("  fully calibrated.")

    out.append("")
    out.append("  Ctrl-C to exit")
    sys.stdout.write("\n".join(out))
    sys.stdout.flush()


def measure_bias(ser, seconds: float) -> float:
    """Averages the yaw rate while stationary. A non zero result is the
    zero offset that slip detection would otherwise mistake for turning."""
    print(f"measuring gyro bias, keep the robot completely still "
          f"for {seconds:.0f}s...")
    dec = Decoder()
    samples = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        for mtype, payload in dec.feed(ser.read(256)):
            if mtype == MSG_TELEMETRY:
                samples.append(parse_telemetry(payload)["gyro_z"] / 16.0)
        left = seconds - (time.time() - t0)
        print(f"\r  {left:4.1f}s remaining, {len(samples)} samples",
              end="", flush=True)
    print()

    if not samples:
        print("no telemetry received")
        return 0.0

    mean = sum(samples) / len(samples)
    var = sum((s - mean) ** 2 for s in samples) / len(samples)
    sd = math.sqrt(var)
    print(f"  bias      {mean:+.3f} deg/s")
    print(f"  noise sd  {sd:.3f} deg/s")
    print(f"  drift     {mean*60:+.1f} deg per minute if uncorrected")
    if abs(mean) < 0.1:
        print("  negligible, no correction needed")
    else:
        print("  subtract this from gyro_z before using it for slip detection")
    return mean


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default="/dev/ttyACM0")
    ap.add_argument("--bias", action="store_true",
                    help="measure the gyro zero offset while stationary")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--log", metavar="FILE", help="record samples as CSV")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, 115200, timeout=0.05)
    except serial.SerialException as e:
        sys.exit(f"cannot open {args.port}: {e}")

    if args.bias:
        try:
            measure_bias(ser, args.seconds)
        finally:
            ser.close()
        return

    log = open(args.log, "w") if args.log else None
    if log:
        log.write("t_ms,roll,pitch,yaw,gyro_z,calib_sys,calib_gyr,"
                  "calib_acc,calib_mag\n")

    dec = Decoder()
    gz_hist = deque(maxlen=100)
    frames = 0
    last_draw = 0.0

    sys.stdout.write(HIDE)
    try:
        while True:
            for mtype, payload in dec.feed(ser.read(256)):
                if mtype != MSG_TELEMETRY:
                    continue
                t = parse_telemetry(payload)
                frames += 1
                gz_hist.append(t["gyro_z"] / 16.0)

                if log:
                    r, p, y = quat_to_euler(t)
                    c = t["imu_calib"]
                    log.write(f"{t['t_ms']},{r:.3f},{p:.3f},{y:.3f},"
                              f"{t['gyro_z']/16.0:.3f},"
                              f"{c>>6},{(c>>4)&3},{(c>>2)&3},{c&3}\n")

                # Redraw at 10Hz; telemetry arrives at 50Hz and the
                # terminal cannot keep up with that without flicker
                now = time.time()
                if now - last_draw > 0.1:
                    render(t, gz_hist, 0.0, frames)
                    last_draw = now
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write(SHOW + "\n")
        if log:
            log.close()
            print(f"wrote {args.log}")
        ser.close()


if __name__ == "__main__":
    main()
