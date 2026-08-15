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
    ./s2m_imu_view.py --accel           live gravity readout, tilt and watch
    ./s2m_imu_view.py --axes            work out the mounting orientation
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

TELEMETRY_FMT = "<IiihhiiihhhhhhhhHHhhHBB"

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


class VersionMismatch(Exception):
    """Payload size does not match what this tool expects."""


def parse_telemetry(p: bytes) -> dict:
    want = struct.calcsize(TELEMETRY_FMT)
    if len(p) != want:
        raise VersionMismatch(
            f"telemetry frame is {len(p)} bytes, this tool expects {want}.\n"
            f"  The board is running different firmware than this checkout.\n"
            f"  Rebuild and reflash:  make flash")
    f = struct.unpack(TELEMETRY_FMT, p)
    return dict(zip((
        "t_ms", "enc_l", "enc_r", "spd_l", "spd_r",
        "x_mm", "y_mm", "th_mrad",
        "qw", "qx", "qy", "qz", "gyro_z",
        "accel_x", "accel_y", "accel_z",
        "dist_mm", "batt_mv", "duty_l", "duty_r", "status",
        "imu_calib", "reserved"), f))


def accel_to_rp(t: dict):
    """Roll and pitch straight from gravity, in degrees.

    Independent of the quaternion and of whatever orientation convention
    the sensor is configured for, so disagreement between this and the
    fused attitude localises the fault to one or the other."""
    ax = t["accel_x"] / 100.0
    ay = t["accel_y"] / 100.0
    az = t["accel_z"] / 100.0
    if abs(ax) + abs(ay) + abs(az) < 1.0:
        return 0.0, 0.0
    # ROS convention: x forward, y left, z up
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
    return roll, pitch


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
    """Small cross hair showing tilt, saturating at 45 degrees.

    Viewed from above with the nose at the top of the screen. The marker
    always sits on the side that is physically lower, so lifting the
    nose sends it toward the bottom and dropping the left side sends it
    to the left."""
    half = width // 2
    rows = []
    height = 7
    # The marker sits on the side that is physically DOWN, so it tracks
    # where the chassis would slide. With ROS convention (x forward,
    # y left, z up) a positive roll lifts the left side, which puts the
    # right side down, hence the positive sign here.
    #
    # This deliberately does NOT compensate for a misaligned sensor. If
    # the marker moves the wrong way, the axis remap is wrong and the
    # SBC is receiving an equally wrong pose; fix it with --axes rather
    # than by flipping this line.
    cx = max(0, min(width - 1, half + int(roll / 45.0 * half)))
    # Screen up is the nose. Positive pitch tips the nose down, which
    # makes the nose the low side, so the marker rises. Negating here is
    # what keeps pitch consistent with roll: both put the marker on the
    # side that is physically lower.
    cy = max(0, min(height - 1,
                    height // 2 - int(pitch / 45.0 * (height // 2))))
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
             (8, "IMU_CAL"), (9, "BATT_DEAD")]
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

    a_roll, a_pitch = accel_to_rp(t)
    gauge = tilt_gauge(roll, pitch)
    out.append(f"  roll {roll:7.2f}      pitch {pitch:7.2f}   (fused)")
    out.append(f"       {a_roll:7.2f}            {a_pitch:7.2f}   (gravity)")
    if abs(roll - a_roll) > 15.0 or abs(pitch - a_pitch) > 15.0:
        out.append("       fused and gravity disagree, see --axes")
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

    ax, ay, az = t["accel_x"]/100.0, t["accel_y"]/100.0, t["accel_z"]/100.0
    out.append(f"  accel     x {ax:6.2f}  y {ay:6.2f}  z {az:6.2f} m/s2"
               f"   |a| {math.sqrt(ax*ax+ay*ay+az*az):5.2f}")
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


# Each step puts one chassis axis vertical so gravity lands on exactly
# the sensor axis aligned with it. Partial tilts do not identify anything.
AXIS_STEPS = [
    ("flat on the floor, wheels down", "z",
     "puts the chassis UP axis vertical"),
    ("standing on its TAIL, nose pointing at the ceiling", "x",
     "puts the chassis FORWARD axis pointing UP, a full 90 degrees"),
    ("lying on its RIGHT side, left flank pointing at the ceiling", "y",
     "puts the chassis LEFT axis pointing UP, a full 90 degrees"),
]


def read_accel(ser, samples: int = 40, settle: int = 25):
    """Averages a burst of accelerometer readings, in m/s2.

    Telemetry arrives at 50Hz and keeps filling the kernel serial buffer
    while the caller waits at an input() prompt, so the first frames read
    after that prompt describe the attitude the chassis was in BEFORE it
    was moved. Flushing and then discarding a settling window is what
    makes the reading correspond to the current pose."""
    ser.reset_input_buffer()

    dec = Decoder()
    acc = [[], [], []]
    seen = 0
    t0 = time.time()

    while len(acc[0]) < samples and time.time() - t0 < 6.0:
        for mtype, payload in dec.feed(ser.read(256)):
            if mtype != MSG_TELEMETRY:
                continue
            seen += 1
            if seen <= settle:
                continue        # frames still in flight from before the flush
            t = parse_telemetry(payload)
            acc[0].append(t["accel_x"] / 100.0)
            acc[1].append(t["accel_y"] / 100.0)
            acc[2].append(t["accel_z"] / 100.0)

    if len(acc[0]) < samples // 2:
        return None
    return [sum(a) / len(a) for a in acc]


def watch_accel(ser):
    """Live accelerometer readout with the dominant axis called out.

    The stepwise identification below depends on holding an attitude and
    trusting a snapshot. This mode instead lets the chassis be tilted
    while watching gravity move, which makes a wrong reading obvious
    rather than silently producing a wrong constant."""
    print()
    print("  live accelerometer. Tilt the chassis and watch which axis")
    print("  takes the gravity. Ctrl-C to stop.")
    print()
    print("  With the axis remap applied the output is already in")
    print("  chassis coordinates, so gravity lands on whichever chassis")
    print("  axis points at the CEILING:")
    print("    flat on the floor, wheels down   -> Z about +9.8  (up)")
    print("    nose on the floor, tail up       -> X about -9.8  (forward down)")
    print("    lying on the right flank         -> Y about +9.8  (left up)")
    print()

    dec = Decoder()
    sys.stdout.write(HIDE)
    try:
        while True:
            for mtype, payload in dec.feed(ser.read(256)):
                if mtype != MSG_TELEMETRY:
                    continue
                t = parse_telemetry(payload)
                a = [t["accel_x"]/100.0, t["accel_y"]/100.0, t["accel_z"]/100.0]
                mag = math.sqrt(sum(v*v for v in a))
                idx = max(range(3), key=lambda i: abs(a[i]))
                purity = abs(a[idx]) / mag if mag > 0.1 else 0.0
                tag = f"{'XYZ'[idx]}{'+' if a[idx] > 0 else '-'}"
                square = "square" if purity > 0.97 else f"{math.degrees(math.acos(min(1.0, purity))):4.0f} deg off"
                sys.stdout.write(
                    f"\r  x {a[0]:+6.2f}  y {a[1]:+6.2f}  z {a[2]:+6.2f}"
                    f"   |a| {mag:5.2f}   gravity on {tag}   {square}      ")
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write(SHOW + "\n")


def identify_axes(ser):
    """Works out how the module is mounted by watching where gravity
    lands in each of three known chassis attitudes, then prints the
    AXIS_MAP values that rotate the sensor into chassis coordinates.

    Reading the silkscreen arrows is not reliable enough for this: they
    describe the chip, not how it ended up on the robot, and a mistake
    propagates all the way into the SLAM pose."""
    print()
    print("  IMU axis identification")
    print("  " + "-" * 60)
    print("  Gravity is the reference. Every step points one chassis")
    print("  axis at the CEILING; the accelerometer then reads +9.8 on")
    print("  whichever sensor axis lines up with it.")
    print()
    print("  Each step needs a FULL 90 degrees, held still. Position the")
    print("  chassis first, then press Enter and hold for two seconds.")
    print()
    print("  Nothing is written to the sensor; the result is two")
    print("  constants to paste into config/board_config.h.")
    print()

    # sensor axis index that carries gravity, and its sign, per attitude
    observed = {}
    for label, chassis_axis, why in AXIS_STEPS:
        print(f"  place the chassis: {label}")
        print(f"    ({why})")
        input("    Enter when steady... ")
        a = read_accel(ser)
        if a is None:
            print("  not enough telemetry, aborting")
            return

        mag = math.sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2])
        idx = max(range(3), key=lambda i: abs(a[i]))
        sign = 1 if a[idx] > 0 else -1
        print(f"    accel = ({a[0]:+6.2f}, {a[1]:+6.2f}, {a[2]:+6.2f}) m/s2"
              f"   |a| {mag:5.2f}"
              f"   -> sensor {'XYZ'[idx]}{'+' if sign > 0 else '-'}")

        # A stationary reading must be pure gravity. Anything else means
        # the chassis was still moving when the samples were taken.
        if abs(mag - 9.81) > 0.6:
            print("    REJECTED the magnitude is not gravity, so the")
            print("             chassis was moving during the reading.")
            print("             Hold it still and repeat this step.")
            return

        # Gravity must land almost entirely on one axis, otherwise the
        # attitude was not square and the mapping would be a guess.
        if abs(a[idx]) < 9.0:
            print("    REJECTED gravity is spread across axes, so the")
            print(f"             chassis was about "
                  f"{math.degrees(math.acos(min(1.0, abs(a[idx])/mag))):.0f} "
                  f"degrees off square.")
            print("             Repeat with the chassis properly square.")
            return

        if idx in [v[0] for v in observed.values()]:
            prev = [k for k, v in observed.items() if v[0] == idx][0]
            print(f"    REJECTED the same sensor axis already carried")
            print(f"             gravity in the '{prev}' step, so the")
            print("             chassis did not actually change attitude.")
            return

        observed[chassis_axis] = (idx, sign)
        print()

    # Two attitudes are enough. The chassis frame is right handed, so
    # once forward and up are known, left is forced by Y = Z x X. The
    # third measurement is kept as a cross-check rather than a third
    # independent input, because a single mispositioned step would
    # otherwise yield a reflection that looks like a valid answer.
    order = ["x", "y", "z"]
    if len(observed) != 3 or len({v[0] for v in observed.values()}) != 3:
        print("  the three attitudes did not resolve to distinct axes.")
        print("  repeat with the chassis square to the floor each time.")
        return

    def as_vec(pair):
        idx, sign = pair
        v = [0, 0, 0]
        v[idx] = sign
        return v

    def cross(a, b):
        return [a[1]*b[2] - a[2]*b[1],
                a[2]*b[0] - a[0]*b[2],
                a[0]*b[1] - a[1]*b[0]]

    fwd = as_vec(observed["x"])
    up = as_vec(observed["z"])
    derived_left = cross(up, fwd)
    measured_left = as_vec(observed["y"])

    if derived_left != measured_left:
        d_idx = max(range(3), key=lambda i: abs(derived_left[i]))
        m_idx = max(range(3), key=lambda i: abs(measured_left[i]))
        print("  the third attitude disagrees with the first two.")
        print(f"    forward and up imply  LEFT = {'XYZ'[d_idx]}"
              f"{'+' if derived_left[d_idx] > 0 else '-'}")
        print(f"    but the step measured LEFT = {'XYZ'[m_idx]}"
              f"{'+' if measured_left[m_idx] > 0 else '-'}")
        print("  the chassis was most likely rolled onto the wrong side.")
        print("  using the value derived from the first two attitudes.")
        observed["y"] = (d_idx, 1 if derived_left[d_idx] > 0 else -1)
        print()

    # A remap is a rotation, so its determinant must be +1. A result of
    # -1 is a reflection, which no mounting can produce, and means one
    # of the three attitudes was the opposite of what was asked for.
    basis = []
    for chassis_axis in order:
        src, sign = observed[chassis_axis]
        col = [0, 0, 0]
        col[src] = sign
        basis.append(col)
    det = (basis[0][0]*(basis[1][1]*basis[2][2] - basis[1][2]*basis[2][1])
         - basis[0][1]*(basis[1][0]*basis[2][2] - basis[1][2]*basis[2][0])
         + basis[0][2]*(basis[1][0]*basis[2][1] - basis[1][1]*basis[2][0]))
    if det != 1:
        print(f"  the three readings imply a reflection (det {det:+d}),")
        print("  which no physical mounting can produce. One attitude was")
        print("  inverted; check that every step pointed the named axis")
        print("  at the CEILING, then repeat.")
        return

    cfg = 0
    sign_bits = 0
    for pos, chassis_axis in enumerate(order):
        src, sign = observed[chassis_axis]
        cfg |= (src & 3) << (pos * 2)
        if sign < 0:
            # sign bit order is X at bit2, Y at bit1, Z at bit0
            sign_bits |= 1 << (2 - pos)

    print("  " + "-" * 60)
    print("  chassis axis  <- sensor axis")
    for pos, chassis_axis in enumerate(order):
        src, sign = observed[chassis_axis]
        print(f"    {chassis_axis} ({['forward','left','up'][pos]:7s})"
              f"  <- {'XYZ'[src]}{'+' if sign > 0 else '-'}")
    print()
    print("  paste into config/board_config.h:")
    print(f"    #define BNO055_AXIS_MAP         0x{cfg:02X}U")
    print(f"    #define BNO055_AXIS_SIGN        0x{sign_bits:02X}U")
    if cfg == 0x24 and sign_bits == 0x00:
        print("  (that is the identity mapping, the module is already")
        print("   aligned with the chassis)")
    print()


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
    ap.add_argument("--axes", action="store_true",
                    help="identify how the module is mounted, using gravity")
    ap.add_argument("--accel", action="store_true",
                    help="live accelerometer readout, tilt and watch")
    ap.add_argument("--log", metavar="FILE", help="record samples as CSV")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, 115200, timeout=0.05)
    except serial.SerialException as e:
        sys.exit(f"cannot open {args.port}: {e}")

    if args.accel:
        try:
            watch_accel(ser)
        except VersionMismatch as e:
            sys.stdout.write(SHOW + "\n")
            print(f"  {e}")
        finally:
            ser.close()
        return

    if args.axes:
        try:
            identify_axes(ser)
        except VersionMismatch as e:
            print(f"  {e}")
        finally:
            ser.close()
        return

    if args.bias:
        try:
            measure_bias(ser, args.seconds)
        except VersionMismatch as e:
            print(f"  {e}")
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
    except VersionMismatch as e:
        sys.stdout.write(SHOW + "\n")
        print(f"  {e}")
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
