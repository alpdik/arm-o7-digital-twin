# Vendored unchanged from ~/Desktop/ti5.py (2026-10-07) so the bridge does not depend on
# a file on the desktop. Edit the original and re-copy if the protocol changes.
#!/usr/bin/env python3
"""Ti5 bionic dexterous hand — RS485 protocol client and command-line control.

    python ti5.py                          # read-only: angles + servo status
    python ti5.py open                     # ease every finger open
    python ti5.py close                    # ease every finger closed
    python ti5.py set index=45 thumb=30    # named fingers; the rest hold still
    python ti5.py set all=20
    python ti5.py control                  # interactive prompt

Fingers can be named (little ring middle index thumb thumb-base), numbered
1..6, or "all". Angles are 0 (open) .. 90 (closed). --torque caps force
(0..1000, default 150) before any motion.

Protocol per 智能仿生灵巧手用户手册 v1.0.0 (2024.07) §3.2 and 机械手控制解析 3.1:
  115200 baud, 8N1, no parity, 16-bit values little-endian ("低字节在前").
  Every frame is exactly 18 bytes: FF FF | addr | cmd | 12 payload | FF FF

  addr 0x03 -> write/motion commands   (NOT used here)
  addr 0xD1 -> read/debug commands

The hand has 6 actuators and closes at 40N, so motion frames are kept behind an
explicit call to `move()` and every target is clamped to the servo's own stored
travel limits (read back via cmd 0x03). Nothing here moves on import.
"""
import argparse
import time

import serial

PORT = "/dev/ttyUSB0"
BAUD = 115200

HEAD = b"\xff\xff"
TAIL = b"\xff\xff"
FRAME_LEN = 18

ADDR_READ = 0xD1
ADDR_WRITE = 0x03      # 上位机 — host-to-hand command frames

CMD_STATUS = 0x01      # 获取常规状态 — voltage, temperature, load, position, flags
CMD_POS_DEBUG = 0x03   # 位置调试 — min / max / correction, with 01 = read
CMD_ALL_ANGLES = 0x10  # 查询全部手指当前角度 — all six joint angles

CMD_POSITION = 0xA3    # 位置模式 — six uint16 ANGLES, one per servo (see below)
CMD_SPEED = 0xA5       # 速度 — six uint16, steps/sec
CMD_TORQUE = 0xA6      # 转矩限制 — six uint16, 0..1000 (0.1% of stall torque)

# 手指代号 §2.4: little=1, ring=2, middle=3, index=4, thumb=5, thumb base joint=6
FINGERS = {
    1: "little",
    2: "ring",
    3: "middle",
    4: "index",
    5: "thumb",
    6: "thumb-base",
}

# 舵机状态 bit flags (§3.4): 0 = no error
STATUS_BITS = ["voltage", "sensor", "temperature", "current", "angle", "overload"]

# 0xA3 takes ANGLES, not the raw step positions that 0x01/0x03 report. Measured
# on this hand by sweeping servo 1 and reading back both quantities:
#
#   angle 0  = fully open      angle 90 = fully closed
#   readback angle tracks the commanded value exactly
#   the servo clamps above 90 rather than ignoring the frame
#
# The scale is normalised per finger: every servo's full travel spans the same
# 0..90, whatever its step range (little 1520 steps, thumb only 510). So angles
# are portable across fingers and step limits are irrelevant to commanding.
ANGLE_OPEN = 0
ANGLE_CLOSED = 90

# Easing for ease_to(): angle units per frame and frame period. A full 0..90
# sweep takes ~1 s, so nothing snaps across its range.
EASE_STEP = 3
EASE_PERIOD = 1 / 30
DEFAULT_TORQUE = 150


def build(cmd, payload=(), addr=ADDR_READ):
    """Assemble an 18-byte frame. Payload is zero-padded to fill the body."""
    body = bytes(payload) + bytes(12 - len(payload))
    if len(body) != 12:
        raise ValueError(f"payload too long: {len(payload)} > 12")
    return HEAD + bytes([addr, cmd]) + body + TAIL


def pack6(values):
    """Six uint16 little-endian — the payload shape of 0xA3 / 0xA5 / 0xA6."""
    if len(values) != 6:
        raise ValueError(f"need 6 values, got {len(values)}")
    out = bytearray()
    for v in values:
        v = int(v)
        if not 0 <= v <= 0xFFFF:
            raise ValueError(f"value out of range: {v}")
        out += bytes([v & 0xFF, (v >> 8) & 0xFF])
    return bytes(out)


def u16(buf, off):
    """16-bit little-endian, per the protocol's 低字节在前 note."""
    return buf[off] | (buf[off + 1] << 8)


def s16(buf, off):
    """Signed 16-bit LE. Load is signed -- a servo pushing against its end stop
    reports e.g. 0xFFB8 (-72), which reads as 65464 and a nonsense 6546% if
    taken unsigned."""
    v = u16(buf, off)
    return v - 0x10000 if v & 0x8000 else v


def decode_flags(value):
    names = [n for i, n in enumerate(STATUS_BITS) if value & (1 << i)]
    return ", ".join(names) if names else "ok"


class Ti5Hand:
    def __init__(self, port=PORT, baud=BAUD):
        self.ser = serial.Serial(port, baud, timeout=0.3)

    def close(self):
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def request(self, cmd, payload=(), settle=0.08):
        """Send a read frame, return (sent, replies). Echo of our own TX is dropped.

        RS485 is half-duplex; many USB adapters loop transmitted bytes back, so a
        reply identical to the request is an echo, not an answer from the hand.
        """
        frame = build(cmd, payload)
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()
        time.sleep(settle)

        raw = self.ser.read(FRAME_LEN * 4)
        replies = [f for f in split_frames(raw) if f != frame]
        return frame, replies, raw

    def limits(self):
        """-> {servo_id: (min, max)} read from each servo's stored 位置调试 values.

        These are much narrower than the nominal 0..4095 and differ per finger,
        so they are the authority for clamping, not the protocol's range.
        """
        out = {}
        for sid in FINGERS:
            _, replies, _ = self.request(CMD_POS_DEBUG, [sid, 0x01])
            if replies:
                b = replies[0]
                out[sid] = (u16(b, 6), u16(b, 8))
        return out

    def send(self, cmd, values):
        """Transmit a 6-value command frame. This is the only method that moves
        the hand; callers are responsible for having clamped the values."""
        self.ser.write(build(cmd, pack6(values), addr=ADDR_WRITE))
        self.ser.flush()

    def set_torque(self, limit, count=6):
        """Cap output torque on every servo. 0..1000 = 0..100% of stall torque."""
        if not 0 <= limit <= 1000:
            raise ValueError(f"torque limit out of range: {limit}")
        self.send(CMD_TORQUE, [limit] * count)

    def set_angles(self, angles):
        """Command all six servos in angle units. 0 = open, 90 = closed.

        Every servo id must be present. 0xA3 is a whole-hand frame with no way
        to say "leave this finger alone", so an omission would silently become a
        command; the safe value for a finger nobody thought about is its current
        angle, which only the caller knows. Use all_angles() to get it.
        """
        missing = set(FINGERS) - set(angles)
        if missing:
            raise ValueError(
                f"set_angles() needs every servo; missing {sorted(missing)}. "
                f"Pass its current angle to hold a finger still."
            )
        frame = [max(ANGLE_OPEN, min(ANGLE_CLOSED, int(angles[sid])))
                 for sid in sorted(FINGERS)]
        self.send(CMD_POSITION, frame)
        return frame

    def ease_to(self, targets, step=EASE_STEP, period=EASE_PERIOD):
        """Move toward {servo_id: angle} a few degrees per frame.

        Servos not in `targets` hold the angle the hand reports for them.
        Returns the final commanded frame.
        """
        current, _ = self.all_angles()
        if not current or set(current) != set(FINGERS):
            raise RuntimeError("could not read all six angles — is the hand powered?")
        goal = dict(current)
        for sid, a in targets.items():
            goal[sid] = max(ANGLE_OPEN, min(ANGLE_CLOSED, int(a)))

        cur = dict(current)
        frame = None
        while cur != goal:
            for sid in cur:
                delta = goal[sid] - cur[sid]
                if abs(delta) > step:
                    cur[sid] += step if delta > 0 else -step
                else:
                    cur[sid] = goal[sid]
            frame = self.set_angles(cur)
            time.sleep(period)
        return frame

    def all_angles(self):
        """-> {servo_id: angle}. Payload is six [id, angle] byte pairs, not words.

        Observed reply: ff ff d1 10 | 01 56 02 54 03 54 04 58 05 96 06 01 | ff ff
        The 01..06 in the odd positions are the servo ids, so angle is one byte.
        """
        _, replies, raw = self.request(CMD_ALL_ANGLES)
        if not replies:
            return None, raw
        body = replies[0]
        return {body[4 + 2 * i]: body[5 + 2 * i] for i in range(6)}, raw

    def status(self, servo_id):
        """-> dict of decoded 常规状态 fields for one servo."""
        _, replies, raw = self.request(CMD_STATUS, [servo_id, 0x01])
        if not replies:
            return None, raw
        b = replies[0]
        return {
            "id": b[4],
            "voltage": b[6] / 10.0,   # 0.1 V per §3.4
            "temp_c": b[7],           # whole degrees C
            "load": s16(b, 10) / 10.0,  # 0.1 %, signed: sign is drive direction
            "position": u16(b, 12),   # steps, 0..4095
            "flags": u16(b, 14),
        }, raw


def split_frames(raw):
    """Pull complete FF FF ... FF FF frames out of a byte stream."""
    frames = []
    i = 0
    while i + FRAME_LEN <= len(raw):
        if raw[i:i + 2] == HEAD and raw[i + 16:i + 18] == TAIL:
            frames.append(raw[i:i + FRAME_LEN])
            i += FRAME_LEN
        else:
            i += 1
    return frames


def hexs(b):
    return " ".join(f"{x:02x}" for x in b)


def parse_finger(name):
    """'index' / '4' / 'all' -> list of servo ids."""
    name = name.strip().lower()
    if name == "all":
        return sorted(FINGERS)
    if name.isdigit() and int(name) in FINGERS:
        return [int(name)]
    for sid, n in FINGERS.items():
        if n == name:
            return [sid]
    raise ValueError(f"unknown finger {name!r}; use {', '.join(FINGERS.values())}, 1-6 or all")


def parse_targets(items):
    """['index=45', 'thumb=30'] -> {4: 45, 5: 30}"""
    targets = {}
    for item in items:
        if "=" not in item:
            raise ValueError(f"expected finger=angle, got {item!r}")
        name, angle = item.split("=", 1)
        for sid in parse_finger(name):
            targets[sid] = int(angle)
    return targets


def print_status(hand):
    print("[0x10] 查询全部手指当前角度 (all finger angles)")
    angles, raw = hand.all_angles()
    print(f"  tx:  {hexs(build(CMD_ALL_ANGLES))}")
    print(f"  rx:  {hexs(raw) if raw else '(nothing)'}")
    if angles:
        for sid, a in sorted(angles.items()):
            print(f"    id{sid} {FINGERS.get(sid, '?'):<11} angle={a:>3}")
    print()

    print("[0x01] 获取常规状态 (per-servo status)")
    print(f"  {'':<16} {'volt':>6} {'temp':>6} {'load':>7} {'pos':>6}  flags")
    for sid in FINGERS:
        st, raw = hand.status(sid)
        label = f"  id{sid} {FINGERS[sid]:<12}"
        if st is None:
            print(f"{label} no reply  raw={hexs(raw) if raw else '-'}")
        else:
            print(
                f"{label} {st['voltage']:>5.1f}V {st['temp_c']:>5}C"
                f" {st['load']:>6.1f}% {st['position']:>6}  {decode_flags(st['flags'])}"
            )


def print_angles(hand):
    angles, _ = hand.all_angles()
    if not angles:
        print("  no reply")
        return
    print("  " + "  ".join(f"{FINGERS[s]}={a}" for s, a in sorted(angles.items())))


CONTROL_HELP = """\
  <finger> <angle>     e.g. index 45, thumb 0, 4 90, all 30   (0 open .. 90 closed)
  open | close         every finger
  torque <0-1000>      cap force
  status               full servo status
  angles               current angles
  help | q"""


def control(hand, torque):
    print(f"Ti5 hand control — torque {torque}/1000. Keep hands clear.")
    print(CONTROL_HELP)
    hand.set_torque(torque)
    while True:
        try:
            line = input("hand> ").strip()
        except EOFError:
            print()
            return
        if not line:
            continue
        words = line.split()
        cmd = words[0].lower()
        try:
            if cmd in ("q", "quit", "exit"):
                return
            elif cmd in ("help", "?"):
                print(CONTROL_HELP)
            elif cmd == "status":
                print_status(hand)
            elif cmd == "angles":
                print_angles(hand)
            elif cmd == "open":
                hand.ease_to({s: ANGLE_OPEN for s in FINGERS})
                print_angles(hand)
            elif cmd == "close":
                hand.ease_to({s: ANGLE_CLOSED for s in FINGERS})
                print_angles(hand)
            elif cmd == "torque" and len(words) == 2:
                hand.set_torque(int(words[1]))
                print(f"  torque {int(words[1])}/1000")
            elif len(words) == 2:
                hand.ease_to({s: int(words[1]) for s in parse_finger(cmd)})
                print_angles(hand)
            else:
                print("  ? type help")
        except (ValueError, RuntimeError) as e:
            print(f"  {e}")
        except KeyboardInterrupt:
            print("\n  stopped — the hand holds its last commanded angle")


def main():
    ap = argparse.ArgumentParser(description="Read or drive the Ti5 hand.")
    ap.add_argument("action", nargs="?", default="status",
                    choices=["status", "open", "close", "set", "control"])
    ap.add_argument("targets", nargs="*", help="for set: finger=angle ...")
    ap.add_argument("--torque", type=int, default=DEFAULT_TORQUE,
                    help="force cap 0..1000 applied before motion (default %(default)s)")
    ap.add_argument("--port", default=PORT)
    args = ap.parse_args()

    if args.action == "set":
        try:
            targets = parse_targets(args.targets)
        except ValueError as e:
            ap.error(str(e))
        if not targets:
            ap.error("set needs at least one finger=angle")
    elif args.action == "open":
        targets = {s: ANGLE_OPEN for s in FINGERS}
    elif args.action == "close":
        targets = {s: ANGLE_CLOSED for s in FINGERS}

    with Ti5Hand(args.port) as hand:
        if args.action == "status":
            print(f"Ti5 hand read-only probe — {args.port} @ {BAUD} 8N1\n")
            print_status(hand)
        elif args.action == "control":
            control(hand, args.torque)
        else:
            hand.set_torque(args.torque)
            time.sleep(0.05)
            hand.ease_to(targets)
            print_angles(hand)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\naborted — the hand holds its last commanded angle")
    except RuntimeError as e:
        raise SystemExit(str(e))
