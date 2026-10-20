"""INAV backend: MSP client + the 'override the mission' logic.

INAV has no GUIDED mode and no MAVLink-style commands. The only way for a companion to change what the
aircraft does is to act like a transmitter: inject RC channels over MSP (MSP_SET_RAW_RC, INAV 8.0+).
So "override the mission and yaw" becomes:

    1. move the RC channel that carries NAV WP so the WP box turns OFF,
    2. move the channel that carries NAV POSHOLD so the POSHOLD box turns ON  (drone holds position),
    3. offset the yaw stick channel          (POSHOLD lets the pilot yaw: stick = yaw rate),
    4. read the active-modes bitmap back to *prove* the FC did it (never assume).

and "resume" is the reverse: drive the WP channel back to a NAV WP value, confirm WP is active again,
stop streaming. Which channel/µs does what comes from the FC's own mode table (MSP_MODE_RANGES), so
nothing has to be hand-configured; `plan_channels` solves it and refuses (loudly) when it cannot.

Pilot authority: this only works while the FC's `MSP RC OVERRIDE` mode is active, so that switch on the
transmitter is the kill switch. Switch it off and the FC ignores us instantly; we notice, stop, and
stay passive until the mission is running again.
"""
import socket
import struct
import threading
import time

try:
    import serial  # pyserial
except ImportError:  # only needed for a real serial link
    serial = None

# MSP message ids
MSP_API_VERSION, MSP_FC_VARIANT, MSP_FC_VERSION = 1, 2, 3
MSP_MODE_RANGES, MSP_ATTITUDE, MSP_BOXIDS = 34, 108, 119
MSP_SET_RAW_RC = 200
MSP2_INAV_STATUS = 0x2000
MSP2_COMMON_SETTING = 0x1003

# INAV permanent box ids
BOX_ARM, BOX_ANGLE, BOX_ALTHOLD, BOX_RTH, BOX_POSHOLD = 0, 1, 3, 10, 11
BOX_MANUAL, BOX_FAILSAFE, BOX_WP, BOX_GCS_NAV, BOX_LAUNCH, BOX_TURTLE, BOX_MSP_OVERRIDE = 12, 27, 28, 31, 36, 52, 50
BOX_NAMES = {0: "ARM", 1: "ANGLE", 3: "NAV ALTHOLD", 10: "NAV RTH", 11: "NAV POSHOLD", 12: "MANUAL",
             27: "FAILSAFE", 28: "NAV WP", 31: "GCS NAV", 36: "NAV LAUNCH", 52: "TURTLE", 50: "MSP RC OVERRIDE"}
ARMED_BIT = 1 << 2  # armingFlags


def box_name(b):
    return BOX_NAMES.get(b, f"BOX{b}")


class MspError(Exception):
    pass


# ---------------------------------------------------------------------------------------------------
# MSP framing / transport
# ---------------------------------------------------------------------------------------------------
def crc8_dvb_s2(data, crc=0):
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def encode_v2(code, payload=b"", flag=0, direction=b"<"):
    body = struct.pack("<BHH", flag, code, len(payload)) + payload
    return b"$X" + direction + body + bytes([crc8_dvb_s2(body)])


def encode_v1(code, payload=b"", direction=b"<"):
    body = bytes([len(payload), code]) + payload
    ck = 0
    for b in body:
        ck ^= b
    return b"$M" + direction + body + bytes([ck])


class MspParser:
    """Incremental MSP v1 + v2 frame parser. feed(bytes) -> [(version, direction, flag, code, payload)]."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(b"$")
            if i < 0:
                self.buf.clear()
                break
            if i:
                del self.buf[:i]
            if len(self.buf) < 3:
                break
            ver, d = self.buf[1], chr(self.buf[2])
            if ver not in (ord("M"), ord("X")) or d not in "<>!":
                del self.buf[0]
                continue
            if ver == ord("M"):
                if len(self.buf) < 5:
                    break
                n = self.buf[3]
                if len(self.buf) < 6 + n:
                    break
                body = bytes(self.buf[3:5 + n])
                ck = 0
                for b in body:
                    ck ^= b
                if ck == self.buf[5 + n]:
                    out.append((1, d, 0, self.buf[4], bytes(self.buf[5:5 + n])))
                    del self.buf[:6 + n]
                else:
                    del self.buf[0]
            else:
                if len(self.buf) < 8:
                    break
                flag, code, n = struct.unpack_from("<BHH", self.buf, 3)
                if len(self.buf) < 9 + n:
                    break
                body = bytes(self.buf[3:8 + n])
                if crc8_dvb_s2(body) == self.buf[8 + n]:
                    out.append((2, d, flag, code, bytes(self.buf[8:8 + n])))
                    del self.buf[:9 + n]
                else:
                    del self.buf[0]
        return out


class MspPort:
    """Blocking request/response MSP over serial ('COM7,115200') or TCP ('tcp:127.0.0.1:5761')."""

    def __init__(self, conn):
        if conn.lower().startswith("tcp:"):
            host, port = conn[4:].rsplit(":", 1)
            self.dev = socket.create_connection((host, int(port)), timeout=3)
            self.dev.settimeout(0.05)
            self._read = lambda: self.dev.recv(4096)
            self._write = self.dev.sendall
            self._close = self.dev.close
        else:
            if serial is None:
                raise MspError("pyserial is not installed: pip install pyserial")
            port, _, baud = conn.partition(",")
            self.dev = serial.Serial(port, int(baud or 115200), timeout=0.05, write_timeout=1)
            self._read = lambda: self.dev.read(4096)
            self._write = self.dev.write
            self._close = self.dev.close
        self.parser = MspParser()
        self.lock = threading.Lock()

    def _recv(self):
        try:
            return self._read()
        except (socket.timeout, TimeoutError):
            return b""

    def request(self, code, payload=b"", timeout=0.6):
        """Send a request, return the response payload (b'' is valid). Raises MspError on timeout/'!'."""
        with self.lock:
            self._write(encode_v2(code, payload))
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                for _, d, _, c, p in self.parser.feed(self._recv()):
                    if c == code:
                        if d == "!":
                            raise MspError(f"FC rejected MSP {code}")
                        return p
            raise MspError(f"no reply to MSP {code}")

    def send(self, code, payload=b""):
        """Fire-and-forget (header flag 1: INAV sends no reply to SET_RAW_RC)."""
        with self.lock:
            self._write(encode_v2(code, payload, flag=1))
            self.parser.feed(self._recv())  # drain any stray replies from older firmware

    def close(self):
        try:
            self._close()
        except OSError:
            pass


# ---------------------------------------------------------------------------------------------------
# Mode table -> RC channel plan
# ---------------------------------------------------------------------------------------------------
class PlanError(Exception):
    pass


def parse_mode_ranges(payload):
    """MSP_MODE_RANGES -> [(box, rc_channel_1based, lo_us, hi_us)]. Empty slots (start==end) dropped.
    A range is active for lo <= value < hi (INAV: 900 + 25*step)."""
    return [(c[0], c[1] + 5, 900 + 25 * c[2], 900 + 25 * c[3])
            for c in (payload[i:i + 4] for i in range(0, len(payload) - 3, 4)) if c[2] != c[3]]


def _in(v, r):
    return r[2] <= v < r[3]


def plan_channels(ranges, on, off, avoid=(BOX_RTH, BOX_FAILSAFE, BOX_MANUAL, BOX_TURTLE, BOX_LAUNCH),
                  protect=(BOX_ARM,), max_channel=16):
    """Find RC values {channel: us} that turn every box in `on` ON and every box in `off` OFF, without
    touching a channel that carries a `protect`ed box (ARM: never disarm in flight) and without lighting
    an `avoid`ed box (RTH, failsafe...) on the channels we do move. Raises PlanError with a plain reason."""
    by_box = {}
    for r in ranges:
        by_box.setdefault(r[0], []).append(r)

    chosen_on = {}
    for b in on:
        if b not in by_box:
            raise PlanError(f"{box_name(b)} is not assigned to any RC channel in the FC's Modes tab")
        chosen_on[b] = by_box[b][0]  # one range is enough to activate a box
    involved = {r[1] for r in chosen_on.values()}
    for b in off:
        involved |= {r[1] for r in by_box.get(b, [])}

    plan = {}
    for ch in sorted(involved):
        if ch > max_channel:
            raise PlanError(f"CH{ch} is above CH{max_channel}; this demo only injects CH1-{max_channel} "
                            f"(assign the modes to AUX channels within that range)")
        guarded = [box_name(r[0]) for r in ranges if r[1] == ch and r[0] in protect]
        if guarded:
            raise PlanError(f"CH{ch} also carries {', '.join(guarded)}; refusing to move a channel "
                            f"that could disarm the aircraft. Put the flight modes on a different AUX channel")
        want_in = [r for r in chosen_on.values() if r[1] == ch]
        want_out = [r for b in off for r in by_box.get(b, []) if r[1] == ch]
        want_out += [r for b in avoid for r in by_box.get(b, []) if r[1] == ch]
        centre = (want_in[0][2] + want_in[0][3]) / 2 if want_in else 1500
        best = None
        for v in range(900, 2101, 25):
            if all(_in(v, r) for r in want_in) and not any(_in(v, r) for r in want_out):
                if best is None or abs(v - centre) < abs(best - centre):
                    best = v
        if best is None:
            raise PlanError(f"no value on CH{ch} satisfies the required modes at once")
        plan[ch] = best
    return plan


def active_boxes(ranges, rc):
    """Which boxes are active for rc = {channel: us}. (Mirror of INAV's range test, OR across ranges.)"""
    return {b for b, ch, lo, hi in ranges if ch in rc and lo <= rc[ch] < hi}


# ---------------------------------------------------------------------------------------------------
# Link used by the tracker (same duck-typed interface as the ArduPilot backend)
# ---------------------------------------------------------------------------------------------------
IDLE, OVERRIDE, RESUME = "idle", "override", "resume"


class InavLink:
    """Owns the MSP port. One thread: streams RC while overriding, polls status + attitude."""

    def __init__(self, conn, a, log=print):
        self.a, self.log = a, log
        self.port = MspPort(conn)
        self.armed = False
        self.yaw_deg = None
        self.active = set()  # active permanent box ids
        self.rc_mode = IDLE
        self.yaw_target = 0.0
        self.last_ok = 0.0
        self._stop = False
        self.box_ids = []
        self.plans = {}
        self.ranges = []
        self.mav_type = 2  # unused by INAV; keeps the multirotor check trivially true

        self._connect_checks()
        threading.Thread(target=self._loop, daemon=True).start()

    # -- start-up: identify the FC, read its mode table, build the plans, say what we will do
    def _connect_checks(self):
        p = self.port
        variant = p.request(MSP_FC_VARIANT).decode(errors="replace")
        if variant != "INAV":
            raise MspError(f"FC reports '{variant}', not INAV")
        ver = tuple(p.request(MSP_FC_VERSION)[:3])
        if ver < (8, 0, 0):
            raise MspError(f"INAV {'.'.join(map(str, ver))}: RC over MSP needs INAV 8.0 or newer")
        self.version = ver
        self.box_ids = list(p.request(MSP_BOXIDS))
        self.ranges = parse_mode_ranges(p.request(MSP_MODE_RANGES))
        a = self.a
        hold = list(a.hold_boxes)
        try:
            self.plans = {
                OVERRIDE: plan_channels(self.ranges, on=hold, off=[a.mission_box]),
                RESUME: plan_channels(self.ranges, on=[a.mission_box], off=hold),
            }
        except PlanError as e:
            raise MspError(f"cannot build an override plan from the FC's mode table: {e}")
        ovr = [r for r in self.ranges if r[0] == BOX_MSP_OVERRIDE]
        if not ovr:
            raise MspError("MSP RC OVERRIDE is not assigned to a switch in the FC's Modes tab; "
                           "without it the FC ignores injected RC. Assign it (and use that switch as the kill switch)")
        self.log(f"[inav] INAV {'.'.join(map(str, ver))}; MSP RC OVERRIDE on CH{ovr[0][1]}")
        self.log(f"[inav] override plan : {self._fmt(self.plans[OVERRIDE])}  + yaw on CH{a.yaw_ch}")
        self.log(f"[inav] resume plan   : {self._fmt(self.plans[RESUME])}")
        try:
            mask = struct.unpack("<I", p.request(MSP2_COMMON_SETTING, b"msp_override_channels\0")[:4])[0]
            need = set(self.plans[OVERRIDE]) | set(self.plans[RESUME]) | {a.yaw_ch}
            missing = [c for c in sorted(need) if not mask & (1 << (c - 1))]
            if missing:
                want = mask | sum(1 << (c - 1) for c in need)
                self.log(f"[inav] WARNING msp_override_channels={mask} does not cover CH{missing}: the FC will "
                         f"ignore those. In the INAV CLI:  set msp_override_channels = {want}  then  save "
                         f"(or use the MaxGCS RC panel's 'Set override bitmask', which is runtime-only)")
        except MspError:
            self.log("[inav] note: could not read msp_override_channels (firmware without MSP RC override?)")

    @staticmethod
    def _fmt(plan):
        return ", ".join(f"CH{c}={v}" for c, v in sorted(plan.items()))

    # -- background loop
    def _frame(self):
        plan = self.plans.get(self.rc_mode)
        vals = dict(plan)
        yaw_us = 1500
        if self.rc_mode == OVERRIDE:
            yaw_us = 1500 + max(-self.a.max_us, min(self.a.max_us, round(self.yaw_target / self.a.inav_yaw_rate * 500)))
        vals[self.a.yaw_ch] = yaw_us
        n = max(vals)
        return struct.pack(f"<{n}H", *[vals.get(c, 0) for c in range(1, n + 1)])  # 0 = leave this channel alone

    def _loop(self):
        tick = 0
        nxt = time.monotonic()
        while not self._stop:
            nxt += 0.05
            try:
                if self.rc_mode != IDLE:
                    self.port.send(MSP_SET_RAW_RC, self._frame())
                if tick % 2 == 0:
                    self._poll()
                tick += 1
            except (MspError, OSError):
                pass
            time.sleep(max(0.0, nxt - time.monotonic()))

    def _poll(self):
        st = self.port.request(MSP2_INAV_STATUS)
        flags = struct.unpack_from("<I", st, 9)[0]
        modes = st[13:-1]
        self.active = {self.box_ids[i * 8 + b] for i, byte in enumerate(modes) for b in range(8)
                       if byte & (1 << b) and i * 8 + b < len(self.box_ids)}
        self.armed = bool(flags & ARMED_BIT)
        att = self.port.request(MSP_ATTITUDE)
        self.yaw_deg = struct.unpack_from("<h", att, 4)[0] % 360
        self.last_ok = time.monotonic()

    # -- interface used by the tracker -----------------------------------------------------------
    @property
    def alive(self):
        return time.monotonic() - self.last_ok < 3.0

    @property
    def mission_running(self):
        return self.armed and self.a.mission_box in self.active

    @property
    def override_allowed(self):
        """The pilot's MSP RC OVERRIDE switch is on. Off = kill switch: do not even try."""
        return BOX_MSP_OVERRIDE in self.active

    @property
    def override_active(self):
        """Our override is really in force: the FC reports MSP override on, mission off, hold on."""
        return (BOX_MSP_OVERRIDE in self.active and self.a.mission_box not in self.active
                and all(b in self.active for b in self.a.hold_boxes))

    @property
    def mode_text(self):
        shown = [box_name(b) for b in sorted(self.active) if b in BOX_NAMES and b != BOX_ARM]
        return " + ".join(shown) or "-"

    def heartbeat(self):
        pass

    def begin_override(self):
        self.rc_mode = OVERRIDE

    def yaw_rate(self, deg_s):
        self.yaw_target = deg_s

    def end_override(self):
        self.yaw_target = 0.0
        self.rc_mode = RESUME

    def release(self):
        """Stop injecting RC altogether: the transmitter is in charge again."""
        self.yaw_target = 0.0
        self.rc_mode = IDLE

    def say(self, text, warn=False):
        self.log(f"[fc-msg] {text}")  # MSP has no status-text channel to a GCS; the HUD + log carry it

    def close(self):
        self._stop = True
        time.sleep(0.1)
        self.port.close()
