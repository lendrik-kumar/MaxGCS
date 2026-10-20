#!/usr/bin/env python3
"""Simulated INAV multirotor (MSP over TCP) for rehearsing the visual-track demo without hardware.

NOT a real aircraft and not a physics simulator. It behaves like an INAV FC in the ways the demo cares
about: it has a mode table (switches -> boxes), a "transmitter" holding those switches, an MSP RC
OVERRIDE gate, and it flies a looping NAV WP square mission until a mode change or injected RC tells it
otherwise. It accepts several TCP clients at once - point MaxGCS and visual_track.py at it together.

    MaxGCS            : connection bar -> TCP, 127.0.0.1, port 5760
    visual_track.py   : --fc inav --connect tcp:127.0.0.1:5760

Mode table (what a sensible INAV setup looks like; it is also what the tracker discovers by itself):
    CH5 (AUX1)  ARM          1700-2100
    CH6 (AUX2)  ANGLE <1300 | NAV POSHOLD 1300-1700 | NAV WP >=1700      (3-position switch)
    CH7 (AUX3)  MSP RC OVERRIDE 1700-2100                                (the pilot's kill switch)
The simulated transmitter starts with ARM on, WP on, override on.

Assumptions the real FC may not share (check on the bench): injected channels revert to the transmitter
after --ovr-timeout seconds without frames; yaw stick = yaw rate (full stick = --yaw-rate deg/s).

Type commands on stdin to play pilot:   tx 7 1000   (flip the kill switch off)   tx 6 1500   tx 7 2000
"""
import argparse
import math
import socket
import struct
import sys
import threading
import time

from fc_inav import (BOX_ARM, BOX_MSP_OVERRIDE, BOX_POSHOLD, BOX_WP, BOX_ANGLE, BOX_NAMES, MspParser,
                     active_boxes, encode_v1, encode_v2)

M_PER_DEG_LAT = 111_320.0
# (box, channel, lo_us, hi_us)
RANGES = [(BOX_ARM, 5, 1700, 2100), (BOX_ANGLE, 6, 900, 1300), (BOX_POSHOLD, 6, 1300, 1700),
          (BOX_WP, 6, 1700, 2100), (BOX_MSP_OVERRIDE, 7, 1700, 2100)]
BOX_IDS = [0, 1, 3, 10, 11, 12, 27, 28, 50]  # order = bit index in the active-modes bitmap


def wrap180(a):
    return (a + 180) % 360 - 180


def slew(cur, target, step):
    return cur + max(-step, min(step, target - cur))


class Sim:
    def __init__(self, a):
        self.a = a
        self.lock = threading.Lock()
        self.home_lat, self.home_lon = (float(v) for v in a.home.split(","))
        s = a.size
        self.wps = [(s, 0.0), (s, s), (0.0, s), (0.0, 0.0)]
        self.n = self.e = 0.0
        self.vn = self.ve = 0.0
        self.yaw = 0.0  # degrees, 0..360
        self.yaw_rate = 0.0
        self.wp = 0
        self.alt = a.alt
        self.tx = {c: 1500 for c in range(1, 17)}
        self.tx.update({5: 2000, 6: 2000, 7: 2000, 8: 1000})
        self.ovr, self.ovr_t = {}, -1e9
        self.mask = a.mask
        self.battery = 100.0
        self.t0 = time.monotonic()
        self.last_boxes = None

    # -- RC / modes
    def effective_rc(self):
        rc = dict(self.tx)
        if BOX_MSP_OVERRIDE in active_boxes(RANGES, self.tx) and time.monotonic() - self.ovr_t < self.a.ovr_timeout:
            for ch, v in self.ovr.items():
                if v and (self.mask >> (ch - 1)) & 1:
                    rc[ch] = v
        return rc

    def boxes(self):
        return active_boxes(RANGES, self.effective_rc())

    def latlon(self):
        return (self.home_lat + self.n / M_PER_DEG_LAT,
                self.home_lon + self.e / (M_PER_DEG_LAT * math.cos(math.radians(self.home_lat))))

    def step(self, dt):
        with self.lock:
            rc = self.effective_rc()
            boxes = active_boxes(RANGES, rc)
            if boxes != self.last_boxes:
                print(f"[mock-inav {time.strftime('%H:%M:%S')}] active: "
                      f"{', '.join(BOX_NAMES.get(b, str(b)) for b in sorted(boxes)) or '-'}", flush=True)
                self.last_boxes = boxes
            if BOX_ARM in boxes and BOX_WP in boxes:
                tn, te = self.wps[self.wp]
                dn, de = tn - self.n, te - self.e
                dist = math.hypot(dn, de)
                if dist < 4.0:
                    self.wp = (self.wp + 1) % len(self.wps)
                    return
                self.vn = slew(self.vn, self.a.speed * dn / dist, 2.5 * dt)
                self.ve = slew(self.ve, self.a.speed * de / dist, 2.5 * dt)
                want = math.degrees(math.atan2(de, dn))
                target_rate = slew(0.0, wrap180(want - self.yaw) * 2.0, 40)
            else:  # POSHOLD / ANGLE / disarmed: stop; yaw stick = yaw rate (only meaningful in POSHOLD)
                self.vn = slew(self.vn, 0.0, 3.0 * dt)
                self.ve = slew(self.ve, 0.0, 3.0 * dt)
                stick = rc[4] - 1500
                target_rate = 0.0 if abs(stick) < 10 or BOX_ARM not in boxes else stick / 500 * self.a.yaw_rate
            self.yaw_rate += (target_rate - self.yaw_rate) * min(1.0, dt / 0.3)
            self.n += self.vn * dt
            self.e += self.ve * dt
            self.yaw = (self.yaw + self.yaw_rate * dt) % 360
            self.battery = max(0.0, self.battery - 0.004 * dt)

    # -- MSP responses
    def wp_payload(self, n):
        if n == 0:
            lat, lon, alt, act, flag = self.home_lat, self.home_lon, 0, 0, 0
        else:
            nn, ee = self.wps[n - 1]
            lat = self.home_lat + nn / M_PER_DEG_LAT
            lon = self.home_lon + ee / (M_PER_DEG_LAT * math.cos(math.radians(self.home_lat)))
            alt, act, flag = int(self.alt * 100), 1, 0xA5 if n == len(self.wps) else 0
        return struct.pack("<BBiiihhhB", n, act, int(lat * 1e7), int(lon * 1e7), alt, 0, 0, 0, flag)

    def respond(self, code, payload, flag):
        """-> (reply_payload | None for no reply | Exception for an MSP error)."""
        a = self.a
        with self.lock:
            if code == 1:
                return bytes([0, 2, 5])
            if code == 2:
                return b"INAV"
            if code == 3:
                return bytes(a.fw)
            if code == 4:
                return b"MOCK" + struct.pack("<H", 0)
            if code == 10:
                return b"MOCK-INAV"
            if code == 0x2010:
                return bytes([0, 0, 0, 0, 0, 3, 0])  # platform 0 = multirotor, preset 3 = quad X
            if code == 118:
                return self.wp_payload(payload[0]) if payload and payload[0] <= len(self.wps) else ValueError()
            if code == 20:
                return bytes([0, 120, 1, len(self.wps)])
            if code == 119:
                return bytes(BOX_IDS)
            if code == 34:
                slots = [bytes([b, ch - 5, (lo - 900) // 25, (hi - 900) // 25]) for b, ch, lo, hi in RANGES]
                return b"".join(slots + [bytes(4)] * (20 - len(slots)))
            if code == 0x1003:
                name = payload.split(b"\0")[0]
                if name == b"msp_override_channels":
                    return struct.pack("<I", self.mask)
                if name == b"receiver_type":
                    return bytes([1])
                return ValueError()
            if code == 0x1004:
                name, _, val = payload.partition(b"\0")
                if name == b"msp_override_channels" and len(val) >= 4:
                    self.mask = struct.unpack("<I", val[:4])[0]
                    return b""
                return ValueError()
            if code == 200:
                vals = struct.unpack(f"<{len(payload) // 2}H", payload[:len(payload) // 2 * 2])
                self.ovr = {i + 1: v for i, v in enumerate(vals) if v}
                self.ovr_t = time.monotonic()
                return None if flag == 1 else b""
            if code == 105:
                rc = self.effective_rc()
                return struct.pack("<16H", *[rc[c] for c in range(1, 17)])
            boxes = active_boxes(RANGES, self.effective_rc())
            if code == 0x2000:
                bits = 0
                for i, b in enumerate(BOX_IDS):
                    if b in boxes:
                        bits |= 1 << i
                armed = 4 if BOX_ARM in boxes else 0
                return struct.pack("<HHHHBI", 1000, 0, 0x0027, 3, 0, armed) + bits.to_bytes(2, "little") + b"\0"
            if code == 108:
                return struct.pack("<hhh", int(-5 * math.hypot(self.vn, self.ve)), 0, int(self.yaw))
            if code == 106:
                lat, lon = self.latlon()
                spd = math.hypot(self.vn, self.ve)
                return struct.pack("<BBiihHH", 2, 14, int(lat * 1e7), int(lon * 1e7), int(540 + self.alt),
                                   int(spd * 100), int(self.yaw * 10))
            if code == 109:
                return struct.pack("<ih", int(self.alt * 100), 0)
            if code == 0x2002:
                return (bytes([0x40]) + struct.pack("<HhIIIIB", 1560, 0, 0, 0, 0, 0, int(self.battery))
                        ).ljust(24, b"\0")
            if code == 151:
                return bytes([1, 1, 1, 1, 1, 1, 0, 0, 0])
            if code == 166:
                return struct.pack("<IIIIHHH", 0, 0, 0, 0, 90, 120, 150)
            if code == 0x203A:
                return struct.pack("<IIBB", int(time.monotonic() - self.t0), int(time.monotonic() - self.t0), 45, 1)
            if code == 121:
                return struct.pack("<BBBBBh", 3 if BOX_WP in boxes else 1, 21 if BOX_WP in boxes else 15, 1,
                                   self.wp + 1, 0, 0)
            return ValueError()


def serve_client(conn, sim, addr):
    parser = MspParser()
    conn.settimeout(1.0)
    print(f"[mock-inav] client connected: {addr[0]}:{addr[1]}", flush=True)
    try:
        while True:
            try:
                data = conn.recv(4096)
            except socket.timeout:
                continue
            if not data:
                break
            for ver, d, flag, code, payload in parser.feed(data):
                if d != "<":
                    continue
                r = sim.respond(code, payload, flag)
                if r is None:
                    continue
                if isinstance(r, Exception):
                    out = encode_v2(code, b"", direction=b"!") if ver == 2 else encode_v1(code, b"", b"!")
                else:
                    out = encode_v2(code, r, direction=b">") if ver == 2 else encode_v1(code, r, b">")
                conn.sendall(out)
    except OSError:
        pass
    finally:
        conn.close()
        print(f"[mock-inav] client left: {addr[0]}:{addr[1]}", flush=True)


def stdin_commands(sim):
    for line in sys.stdin:
        p = line.split()
        try:
            if len(p) == 3 and p[0] == "tx":
                with sim.lock:
                    sim.tx[int(p[1])] = int(p[2])
                print(f"[mock-inav] transmitter CH{p[1]} = {p[2]}", flush=True)
        except (ValueError, KeyError):
            print("usage: tx <channel> <us>", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=5760, help="TCP port (default 5760, like INAV SITL UART1)")
    ap.add_argument("--home", default="12.9716,77.5946", help="home lat,lon (default Bengaluru)")
    ap.add_argument("--size", type=float, default=120.0, help="square mission side, metres")
    ap.add_argument("--alt", type=float, default=30.0)
    ap.add_argument("--speed", type=float, default=5.0)
    ap.add_argument("--fw", type=lambda s: tuple(int(x) for x in s.split(".")), default=(9, 0, 0),
                    help="firmware version to report (default 9.0.0)")
    ap.add_argument("--mask", type=lambda s: int(s, 0), default=0xFF,
                    help="msp_override_channels bitmask (CH1 = bit 0); 0xFF = CH1-8. Use 0 to see the tracker warn")
    ap.add_argument("--yaw-rate", type=float, default=200.0, help="deg/s at full yaw stick (INAV default 200)")
    ap.add_argument("--ovr-timeout", type=float, default=1.0, help="injected RC expires after this many s")
    a = ap.parse_args()

    sim = Sim(a)
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        srv.bind(("0.0.0.0", a.port))
    except OSError as e:
        sys.exit(f"mock_inav: cannot listen on {a.port}: {e}")
    srv.listen(5)
    srv.settimeout(0.05)
    print(f"[mock-inav] simulated INAV {'.'.join(map(str, a.fw))} listening on tcp/{a.port}. "
          f"NAV WP square loop. stdin: 'tx <ch> <us>' to play pilot. Ctrl+C stops.", flush=True)
    threading.Thread(target=stdin_commands, args=(sim,), daemon=True).start()
    nxt = time.monotonic()
    try:
        while True:
            try:
                c, addr = srv.accept()
                threading.Thread(target=serve_client, args=(c, sim, addr), daemon=True).start()
            except socket.timeout:
                pass
            if time.monotonic() >= nxt:
                nxt += 0.05
                sim.step(0.05)
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    main()
