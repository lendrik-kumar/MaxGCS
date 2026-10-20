#!/usr/bin/env python3
"""Simulated ArduCopter flight controller (MAVLink 2) for rehearsing the visual-track demo.

NOT a real aircraft and not a physics simulator. It exists so the demo runs on a laptop with no
hardware: it flies a looping square AUTO mission, answers the MAVLink calls MaxGCS and
visual_track.py make, and obeys the same two commands the real FC would:

  * mode change   (COMMAND_LONG MAV_CMD_DO_SET_MODE / SET_MODE)  AUTO <-> GUIDED
  * yaw-rate hold (SET_POSITION_TARGET_LOCAL_NED, velocity 0 + yaw_rate, honoured in GUIDED only)

Topology (all UDP on localhost, same convention as ArduPilot SITL's --out):

    mock_fc --telemetry--> MaxGCS          (MaxGCS UDP connection, port 14550)
       |   <---commands--- MaxGCS
       +----telemetry---> visual_track.py  (--connect udpin:0.0.0.0:14551)
       |   <---commands--- visual_track.py (its STATUSTEXT is relayed on to MaxGCS)

For a real FC / ArduPilot SITL, skip this file and point visual_track.py at that link instead.
"""
import argparse
import math
import select
import socket
import sys
import time

from pymavlink.dialects.v20 import ardupilotmega as mav2

# ArduCopter custom modes
AUTO, GUIDED, LOITER, RTL, LAND = 3, 4, 5, 6, 9
MODE_NAMES = {3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL", 9: "LAND"}

M_PER_DEG_LAT = 111_320.0
YAW_IGNORE, YAW_RATE_IGNORE = 1024, 2048  # POSITION_TARGET_TYPEMASK bits
CMD_DO_SET_MODE, CMD_REQUEST_MESSAGE = 176, 512
MSG_AUTOPILOT_VERSION = 148


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def slew(cur, target, step):
    return cur + max(-step, min(step, target - cur))


class _Writer:
    """File-like sink for pymavlink: fan every encoded frame out to all current peers."""

    def __init__(self, sock, peers):
        self.sock, self.peers = sock, peers

    def write(self, buf):
        for addr in list(self.peers):
            try:
                self.sock.sendto(buf, addr)
            except OSError:
                pass


class MockFc:
    def __init__(self, args):
        self.a = args
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if hasattr(socket, "SIO_UDP_CONNRESET"):  # Windows: ignore ICMP "port unreachable" resets
            self.sock.ioctl(socket.SIO_UDP_CONNRESET, False)
        self.sock.bind(("0.0.0.0", args.port))
        self.static_peers = {(h, int(p)) for h, p in (o.rsplit(":", 1) for o in args.out)}
        self.peers = {p: float("inf") for p in self.static_peers}  # addr -> last_seen
        self.out = _Writer(self.sock, self.peers)
        self.mav = mav2.MAVLink(self.out, srcSystem=1, srcComponent=1)
        self.decoders = {}  # addr -> parser

        lat, lon = (float(v) for v in args.home.split(","))
        self.home_lat, self.home_lon = lat, lon
        self.home_msl = 540.0
        s = args.size
        self.wps = [(s, 0.0), (s, s), (0.0, s), (0.0, 0.0)]  # (north, east) metres from home
        self.alt = args.alt
        self.t0 = time.monotonic()

        # airborne, armed, flying the loop towards WP 1
        self.n, self.e = 0.0, 0.0
        self.vn = self.ve = 0.0
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.mode = AUTO
        self.wp = 0  # index into self.wps; mission seq = wp + 1 (seq 0 is home)
        self.armed = True
        self.yaw_rate_cmd, self.yaw_cmd_t = 0.0, -1e9
        self.battery = 100.0

    # ---- helpers -------------------------------------------------------------------------------
    def now_ms(self):
        return int((time.monotonic() - self.t0) * 1000) & 0xFFFFFFFF

    def latlon(self):
        lat = self.home_lat + self.n / M_PER_DEG_LAT
        lon = self.home_lon + self.e / (M_PER_DEG_LAT * math.cos(math.radians(self.home_lat)))
        return lat, lon

    def log(self, text):
        print(f"[mock-ardupilot {time.strftime('%H:%M:%S')}] {text}", flush=True)

    def statustext(self, text, sev=6):
        self.mav.statustext_send(sev, text.encode()[:50])

    # ---- simulation ----------------------------------------------------------------------------
    def step(self, dt):
        now = time.monotonic()
        if self.mode == AUTO and self.armed:
            tn, te = self.wps[self.wp]
            dn, de = tn - self.n, te - self.e
            dist = math.hypot(dn, de)
            if dist < 4.0:
                self.wp = (self.wp + 1) % len(self.wps)  # loop the mission forever
                self.log(f"reached WP, next seq {self.wp + 1}")
                return
            want_vn, want_ve = self.a.speed * dn / dist, self.a.speed * de / dist
            acc = 2.5 * dt
            self.vn = slew(self.vn, want_vn, acc)
            self.ve = slew(self.ve, want_ve, acc)
            want_yaw = math.atan2(de, dn)  # AUTO: nose towards the next waypoint
            rate_t = slew(0.0, wrap_pi(want_yaw - self.yaw) * 2.0, math.radians(40))
            self.yaw_rate += (rate_t - self.yaw_rate) * min(1.0, dt / 0.3)
        else:
            # GUIDED / other: hold position, yaw only as commanded
            self.vn = slew(self.vn, 0.0, 3.0 * dt)
            self.ve = slew(self.ve, 0.0, 3.0 * dt)
            cmd = self.yaw_rate_cmd if (self.mode == GUIDED and now - self.yaw_cmd_t < 1.0) else 0.0
            self.yaw_rate += (cmd - self.yaw_rate) * min(1.0, dt / 0.3)
        self.n += self.vn * dt
        self.e += self.ve * dt
        self.yaw = wrap_pi(self.yaw + self.yaw_rate * dt)
        self.battery = max(0.0, self.battery - 0.004 * dt)

    # ---- outgoing telemetry --------------------------------------------------------------------
    def send_heartbeat(self):
        base = 1 | 16 | (128 if self.armed else 0)  # custom-mode | stabilize | armed
        base |= 4 if self.mode == AUTO else 8 if self.mode == GUIDED else 0
        self.mav.heartbeat_send(mav2.MAV_TYPE_QUADROTOR, mav2.MAV_AUTOPILOT_ARDUPILOTMEGA,
                                base, self.mode, mav2.MAV_STATE_ACTIVE, 3)

    def send_telemetry_fast(self):
        t = self.now_ms()
        spd = math.hypot(self.vn, self.ve)
        pitch = -0.12 * spd / max(self.a.speed, 0.1)
        roll = max(-0.5, min(0.5, self.yaw_rate * spd * 0.05))
        self.mav.attitude_send(t, roll, pitch, self.yaw, 0.0, pitch * 0.0, self.yaw_rate)
        lat, lon = self.latlon()
        hdg = int(math.degrees(self.yaw) % 360 * 100)
        self.mav.global_position_int_send(
            t, int(lat * 1e7), int(lon * 1e7), int((self.home_msl + self.alt) * 1000),
            int(self.alt * 1000), int(self.vn * 100), int(self.ve * 100), 0, hdg)

    def send_telemetry_slow(self):
        t = self.now_ms()
        spd = math.hypot(self.vn, self.ve)
        lat, lon = self.latlon()
        self.mav.vfr_hud_send(spd, spd, int(math.degrees(self.yaw) % 360), 55, self.alt, 0.0)
        self.mav.gps_raw_int_send(t * 1000, 3, int(lat * 1e7), int(lon * 1e7),
                                  int((self.home_msl + self.alt) * 1000), 80, 120,
                                  int(spd * 100), int(math.degrees(self.yaw) % 360 * 100), 14)
        sensors = 1 | 2 | 4 | 8 | 32
        volts = int((14.0 + 2.6 * self.battery / 100) * 1000)
        self.mav.sys_status_send(sensors, sensors, sensors, 100, volts, -1, int(self.battery),
                                 0, 0, 0, 0, 0, 0)
        self.mav.mission_current_send(self.wp + 1, len(self.wps) + 1)
        self.mav.home_position_send(int(self.home_lat * 1e7), int(self.home_lon * 1e7),
                                    int(self.home_msl * 1000), 0, 0, 0, [1, 0, 0, 0], 0, 0, 0)

    # ---- incoming ------------------------------------------------------------------------------
    def set_mode(self, mode, who):
        if mode not in MODE_NAMES:
            self.log(f"{who}: unsupported mode {mode}")
            return False
        if mode != self.mode:
            self.log(f"mode {MODE_NAMES[self.mode]} -> {MODE_NAMES[mode]}  (requested by {who})")
        self.mode = mode
        return True

    def ack(self, cmd, result, msg):
        self.mav.command_ack_send(cmd, result, 0, 0, msg.get_srcSystem(), msg.get_srcComponent())

    def handle(self, msg, addr, raw):
        mt = msg.get_type()
        who = f"{addr[0]}:{addr[1]}"
        if mt == "COMMAND_LONG" and msg.target_system in (0, 1):
            if msg.command == CMD_DO_SET_MODE:
                ok = self.set_mode(int(msg.param2), who)
                self.ack(msg.command, 0 if ok else 4, msg)
            elif msg.command == CMD_REQUEST_MESSAGE and int(msg.param1) == MSG_AUTOPILOT_VERSION:
                self.ack(msg.command, 0, msg)
                self.mav.autopilot_version_send(0, (4 << 24) | (5 << 16) | (7 << 8) | 255, 0, 0, 0,
                                                [0] * 8, [0] * 8, [0] * 8, 0, 0, 0)
            else:
                self.ack(msg.command, 3, msg)  # MAV_RESULT_UNSUPPORTED
        elif mt == "SET_MODE":
            self.set_mode(int(msg.custom_mode), who)
        elif mt == "SET_POSITION_TARGET_LOCAL_NED":
            if self.mode == GUIDED and not (msg.type_mask & YAW_RATE_IGNORE):
                self.yaw_rate_cmd, self.yaw_cmd_t = msg.yaw_rate, time.monotonic()
        elif mt == "MISSION_REQUEST_LIST":
            self.mav.mission_count_send(msg.get_srcSystem(), msg.get_srcComponent(),
                                        len(self.wps) + 1, 0)
        elif mt in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            self.send_mission_item(msg.seq, msg.get_srcSystem(), msg.get_srcComponent())
        elif mt == "STATUSTEXT":
            # visual_track.py narrates the demo through STATUSTEXT; forward it to the GCS the way a
            # MAVLink router would, so it shows up as a toast in MaxGCS.
            for peer in self.peers:
                if peer != addr:
                    self.sock.sendto(raw, peer)
            self.log(f"STATUSTEXT from {who}: {msg.text}")

    def send_mission_item(self, seq, tsys, tcomp):
        if seq == 0:
            lat, lon, alt, frame = self.home_lat, self.home_lon, self.home_msl, 0
        else:
            n, e = self.wps[(seq - 1) % len(self.wps)]
            lat = self.home_lat + n / M_PER_DEG_LAT
            lon = self.home_lon + e / (M_PER_DEG_LAT * math.cos(math.radians(self.home_lat)))
            alt, frame = self.alt, 3  # GLOBAL_RELATIVE_ALT
        self.mav.mission_item_int_send(tsys, tcomp, seq, frame, 16, 1 if seq == self.wp + 1 else 0, 1,
                                       0, 0, 0, 0, int(lat * 1e7), int(lon * 1e7), alt, 0)

    def pump_rx(self, timeout):
        r, _, _ = select.select([self.sock], [], [], timeout)
        while r:
            try:
                data, addr = self.sock.recvfrom(2048)
            except (ConnectionResetError, OSError):
                break
            dec = self.decoders.setdefault(addr, mav2.MAVLink(None))
            try:
                msgs = dec.parse_buffer(data) or []
            except Exception:
                msgs = []
            if msgs and addr not in self.peers:
                self.log(f"peer connected: {addr[0]}:{addr[1]}")
            if msgs:
                self.peers[addr] = time.monotonic()
            for m in msgs:
                self.handle(m, addr, m.get_msgbuf())
            r, _, _ = select.select([self.sock], [], [], 0)
        stale = [a for a, t in self.peers.items() if a not in self.static_peers and time.monotonic() - t > 10]
        for a in stale:
            del self.peers[a]
            self.log(f"peer timed out: {a[0]}:{a[1]}")

    # ---- main loop -----------------------------------------------------------------------------
    def run(self):
        self.log(f"simulated ArduCopter up. FC port udp/{self.a.port}; streaming to {', '.join(self.a.out)}")
        self.log("AUTO mission: square loop, 4 waypoints. Ctrl+C to stop.")
        dt = 0.05
        nxt = time.monotonic()
        tick = 0
        self.statustext("SIMULATED ArduCopter (mock_fc) - not a real aircraft")
        while True:
            self.pump_rx(max(0.0, nxt - time.monotonic()))
            if time.monotonic() < nxt:
                continue
            nxt += dt
            tick += 1
            self.step(dt)
            self.send_telemetry_fast()  # 20 Hz
            if tick % 20 == 0:
                self.send_heartbeat()
                self.send_telemetry_slow()
            elif tick % 5 == 0:
                self.mav.vfr_hud_send(math.hypot(self.vn, self.ve), math.hypot(self.vn, self.ve),
                                      int(math.degrees(self.yaw) % 360), 55, self.alt, 0.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=14560, help="UDP port the FC sends from / listens on (default 14560)")
    ap.add_argument("--out", nargs="+", default=["127.0.0.1:14550", "127.0.0.1:14551"], metavar="HOST:PORT",
                    help="where to push telemetry: MaxGCS and visual_track.py (default 127.0.0.1:14550 127.0.0.1:14551)")
    ap.add_argument("--home", default="12.9716,77.5946", help="home lat,lon (default Bengaluru)")
    ap.add_argument("--size", type=float, default=120.0, help="square mission side in metres")
    ap.add_argument("--alt", type=float, default=30.0, help="mission altitude AGL, metres")
    ap.add_argument("--speed", type=float, default=5.0, help="cruise speed m/s")
    args = ap.parse_args()
    try:
        MockFc(args).run()
    except KeyboardInterrupt:
        print()
    except OSError as e:
        sys.exit(f"mock_fc: {e}")


if __name__ == "__main__":
    main()
