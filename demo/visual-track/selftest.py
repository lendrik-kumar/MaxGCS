#!/usr/bin/env python3
"""End-to-end check without a camera or MaxGCS, for both firmwares.

For each of INAV and ArduPilot: start the simulated FC, play a synthetic camera scene (a white block
standing in for the person, fixed in the world) through visual_track.py with --sim-pan, and watch the
aircraft from a *second, independent* link - the way MaxGCS would. Scenarios:

    normal   trigger -> override -> heading turns to centre the person -> person leaves -> mission resumes
    pilot    the pilot vetoes mid-track (INAV: MSP RC OVERRIDE switch off; ArduPilot: mode -> LOITER);
             the tracker must let go and must not fight back

Run:  python selftest.py [inav|ardupilot]      (~1 min per firmware)
"""
import math
import os
import struct
import subprocess
import sys
import threading
import time

os.environ["MAVLINK20"] = "1"
import cv2  # noqa: E402
import numpy as np  # noqa: E402

import visual_track as vt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
W, H = 640, 480
APPEAR, VANISH, END = 2.0, 14.0, 26.0
PERSON_X = 500  # block centre, px (right of centre -> expect a clockwise yaw of ~21 deg)
PILOT_AT = 8.0


class BlockDetector:
    """Stand-in for HOG: finds the bright block painted into the synthetic frame."""

    name = "block"

    def detect(self, frame):
        mask = frame[:, :, 0] > 200
        if mask.sum() < 400:
            return []
        ys, xs = np.nonzero(mask)
        return [vt.Det(int(xs.min()), int(ys.min()), int(xs.max() - xs.min()), int(ys.max() - ys.min()))]


def scene(t0):
    while True:
        t = time.monotonic() - t0
        if t > END:
            return
        img = np.full((H, W, 3), 40, np.uint8)
        if APPEAR <= t < VANISH:
            cv2.rectangle(img, (PERSON_X - 25, 160), (PERSON_X + 25, 400), (255, 255, 255), -1)
        time.sleep(0.05)
        yield img


# ---- independent observers: normalise both firmwares to the same log ---------------------------------
# log: modes [(t, 'MISSION'|'OVERRIDE'|'OTHER')], yaw [(t, deg)], pos [(t, lat, lon)], text [str]
class MavObserver:
    def __init__(self, port):
        from pymavlink import mavutil
        self.m = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}")

    def run(self, log, stop):
        t0 = time.monotonic()
        while not stop.is_set():
            m = self.m.recv_match(blocking=True, timeout=0.3)
            if not m:
                continue
            t, ty = time.monotonic() - t0, m.get_type()
            if ty == "HEARTBEAT":
                label = {3: "MISSION", 4: "OVERRIDE"}.get(m.custom_mode, "OTHER")
                if not log["modes"] or log["modes"][-1][1] != label:
                    log["modes"].append((t, label))
            elif ty == "ATTITUDE":
                log["yaw"].append((t, math.degrees(m.yaw) % 360))
            elif ty == "GLOBAL_POSITION_INT":
                log["pos"].append((t, m.lat / 1e7, m.lon / 1e7))
            elif ty == "STATUSTEXT":
                log["text"].append(m.text)

    def pilot_veto(self):
        self.m.mav.command_long_send(1, 1, 176, 0, 1, 5, 0, 0, 0, 0, 0)  # DO_SET_MODE LOITER


class InavObserver:
    def __init__(self, port):
        import fc_inav
        self.f = fc_inav
        self.port = fc_inav.MspPort(f"tcp:127.0.0.1:{port}")
        self.box_ids = list(self.port.request(fc_inav.MSP_BOXIDS))
        self.veto = None

    def run(self, log, stop):
        f, t0 = self.f, time.monotonic()
        while not stop.is_set():
            try:
                st = self.port.request(f.MSP2_INAV_STATUS)
                att = self.port.request(f.MSP_ATTITUDE)
                gps = self.port.request(106)
            except f.MspError:
                continue
            t = time.monotonic() - t0
            bits = st[13:-1]
            on = {self.box_ids[i * 8 + b] for i, by in enumerate(bits) for b in range(8)
                  if by & (1 << b) and i * 8 + b < len(self.box_ids)}
            if f.BOX_WP in on:
                label = "MISSION"
            elif f.BOX_POSHOLD in on and f.BOX_MSP_OVERRIDE in on:
                label = "OVERRIDE"
            else:
                label = "OTHER"
            if not log["modes"] or log["modes"][-1][1] != label:
                log["modes"].append((t, label))
            log["yaw"].append((t, struct.unpack_from("<h", att, 4)[0] % 360))
            lat, lon = struct.unpack_from("<ii", gps, 2)
            log["pos"].append((t, lat / 1e7, lon / 1e7))
            time.sleep(0.1)


def run_case(fc, pilot):
    ports = {"inav": (5790, 0), "ardupilot": (14590, 14591)}[fc]
    if pilot:
        ports = tuple(p + 10 if p else 0 for p in ports)
    print(f"\n=========== {fc.upper()} - {'pilot veto mid-track' if pilot else 'trigger -> track -> resume'}")
    log = {"modes": [], "yaw": [], "text": [], "pos": []}
    stop = threading.Event()

    if fc == "inav":
        mock = subprocess.Popen([sys.executable, os.path.join(HERE, "mock_inav.py"), "--port", str(ports[0])],
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        time.sleep(1.5)
        obs = InavObserver(ports[0])
        argv = ["--fc", "inav", "--connect", f"tcp:127.0.0.1:{ports[0]}"]
    else:
        gcs_port, trk_port = ports[0] + 1000, ports[1] + 1000
        from pymavlink import mavutil  # noqa: F401
        obs = MavObserver(gcs_port)
        mock = subprocess.Popen([sys.executable, os.path.join(HERE, "mock_ardupilot.py"), "--port", str(ports[0]),
                                 "--out", f"127.0.0.1:{gcs_port}", f"127.0.0.1:{trk_port}"],
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        time.sleep(1.5)
        argv = ["--fc", "ardupilot", "--connect", f"udpin:0.0.0.0:{trk_port}"]

    threading.Thread(target=obs.run, args=(log, stop), daemon=True).start()
    vt.DETECTORS["block"] = BlockDetector
    args = vt.parse_args(argv + ["--detector", "block", "--headless", "--sim-pan", "--lost-timeout", "2.5"])

    if pilot:
        def veto():
            if fc == "inav":
                mock.stdin.write("tx 7 1000\n")  # the pilot flips the MSP RC OVERRIDE (kill) switch off
                mock.stdin.flush()
            else:
                obs.pilot_veto()
        threading.Timer(PILOT_AT, veto).start()

    try:
        trk = vt.run(args, frames=scene(time.monotonic()))
    finally:
        time.sleep(0.6)
        stop.set()
        mock.terminate()
        mock_out = mock.communicate(timeout=5)[0]

    print("--- mock log ---\n" + mock_out.strip())
    seq = [m for _, m in log["modes"]]
    print("\nmodes seen by the independent observer:", " -> ".join(seq))
    checks = []

    def check(name, ok, detail=""):
        checks.append(ok)
        print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}")

    overrides = seq.count("OVERRIDE")
    on = next((t for t, m in log["modes"] if m == "OVERRIDE"), None)
    off = next((t for t, m in log["modes"] if on is not None and t > on and m != "OVERRIDE"), None)

    if pilot:
        check("exactly one override, then released", overrides == 1 and off is not None, f"({seq})")
        check("tracker let go and is not commanding", trk.state in (vt.PILOT, vt.PATROL),
              f"(state {trk.state})")
        if fc == "ardupilot":
            check("aircraft stayed where the pilot put it (LOITER)", seq[-1] == "OTHER", f"({seq[-1]})")
        else:
            check("mission came back by itself once override was refused", seq[-1] == "MISSION", f"({seq[-1]})")
        return all(checks)

    check("MISSION -> OVERRIDE -> MISSION", seq[:3] == ["MISSION", "OVERRIDE", "MISSION"] and overrides == 1, f"({seq})")
    if on is not None and off is not None:
        yaw_in = [y for t, y in log["yaw"] if on <= t <= off - 0.5]
        turned = (yaw_in[-1] - yaw_in[0] + 180) % 360 - 180
        print("yaw while overridden (every ~1 s):", [round(y) for y in yaw_in[::max(1, len(yaw_in) // 14)]])
        check("heading turned clockwise to centre the person (~21 deg)", 14 < turned < 28, f"(turned {turned:+.1f} deg)")
        pos = [(la, lo) for t, la, lo in log["pos"] if on + 2 <= t <= off]
        drift = math.hypot((pos[-1][0] - pos[0][0]) * 111320, (pos[-1][1] - pos[0][1]) * 111320)
        check("position held while overridden", drift < 6.0, f"(moved {drift:.1f} m)")
    else:
        check("override window found", False)
    if fc == "ardupilot":
        check("STATUSTEXT narration reached the GCS",
              any("MISSION OVERRIDE" in t for t in log["text"]) and any("RESUMING" in t for t in log["text"]))
    check("final state is PATROL", trk.state == vt.PATROL, f"(state {trk.state})")
    return all(checks)


def unit():
    """Dwell lock, search-before-resume and colour detector, on the fake FC (no processes, ~8 s)."""
    print("\n=========== UNIT - dwell lock / search / colour detector")
    checks = []

    def check(name, ok, detail=""):
        checks.append(ok)
        print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}")

    def trk_for(*extra):
        a = vt.parse_args(["--no-fc", "--headless", "--confirm-frames", "2", *extra])
        fc = vt.NullLink()
        return vt.VisualTrack(fc, a), fc

    def feed(trk, det, secs, fps=20):
        for _ in range(int(secs * fps)):
            trk.update([det] if det else [], W, H)
            time.sleep(1 / fps)

    centred, edge = vt.Det(300, 200, 40, 100), vt.Det(560, 200, 40, 100)

    trk, _ = trk_for("--lock-seconds", "1.0")
    feed(trk, centred, 0.6)
    check("dwell: not triggered before --lock-seconds", trk.state == vt.PATROL, f"({trk.state})")
    feed(trk, centred, 0.8)
    check("dwell: triggered once the target has dwelt long enough", trk.state != vt.PATROL, f"({trk.state})")

    trk, _ = trk_for("--lock-seconds", "1.0")
    feed(trk, edge, 2.0)
    check("dwell: target outside the centre box never triggers", trk.state == vt.PATROL, f"({trk.state})")
    feed(trk, centred, 0.7)
    feed(trk, edge, 0.8)
    feed(trk, centred, 0.7)
    check("dwell: leaving the box restarts the clock", trk.state == vt.PATROL, f"({trk.state})")

    trk, _ = trk_for()
    feed(trk, edge, 0.4)
    check("dwell off (default): triggers anywhere in the frame", trk.state != vt.PATROL, f"({trk.state})")

    def lose_target(*extra):
        trk, fc = trk_for("--lost-timeout", "1.5", *extra)
        rates = []
        fc.yaw_rate = rates.append
        feed(trk, vt.Det(500, 200, 40, 100), 0.5)  # right of centre -> clockwise error
        feed(trk, None, 1.0)
        return trk, rates

    trk, rates = lose_target("--search-rate", "20")
    check("search: keeps turning toward the side it vanished on", trk.state == vt.LOCKED and rates[-1] == 20.0,
          f"(last rate {rates[-1]:+.1f})")
    trk, rates = lose_target()
    check("search off (default): holds heading when the target drops out", rates[-1] == 0.0, f"(last rate {rates[-1]:+.1f})")

    for colour, bgr in (("green", (0, 200, 0)), ("orange", (0, 140, 255)), ("red", (0, 0, 220)), ("blue", (220, 60, 0))):
        img = np.full((H, W, 3), 40, np.uint8)
        cv2.rectangle(img, (380, 120), (460, 260), bgr, -1)
        found = vt.ColorDetector(colour).detect(img)
        ok = len(found) == 1 and abs(found[0].cx - 420) < 12 and abs(found[0].cy - 190) < 12
        check(f"colour detector finds a {colour} block", ok, f"({[(d.x, d.y, d.w, d.h) for d in found]})")
    check("colour detector ignores other colours", vt.ColorDetector("green").detect(
        cv2.rectangle(np.full((H, W, 3), 40, np.uint8), (380, 120), (460, 260), (0, 0, 220), -1)) == [])
    return all(checks)


if __name__ == "__main__":
    which = sys.argv[1:] or ["unit", "inav", "ardupilot"]
    ok = True
    if "unit" in which:
        r = unit()
        print("RESULT:", "ALL PASS" if r else "FAILURES")
        ok = ok and r
        which = [w for w in which if w != "unit"]
    for fc in which:
        for pilot in (False, True):
            r = run_case(fc, pilot)
            print("RESULT:", "ALL PASS" if r else "FAILURES")
            ok = ok and r
    print("\nOVERALL:", "ALL PASS" if ok else "FAILURES")
    sys.exit(0 if ok else 1)
