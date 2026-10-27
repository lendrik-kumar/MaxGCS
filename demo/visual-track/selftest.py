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
REC_VIDEO = os.path.join(HERE, "runs", "selftest-mockcam.mp4")  # runs/ is git-ignored
REC_REPORT = os.path.join(HERE, "runs", "selftest-mockcam.png")
vt_tunes = {k: v.decode() for k, v in __import__("fc_ardupilot").ArduPilotLink.TUNES.items()}
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
        log["abs0"] = t0  # log times are seconds since this; lets a test line them up with other clocks
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


def run_mockcam():
    """Closed loop on the simulated ArduPilot with the simulated camera (`--source mock`).

    The target building sways +/-18 deg left and right. Nothing here tells the tracker where it is: the
    picture is drawn from the heading the FC reports, the colour detector finds the building in it, and
    the yaw commands must keep the drone pointed at the building as it moves. Checks that mission override,
    lock, yaw and the camera view stay in step, then that the mission resumes when the target goes away."""
    print("\n=========== ARDUPILOT - simulated camera: lock and track a swaying building")
    from pymavlink import mavutil  # noqa: F401
    gcs_port, trk_port, fc_port = 15710, 15711, 14710
    log = {"modes": [], "yaw": [], "text": [], "pos": []}
    stop = threading.Event()
    obs = MavObserver(gcs_port)
    mock = subprocess.Popen([sys.executable, os.path.join(HERE, "mock_ardupilot.py"), "--port", str(fc_port),
                             "--size", "40", "--out", f"127.0.0.1:{gcs_port}", f"127.0.0.1:{trk_port}"],
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    time.sleep(1.5)
    threading.Thread(target=obs.run, args=(log, stop), daemon=True).start()
    args = vt.parse_args(["--fc", "ardupilot", "--connect", f"udpin:0.0.0.0:{trk_port}", "--source", "mock",
                          "--detector", "color", "--color", "orange", "--headless", "--lost-timeout", "2.5",
                          "--kp", "1.5", "--mock-hold", "24", "--mock-hide", "999", "--duration", "75",
                          "--record", REC_VIDEO, "--report", REC_REPORT])
    try:
        trk = vt.run(args)
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

    on = next((t for t, m in log["modes"] if m == "OVERRIDE"), None)
    off = next((t for t, m in log["modes"] if on is not None and t > on and m != "OVERRIDE"), None)
    check("MISSION -> OVERRIDE -> MISSION", seq[:3] == ["MISSION", "OVERRIDE", "MISSION"] and seq.count("OVERRIDE") == 1,
          f"({seq})")
    if on is None or off is None:
        check("override window found", False)
        return False

    cam, abs0 = trk.cam, log["abs0"]
    win = [(t, y) for t, y in log["yaw"] if on + 6.0 <= t <= off - 0.5]  # skip the initial swing-round
    errs = [abs((y - cam.target_bearing(abs0 + t) + 180) % 360 - 180) for t, y in win]
    errs_sorted = sorted(errs)
    med, p95 = errs_sorted[len(errs) // 2], errs_sorted[int(len(errs) * 0.95)]
    print(f"locked for {off - on:.1f} s; heading-vs-target error: median {med:.1f} deg, 95th pct {p95:.1f} deg")
    check("heading stays on the swaying target", med < 10 and p95 < 22, f"(median {med:.1f}, p95 {p95:.1f})")

    yaws = [y for t, y in win]
    swing = max(((y - yaws[0] + 180) % 360 - 180) for y in yaws) - min(((y - yaws[0] + 180) % 360 - 180) for y in yaws)
    check("tracked it left AND right", swing > 20, f"(heading swung {swing:.0f} deg)")

    first = [y for t, y in log["yaw"] if on <= t <= on + 0.3]
    turned = abs((yaws[0] - first[0] + 180) % 360 - 180) if first else 0
    check("turned toward the target from the mission heading", turned > 8, f"(turned {turned:.0f} deg after the override)")

    pos = [(la, lo) for t, la, lo in log["pos"] if on + 2 <= t <= off]
    drift = math.hypot((pos[-1][0] - pos[0][0]) * 111320, (pos[-1][1] - pos[0][1]) * 111320)
    check("position held while overridden", drift < 6.0, f"(moved {drift:.1f} m)")
    check("narration reached the GCS in order",
          [i for i, t in enumerate(log["text"]) if "MISSION OVERRIDE" in t][:1] <
          [i for i, t in enumerate(log["text"]) if "RESUMING" in t][:1] and
          any("MISSION OVERRIDE" in t for t in log["text"]))
    tunes = [ln.split("BUZZER", 1)[1] for ln in mock_out.splitlines() if "BUZZER" in ln]
    got = [t.rsplit(":", 1)[1].strip().strip(chr(0)) for t in tunes]
    print("buzzer cues the FC received:", got)
    check("FC buzzer played detected -> locked -> lost -> resumed", got[:4] == [vt_tunes[k] for k in ("detected", "locked", "lost", "resumed")],
          f"(got {got[:4]!r}, want {[vt_tunes[k] for k in ("detected", "locked", "lost", "resumed")]!r})")
    check("yaw-controller history recorded for the panel", len(trk.hist) > 100, f"({len(trk.hist)} samples)")

    cap = cv2.VideoCapture(REC_VIDEO)
    n, w, h = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)), int(cap.get(3)), int(cap.get(4))
    mid = None
    if n > 0:
        cap.set(cv2.CAP_PROP_POS_FRAMES, n // 2)
        _, mid = cap.read()
    cap.release()
    check("--record: video plays back", n > 500 and (w, h) == (640, 480) and mid is not None and mid.std() > 10,
          f"({n} frames, {w}x{h})")
    rep = cv2.imread(REC_REPORT)
    check("--report: timeline PNG written", rep is not None and rep.shape == (900, 1400, 3) and rep.std() > 20,
          f"({None if rep is None else rep.shape})")
    check("final state is PATROL", trk.state == vt.PATROL, f"(state {trk.state})")
    return all(checks)


def run_predict():
    """Blind-spot tracking without hardware: the predictor's maths, then the tracker keeping its lock."""
    print("\n=========== PREDICT - tracking through blind spots")
    from predictor import BearingPredictor, wrap180
    checks = []

    def check(name, ok, detail=""):
        checks.append(ok)
        print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}")

    p = BearingPredictor()
    for i in range(60):
        p.update(50 + 10 * (i * 0.05), i * 0.05)
    check("filter learns a 10 deg/s target speed", 8.5 < p.vel < 11.5, f"({p.vel:.1f} deg/s)")
    start, t0 = p.est, 59 * 0.05
    for i in range(20):
        p.coast(t0 + (i + 1) * 0.05)
    moved = wrap180(p.est - start)
    check("coasting for 1 s carries on in the same direction", 6.5 < moved < 10.0, f"(moved {moved:+.1f} deg)")
    check("...and the speed fades while it cannot see the target", 0 < p.vel < 9.0, f"({p.vel:.1f} deg/s)")
    p = BearingPredictor()
    for i in range(40):
        p.update((355 + 10 * i * 0.05) % 360, i * 0.05)
    check("works across north (359 -> 1 deg)", 8.0 < p.vel < 12.0, f"({p.vel:.1f} deg/s, est {p.est:.1f})")

    # the tracker in a closed loop with a fake drone that turns as commanded (NullLink integrates yaw)
    a = vt.parse_args(["--no-fc", "--headless", "--confirm-frames", "2", "--lost-timeout", "1.0",
                       "--cooldown", "0", "--predict-seconds", "1.5", "--kp", "1.0"])
    fc = vt.NullLink()
    trk = vt.VisualTrack(fc, a)
    fx = 320 / math.tan(math.radians(a.hfov) / 2)
    t_start = time.monotonic()

    def bearing(t):
        return 10 + 12 * t  # a target walking round at 12 deg/s

    def det_at(world):
        rel = wrap180(world - fc.yaw_deg)
        return vt.Det(int(320 + fx * math.tan(math.radians(rel))) - 20, 200, 40, 100)

    def step(visible, far=False):
        dets = []
        if visible:
            dets = [vt.Det(20, 200, 40, 100)] if far else [det_at(bearing(time.monotonic() - t_start))]
        got = trk.update(dets, 640)
        time.sleep(0.05)
        return got

    for _ in range(60):
        step(True)
    check("locked on the moving target", trk.state == vt.LOCKED, f"({trk.state})")
    rates = []
    for _ in range(20):  # 1 s hidden
        step(False)
        rates.append(trk.last_cmd)
    check("hidden: still LOCKED, not given up", trk.state == vt.LOCKED and trk.occluded_since is not None)
    check("hidden: keeps turning the way the target was going", all(r >= 0 for r in rates) and max(rates) > 2.0,
          f"(rates {min(rates):+.1f}..{max(rates):+.1f} deg/s; holding heading would be 0)")
    check("hidden: a predicted position is available for the display", trk.pred_px is not None)
    got = step(True)
    check("target back in the right place: lock kept, prediction cleared",
          got is not None and trk.state == vt.LOCKED and trk.occluded_since is None)
    for _ in range(6):
        step(False)
    got = step(True, far=True)
    check("someone else far from the prediction is not taken as the target",
          got is None and trk.occluded_since is not None)
    t_gone = time.monotonic()
    while trk.state == vt.LOCKED and time.monotonic() - t_gone < 6.0:
        step(False)
    gone = time.monotonic() - t_gone
    check("gone for good: mission handed back after prediction window + lost timeout",
          trk.state in (vt.RESUME, vt.PATROL) and 2.0 < gone < 3.4, f"(after {gone:.1f} s; expect about 1.5 + 1.0)")

    a0 = vt.parse_args(["--no-fc", "--headless", "--confirm-frames", "2", "--lost-timeout", "1.0",
                        "--cooldown", "0", "--predict-seconds", "0"])
    trk0 = vt.VisualTrack(vt.NullLink(), a0)
    for _ in range(10):
        trk0.update([vt.Det(400, 200, 40, 100)], 640)
        time.sleep(0.05)
    for _ in range(10):
        trk0.update([], 640)
        time.sleep(0.05)
    check("--predict-seconds 0 keeps the old behaviour (hold heading)",
          trk0.state == vt.LOCKED and trk0.last_cmd == 0.0 and trk0.occluded_since is None)
    return all(checks)


def run_occlusion():
    """A/B on the closed-loop simulated ArduPilot: the same swaying target passes behind a tree trunk again and
    again; compare how well the heading stays on it with prediction on and off."""
    print("\n=========== OCCLUSION - prediction on vs off, target swaying behind a tree trunk")
    from pymavlink import mavutil  # noqa: F401
    checks = []

    def check(name, ok, detail=""):
        checks.append(ok)
        print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}")

    def one(predict_s, base):
        gcs_port, trk_port, fc_port = base + 1, base + 2, base
        log = {"modes": [], "yaw": [], "text": [], "pos": []}
        stop = threading.Event()
        obs = MavObserver(gcs_port)
        mock = subprocess.Popen([sys.executable, os.path.join(HERE, "mock_ardupilot.py"), "--port", str(fc_port),
                                 "--size", "40", "--out", f"127.0.0.1:{gcs_port}", f"127.0.0.1:{trk_port}"],
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        time.sleep(1.5)
        threading.Thread(target=obs.run, args=(log, stop), daemon=True).start()
        args = vt.parse_args(["--fc", "ardupilot", "--connect", f"udpin:0.0.0.0:{trk_port}", "--source", "mock",
                              "--detector", "color", "--color", "orange", "--headless", "--kp", "1.5",
                              "--mock-sway", "25", "--mock-period", "10", "--mock-occluder", "60", "--mock-hold", "0",
                              "--predict-seconds", str(predict_s), "--lost-timeout", "3", "--duration", "62"])
        try:
            trk = vt.run(args)
        finally:
            time.sleep(0.6)
            stop.set()
            mock.terminate()
            mock.communicate(timeout=5)
        seq = [m for _, m in log["modes"]]
        on = next((t for t, m in log["modes"] if m == "OVERRIDE"), None)
        off = next((t for t, m in log["modes"] if on is not None and t > on and m != "OVERRIDE"), log["yaw"][-1][0])
        cam, abs0 = trk.cam, log["abs0"]
        errs = []
        for t, y in log["yaw"]:
            if on is not None and on + 8.0 <= t <= off - 1.0:
                b = cam.target_bearing(abs0 + t)
                if abs((b - 60 + 180) % 360 - 180) < 16.0:  # the target is (at least partly) behind the trunk
                    errs.append(abs((y - b + 180) % 360 - 180))
        return trk, seq, errs

    trk_on, seq_on, e_on = one(2.0, 14740)
    trk_off, seq_off, e_off = one(0.0, 14750)
    print(f"seen by the independent observer, prediction on:  {' -> '.join(seq_on)}")
    print(f"seen by the independent observer, prediction off: {' -> '.join(seq_off)}")
    check("each run overrode the mission once and held it (no early give-up, no re-trigger)",
          seq_on.count("OVERRIDE") == 1 and seq_off.count("OVERRIDE") == 1)
    check("the target really did go behind the trunk, several times", trk_on.occlusions >= 3,
          f"({trk_on.occlusions} blind spots handled)")
    if len(e_on) < 20 or len(e_off) < 20:
        check("enough samples behind the trunk to compare", False, f"(on {len(e_on)}, off {len(e_off)})")
        return False
    m_on, m_off = sum(e_on) / len(e_on), sum(e_off) / len(e_off)
    print(f"heading error while the target is behind the trunk (mean): prediction ON {m_on:.1f} deg, OFF {m_off:.1f} deg "
          f"({len(e_on)} / {len(e_off)} samples)")
    check("prediction keeps the heading closer to the hidden target", m_on < 0.9 * m_off,
          f"(on {m_on:.1f} deg vs off {m_off:.1f} deg)")
    return all(checks)


def run_cues():
    """Pilot cues without hardware: rumble patterns on a fake gamepad, and which tracker event fires when."""
    print("\n=========== CUES - gamepad rumble patterns and tracker events")
    import haptics
    checks = []

    def check(name, ok, detail=""):
        checks.append(ok)
        print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}")

    class FakePad:
        def __init__(self):
            self.calls = []

        def set(self, low, high):
            self.calls.append((low, high))
            return True

    class FakeClock:
        t = 100.0

        def __call__(self):
            return self.t

    for name, pattern in haptics.PATTERNS.items():
        pad, clock = FakePad(), FakeClock()
        h = haptics.Haptics(pad, clock)
        h.cue(name)
        for _ in range(int(sum(d for d, _, _ in pattern) / 0.01) + 20):
            clock.t += 0.01
            h.tick()
        buzzes = sum(1 for i, c in enumerate(pad.calls) if c != (0.0, 0.0) and (i == 0 or pad.calls[i - 1] == (0.0, 0.0)))
        want = sum(1 for i, (_, lo, hi) in enumerate(pattern) if (lo or hi) and (i == 0 or not (pattern[i - 1][1] or pattern[i - 1][2])))
        check(f"rumble '{name}': {want} buzz(es), then silence", buzzes == want and pad.calls[-1] == (0.0, 0.0),
              f"({buzzes} buzzes, ends {pad.calls[-1]})")
    check("the five events feel different", len({tuple(p) for p in haptics.PATTERNS.values()}) == 5)

    a = vt.parse_args(["--no-fc", "--headless", "--confirm-frames", "2", "--lost-timeout", "1.0", "--cooldown", "0",
                       "--predict-seconds", "0"])
    fc, events = vt.NullLink(), []
    trk = vt.VisualTrack(fc, a)
    trk.cues.append(events.append)
    det = vt.Det(450, 200, 40, 100)
    for dets, secs in (([det], 0.6), ([], 2.5)):
        for _ in range(int(secs * 20)):
            trk.update(dets, 640)
            time.sleep(0.05)
    check("events fire once each, in order", events == ["detected", "locked", "lost", "resumed"], f"({events})")
    check("controller history has error and rate samples", len(trk.hist) > 5 and any(c != 0 for _, _, c in trk.hist),
          f"({len(trk.hist)} samples)")

    def boom(_):
        raise RuntimeError("no gamepad")
    a2 = vt.parse_args(["--no-fc", "--headless", "--confirm-frames", "2"])
    trk2 = vt.VisualTrack(vt.NullLink(), a2)
    trk2.cues.append(boom)
    for _ in range(10):
        trk2.update([det], 640)
    check("a failing cue device never stops the tracker", trk2.state != vt.PATROL, f"(state {trk2.state})")
    return all(checks)


if __name__ == "__main__":
    which = sys.argv[1:] or ["inav", "ardupilot", "mockcam", "cues", "predict", "occlusion"]
    ok = True
    if "predict" in which:
        r = run_predict()
        print("RESULT:", "ALL PASS" if r else "FAILURES")
        ok = ok and r
        which = [w for w in which if w != "predict"]
    if "occlusion" in which:
        r = run_occlusion()
        print("RESULT:", "ALL PASS" if r else "FAILURES")
        ok = ok and r
        which = [w for w in which if w != "occlusion"]
    if "cues" in which:
        r = run_cues()
        print("RESULT:", "ALL PASS" if r else "FAILURES")
        ok = ok and r
        which = [w for w in which if w != "cues"]
    if "mockcam" in which:
        r = run_mockcam()
        print("RESULT:", "ALL PASS" if r else "FAILURES")
        ok = ok and r
        which = [w for w in which if w != "mockcam"]
    for fc in which:
        for pilot in (False, True):
            r = run_case(fc, pilot)
            print("RESULT:", "ALL PASS" if r else "FAILURES")
            ok = ok and r
    print("\nOVERALL:", "ALL PASS" if ok else "FAILURES")
    sys.exit(0 if ok else 1)
