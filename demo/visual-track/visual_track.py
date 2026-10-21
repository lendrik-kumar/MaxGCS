"""Visual lock-and-track: a camera trigger overrides the mission plan and yaws the drone.

Companion-computer process (runs next to the camera, talks to the flight controller):

    PATROL    drone flies its mission. Camera frames are scanned for a person.
    OVERRIDE  person confirmed for N frames  ->  take the mission away from the FC and hold position.
    LOCKED    steer the drone's yaw so the person stays centred in the frame.
    RESUME    person gone for --lost-timeout seconds  ->  hand the mission back; the FC carries on.

How "take the mission away" works depends on the firmware (see the fc_*.py backends):
    INAV        MSP RC injection: NAV WP off, NAV POSHOLD on, yaw stick offset (needs INAV 8.0+).
    ArduPilot   MAVLink: AUTO -> GUIDED, velocity 0 + yaw rate.

Safety: the pilot always wins. If the FC leaves the override state while we are tracking (INAV: the
MSP RC OVERRIDE switch is turned off; ArduPilot: any mode change) we stop commanding immediately and
stay passive until the mission is running again. We also hand the mission back if this script exits.

Examples:
    python visual_track.py --detector face --mirror --sim-pan      # INAV on the mock FC (mock_inav.py)
    python visual_track.py --connect COM7,115200                   # real INAV, a spare UART set to MSP
    python visual_track.py --fc ardupilot                          # ArduPilot / SITL / mock_ardupilot.py
    python visual_track.py --no-fc                                 # vision only, to check the camera
"""
import argparse
import math
import sys
import time
from dataclasses import dataclass

import cv2  # noqa: E402
import numpy as np  # noqa: E402


# ===================================================================================================
# Detectors: frame (BGR) -> list of (x, y, w, h, confidence) in frame pixels
# ===================================================================================================
@dataclass
class Det:
    x: int
    y: int
    w: int
    h: int
    conf: float = 1.0

    @property
    def cx(self):
        return self.x + self.w / 2

    @property
    def cy(self):
        return self.y + self.h / 2

    @property
    def area(self):
        return self.w * self.h


class _Scaled:
    """Run a detector on a downscaled copy (speed) and map boxes back to full-frame pixels."""

    max_w = 480

    def detect(self, frame):
        h, w = frame.shape[:2]
        s = min(1.0, self.max_w / w)
        small = cv2.resize(frame, (int(w * s), int(h * s))) if s < 1.0 else frame
        return [Det(int(d.x / s), int(d.y / s), int(d.w / s), int(d.h / s), d.conf)
                for d in self._detect(small)]


class HogDetector(_Scaled):
    """OpenCV's built-in full-body pedestrian detector. Zero downloads; best for a standing person
    seen from the side/front at moderate range. Weak from directly above or at long range."""

    name = "hog"

    def __init__(self, min_conf=0.5):
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        self.min_conf = min_conf

    def _detect(self, img):
        rects, weights = self.hog.detectMultiScale(img, winStride=(8, 8), padding=(8, 8), scale=1.06)
        dets = [Det(int(x), int(y), int(w), int(h), float(c))
                for (x, y, w, h), c in zip(rects, np.ravel(weights)) if c >= self.min_conf]
        # HOG fires several overlapping boxes per person; drop any box mostly inside a bigger one
        keep = []
        for d in sorted(dets, key=lambda d: -d.area):
            if not any(_overlap(d, k) > 0.6 for k in keep):
                keep.append(d)
        return keep


class FaceDetector(_Scaled):
    """OpenCV Haar frontal-face cascade. Zero downloads; the most reliable choice for a laptop
    webcam on a desk (the person sits close and faces the camera)."""

    name = "face"

    def __init__(self):
        self.cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
        if self.cascade.empty():
            sys.exit("face cascade not found in this OpenCV build")

    def _detect(self, img):
        gray = cv2.equalizeHist(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        rects = self.cascade.detectMultiScale(gray, scaleFactor=1.15, minNeighbors=5, minSize=(40, 40))
        return [Det(int(x), int(y), int(w), int(h)) for x, y, w, h in rects]


class YoloDetector:
    """Optional: Ultralytics YOLO, class 'person'. Far more robust (any pose, aerial view, range).
    Needs `pip install ultralytics` (pulls PyTorch); the weights file downloads on first use."""

    name = "yolo"

    def __init__(self, weights="yolov8n.pt", min_conf=0.4):
        try:
            from ultralytics import YOLO
        except ImportError:
            sys.exit("--detector yolo needs: pip install ultralytics")
        self.model, self.min_conf = YOLO(weights), min_conf

    def detect(self, frame):
        out = []
        for r in self.model.predict(frame, classes=[0], conf=self.min_conf, verbose=False):
            for (x1, y1, x2, y2), c in zip(r.boxes.xyxy.tolist(), r.boxes.conf.tolist()):
                out.append(Det(int(x1), int(y1), int(x2 - x1), int(y2 - y1), float(c)))
        return out


def _overlap(a, b):
    ix = max(0, min(a.x + a.w, b.x + b.w) - max(a.x, b.x))
    iy = max(0, min(a.y + a.h, b.y + b.h) - max(a.y, b.y))
    return ix * iy / max(1, min(a.area, b.area))


class ColorDetector(_Scaled):
    """Brightly coloured object (a vest, ball, panel) found by HSV thresholding. Zero downloads, works from
    any angle and range the colour is visible, and far steadier than HOG/face for a camera demo. The 'target'
    is then the coloured object rather than a person."""

    name = "color"
    # colour -> list of (lower HSV, upper HSV); OpenCV hue runs 0-179, red wraps around it
    RANGES = {
        "green": [((35, 80, 60), (85, 255, 255))],
        "orange": [((5, 120, 100), (22, 255, 255))],
        "yellow": [((22, 100, 100), (35, 255, 255))],
        "blue": [((95, 120, 60), (130, 255, 255))],
        "red": [((0, 120, 80), (8, 255, 255)), ((172, 120, 80), (179, 255, 255))],
    }
    min_area_frac = 0.002  # smallest blob, as a fraction of the (downscaled) frame

    def __init__(self, colour="green"):
        if colour not in self.RANGES:
            sys.exit(f"--color must be one of {', '.join(self.RANGES)}")
        self.colour = colour

    def _detect(self, img):
        hsv = cv2.cvtColor(cv2.GaussianBlur(img, (7, 7), 0), cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in self.RANGES[self.colour]:
            mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel), cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        floor = self.min_area_frac * img.shape[0] * img.shape[1]
        return [Det(*cv2.boundingRect(c)) for c in contours if cv2.contourArea(c) >= floor]


DETECTORS = {"hog": HogDetector, "face": FaceDetector, "yolo": YoloDetector, "color": ColorDetector}


def make_detector(args):
    cls = DETECTORS[args.detector]
    return cls(args.color) if cls is ColorDetector else cls()


# ===================================================================================================
# Flight-controller backends (fc_inav.InavLink, fc_ardupilot.ArduPilotLink) share this interface:
#   armed, yaw_deg, alive, mission_running, override_allowed, override_active, mode_text
#   heartbeat() begin_override() yaw_rate(deg_s) end_override() release() say(text, warn) close()
# ===================================================================================================
class NullLink:
    """--no-fc: vision only. Acts like an armed aircraft flying its mission so the whole state machine runs."""

    armed, yaw_deg, alive = True, 0.0, True

    def __init__(self):
        self.overridden = False

    override_allowed = True
    mission_running = property(lambda self: not self.overridden)
    override_active = property(lambda self: self.overridden)
    mode_text = property(lambda self: "OVERRIDE (no FC)" if self.overridden else "MISSION (no FC)")

    def heartbeat(self):
        pass

    def begin_override(self):
        self.overridden = True

    def end_override(self):
        self.overridden = False

    def release(self):
        pass

    def yaw_rate(self, rate):
        now = time.monotonic()  # integrate so --sim-pan still has something to react to
        dt = now - getattr(self, "_t", now)
        self._t = now
        self.yaw_deg = (self.yaw_deg + rate * min(dt, 0.2)) % 360

    def say(self, text, warn=False):
        print(f"[fc-msg] {text}", flush=True)

    def close(self):
        pass


def connect_fc(args):
    if args.no_fc:
        return NullLink()
    print(f"[link] {args.fc}: connecting {args.connect} ...", flush=True)
    if args.fc == "inav":
        from fc_inav import InavLink, MspError
        try:
            fc = InavLink(args.connect, args)
        except (MspError, OSError) as e:
            sys.exit(f"INAV link failed: {e}")
    else:
        from fc_ardupilot import ArduPilotLink
        fc = ArduPilotLink(args)
    t0 = time.monotonic()
    while not fc.alive:
        if time.monotonic() - t0 > 5:
            sys.exit("connected, but the flight controller is not answering telemetry polls")
        time.sleep(0.1)
    print(f"[link] FC: {fc.mode_text}, {'ARMED' if fc.armed else 'DISARMED'}, "
          f"mission {'running' if fc.mission_running else 'NOT running'}", flush=True)
    return fc


# ===================================================================================================
# Trigger -> override -> track -> resume
# ===================================================================================================
PATROL, OVERRIDE, LOCKED, RESUME, PILOT = "PATROL", "OVERRIDE", "LOCKED", "RESUME", "PILOT"
DWELL_GRACE = 0.5  # seconds a dwell survives without the target in the lock zone (detector flicker)


class VisualTrack:
    def __init__(self, fc, a):
        self.fc, self.a = fc, a
        self.state = PATROL
        self.hits = 0
        self.t_state = time.monotonic()
        self.t_seen = 0.0
        self.cooldown_until = 0.0
        self.sm_cx = None  # smoothed target centre x
        self.last_cmd = 0.0
        self.err_deg = 0.0
        self.fx = None  # focal length in px, set from first frame size
        self.ref_yaw = None  # heading when lock started (for --sim-pan)
        self.dwell_t0 = None  # when the target first sat inside the lock zone (--lock-seconds)
        self.dwell_ok = 0.0  # last time it was inside
        self.search_dir = 0  # +1 / -1: side the target was last seen on, for --search-rate
        self.frame_h = None

    def _go(self, state):
        self.state, self.t_state = state, time.monotonic()
        print(f"[track] -> {state}", flush=True)

    def _pick(self, dets):
        if not dets:
            return None
        if self.sm_cx is not None and self.state == LOCKED:  # stay on the person we were following
            near = min(dets, key=lambda d: abs(d.cx - self.sm_cx))
            if abs(near.cx - self.sm_cx) < 0.25 * self.frame_w:
                return near
        return max(dets, key=lambda d: d.area)

    def lock_zone(self):
        """Centre box (x0, y0, x1, y1) the target must sit in for --lock-seconds; None if no frame seen yet."""
        if not self.frame_w or not self.frame_h:
            return None
        half = self.a.lock_zone / 2
        return (self.frame_w * (0.5 - half), self.frame_h * 0.1, self.frame_w * (0.5 + half), self.frame_h * 0.9)

    def _in_zone(self, target):
        z = self.lock_zone()
        return z is not None and z[0] < target.cx < z[2] and z[1] < target.cy < z[3]

    @property
    def dwell_progress(self):
        """Seconds the target has dwelt in the lock zone so far (0 when --lock-seconds is off)."""
        return 0.0 if self.dwell_t0 is None else time.monotonic() - self.dwell_t0

    def update(self, dets, frame_w, frame_h=None):
        """Advance one frame. Returns the target Det being followed (or None) for the HUD."""
        a, fc, now = self.a, self.fc, time.monotonic()
        self.frame_w, self.frame_h = frame_w, frame_h
        self.fx = (frame_w / 2) / math.tan(math.radians(a.hfov) / 2)
        target = self._pick(dets)

        if self.state == PATROL:
            armed_trigger = target and fc.mission_running and fc.override_allowed and now >= self.cooldown_until
            dwell_met = True
            if a.lock_seconds > 0:
                # Dwell lock: the target must stay in the centre box for lock_seconds before we trigger. A
                # short detector dropout (DWELL_GRACE) does not restart the clock; leaving the box does.
                if armed_trigger and self._in_zone(target):
                    if self.dwell_t0 is None:
                        self.dwell_t0 = now
                    self.dwell_ok = now
                elif self.dwell_t0 is not None and now - self.dwell_ok > DWELL_GRACE:
                    self.dwell_t0 = None
                # dwell_ok (not now) so the clock only completes on a frame where the target is in the box
                dwell_met = self.dwell_t0 is not None and self.dwell_ok - self.dwell_t0 >= a.lock_seconds
            if armed_trigger and dwell_met:
                self.hits += 1
            else:
                self.hits = max(0, self.hits - 1)
            if self.hits >= a.confirm_frames:
                self.hits = 0
                self.dwell_t0 = None
                self.sm_cx = target.cx
                self.ref_yaw = fc.yaw_deg
                fc.say("VISUAL TRACK: human detected - MISSION OVERRIDE")
                fc.begin_override()
                self._go(OVERRIDE)

        elif self.state == OVERRIDE:
            if fc.override_active:
                self.t_seen = now
                self.ref_yaw = fc.yaw_deg
                self.search_dir = 0
                fc.say("VISUAL TRACK: override confirmed - locking on")
                self._go(LOCKED)
            elif now - self.t_state > 1.0:
                if now - self.t_state > 4.0:
                    fc.say("VISUAL TRACK: FC did not accept override - abort", warn=True)
                    fc.release()
                    self.cooldown_until = now + a.cooldown
                    self._go(PATROL)
                else:
                    fc.begin_override()  # re-send once a second until the FC confirms

        elif self.state == LOCKED:
            if not fc.override_active:  # the pilot (or a failsafe) changed mode: stop commanding at once
                fc.say("VISUAL TRACK: pilot took over - released", warn=True)
                fc.release()
                self._go(PILOT)
                return target
            if target:
                self.t_seen = now
                self.sm_cx = target.cx if self.sm_cx is None else a.smooth * target.cx + (1 - a.smooth) * self.sm_cx
                self.err_deg = math.degrees(math.atan2(self.sm_cx - frame_w / 2, self.fx))
                rate = 0.0 if abs(self.err_deg) < a.deadband else max(-a.max_rate, min(a.max_rate, a.kp * self.err_deg))
                if abs(self.err_deg) >= a.deadband:
                    self.search_dir = 1 if self.err_deg > 0 else -1
            else:
                # Target momentarily gone: keep the override and either hold heading or, with --search-rate,
                # keep turning toward the side it disappeared on until --lost-timeout runs out.
                self.err_deg = 0.0
                rate = self.search_dir * min(a.search_rate, a.max_rate)
            fc.yaw_rate(rate)
            self.last_cmd = rate
            if now - self.t_seen > a.lost_timeout:
                self.last_cmd = 0.0
                fc.say("VISUAL TRACK: target lost - RESUMING MISSION")
                fc.end_override()
                self._go(RESUME)

        elif self.state == RESUME:
            if fc.mission_running:
                fc.release()
                self.cooldown_until = now + a.cooldown
                self.sm_cx = None
                self._go(PATROL)
            elif now - self.t_state > 1.0:
                if now - self.t_state > 6.0:
                    fc.say("VISUAL TRACK: could not resume mission - handing over", warn=True)
                    fc.release()
                    self._go(PILOT)  # leave it to the pilot
                else:
                    fc.end_override()

        elif self.state == PILOT:
            if fc.mission_running:  # mission is running again: re-arm the trigger
                self.cooldown_until = now + a.cooldown
                self._go(PATROL)

        return target

    def release(self):
        """Called on exit: never leave the aircraft parked in the override because this script quit."""
        fc = self.fc
        if self.state in (OVERRIDE, LOCKED, RESUME):
            fc.say("VISUAL TRACK: stopped - RESUMING MISSION")
            fc.end_override()
            t0 = time.monotonic()
            while not fc.mission_running and time.monotonic() - t0 < 3.0:
                time.sleep(0.1)
        fc.release()


# ===================================================================================================
# Camera, HUD, main loop
# ===================================================================================================
STATE_COLOUR = {PATROL: (0, 190, 0), OVERRIDE: (0, 140, 255), LOCKED: (0, 0, 230),
                RESUME: (0, 140, 255), PILOT: (200, 120, 0)}
STATE_TEXT = {
    PATROL: "PATROL  -  flying mission",
    OVERRIDE: "TRIGGER  -  taking over from mission",
    LOCKED: "MISSION OVERRIDDEN  -  tracking",
    RESUME: "TARGET LOST  -  resuming mission",
    PILOT: "PILOT IN CONTROL  -  tracking released",
}


def open_camera(src, w, h):
    s = int(src) if str(src).isdigit() else src
    cap = cv2.VideoCapture(s, cv2.CAP_DSHOW) if isinstance(s, int) and sys.platform == "win32" else cv2.VideoCapture(s)
    if not cap.isOpened():
        sys.exit(f"cannot open video source {src!r}")
    if isinstance(s, int):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    return cap


def draw_hud(frame, trk, fc, target, args, fps):
    h, w = frame.shape[:2]
    col = STATE_COLOUR[trk.state]
    cv2.rectangle(frame, (0, 0), (w, 34), col, -1)
    cv2.putText(frame, STATE_TEXT[trk.state], (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.line(frame, (w // 2, h // 2 - 14), (w // 2, h // 2 + 14), (255, 255, 255), 1)
    cv2.line(frame, (w // 2 - 14, h // 2), (w // 2 + 14, h // 2), (255, 255, 255), 1)
    if target:
        tcol = (0, 0, 255) if trk.state == LOCKED else (0, 255, 255)
        cv2.rectangle(frame, (target.x, target.y), (target.x + target.w, target.y + target.h), tcol, 2)
        cv2.circle(frame, (int(target.cx), int(target.cy)), 5, tcol, -1)
        if trk.state == LOCKED:
            cv2.line(frame, (w // 2, h // 2), (int(target.cx), int(target.cy)), tcol, 1)
        label = f"{'LOCK' if trk.state == LOCKED else 'target'} {target.conf:.2f}" if target.conf != 1.0 else "target"
        cv2.putText(frame, label, (target.x, max(14, target.y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, tcol, 1, cv2.LINE_AA)
    zone = trk.lock_zone() if args.lock_seconds > 0 and trk.state == PATROL else None
    if zone:
        cv2.rectangle(frame, (int(zone[0]), int(zone[1])), (int(zone[2]), int(zone[3])), (255, 255, 255), 1)
        cv2.putText(frame, f"locking {trk.dwell_progress:.1f}/{args.lock_seconds:.1f} s", (int(zone[0]) + 6, int(zone[1]) + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    yaw = f"{fc.yaw_deg:5.1f}" if fc.yaw_deg is not None else "  ?  "
    lines = [f"FC: {fc.mode_text}  {'ARMED' if fc.armed else 'disarmed'}   heading {yaw} deg",
             f"yaw cmd {trk.last_cmd:+5.1f} deg/s   err {trk.err_deg:+5.1f} deg   {fps:4.1f} fps  [{args.detector}]"]
    if args.sim_pan:
        lines.append("SIMULATED CAMERA PAN (view shifts with drone yaw)")
    for i, t in enumerate(reversed(lines)):
        cv2.putText(frame, t, (10, h - 10 - 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, t, (10, h - 10 - 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)


def sim_pan(frame, trk, fc):
    """Emulate a camera bolted to the airframe: shift the picture opposite to the yaw since lock."""
    if trk.state != LOCKED or trk.ref_yaw is None or fc.yaw_deg is None or trk.fx is None:
        return frame
    d = (fc.yaw_deg - trk.ref_yaw + 180) % 360 - 180
    shift = trk.fx * math.tan(math.radians(max(-80, min(80, d))))
    h, w = frame.shape[:2]
    return cv2.warpAffine(frame, np.float32([[1, 0, -shift], [0, 1, 0]]), (w, h))


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_argument_group("flight controller link")
    g.add_argument("--fc", choices=("inav", "ardupilot"), default="inav", help="firmware (default inav)")
    g.add_argument("--connect", default=None,
                   help="inav: 'COM7,115200' or 'tcp:host:port' (default tcp:127.0.0.1:5760 = mock_inav.py). "
                        "ardupilot: pymavlink string (default udpin:0.0.0.0:14551)")
    g.add_argument("--no-fc", action="store_true", help="vision only: no link, fake FC flying a mission")
    g = ap.add_argument_group("INAV")
    g.add_argument("--mission-box", type=int, default=28, help="permanent box id of the mission mode (28 = NAV WP)")
    g.add_argument("--hold-boxes", type=int, nargs="+", default=[11],
                   help="box ids to turn on while overriding (11 = NAV POSHOLD; add 3 = NAV ALTHOLD if your FC needs it)")
    g.add_argument("--yaw-ch", type=int, default=4, help="RC channel of the yaw stick (default 4, AETR order)")
    g.add_argument("--inav-yaw-rate", type=float, default=200.0,
                   help="deg/s the FC turns at full yaw stick (INAV 'yaw_rate' setting x10; default 200)")
    g.add_argument("--max-us", type=int, default=250, help="never move the yaw stick more than this from centre")
    g = ap.add_argument_group("ArduPilot")
    g.add_argument("--baud", type=int, default=57600, help="serial baud if --connect is a COM port")
    g.add_argument("--sysid", type=int, default=1, help="our MAVLink system id (default 1 = same as FC)")
    g.add_argument("--compid", type=int, default=191, help="our component id (default 191 = onboard computer)")
    g = ap.add_argument_group("camera / detection")
    g.add_argument("--source", default="0", help="camera index, video file or rtsp:// URL")
    g.add_argument("--width", type=int, default=640)
    g.add_argument("--height", type=int, default=480)
    g.add_argument("--hfov", type=float, default=70.0, help="camera horizontal field of view, degrees")
    g.add_argument("--detector", choices=DETECTORS, default="hog",
                   help="hog = full body (default), face = best for a desk webcam, yolo = optional/robust, "
                        "color = track a coloured object (see --color)")
    g.add_argument("--color", default="green", choices=list(ColorDetector.RANGES),
                   help="colour for --detector color (default green)")
    g.add_argument("--mirror", action="store_true", help="flip the image horizontally (selfie view)")
    g.add_argument("--headless", action="store_true", help="no preview window")
    g.add_argument("--sim-pan", action="store_true",
                   help="demo aid: shift the picture with the drone's yaw so a fixed webcam closes the loop")
    g = ap.add_argument_group("behaviour")
    g.add_argument("--confirm-frames", type=int, default=5, help="consecutive hits before overriding")
    g.add_argument("--lock-seconds", type=float, default=0.0,
                   help="dwell lock: the target must stay inside the centre box this many seconds before "
                        "the mission is overridden (default 0 = trigger on --confirm-frames alone)")
    g.add_argument("--lock-zone", type=float, default=0.5,
                   help="width of the centre box for --lock-seconds, as a fraction of the frame (default 0.5)")
    g.add_argument("--search-rate", type=float, default=0.0,
                   help="deg/s to keep turning toward the side the target vanished on, until --lost-timeout "
                        "(default 0 = hold heading)")
    g.add_argument("--lost-timeout", type=float, default=3.0, help="seconds without target before resuming")
    g.add_argument("--cooldown", type=float, default=3.0, help="seconds before a new trigger is allowed")
    g.add_argument("--kp", type=float, default=1.0, help="yaw rate (deg/s) per degree of error")
    g.add_argument("--max-rate", type=float, default=45.0, help="yaw rate limit, deg/s")
    g.add_argument("--deadband", type=float, default=2.5, help="no yaw inside +/- this many degrees")
    g.add_argument("--smooth", type=float, default=0.5, help="EMA weight of the newest target position")
    args = ap.parse_args(argv)
    if args.connect is None:
        args.connect = "tcp:127.0.0.1:5760" if args.fc == "inav" else "udpin:0.0.0.0:14551"
    return args


def run(args, frames=None):
    """frames: optional iterator of BGR images (tests); default is the camera."""
    detector = make_detector(args)
    cap = None
    if frames is None:
        cap = open_camera(args.source, args.width, args.height)
    fc = connect_fc(args)

    trk = VisualTrack(fc, args)
    last_hb, fps, t_prev = 0.0, 0.0, time.monotonic()
    try:
        while True:
            if frames is None:
                ok, frame = cap.read()
                if not ok:
                    if str(args.source).isdigit():
                        print("camera read failed", file=sys.stderr)
                        break
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # loop video files
                    continue
            else:
                frame = next(frames, None)
                if frame is None:
                    break
            if args.mirror:
                frame = cv2.flip(frame, 1)
            if args.sim_pan:
                frame = sim_pan(frame, trk, fc)

            dets = detector.detect(frame)
            target = trk.update(dets, frame.shape[1], frame.shape[0])

            now = time.monotonic()
            if now - last_hb > 1.0:
                fc.heartbeat()
                last_hb = now
            fps = 0.9 * fps + 0.1 / max(now - t_prev, 1e-3)
            t_prev = now

            if not args.headless:
                for d in dets:
                    if d is not target:
                        cv2.rectangle(frame, (d.x, d.y), (d.x + d.w, d.y + d.h), (120, 120, 120), 1)
                draw_hud(frame, trk, fc, target, args, fps)
                cv2.imshow("MaxGCS visual track  (q = quit)", frame)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        trk.release()
        fc.close()
        if cap:
            cap.release()
        cv2.destroyAllWindows()
    return trk


if __name__ == "__main__":
    run(parse_args())
