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
from collections import deque
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
    """Optional: Ultralytics YOLO. By default the COCO 'person' class (far more robust than HOG: any pose,
    aerial view, range). Point --weights at your own model and --classes at its class ids to lock onto
    something else, e.g. a building model you trained. Needs `pip install ultralytics` (pulls PyTorch);
    the default weights file downloads on first use."""

    name = "yolo"

    def __init__(self, weights="yolov8n.pt", classes=(0,), min_conf=0.4):
        try:
            from ultralytics import YOLO
        except ImportError:
            sys.exit("--detector yolo needs: pip install ultralytics")
        self.model, self.classes, self.min_conf = YOLO(weights), list(classes), min_conf

    def detect(self, frame):
        out = []
        for r in self.model.predict(frame, classes=self.classes, conf=self.min_conf, verbose=False):
            for (x1, y1, x2, y2), c in zip(r.boxes.xyxy.tolist(), r.boxes.conf.tolist()):
                out.append(Det(int(x1), int(y1), int(x2 - x1), int(y2 - y1), float(c)))
        return out


def _overlap(a, b):
    ix = max(0, min(a.x + a.w, b.x + b.w) - max(a.x, b.x))
    iy = max(0, min(a.y + a.h, b.y + b.h) - max(a.y, b.y))
    return ix * iy / max(1, min(a.area, b.area))


class ColorDetector(_Scaled):
    """A brightly coloured target (a painted building, a panel, a vest) found by HSV thresholding. Zero
    downloads, works from any angle and range the colour is visible, and far steadier than HOG/face for a
    demo. Also what the simulated camera's orange building is found with."""

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
    if cls is ColorDetector:
        return cls(args.color)
    if cls is YoloDetector:
        return cls(args.weights, args.classes)
    return cls()


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

    def cue(self, name):
        pass

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
        self.hist = deque(maxlen=400)  # (t, error deg, rate cmd deg/s) of the yaw controller while locked
        self.cues = []  # callables fed the event name: FC buzzer, gamepad rumble, ...
        self.last_cue = ("", 0.0)  # (event, time) for the on-screen banner

    # state entered -> event the pilot is told about
    CUE_FOR = {OVERRIDE: "detected", LOCKED: "locked", RESUME: "lost", PILOT: "released"}

    def _go(self, state):
        prev, now = self.state, time.monotonic()
        self.state, self.t_state = state, now
        print(f"[track] -> {state}", flush=True)
        event = "resumed" if (state == PATROL and prev == RESUME) else self.CUE_FOR.get(state)
        if event:
            self.last_cue = (event, now)
            for fn in self.cues:
                try:
                    fn(event)
                except Exception as e:  # a missing buzzer or gamepad must never stop the tracker
                    print(f"[cue] {event}: {e}", file=sys.stderr, flush=True)

    def _pick(self, dets):
        if not dets:
            return None
        if self.sm_cx is not None and self.state == LOCKED:  # stay on the person we were following
            near = min(dets, key=lambda d: abs(d.cx - self.sm_cx))
            if abs(near.cx - self.sm_cx) < 0.25 * self.frame_w:
                return near
        return max(dets, key=lambda d: d.area)

    def update(self, dets, frame_w):
        """Advance one frame. Returns the target Det being followed (or None) for the HUD."""
        a, fc, now = self.a, self.fc, time.monotonic()
        self.frame_w = frame_w
        self.fx = (frame_w / 2) / math.tan(math.radians(a.hfov) / 2)
        target = self._pick(dets)

        if self.state == PATROL:
            if target and fc.mission_running and fc.override_allowed and now >= self.cooldown_until:
                self.hits += 1
            else:
                self.hits = max(0, self.hits - 1)
            if self.hits >= a.confirm_frames:
                self.hits = 0
                self.sm_cx = target.cx
                self.ref_yaw = fc.yaw_deg
                fc.say("VISUAL TRACK: target detected - MISSION OVERRIDE")
                fc.begin_override()
                self._go(OVERRIDE)

        elif self.state == OVERRIDE:
            if fc.override_active:
                self.t_seen = now
                self.ref_yaw = fc.yaw_deg
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
            else:
                self.err_deg, rate = 0.0, 0.0  # target momentarily gone: hold heading, keep the override
            fc.yaw_rate(rate)
            self.last_cmd = rate
            self.hist.append((now, self.err_deg, rate))
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


def lock_brackets(frame, t, col, pad=10, arm=16):
    """Corner brackets around the target box: the 'locked on' cue."""
    x1, y1, x2, y2 = t.x - pad, t.y - pad, t.x + t.w + pad, t.y + t.h + pad
    for (px, py, dx, dy) in ((x1, y1, 1, 1), (x2, y1, -1, 1), (x1, y2, 1, -1), (x2, y2, -1, -1)):
        cv2.line(frame, (px, py), (px + dx * arm, py), col, 3)
        cv2.line(frame, (px, py), (px, py + dy * arm), col, 3)


CUE_TEXT = {"detected": ("TARGET DETECTED", (0, 200, 255)), "locked": ("TARGET LOCKED", (0, 0, 230)),
            "lost": ("TARGET LOST", (0, 140, 255)), "resumed": ("MISSION RESUMED", (0, 170, 0)),
            "released": ("PILOT TOOK OVER", (200, 120, 0))}
CUE_SECONDS = 2.0


def draw_cue_banner(frame, trk):
    """Big banner for a couple of seconds after each event, matching the buzzer / gamepad cue."""
    event, t = trk.last_cue
    if not event or time.monotonic() - t > CUE_SECONDS:
        return
    text, col = CUE_TEXT[event]
    h, w = frame.shape[:2]
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_DUPLEX, 1.2, 2)
    x, y = (w - tw) // 2, h // 2 - 60
    cv2.rectangle(frame, (x - 14, y - th - 12), (x + tw + 14, y + 14), (0, 0, 0), -1)
    cv2.rectangle(frame, (x - 14, y - th - 12), (x + tw + 14, y + 14), col, 2)
    cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_DUPLEX, 1.2, col, 2, cv2.LINE_AA)


def draw_controller(frame, trk, a, target=None):
    """Live view of the yaw controller: error in, rate command out, last ~10 s, with its limits.
    Sits on whichever side of the picture the target is not."""
    h, w = frame.shape[:2]
    pw, ph = 230, 150
    x0, y0 = (8 if target is not None and target.cx > w / 2 else w - pw - 8), 42
    ov = frame.copy()
    cv2.rectangle(ov, (x0, y0), (x0 + pw, y0 + ph), (0, 0, 0), -1)
    cv2.addWeighted(ov, 0.6, frame, 0.4, 0, frame)
    cv2.rectangle(frame, (x0, y0), (x0 + pw, y0 + ph), (160, 160, 160), 1)
    cv2.putText(frame, "YAW CONTROLLER (P)", (x0 + 6, y0 + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    locked = trk.state == LOCKED
    sat = locked and abs(trk.last_cmd) >= a.max_rate - 1e-6
    inband = locked and abs(trk.err_deg) < a.deadband
    cv2.putText(frame, f"error {trk.err_deg:+6.1f} deg" if locked else "error    --", (x0 + 6, y0 + 34),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    cmd_col = (0, 0, 255) if sat else (255, 255, 255)
    cv2.putText(frame, f"rate  {trk.last_cmd:+6.1f} deg/s" if locked else "rate     --", (x0 + 6, y0 + 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, cmd_col, 1, cv2.LINE_AA)
    flag = "LIMIT" if sat else "DEADBAND" if inband else ""
    cv2.putText(frame, flag, (x0 + 170, y0 + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 0, 255) if sat else (0, 220, 0), 1, cv2.LINE_AA)
    # plot area: error (cyan, scaled to +/-45 deg) and command (orange, scaled to +/-max rate), last 10 s
    gx0, gy0, gw, gh = x0 + 6, y0 + 58, pw - 12, 66
    cv2.rectangle(frame, (gx0, gy0), (gx0 + gw, gy0 + gh), (90, 90, 90), 1)
    mid = gy0 + gh // 2
    cv2.line(frame, (gx0, mid), (gx0 + gw, mid), (90, 90, 90), 1)
    db = int(a.deadband / 45.0 * gh / 2)
    cv2.rectangle(frame, (gx0, mid - db), (gx0 + gw, mid + db), (60, 90, 60), 1)  # deadband
    now = time.monotonic()
    pts_e, pts_c = [], []
    for t, e, c in trk.hist:
        age = now - t
        if age > 10.0:
            continue
        px = int(gx0 + gw - age / 10.0 * gw)
        pts_e.append((px, int(mid - max(-45, min(45, e)) / 45.0 * gh / 2)))
        pts_c.append((px, int(mid - max(-a.max_rate, min(a.max_rate, c)) / a.max_rate * gh / 2)))
    if len(pts_e) > 1:
        cv2.polylines(frame, [np.array(pts_e, np.int32)], False, (255, 220, 0), 1, cv2.LINE_AA)
        cv2.polylines(frame, [np.array(pts_c, np.int32)], False, (0, 140, 255), 1, cv2.LINE_AA)
    cv2.putText(frame, f"kp {a.kp:g}  dead +/-{a.deadband:g}  max {a.max_rate:g}/s", (x0 + 6, y0 + ph - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1, cv2.LINE_AA)
    cv2.putText(frame, "err", (gx0 + 2, gy0 + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.33, (255, 220, 0), 1, cv2.LINE_AA)
    cv2.putText(frame, "cmd", (gx0 + 26, gy0 + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.33, (0, 140, 255), 1, cv2.LINE_AA)


def draw_hud(frame, trk, fc, target, args, fps, extra=()):
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
            lock_brackets(frame, target, tcol)
            cv2.line(frame, (w // 2, h // 2), (int(target.cx), int(target.cy)), tcol, 1)
            cv2.putText(frame, f"TARGET LOCKED  {trk.err_deg:+.1f} deg", (target.x - 10, max(14, target.y - 22)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, tcol, 2, cv2.LINE_AA)
        else:
            label = f"target {target.conf:.2f}" if target.conf != 1.0 else "target detected"
            cv2.putText(frame, label, (target.x, max(14, target.y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, tcol, 1, cv2.LINE_AA)
    draw_controller(frame, trk, args, target)
    draw_cue_banner(frame, trk)
    yaw = f"{fc.yaw_deg:5.1f}" if fc.yaw_deg is not None else "  ?  "
    lines = [f"FC: {fc.mode_text}  {'ARMED' if fc.armed else 'disarmed'}   heading {yaw} deg",
             f"yaw cmd {trk.last_cmd:+5.1f} deg/s   err {trk.err_deg:+5.1f} deg   {fps:4.1f} fps  [{args.detector}]"]
    lines.extend(extra)
    if args.sim_pan and not extra:
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
    g.add_argument("--source", default="0",
                   help="camera index, video file, rtsp:// URL, or 'mock' = the simulated drone camera "
                        "(a swaying target building, drawn from the drone's real heading)")
    g.add_argument("--width", type=int, default=640)
    g.add_argument("--height", type=int, default=480)
    g.add_argument("--hfov", type=float, default=70.0, help="camera horizontal field of view, degrees")
    g.add_argument("--detector", choices=DETECTORS, default="hog",
                   help="hog = full body (default), face = best for a desk webcam, yolo = optional/robust, "
                        "color = lock onto a coloured target (see --color; what --source mock uses)")
    g.add_argument("--color", default="green", choices=list(ColorDetector.RANGES),
                   help="colour for --detector color (default green; the mock building is orange)")
    g.add_argument("--weights", default="yolov8n.pt",
                   help="--detector yolo: model file (default yolov8n.pt = COCO; use your own building model)")
    g.add_argument("--classes", type=int, nargs="+", default=[0],
                   help="--detector yolo: class ids to lock onto (default 0 = person)")
    g.add_argument("--mirror", action="store_true", help="flip the image horizontally (selfie view)")
    g.add_argument("--headless", action="store_true", help="no preview window")
    g.add_argument("--sim-pan", action="store_true",
                   help="demo aid: shift the picture with the drone's yaw so a fixed webcam closes the loop")
    g.add_argument("--duration", type=float, default=0.0, help="stop after this many seconds (default 0 = run until quit)")
    g = ap.add_argument_group("simulated camera (--source mock)")
    g.add_argument("--mock-bearing", type=float, default=60.0, help="compass bearing of the target building, deg (default 60)")
    g.add_argument("--mock-sway", type=float, default=18.0, help="target swings +/- this many deg left and right (default 18)")
    g.add_argument("--mock-period", type=float, default=12.0, help="seconds per left-right swing (default 12)")
    g.add_argument("--mock-hold", type=float, default=25.0,
                   help="hide the target this long after the lock started, so the mission resumes (0 = never)")
    g.add_argument("--mock-hide", type=float, default=20.0, help="seconds the target stays hidden (default 20)")
    g = ap.add_argument_group("behaviour")
    g.add_argument("--confirm-frames", type=int, default=5, help="consecutive hits before overriding")
    g.add_argument("--rumble", action="store_true",
                   help="pilot feedback: buzz an XInput gamepad on detect / lock / lost / resume / pilot-takeover")
    g.add_argument("--no-buzzer", action="store_true",
                   help="ArduPilot: don't play the cue tunes on the FC's buzzer (they are on by default)")
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
    cap = cam = None
    if frames is None:
        if args.source == "mock":
            from mock_camera import MockCamera
            cam = MockCamera(args)
        else:
            cap = open_camera(args.source, args.width, args.height)
    fc = connect_fc(args)

    trk = VisualTrack(fc, args)
    trk.cam = cam  # None unless --source mock; lets tests read the target's true bearing
    if hasattr(fc, "cue"):
        trk.cues.append(fc.cue)  # FC buzzer (ArduPilot)
    haptics = None
    if args.rumble:
        from haptics import Haptics
        haptics = Haptics()
        trk.cues.append(haptics.cue)
    last_hb, fps, t_prev = 0.0, 0.0, time.monotonic()
    t_start = t_next = time.monotonic()
    try:
        while True:
            if args.duration and time.monotonic() - t_start > args.duration:
                break
            if cam is not None:
                time.sleep(max(0.0, t_next - time.monotonic()))  # ~20 fps
                t_next = max(t_next, time.monotonic() - 0.05) + 0.05
                # the picture is drawn from the heading the FC reports right now, so view, lock and yaw stay in step
                frame = cam.render(fc.yaw_deg, trk.state == LOCKED)
            elif frames is None:
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
            if args.mirror and cam is None:
                frame = cv2.flip(frame, 1)
            if args.sim_pan and cam is None:  # the mock camera already draws from the real heading
                frame = sim_pan(frame, trk, fc)

            dets = detector.detect(frame)
            target = trk.update(dets, frame.shape[1])
            if haptics:
                haptics.tick()

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
                draw_hud(frame, trk, fc, target, args, fps, cam.info(fc.yaw_deg) if cam else ())
                cv2.imshow("MaxGCS visual track  (q = quit)", frame)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        trk.release()
        if haptics:
            haptics.close()
        fc.close()
        if cap:
            cap.release()
        cv2.destroyAllWindows()
    return trk


if __name__ == "__main__":
    run(parse_args())
