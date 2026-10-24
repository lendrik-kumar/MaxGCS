"""Visual record of a run, so there is something to look at after it ends.

    --record [FILE]   video of exactly what the live window shows (camera picture, lock brackets, event
                      banners, yaw-controller panel). Plays in any video player.
    --report [FILE]   one PNG timeline: tracker state and FC mode, heading against the target's bearing,
                      yaw error and rate command, with the event cues marked and a summary of each lock.

Give no FILE to save into runs/<date-time>.mp4 / .png next to the script. No packages beyond OpenCV.
"""
import math
import os
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FPS = 20.0

STATE_COL = {"PATROL": (80, 170, 60), "OVERRIDE": (0, 140, 255), "LOCKED": (50, 50, 220),
             "RESUME": (0, 140, 255), "PILOT": (200, 120, 0)}
EVENT_COL = {"detected": (0, 150, 220), "locked": (50, 50, 220), "lost": (0, 120, 230),
             "resumed": (60, 150, 40), "released": (200, 110, 0)}
INK, GRID, FAINT = (40, 40, 40), (215, 215, 215), (120, 120, 120)
HEADING_COL, TARGET_COL, ERR_COL, CMD_COL = (60, 60, 60), (0, 140, 255), (200, 150, 0), (40, 90, 230)


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class RunRecorder:
    """Collects frames and per-frame samples during run(); writes the video live and the report at the end."""

    def __init__(self, record, report, t0=None):
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self.video_path = self._path(record, stamp, ".mp4")
        self.report_path = self._path(report, stamp, ".png")
        self.t0 = time.monotonic() if t0 is None else t0
        self.writer = None
        self.frames = 0
        self.samples = []  # (t, state, override_active, mission_running, mode_text, heading, bearing, err, cmd, seen)
        self.events = []  # (t, name)

    @staticmethod
    def _path(value, stamp, ext):
        if not value:
            return None
        path = os.path.join(HERE, "runs", stamp + ext) if value == "auto" else value
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        return path

    @property
    def active(self):
        return bool(self.video_path or self.report_path)

    def event(self, name):
        self.events.append((time.monotonic() - self.t0, name))

    def sample(self, state, fc, heading, bearing, err, cmd, seen):
        self.samples.append((time.monotonic() - self.t0, state, bool(fc.override_active), bool(fc.mission_running),
                             fc.mode_text, heading, bearing, err, cmd, seen))

    def write(self, frame):
        if not self.video_path:
            return
        if self.writer is None:
            h, w = frame.shape[:2]
            self.writer = cv2.VideoWriter(self.video_path, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (w, h))
            if not self.writer.isOpened():
                print(f"[record] cannot write {self.video_path}", flush=True)
                self.video_path, self.writer = None, None
                return
        self.writer.write(frame)
        self.frames += 1

    def finish(self, args):
        if self.writer is not None:
            self.writer.release()
            print(f"[record] video  {self.video_path}  ({self.frames} frames, {self.frames / FPS:.0f} s)", flush=True)
        if self.report_path and self.samples:
            img = render_report(self.samples, self.events, args)
            cv2.imwrite(self.report_path, img)
            print(f"[record] report {self.report_path}", flush=True)


# ===================================================================================================
# Report rendering
# ===================================================================================================
def _runs(samples, key):
    """Collapse consecutive samples with the same key into [(t_start, t_end, key_value, first_sample)]."""
    out = []
    for i, s in enumerate(samples):
        k = key(s)
        t1 = samples[i + 1][0] if i + 1 < len(samples) else s[0]
        if out and out[-1][2] == k:
            out[-1][1] = t1
        else:
            out.append([s[0], t1, k, s])
    return out


def _nice_step(span, target=8):
    raw = span / target
    for step in (1, 2, 5, 10, 15, 20, 30, 60, 120, 300):
        if step >= raw:
            return step
    return 600


def _segments(xs, ys):
    """Split into runs of finite points so gaps (NaN) are not joined by a line."""
    seg = []
    for x, y in zip(xs, ys):
        if y is None or not math.isfinite(y):
            if len(seg) > 1:
                yield seg
            seg = []
        else:
            seg.append((x, y))
    if len(seg) > 1:
        yield seg


class _Panel:
    def __init__(self, img, box, tmax, ylo, yhi):
        self.img, self.box, self.tmax, self.ylo, self.yhi = img, box, tmax, ylo, yhi

    def X(self, t):
        x0, _, x1, _ = self.box
        return int(x0 + (x1 - x0) * t / self.tmax)

    def Y(self, v):
        _, y0, _, y1 = self.box
        return int(y1 - (y1 - y0) * (v - self.ylo) / (self.yhi - self.ylo))

    def frame(self, title, unit):
        x0, y0, x1, y1 = self.box
        for i in range(6):
            v = self.ylo + (self.yhi - self.ylo) * i / 5
            y = self.Y(v)
            cv2.line(self.img, (x0, y), (x1, y), GRID, 1)
            cv2.putText(self.img, f"{v:.0f}", (x0 - 46, y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.42, FAINT, 1, cv2.LINE_AA)
        cv2.rectangle(self.img, (x0, y0), (x1, y1), (170, 170, 170), 1)
        cv2.putText(self.img, f"{title} ({unit})", (x0, y0 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, INK, 1, cv2.LINE_AA)

    def line(self, ts, vs, col, thickness=2):
        """Draw clipped to the panel, so a zoomed axis lets the data run off the edge instead of over the page."""
        x0, y0, x1, y1 = self.box
        view = self.img[y0:y1 + 1, x0:x1 + 1]  # a numpy view: OpenCV cannot draw outside it
        for seg in _segments(ts, vs):
            pts = np.array([(self.X(t) - x0, self.Y(v) - y0) for t, v in seg], np.int32)
            pts[:, 1] = np.clip(pts[:, 1], -4000, 4000)  # keep int32 maths sane for huge off-axis values
            cv2.polylines(view, [pts], False, col, thickness, cv2.LINE_AA)


def _shade(img, panel_boxes, spans, tmax, col, alpha=0.12):
    ov = img.copy()
    for (x0, y0, x1, y1) in panel_boxes:
        for a, b in spans:
            xa = int(x0 + (x1 - x0) * a / tmax)
            xb = int(x0 + (x1 - x0) * b / tmax)
            cv2.rectangle(ov, (xa, y0), (xb, y1), col, -1)
    cv2.addWeighted(ov, alpha, img, 1 - alpha, 0, img)


def render_report(samples, events, args):
    W, H = 1400, 900
    img = np.full((H, W, 3), 255, np.uint8)
    L, R = 100, 30
    tmax = max(samples[-1][0], 1.0)
    t = [s[0] for s in samples]

    # ---- data -----------------------------------------------------------------------------------
    heading = np.array([np.nan if s[5] is None else s[5] for s in samples], float)
    bearing = np.array([np.nan if s[6] is None else s[6] for s in samples], float)
    err = np.array([np.nan if s[7] is None else s[7] for s in samples], float)
    cmd = np.array([np.nan if s[8] is None else s[8] for s in samples], float)
    locked = np.array([s[1] == "LOCKED" for s in samples])
    err[~locked], cmd[~locked] = np.nan, np.nan
    ok = np.isfinite(heading)
    hp = np.full_like(heading, np.nan)  # unwrapped heading, so a turn through north is not a 360 deg jump
    if ok.any():
        hp[ok] = np.degrees(np.unwrap(np.radians(heading[ok])))
    bp = hp + np.array([wrap180(b - h) if np.isfinite(b) and np.isfinite(h) else np.nan
                        for b, h in zip(bearing, heading)])
    lock_spans = [(a, b) for a, b, k, _ in _runs(samples, lambda s: s[1] == "LOCKED") if k]

    # ---- header ---------------------------------------------------------------------------------
    cv2.putText(img, "MaxGCS visual track - run report", (L, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, INK, 2, cv2.LINE_AA)
    sub = (f"{time.strftime('%Y-%m-%d %H:%M:%S')}   {tmax:.0f} s   fc {args.fc}   source {args.source}   "
           f"detector {args.detector}   kp {args.kp:g}  deadband {args.deadband:g}  max {args.max_rate:g} deg/s")
    cv2.putText(img, sub, (L, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.5, FAINT, 1, cv2.LINE_AA)
    if not lock_spans:
        cv2.putText(img, "no lock happened in this run", (L, 84), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 200), 1, cv2.LINE_AA)
    for n, (a, b) in enumerate(lock_spans[:4], 1):
        m = locked & (np.array(t) >= a) & (np.array(t) <= b)
        e = np.abs(err[m])
        e = e[np.isfinite(e)]
        line = f"lock {n}:  {a:5.1f} s -> {b:5.1f} s  ({b - a:.1f} s)"
        if len(e):
            line += f"   yaw error median {np.median(e):.1f} deg, 95% {np.percentile(e, 95):.1f} deg"
        tr = m & np.isfinite(bp) & np.isfinite(hp)
        if tr.sum() > 20 and np.isfinite(bearing[tr]).any():
            d = np.abs(hp[tr] - bp[tr])
            line += f"   heading vs target median {np.median(d):.1f} deg"
        cv2.putText(img, line, (L, 82 + 20 * (n - 1)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, INK, 1, cv2.LINE_AA)

    # ---- bands: tracker state and FC mode -------------------------------------------------------
    bx0, bx1 = L, W - R
    boxes = []
    cv2.putText(img, "tracker state / FC mode", (bx0, 168), cv2.FONT_HERSHEY_SIMPLEX, 0.55, INK, 1, cv2.LINE_AA)
    for row, (label, key) in enumerate((("state", lambda s: s[1]),
                                        ("mode", lambda s: (s[2], s[3])))):
        y0, y1 = 176 + row * 30, 202 + row * 30
        boxes.append((bx0, y0, bx1, y1))
        cv2.putText(img, label, (bx0 - 56, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, FAINT, 1, cv2.LINE_AA)
        for a, b, k, first in _runs(samples, key):
            xa, xb = int(bx0 + (bx1 - bx0) * a / tmax), int(bx0 + (bx1 - bx0) * b / tmax)
            if row == 0:
                col, text = STATE_COL.get(k, (150, 150, 150)), k
            else:
                col = (60, 60, 220) if k[0] else (80, 170, 60) if k[1] else (160, 160, 160)
                text = first[4] if len(first[4]) < 14 else ("OVERRIDE" if k[0] else "MISSION" if k[1] else "other")
            cv2.rectangle(img, (xa, y0), (max(xb, xa + 1), y1), col, -1)
            if xb - xa > 12 * len(text):
                cv2.putText(img, text, (xa + 5, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

    # ---- heading vs target ----------------------------------------------------------------------
    # y range: zoom on the lock (that is the interesting part); the mission's own turns before and after
    # would otherwise stretch the axis and flatten the tracking into a line.
    tt = np.array(t)
    zoom = np.zeros(len(t), bool)
    if lock_spans:
        zoom = (tt >= lock_spans[0][0] - 4) & (tt <= lock_spans[-1][1] + 4)
    pick = zoom if zoom.any() else np.ones(len(t), bool)
    finite = np.concatenate([hp[pick & np.isfinite(hp)], bp[pick & np.isfinite(bp)]])
    if not len(finite):
        finite = np.array([0.0, 90.0])
    lo, hi = float(finite.min()), float(finite.max())
    pad = max(8.0, (hi - lo) * 0.15)
    pb = _Panel(img, (L, 300, W - R, 530), tmax, lo - pad, hi + pad)
    ec = np.concatenate([np.abs(err[np.isfinite(err)]), np.abs(cmd[np.isfinite(cmd)])]) if np.isfinite(err).any() else np.array([10.0])
    pc_lim = max(10.0, float(np.ceil(ec.max() / 10.0) * 10.0))
    pc = _Panel(img, (L, 590, W - R, 820), tmax, -pc_lim, pc_lim)
    _shade(img, [pb.box, pc.box], lock_spans, tmax, (50, 50, 220))
    pb.frame("heading and target bearing", "deg" + (", zoomed on lock" if zoom.any() else ""))
    if np.isfinite(bearing).any():
        pb.line(t, list(bp), TARGET_COL, 2)
    pb.line(t, list(hp), HEADING_COL, 2)
    lx = pb.box[0] + 560  # legends sit on the title row, clear of the data
    ly = pb.box[1] - 12
    cv2.line(img, (lx, ly), (lx + 24, ly), HEADING_COL, 2)
    cv2.putText(img, "drone heading (from FC)", (lx + 30, ly + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, INK, 1, cv2.LINE_AA)
    if np.isfinite(bearing).any():
        cv2.line(img, (lx + 230, ly), (lx + 254, ly), TARGET_COL, 2)
        cv2.putText(img, "target bearing (simulated camera truth)", (lx + 260, ly + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, INK, 1, cv2.LINE_AA)

    # ---- yaw controller -------------------------------------------------------------------------
    pc.frame("yaw controller: error and rate command", "deg and deg/s")
    ov = img.copy()
    cv2.rectangle(ov, (pc.box[0], pc.Y(args.deadband)), (pc.box[2], pc.Y(-args.deadband)), (120, 200, 120), -1)
    cv2.addWeighted(ov, 0.35, img, 0.65, 0, img)
    cv2.line(img, (pc.box[0], pc.Y(0)), (pc.box[2], pc.Y(0)), (150, 150, 150), 1)
    pc.line(t, list(err), ERR_COL, 2)
    pc.line(t, list(cmd), CMD_COL, 2)
    lx = pc.box[0] + 560
    ly = pc.box[1] - 12
    for off, col, text in ((0, ERR_COL, "error (target off-centre)"), (230, CMD_COL, "rate command"),
                           (390, (120, 200, 120), f"deadband +/-{args.deadband:g}")):
        cv2.line(img, (lx + off, ly), (lx + off + 24, ly), col, 3)
        cv2.putText(img, text, (lx + off + 30, ly + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, INK, 1, cv2.LINE_AA)

    # ---- time axis ------------------------------------------------------------------------------
    step = _nice_step(tmax)
    for s in range(0, int(tmax) + 1, step):
        x = pc.X(s)
        cv2.line(img, (x, pc.box[3]), (x, pc.box[3] + 6), (120, 120, 120), 1)
        cv2.putText(img, f"{s} s", (x - 14, pc.box[3] + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.45, FAINT, 1, cv2.LINE_AA)

    # ---- event cues through every panel ---------------------------------------------------------
    top, bottom = 170, pc.box[3]
    row_end = [0, 0, 0]  # x where each label row is used up, so neighbouring labels stack instead of colliding
    for te, name in events:
        x = pb.X(te)
        col = EVENT_COL.get(name, (100, 100, 100))
        for y in range(top, bottom, 10):
            cv2.line(img, (x, y), (x, min(y + 5, bottom)), col, 1)
        width = 9 * len(name) + 8
        row = next((r for r in range(3) if x >= row_end[r]), 2)
        row_end[row] = x + width
        cv2.putText(img, name, (x + 3, 248 + row * 13), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
    cv2.putText(img, "dashed lines = cues sent to the pilot (screen banner, FC buzzer, gamepad rumble)",
                (L, H - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.45, FAINT, 1, cv2.LINE_AA)
    cv2.putText(img, "red shading = mission overridden and locked", (L, H - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, FAINT, 1, cv2.LINE_AA)
    return img
