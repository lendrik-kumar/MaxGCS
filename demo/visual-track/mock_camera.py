"""Simulated drone camera for the visual-track demo (`visual_track.py --source mock`).

A tiny world seen from the aircraft: a horizon, a few distant buildings and posts at fixed compass
bearings, and one orange TARGET building that sways left and right around a base bearing. Each frame is
drawn from the drone's REAL heading (read from the flight controller), so when the tracker yaws the
drone the whole view slides sideways, exactly as a camera bolted to the airframe would see it.

That closes the loop honestly: heading -> picture -> detection -> yaw command -> heading.
The camera only looks along the nose (no roll/pitch/position parallax: the scenery is "far away").

    target on screen at x = cx + fx * tan(bearing_of_target - drone_heading)

After the lock has lasted `--mock-hold` seconds the target is hidden for `--mock-hide` seconds, which
makes the tracker hand the mission back; it then reappears and can be triggered again.
"""
import math
import time

import cv2
import numpy as np

# (compass bearing deg, angular width deg, height px above the horizon, BGR). Never orange.
SCENERY = [
    (10, 7, 70, (120, 120, 125)), (28, 5, 45, (150, 100, 60)), (140, 9, 90, (115, 115, 120)),
    (165, 6, 55, (170, 150, 120)), (200, 8, 75, (125, 125, 130)), (232, 5, 40, (150, 100, 60)),
    (260, 10, 85, (110, 110, 118)), (300, 6, 60, (160, 140, 110)), (330, 8, 80, (118, 118, 124)),
    (112, 4, 35, (70, 140, 80)), (352, 4, 30, (70, 140, 80)), (82, 4, 38, (70, 140, 80)),
]
TARGET_COLOUR = (30, 120, 235)  # BGR orange
TARGET_HALF_WIDTH_DEG = 4.0
TARGET_HEIGHT = 130


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class MockCamera:
    def __init__(self, a, w=640, h=480):
        self.w, self.h = w, h
        self.fx = (w / 2) / math.tan(math.radians(a.hfov) / 2)
        self.hfov = a.hfov
        self.base, self.sway, self.period = a.mock_bearing, a.mock_sway, a.mock_period
        self.hold, self.hide = a.mock_hold, a.mock_hide
        self.occ, self.occ_width = a.mock_occluder, a.mock_occluder_width  # None = no occluder
        self.t0 = time.monotonic()
        self.locked_since = None
        self.hidden_until = 0.0
        self.horizon = h // 2
        self._bg = self._background()
        self._warned = False

    def _background(self):
        h, w, hz = self.h, self.w, self.horizon
        img = np.zeros((h, w, 3), np.uint8)
        for y in range(h):
            if y < hz:
                f = y / hz
                img[y, :] = (int(200 - 40 * f), int(150 + 10 * f), int(70 + 40 * f))
            else:
                f = (y - hz) / (h - hz)
                img[y, :] = (int(70 + 25 * f), int(125 - 35 * f), int(80 - 20 * f))
        return img

    # ---- ground truth (for tests and the HUD) ----------------------------------------------------
    def target_bearing(self, now=None):
        t = (time.monotonic() if now is None else now) - self.t0
        return (self.base + self.sway * math.sin(2 * math.pi * t / self.period)) % 360.0

    def target_visible(self, now=None):
        return (time.monotonic() if now is None else now) >= self.hidden_until

    def info(self, heading):
        if heading is None:
            return ["mock camera: waiting for the FC heading"]
        rel = wrap180(self.target_bearing() - heading)
        vis = "visible" if self.target_visible() else "HIDDEN"
        return [f"MOCK CAMERA  target bearing {self.target_bearing():5.1f}  heading {heading:5.1f}  "
                f"offset {rel:+5.1f} deg  ({vis})"]

    # ---- rendering -------------------------------------------------------------------------------
    def _x(self, rel_deg):
        return self.w / 2 + self.fx * math.tan(math.radians(rel_deg))

    def render(self, heading, locked):
        now = time.monotonic()
        if locked:
            if self.locked_since is None:
                self.locked_since = now
            elif self.hold > 0 and now - self.locked_since >= self.hold:
                self.hidden_until = now + self.hide
                self.locked_since = None
        else:
            self.locked_since = None
        if heading is None:
            heading = 0.0
            if not self._warned:
                print("[mock-cam] no heading from the FC yet - drawing with heading 0", flush=True)
                self._warned = True

        img = self._bg.copy()
        hz = self.horizon
        # posts every 10 deg: cheap texture so the sideways motion is obvious even with no building in view
        for b in range(0, 360, 10):
            rel = wrap180(b - heading)
            if abs(rel) < self.hfov / 2 + 3:
                x = int(self._x(rel))
                cv2.line(img, (x, hz + 6), (x, hz + 70), (40, 80, 55), 2)
        for b, wdeg, hpx, col in SCENERY:
            rel = wrap180(b - heading)
            if abs(rel) < self.hfov / 2 + wdeg:
                x1, x2 = int(self._x(rel - wdeg / 2)), int(self._x(rel + wdeg / 2))
                cv2.rectangle(img, (x1, hz - hpx), (x2, hz + 25), col, -1)
                cv2.rectangle(img, (x1, hz - hpx), (x2, hz + 25), (60, 60, 60), 1)
        if self.target_visible():
            rel = wrap180(self.target_bearing(now) - heading)
            if abs(rel) < self.hfov / 2 + TARGET_HALF_WIDTH_DEG + 2:
                x1 = int(self._x(rel - TARGET_HALF_WIDTH_DEG))
                x2 = int(self._x(rel + TARGET_HALF_WIDTH_DEG))
                y1, y2 = hz - TARGET_HEIGHT, hz + 25
                cv2.rectangle(img, (x1, y1), (x2, y2), TARGET_COLOUR, -1)
                for wy in range(y1 + 12, y2 - 10, 24):  # windows
                    for wx in range(x1 + 8, x2 - 10, 18):
                        cv2.rectangle(img, (wx, wy), (wx + 8, wy + 12), (20, 70, 170), -1)
        if self.occ is not None:  # a tree trunk in front of the target: the target goes out of sight behind it
            rel = wrap180(self.occ - heading)
            if abs(rel) < self.hfov / 2 + self.occ_width:
                x1 = int(self._x(rel - self.occ_width / 2))
                x2 = int(self._x(rel + self.occ_width / 2))
                cv2.rectangle(img, (x1, hz - 175), (x2, hz + 85), (35, 55, 35), -1)
                cv2.rectangle(img, (x1, hz - 175), (x2, hz + 85), (20, 35, 20), 2)
        return img
