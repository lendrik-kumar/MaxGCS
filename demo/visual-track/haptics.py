"""Gamepad rumble for the visual-track cues (`visual_track.py --rumble`).

Each tracker event buzzes the pilot's gamepad in its own pattern, so the pilot can feel what the tracker
is doing without looking at the screen:

    detected   two light taps          target seen, taking over from the mission
    locked     two strong thumps       override confirmed, yawing onto the target
    lost       one long soft rumble    target gone, handing the mission back
    resumed    one short pulse         the FC is flying its mission again
    released   three sharp buzzes      the pilot (or a failsafe) took control - tracker let go

Windows + an XInput pad (Xbox 360/One/Series, or anything XInput-compatible). No extra packages: it calls
XInput through ctypes. A PlayStation pad needs an XInput wrapper such as DS4Windows. With no pad, on another
OS, or on a pad without rumble, every call quietly does nothing.
"""
import ctypes
import sys
import time

# event -> [(seconds, low-frequency motor 0..1, high-frequency motor 0..1), ...]
PATTERNS = {
    "detected": [(0.12, 0.0, 0.6), (0.10, 0.0, 0.0), (0.12, 0.0, 0.6)],
    "locked": [(0.22, 0.9, 0.9), (0.10, 0.0, 0.0), (0.22, 0.9, 0.9)],
    "lost": [(0.70, 0.5, 0.0)],
    "resumed": [(0.15, 0.4, 0.4)],
    "released": [(0.10, 1.0, 1.0), (0.08, 0.0, 0.0), (0.10, 1.0, 1.0), (0.08, 0.0, 0.0), (0.10, 1.0, 1.0)],
}


class XInputPad:
    """Vibration output for the first connected XInput controller."""

    class _Vibration(ctypes.Structure):
        _fields_ = [("left", ctypes.c_ushort), ("right", ctypes.c_ushort)]

    def __init__(self):
        self.dll = None
        self.index = None
        if sys.platform != "win32":
            return
        for name in ("xinput1_4", "xinput1_3", "xinput9_1_0"):
            try:
                self.dll = getattr(ctypes.windll, name)
                break
            except (OSError, AttributeError):
                continue

    def set(self, low, high):
        if self.dll is None:
            return False
        vib = self._Vibration(int(max(0.0, min(1.0, low)) * 65535), int(max(0.0, min(1.0, high)) * 65535))
        for i in ([self.index] if self.index is not None else range(4)):
            if self.dll.XInputSetState(i, ctypes.byref(vib)) == 0:  # 0 = ERROR_SUCCESS, 1167 = not connected
                self.index = i
                return True
        self.index = None
        return False

    @property
    def available(self):
        return self.set(0.0, 0.0)


class Haptics:
    """Plays PATTERNS on a pad. cue(name) starts one; tick() (call it every frame) advances it."""

    def __init__(self, pad=None, clock=time.monotonic):
        self.pad = pad if pad is not None else XInputPad()
        self.clock = clock
        self.segments = []  # [(t_end, low, high)]
        self.level = (0.0, 0.0)
        self.found = None

    def cue(self, name):
        pattern = PATTERNS.get(name)
        if not pattern:
            return
        t = self.clock()
        self.segments = []
        for dur, low, high in pattern:
            t += dur
            self.segments.append((t, low, high))
        self.tick()

    def tick(self):
        now = self.clock()
        while self.segments and now >= self.segments[0][0]:
            self.segments.pop(0)
        level = (self.segments[0][1], self.segments[0][2]) if self.segments else (0.0, 0.0)
        if level != self.level:
            ok = self.pad.set(*level)
            if self.found is None and level != (0.0, 0.0):
                self.found = ok
                if not ok:
                    print("[rumble] no XInput gamepad found - rumble cues are off", flush=True)
            self.level = level

    def close(self):
        self.segments = []
        self.level = (0.0, 0.0)
        self.pad.set(0.0, 0.0)
