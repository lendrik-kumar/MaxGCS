"""ArduPilot Copter backend: override = switch AUTO -> GUIDED, steer with a velocity-0 + yaw-rate setpoint.

Same duck-typed interface as fc_inav.InavLink, so visual_track.py does not care which one it drives.
"""
import math
import os
import sys
import threading
import time

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

AUTO, GUIDED = 3, 4
MODE_NAMES = {0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL", 9: "LAND",
              16: "POSHOLD"}
COPTER_TYPES = {2, 3, 4, 13, 14, 15}  # quad, coax, heli, hexa, octo, tri
# POSITION_TARGET_TYPEMASK: ignore position, acceleration and absolute yaw -> use velocity + yaw_rate
TYPEMASK_VEL_YAWRATE = 0b0000_0101_1100_0111
MAV_FRAME_LOCAL_NED = 1
CMD_DO_SET_MODE = 176


class ArduPilotLink:
    """MAVLink wrapper. A reader thread keeps `mode`, `armed`, `yaw_deg` current."""

    def __init__(self, a, log=print):
        self.log = log
        self.m = mavutil.mavlink_connection(a.connect, baud=a.baud, source_system=a.sysid,
                                            source_component=a.compid, autoreconnect=True)
        self.target_sys = 0
        self.lock = threading.Lock()
        self.mode = None
        self.armed = False
        self.mav_type = None
        self.yaw_deg = None
        self.last_hb = 0.0
        self._stop = False
        self.heartbeat()
        threading.Thread(target=self._reader, daemon=True).start()

        t0 = time.monotonic()
        while not self.alive:
            if time.monotonic() - t0 > 15:
                sys.exit("no HEARTBEAT from the flight controller in 15 s - check --connect")
            self.heartbeat()
            time.sleep(0.25)
        if self.mav_type not in COPTER_TYPES:
            sys.exit(f"vehicle MAV_TYPE {self.mav_type} is not a multirotor; this backend drives ArduCopter only")
        log(f"[link] ArduPilot sysid {self.target_sys}, mode {self.mode_text}, "
            f"{'armed' if self.armed else 'DISARMED'}")

    def _reader(self):
        while not self._stop:
            try:
                msg = self.m.recv_match(blocking=True, timeout=0.5)
            except Exception:
                time.sleep(0.2)
                continue
            if msg is None:
                continue
            t = msg.get_type()
            if t == "HEARTBEAT":
                # only the autopilot (not gimbals/companions/GCS) defines mode and armed state
                if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID or msg.get_srcComponent() != 1:
                    continue
                if self.target_sys and msg.get_srcSystem() != self.target_sys:
                    continue
                self.target_sys = msg.get_srcSystem()
                self.mode = msg.custom_mode
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mav_type = msg.type
                self.last_hb = time.monotonic()
            elif t == "ATTITUDE":
                self.yaw_deg = math.degrees(msg.yaw) % 360

    # -- state the tracker asks about
    @property
    def alive(self):
        return time.monotonic() - self.last_hb < 3.0

    @property
    def mission_running(self):
        return self.armed and self.mode == AUTO

    override_allowed = True  # the pilot's veto is any mode change, handled by the tracker

    @property
    def override_active(self):
        return self.mode == GUIDED

    @property
    def mode_text(self):
        return MODE_NAMES.get(self.mode, f"MODE{self.mode}") if self.mode is not None else "?"

    # -- commands (pymavlink's sequence counter is not thread-safe, so serialise sends)
    def heartbeat(self):
        with self.lock:
            self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                      mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                      mavutil.mavlink.MAV_STATE_ACTIVE)

    def _set_mode(self, mode):
        with self.lock:
            self.m.mav.command_long_send(self.target_sys or 1, 1, CMD_DO_SET_MODE, 0,
                                         mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                         mode, 0, 0, 0, 0, 0)

    def begin_override(self):
        self._set_mode(GUIDED)

    def end_override(self):
        self._set_mode(AUTO)

    def release(self):
        pass  # nothing is streamed once the FC is back in AUTO

    def yaw_rate(self, rate_deg_s):
        """Hold position and rotate at rate_deg_s (+ = clockwise seen from above). GUIDED only."""
        with self.lock:
            self.m.mav.set_position_target_local_ned_send(
                0, self.target_sys or 1, 1, MAV_FRAME_LOCAL_NED, TYPEMASK_VEL_YAWRATE,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, math.radians(rate_deg_s))

    def say(self, text, warn=False):
        self.log(f"[fc-msg] {text}")
        sev = mavutil.mavlink.MAV_SEVERITY_WARNING if warn else mavutil.mavlink.MAV_SEVERITY_NOTICE
        with self.lock:
            self.m.mav.statustext_send(sev, text.encode()[:50])

    def close(self):
        self._stop = True
