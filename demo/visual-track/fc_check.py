"""Camera-free check of the FC half of the tracker: override the mission, yaw, hand the mission back.

Uses the exact same FC backend (fc_ardupilot / fc_inav) as visual_track.py, but the "trigger" is you
running this script instead of a person in front of a camera:

    wait until the FC is armed and flying its mission (AUTO)
    -> override           (ArduPilot: AUTO -> GUIDED.  INAV: NAV WP off, NAV POSHOLD on)
    -> hold still 2 s, yaw clockwise, pause, yaw anticlockwise, pause
    -> resume the mission (back to AUTO / NAV WP)

If the pilot changes mode at any point (the switch, or the INAV kill switch) the script lets go at once.
All of visual_track.py's link options work here too (--fc, --connect, --baud, --yaw-ch, ...).

    python fc_check.py --fc ardupilot --connect COM8 --baud 115200
    python fc_check.py --fc inav --connect COM7,115200
    python fc_check.py --fc ardupilot --connect COM8 --rate 30 --seconds 6
"""
import argparse
import sys
import time

import visual_track as vt

TICK = 0.1  # s; ArduPilot GUIDED drops a velocity setpoint after GUID_TIMEOUT, so keep streaming


def wait_for(cond, timeout, every=None, again=None):
    t0 = last = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True
        if every and again and time.monotonic() - last >= every:
            again()
            last = time.monotonic()
        time.sleep(0.05)
    return cond()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
                                 add_help=False)
    ap.add_argument("--rate", type=float, default=20.0, help="yaw rate to command, deg/s (default 20)")
    ap.add_argument("--seconds", type=float, default=4.0, help="how long to yaw each way (default 4)")
    ap.add_argument("--wait", type=float, default=120.0, help="seconds to wait for the FC to be armed in its mission")
    ap.add_argument("-h", "--help", action="store_true")
    own, rest = ap.parse_known_args(argv)
    if own.help:
        print(__doc__)
        return 0
    args = vt.parse_args(rest)
    if abs(own.rate) > args.max_rate:
        sys.exit(f"--rate {own.rate} is above --max-rate {args.max_rate}")

    fc = vt.connect_fc(args)
    overriding = False

    def ok(msg):
        print(f"[ OK ] {msg}", flush=True)

    def fail(msg):
        print(f"[FAIL] {msg}", flush=True)
        return 1

    def heading():
        return "?" if fc.yaw_deg is None else f"{fc.yaw_deg:6.1f}"

    try:
        print("waiting for the FC to be ARMED and flying its mission (AUTO / NAV WP) ...", flush=True)
        if not wait_for(lambda: fc.mission_running and fc.override_allowed, own.wait):
            return fail(f"FC never reached 'armed + mission running' (mode {fc.mode_text}, "
                        f"{'armed' if fc.armed else 'DISARMED'})")
        ok(f"mission running: {fc.mode_text}, heading {heading()}")
        start_heading = fc.yaw_deg

        fc.say("FC CHECK: overriding mission")
        fc.begin_override()
        overriding = True
        if not wait_for(lambda: fc.override_active, 5.0, every=1.0, again=fc.begin_override):
            return fail(f"FC did not accept the override within 5 s (mode {fc.mode_text})")
        ok(f"MISSION OVERRIDDEN: {fc.mode_text}")

        def stream(rate, secs, label):
            print(f"       {label}: yaw rate {rate:+.0f} deg/s for {secs:.0f} s", flush=True)
            t0 = time.monotonic()
            while time.monotonic() - t0 < secs:
                if not fc.override_active:
                    return False
                fc.yaw_rate(rate)
                fc.heartbeat()
                time.sleep(TICK)
            return True

        for rate, secs, label in ((0.0, 2.0, "hold still"), (own.rate, own.seconds, "clockwise"),
                                  (0.0, 1.0, "pause"), (-own.rate, own.seconds, "anticlockwise"),
                                  (0.0, 1.0, "pause")):
            if not stream(rate, secs, label):
                fc.release()
                overriding = False
                return fail(f"pilot/FC left the override (mode {fc.mode_text}) - released, not commanding")
        ok(f"yaw commands sent and the FC stayed in {fc.mode_text}")

        fc.say("FC CHECK: resuming mission")
        fc.end_override()
        if not wait_for(lambda: fc.mission_running, 6.0, every=1.0, again=fc.end_override):
            return fail(f"mission did not resume (mode {fc.mode_text})")
        overriding = False
        ok(f"MISSION RESUMED: {fc.mode_text}")

        if start_heading is not None and fc.yaw_deg is not None:
            net = (fc.yaw_deg - start_heading + 180) % 360 - 180
            print(f"       heading {start_heading:.1f} -> {fc.yaw_deg:.1f} deg (net {net:+.1f}); "
                  f"with props off it will not turn, in flight expect it to return near the start")
        print("DONE")
        return 0
    except KeyboardInterrupt:
        print("\ninterrupted", flush=True)
        return 1
    finally:
        if overriding:  # never leave the aircraft parked in the override because this script quit
            fc.say("FC CHECK: stopped - resuming mission")
            fc.end_override()
            wait_for(lambda: fc.mission_running, 3.0)
        fc.release()
        fc.close()


if __name__ == "__main__":
    sys.exit(main())
