# Visual lock-and-track demo (INAV)

A camera sees a person → the flight controller **drops its mission plan** → the drone **yaws to keep
the person centred** → when the person leaves, the **mission resumes**. MaxGCS shows it live: the mode
flips NAV WP → NAV POSHOLD → NAV WP and the heading turns.

```
 camera ─> visual_track.py (OpenCV) ── MSP over a spare UART ──> INAV FC ── MSP over USB ──> MaxGCS
           detect person               RC injection:                        modes / heading
           state machine               NAV WP off, NAV POSHOLD on, yaw stick
```

INAV has no "guided mode" and no command to say "yaw to X". The only thing a companion can do is
behave like a **transmitter**: inject RC channels over MSP (`MSP_SET_RAW_RC`, INAV 8.0+). So:

| State | What the script does |
|---|---|
| **PATROL** | Nothing is injected; the FC flies its NAV WP mission exactly as configured. Frames are scanned for a person. |
| **OVERRIDE** | Person seen in 5 frames in a row → inject: NAV WP **off**, NAV POSHOLD **on** (drone holds position). It then **reads the active modes back** and only proceeds when the FC confirms; otherwise it aborts. |
| **LOCKED** | Offsets the yaw stick in proportion to the angle between the person and the image centre (POSHOLD lets the pilot yaw; stick = yaw rate). |
| **RESUME** | No person for 3 s → inject NAV WP **on**, confirm, stop injecting. The FC carries on with the mission (see `nav_wp_mission_restart` below). |
| **PILOT** | The kill switch was thrown: stop at once and stay passive. |

You don't type channel numbers: at start-up it reads the FC's own Modes table (`MSP_MODE_RANGES`),
works out which switch positions give "WP off / POSHOLD on", prints the plan, and **refuses** with a
plain reason if it can't (mode not assigned, on the ARM channel, above CH16…). It never moves a channel
that carries ARM.

## Pilot authority: the kill switch

Injected RC only counts while the FC's **MSP RC OVERRIDE** mode is active, and you assign that mode to a
transmitter switch. **That switch is the tracker's kill switch**: turn it off and the FC ignores the
script instantly; the script notices, releases, and won't try again while it's off. Don't rely on other
transmitter switches while the override is on — the injected channels win over the radio.

## Setup (Windows, once)

```powershell
cd D:\MaxGCS\demo\visual-track
python -m venv .venv
.\.venv\Scripts\python -m pip install -r requirements.txt
```

`opencv-python` must be 4.x — OpenCV 5.0 removed the HOG people detector.

## Run the demo without hardware (simulated INAV)

1. **Simulated FC** (a toy INAV: mode table, transmitter, override gate, looping NAV WP square; not a real aircraft):
   ```powershell
   .\.venv\Scripts\python mock_inav.py
   ```
2. **MaxGCS** — connection bar → **TCP**, `127.0.0.1`, port **5760**, Connect. Download the mission to
   see the square. The mock accepts several clients, so MaxGCS and the tracker connect together.
3. **Tracker** (a webcam stands in for the drone camera):
   ```powershell
   .\.venv\Scripts\python visual_track.py --detector face --mirror --sim-pan
   ```
   Use `--detector hog` if the camera can see your whole body.

Walk into frame → the HUD turns red, MaxGCS mode goes **NAV POSHOLD**, the drone leaves the mission line
and turns to face you; step sideways and it follows. Walk out → after 3 s the mode returns to **NAV WP**
and it flies on. To show the safety rule, type `tx 7 1000` in the mock's window (the pilot flips the
kill switch off) while it is locked: the tracker lets go and the mission continues by itself.

`--sim-pan` is a demo aid: a webcam on your desk doesn't turn when the drone yaws, so the picture is
shifted by the drone's yaw to close the loop (the window says so). **Drop it on a real drone.**
`q` / `Esc` quits and hands the mission back if it was overridden.

## Real INAV FC

**Wiring.** MSP is point-to-point, so MaxGCS and the tracker each need their own FC port: MaxGCS on
**USB**, the tracker on a **spare UART** with MSP enabled (Configurator → Ports, e.g. 115200) through a
USB-serial adapter (or a Pi/Jetson UART on the aircraft):

```powershell
.\.venv\Scripts\python visual_track.py --connect COM7,115200 --hfov <your camera's horizontal FOV>
```

**FC configuration** (INAV 8.0+):
- Modes tab: **MSP RC OVERRIDE** on a transmitter switch (your kill switch), **NAV WP** and
  **NAV POSHOLD** on channels CH1–16, **not** on the ARM channel. A 3-position switch (ANGLE / POSHOLD /
  WP) works well, which is also what `mock_inav.py` uses.
- `msp_override_channels` must cover the yaw channel and the mode channels. The tracker tells you the exact
  `set msp_override_channels = …` line if it doesn't (MaxGCS's RC panel can also set it at runtime).
- Yaw: `--yaw-ch` (default 4, AETR) and `--inav-yaw-rate` (your `yaw_rate` × 10, default 200 deg/s)
  set the stick-to-turn-rate scale. If your FC wants NAV ALTHOLD beside POSHOLD, add `--hold-boxes 11 3`.

**Bench test first, props off:** arm, throw the kill switch on, walk in front of the camera, and watch
the modes toggle in MaxGCS. Only then fly it, with a safety pilot's hand on the kill switch.

### Not verified on a real INAV (check these on the bench)
- That NAV POSHOLD turns the yaw stick into a yaw *rate* and also holds altitude on your build.
- What the FC does with injected channels after the script stops: the mock lets them expire after 1 s,
  the script also actively re-asserts the NAV WP position before it stops, so either way it should end up
  in NAV WP — but confirm.
- Whether the mission **resumes** or restarts depends on the `nav_wp_mission_restart` setting.
- INAV has no status-text channel to a GCS, so the narration lives in the tracker window and console;
  MaxGCS shows the mode and heading change.
- MaxGCS against `mock_inav.py`: the mock answers the messages MaxGCS's connect and telemetry code
  decode (read from its source), but the app itself has not been run against it here.

## ArduPilot

`--fc ardupilot` drives an ArduCopter instead (AUTO → GUIDED, velocity 0 + yaw rate, status text to
the GCS) against `mock_ardupilot.py`, SITL or MAVProxy. See the docstrings in `fc_ardupilot.py`.

## Building / target lock with the simulated drone camera (ArduPilot or INAV)

No camera, no person: `--source mock` is a simulated drone camera. It draws a small world (horizon,
distant buildings, posts) and one orange **target building** that sways **left and right** around a
compass bearing. Every frame is drawn from the drone's **real heading** as the FC reports it, so when
the tracker yaws the drone the whole view slides sideways. The loop is closed honestly:

```
 FC heading ─> mock camera picture ─> colour detector ─> state machine ─> yaw command ─> FC ─> heading
```

```powershell
.\.venv\Scripts\python mock_ardupilot.py                    # simulated FC (square mission); MaxGCS: UDP 14550
.\.venv\Scripts\python visual_track.py --fc ardupilot --connect udpin:0.0.0.0:14551 --source mock --detector color --color orange
```

What you see in the camera window and in MaxGCS, in step:
1. **PATROL**: the drone flies the mission; the camera view turns with each leg. The building comes into view as the nose swings past its bearing.
2. **Trigger**: building detected for 5 frames → "target detected - MISSION OVERRIDE", FC mode AUTO → GUIDED.
3. **Lock**: orange bracket cue + `TARGET LOCKED`. The drone yaws toward the building *whatever the mission heading was*, and holds position.
4. **Track**: the building sways left/right; the drone yaws left/right after it. The scenery slides the opposite way, the building stays near the crosshair.
5. **Resume**: after `--mock-hold` seconds the building is hidden; 3 s later the mission resumes (GUIDED → AUTO). It returns after `--mock-hide` s and can trigger again.

Tune it with `--mock-bearing`, `--mock-sway`, `--mock-period`, `--mock-hold`, `--mock-hide`. `--duration N` stops after N seconds.
With the INAV mock use `--fc inav` and its default connect string; the camera only needs a heading.

**Real buildings:** the colour detector only finds painted/marked targets. For real buildings train a YOLO
model and run `--detector yolo --weights building.pt --classes 0` (not exercised here).

`selftest.py mockcam` runs this closed loop against the simulated ArduPilot and asserts the heading stays
on the swaying target (median error under 10°), swings both ways, held position, and the mission resumed.

## Feedback for the pilot and for the yaw controller

**Controller view (PID).** The camera window has a **YAW CONTROLLER (P)** panel while locked: the live
error (degrees off-centre) and the rate command (deg/s), a 10-second plot of both, the deadband, and a
`DEADBAND` / `LIMIT` flag when the command is inside the dead zone or hitting `--max-rate`. It sits on the
side of the picture the target is not on. Tune with `--kp`, `--deadband`, `--max-rate` and watch the plot.

**Pilot cues.** Every tracker event is announced the same way on all channels, so the pilot always knows
who is flying:

| Event | On screen | FC buzzer (ArduPilot) | Gamepad `--rumble` |
|---|---|---|---|
| `detected` (taking over) | TARGET DETECTED | rising two-note chirp | two light taps |
| `locked` (yawing onto it) | TARGET LOCKED + brackets | rising three-note | two strong thumps |
| `lost` (handing back) | TARGET LOST | falling two-note | one long soft rumble |
| `resumed` (mission flying again) | MISSION RESUMED | short double beep | one short pulse |
| `released` (pilot/failsafe took over) | PILOT TOOK OVER | fast high chirp | three sharp buzzes |

- **FC buzzer:** sent as MAVLink `PLAY_TUNE`, on by default for ArduPilot (`--no-buzzer` turns it off). It is
  played by the FC's ToneAlarm output; a board with only a plain on/off buzzer ignores it. The same events
  also go out as `STATUSTEXT`, which MaxGCS shows and a telemetry-capable radio can display.
- **Gamepad rumble:** `--rumble` buzzes an XInput pad (Xbox-style) plugged into the PC running the tracker.
  Windows only, no extra packages; a PlayStation pad needs an XInput wrapper such as DS4Windows.
  The patterns are tested with a fake pad; no real pad was used.
- INAV has no buzzer command over MSP, so INAV gets the on-screen cues and rumble only.

## One real FC, MaxGCS and the tracker at once (`fc_bridge.py`)

A serial port can only be opened by one program, so MaxGCS and the tracker can't both open the FC's COM
port. `fc_bridge.py` opens it once and relays MAVLink over UDP to both (close Mission Planner / QGC first):

```powershell
.\.venv\Scripts\python fc_bridge.py --fc COM11,115200
```

Then **MaxGCS → MAVLink → UDP → port 14550**, and the tracker / `fc_check.py` with
`--connect udpin:0.0.0.0:14551`. Every FC message is copied to both; anything either sends goes to the FC.
Tested against the simulated FC (`--fc udp:127.0.0.1:14560`); the serial side is not tested on hardware.

## Seeing what happened afterwards

The console only prints text, so two options save something you can look at once the run is over. Give
a file name, or none to save into `runs/<date-time>.mp4` / `.png` (that folder is git-ignored):

```powershell
.\.venv\Scripts\python visual_track.py --fc ardupilot --connect udpin:0.0.0.0:14551 --source mock --detector color --color orange --record --report
```

- **`--record [FILE]`**: a video of exactly what the live window shows: the camera picture, lock brackets,
  event banners and the yaw-controller panel. It also records with `--headless`, so a Pi with no screen can
  still keep a video.
- **`--report [FILE]`**: one PNG timeline written when the run ends:
  - tracker state and FC mode (PATROL/LOCKED, AUTO/GUIDED) over time;
  - drone heading against the target's bearing (the simulated camera knows the truth), zoomed on the lock
    so you can see the heading follow the sway;
  - yaw error and rate command against the deadband;
  - the pilot cues (detected / locked / lost / resumed / released) as dashed lines;
  - a summary line per lock: how long, median and 95th-percentile yaw error, heading-vs-target error.

With a real camera there is no target-bearing truth, so the heading panel shows the heading alone; the
error and rate panel is the same. `selftest.py mockcam` also checks that both files are produced and valid.

## Options

| Flag | Default | |
|---|---|---|
| `--detector` | `hog` | `hog` full body · `face` best for a desk webcam · `yolo` optional (`pip install ultralytics`) |
| `--hfov` | 70 | Camera horizontal FOV (°): turns pixels into degrees. Look yours up. |
| `--kp` / `--max-rate` / `--deadband` | 1.0 / 45 / 2.5 | Tracking feel (deg/s per degree of error, rate cap, ± dead zone) |
| `--confirm-frames` / `--lost-timeout` / `--cooldown` | 5 / 3 s / 3 s | Trigger debounce, resume delay, re-trigger delay |
| `--source` | 0 | Camera index, video file, or `rtsp://` URL (the drone's own stream) |
| `--no-fc` | | Vision only, no FC link — to check the camera and detector |

## Check it works

```powershell
.\.venv\Scripts\python selftest.py            # or: selftest.py inav | ardupilot | mockcam
```

For INAV and ArduPilot, starts the simulated FC, plays a synthetic scene through the tracker, and watches
the aircraft from an independent second link. Asserts: mission → override → mission, heading turns to
centre the person, position is held, and a pilot veto makes the tracker let go and not fight back.

## Honest limits

- The built-in detectors are demo-grade: weak from above, at range or in odd poses. A YOLO person model
  is the realistic upgrade (`--detector yolo`, not exercised here).
- Yaw only: a P-controller on horizontal error. No gimbal, no pitch/roll, no following.
- The simulated FCs are toys; they prove the logic and the protocol exchange, not flight behaviour.
