# Intruder Detection — Proof of Concept

Standalone exploration for an "intruder detection" feature idea for MaxGCS.
Nothing here is wired into the app — it's a separate script on a separate
branch (`intruder-detection-poc`) so it doesn't touch or risk the `mannat`
branch's ongoing work.

## What's here

`fake_intruder_trigger.py` — a fake trigger that prints `INTRUDER DETECTED`
at a random interval (10-30s). It stands in for a real detector (e.g. a
motion/object-detection model watching the video feed) so the rest of the
feature — what the GCS should actually *do* when an intruder is detected
(alert banner, sound, log entry, recording start, etc.) — can be designed
and demoed before any real detection logic exists.

## Run it

```bash
python fake_intruder_trigger.py
```

## Not yet decided / next steps

- How a real detector would actually see the intruder (a model run against
  the native-capture/RTSP feed? a separate camera?)
- How a detection event should reach the GCS app (a file it polls, a local
  HTTP call, a Tauri event, something else)
- What the GCS should do when it receives one
