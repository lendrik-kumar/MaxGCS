#!/usr/bin/env python3
"""Fake intruder-detection trigger — standalone proof of concept.

Not wired into MaxGCS in any way. It doesn't watch a camera, doesn't talk
to the app, doesn't touch any GCS code — it just prints an "intruder
detected" line at a random interval, to stand in for a real detector while
the rest of this feature (what should actually happen in the GCS when an
intruder is detected) gets designed.

Run it standalone:
    python fake_intruder_trigger.py
"""

import random
import time
from datetime import datetime

MIN_INTERVAL_S = 10
MAX_INTERVAL_S = 30


def main() -> None:
    print("[intruder-sim] Fake intruder detector started (Ctrl+C to stop).")
    print(f"[intruder-sim] Firing at a random interval between {MIN_INTERVAL_S}-{MAX_INTERVAL_S}s.")
    try:
        while True:
            time.sleep(random.uniform(MIN_INTERVAL_S, MAX_INTERVAL_S))
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            print(f"[{timestamp}] INTRUDER DETECTED")
    except KeyboardInterrupt:
        print("\n[intruder-sim] Stopped.")


if __name__ == "__main__":
    main()
