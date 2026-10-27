"""Where is the target when we cannot see it? A tiny alpha-beta filter on the target's WORLD bearing.

The camera turns with the drone, so a pixel position alone says nothing once the picture has moved. The
tracker therefore works in world terms: world bearing = drone heading + the angle of the target in the
picture. While the target is visible the filter smooths that bearing and learns how fast it is changing
(deg/s). When the target vanishes behind a tree or a corner, `coast()` keeps moving the estimate along its
last velocity, with the velocity fading out (a target seldom keeps going in a straight line for long), so the
tracker can keep yawing toward where it is expected to reappear.
"""
import math


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class BearingPredictor:
    def __init__(self, alpha=0.6, beta=0.25, max_rate=60.0, fade_s=3.0):
        self.alpha, self.beta, self.max_rate, self.fade_s = alpha, beta, max_rate, fade_s
        self.valid = False
        self.est = 0.0  # degrees, kept near the data (wrap with wrap180 when comparing)
        self.vel = 0.0  # deg/s
        self.t = 0.0

    def reset(self, bearing, t):
        self.est, self.vel, self.t, self.valid = bearing % 360.0, 0.0, t, True

    def update(self, bearing, t):
        """Feed one measured world bearing (deg) taken at time t (s)."""
        if not self.valid:
            self.reset(bearing, t)
            return self.est
        dt = max(1e-3, min(0.5, t - self.t))
        pred = self.est + self.vel * dt
        r = wrap180(bearing - pred)
        self.est = (pred + self.alpha * r) % 360.0
        self.vel = max(-self.max_rate, min(self.max_rate, self.vel + self.beta * r / dt))
        self.t = t
        return self.est

    def coast(self, t):
        """No measurement: advance the estimate along the (fading) velocity and return it."""
        if not self.valid:
            return self.est
        dt = max(0.0, min(0.5, t - self.t))
        self.est = (self.est + self.vel * dt) % 360.0
        self.vel *= math.exp(-dt / self.fade_s)
        self.t = t
        return self.est
