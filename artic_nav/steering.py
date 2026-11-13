"""Sensor-only articulation-rate feedback; no plant configuration or truth IO."""
from collections import deque
import numpy as np


def remote_to_steer(value):
    """Remote 0=left, .5=neutral, 1=right; internal +steer increases alpha."""
    if not np.isfinite(value) or not 0 <= value <= 1:
        raise ValueError("Remote steering must be finite and in [0, 1]")
    return 1-2*value


def steer_to_remote(value):
    if not np.isfinite(value) or not -1 <= value <= 1:
        raise ValueError("Steering must be finite and in [-1, 1]")
    return (1-value)/2


class SteeringFeedback:
    """PI rate loop with separately learned local gains in both directions.

    The gain is an operating-point approximation, not an identified global motor
    curve. Learning uses only settled commands and bias-corrected gyro differences.
    No calibration manoeuvre is generated: excitation comes from normal tracking.
    """

    def __init__(self, cfg):
        self.n = cfg["navigation"]["steering"]
        self.limit = np.deg2rad(cfg["geometry"]["max_articulation_deg"])
        self.margin = np.deg2rad(self.n["limit_margin_deg"])
        self.gains = np.full(2, np.deg2rad(self.n["prior_rate_deg"]))
        self.updates = [0, 0]
        self.rate = None
        self.observation_time = -np.inf
        self.commands = deque(maxlen=100)
        self.integral = 0.
        self.previous_target = 0.
        self.last_control_time = None

    def observe(self, packets, biases):
        for packet in packets:
            if packet["kind"] != "imu" or packet["time"] <= self.observation_time:
                continue
            measured = (packet["front"]["gyro"][2]-biases[0]
                        -packet["rear"]["gyro"][2]+biases[1])
            dt = packet["time"]-self.observation_time
            weight = 1. if self.rate is None or dt > .2 else 1-np.exp(-dt/.025)
            self.rate = measured if self.rate is None else self.rate+weight*(measured-self.rate)
            self.observation_time = packet["time"]

    def fresh(self, now):
        return self.rate is not None and 0 <= now-self.observation_time <= self.n["feedback_max_age"]

    def _learn(self, now, alpha):
        # Reject transients, command reversals, stale samples and joint stops.
        if not self.fresh(now) or abs(alpha) >= self.limit-2*self.margin:
            return
        start = now-self.n["learning_settle_seconds"]
        before = [row for row in self.commands if row[0] <= start+1e-8]
        if not before:
            return
        rows = [before[-1]]+[row for row in self.commands if row[0] > start+1e-8]
        values = np.array([row[1] for row in rows])
        u = float(np.mean(values))
        if abs(u) < .12 or np.ptp(values) > .06 or np.min(values)*np.max(values) <= 0:
            return
        if self.rate*u <= 0 or abs(self.rate) < .015:
            return
        side = 0 if u > 0 else 1
        measured_gain = np.clip(self.rate/u, .06, 1.6)
        self.gains[side] += self.n["learning_rate"]*(measured_gain-self.gains[side])
        self.updates[side] += 1

    def control(self, desired_rate, now, alpha):
        dt = 0. if self.last_control_time is None else np.clip(now-self.last_control_time, 0, .2)
        if self.last_control_time is not None and now-self.last_control_time > .2:
            self.integral = 0.
            self.commands.clear()
        self._learn(now, alpha)
        self.last_control_time = now
        outward = desired_rate*alpha > 0 and abs(alpha) >= self.limit-self.margin
        if not self.fresh(now) or abs(desired_rate) < 1e-4 or outward:
            self.integral = 0.
            self.previous_target = 0.
            self.commands.append((now, 0.))
            return 0.
        if desired_rate*self.previous_target <= 0:
            self.integral = 0.
        side = 0 if desired_rate > 0 else 1
        gain = self.gains[side]
        error = desired_rate-self.rate
        # Anti-windup: only integrate while unsaturated or towards release.
        base = desired_rate/gain+self.n["feedback_gain"]*error/gain
        proposed = self.integral+self.n["integral_gain"]*error/gain*dt
        if abs(base+proposed) <= 1 or (base+proposed)*error < 0:
            self.integral = float(np.clip(proposed, -.45, .45))
        command = float(np.clip(base+self.integral, -1, 1))
        # Braking is neutral; never reverse the steering motor merely to reduce
        # a still-positive/negative rate. This also prevents deadzone chatter.
        if command*desired_rate < 0:
            command = 0.
        self.previous_target = desired_rate
        self.commands.append((now, command))
        return command

    def diagnostics(self):
        return {"rate_estimate": None if self.rate is None else float(self.rate),
                "target_rate": float(self.previous_target),
                "remote_value": steer_to_remote(self.commands[-1][1]) if self.commands else .5,
                "fresh": (self.last_control_time is not None
                          and self.fresh(max(self.last_control_time, self.observation_time))),
                "observation_time": None if not np.isfinite(self.observation_time) else self.observation_time,
                "positive_alpha_gain": float(self.gains[0]), "negative_alpha_gain": float(self.gains[1]),
                "gain_updates": self.updates.copy(), "integral": self.integral}
