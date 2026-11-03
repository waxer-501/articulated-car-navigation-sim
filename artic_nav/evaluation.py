"""Privileged scoring: never called by, or sent to, the navigator."""
import numpy as np
from .geometry import wrap


class Evaluator:
    def __init__(self, cfg):
        self.cfg = cfg
        self.max_extent = 0.
        self.max_alpha = 0.
        self.samples = []
        self.stable_since = None
        self.stable_duration = 0.
        self.goal = None
        self.reference = None
        self.path_errors = []
        self.estimation_errors = []

    def sample(self, truth, output=None):
        self.max_extent = max(self.max_extent, truth["extent"])
        self.max_alpha = max(self.max_alpha, abs(truth["state"][3]))
        if output is None:
            return
        state = np.asarray(truth["state"])
        self.samples.append(truth)
        if output.get("goal") is not None:
            self.goal = np.asarray(output["goal"])
        if output.get("path") is not None:
            self.reference = np.asarray(output["path"])
        if self.reference is not None and output["mode"] == "running":
            self.path_errors.append(float(np.min(np.linalg.norm(self.reference[:, :2]-state[:2], axis=1))))
        if output.get("estimate") is not None:
            self.estimation_errors.append(float(np.linalg.norm(np.asarray(output["estimate"])[:2]-state[:2])))
        if self.goal is None:
            return
        e = self.cfg["evaluation"]
        okay = (np.linalg.norm(state[:2]-self.goal[:2]) <= e["position_tolerance"]
                and abs(wrap(state[2]-self.goal[2])) <= np.deg2rad(e["heading_tolerance_deg"])
                and truth["speed"] <= e["stopped_speed"]
                and abs(truth["yaw_rate"]) <= np.deg2rad(e["stopped_yaw_rate_deg"]))
        if okay:
            if self.stable_since is None:
                self.stable_since = truth["time"]
            self.stable_duration = truth["time"]-self.stable_since
        else:
            self.stable_since = None
            self.stable_duration = 0.

    def report(self, output, reason, curve=False):
        final = self.samples[-1] if self.samples else None
        state = None if final is None else np.asarray(final["state"])
        pos_error = None if self.goal is None or state is None else float(np.linalg.norm(state[:2]-self.goal[:2]))
        yaw_error = None if self.goal is None or state is None else float(np.rad2deg(abs(wrap(state[2]-self.goal[2]))))
        rms = float(np.sqrt(np.mean(np.square(self.path_errors)))) if self.path_errors else None
        boundary_ok = self.max_extent <= 1.+1e-6
        articulation_ok = self.max_alpha <= np.deg2rad(self.cfg["geometry"]["max_articulation_deg"])+1e-5
        success = (output.get("mode") == "complete" and self.stable_duration >= self.cfg["evaluation"]["settle_seconds"]
                   and boundary_ok and articulation_ok and (not curve or (rms is not None and rms <= self.cfg["evaluation"]["path_rms_tolerance"])))
        failures = []
        if output.get("mode") != "complete":
            failures.append("navigator_did_not_complete")
        if pos_error is None or pos_error > self.cfg["evaluation"]["position_tolerance"]:
            failures.append("true_position_outside_tolerance")
        if yaw_error is None or yaw_error > self.cfg["evaluation"]["heading_tolerance_deg"]:
            failures.append("true_front_heading_outside_tolerance")
        if self.stable_duration < self.cfg["evaluation"]["settle_seconds"]:
            failures.append("true_pose_and_stop_not_stable_for_one_second")
        if not boundary_ok:
            failures.append("vehicle_crossed_boundary")
        if not articulation_ok:
            failures.append("mechanical_angle_limit_exceeded")
        if curve and (rms is None or rms > self.cfg["evaluation"]["path_rms_tolerance"]):
            failures.append("curve_tracking_rms_exceeded")
        return {"success": bool(success), "reason": reason, "navigator_mode": output.get("mode"),
                "failure_causes": failures,
                "navigator_status": output.get("status"), "position_error_m": pos_error,
                "front_heading_error_deg": yaw_error, "stable_duration_s": self.stable_duration,
                "boundary_ok": boundary_ok, "max_absolute_extent_m": self.max_extent,
                "articulation_ok": bool(articulation_ok), "max_articulation_deg": float(np.rad2deg(self.max_alpha)),
                "path_rms_m": rms,
                "position_estimation_rms_m": float(np.sqrt(np.mean(np.square(self.estimation_errors)))) if self.estimation_errors else None,
                "sim_time_s": None if final is None else final["time"],
                "seed": self.cfg["seed"], "profile": self.cfg["profile"], "curve": curve,
                "scope": "simulation only; synthetic detector; unknown toy actuator parameters"}
