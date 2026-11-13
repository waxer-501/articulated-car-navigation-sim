"""Unprivileged algorithm; all inputs are serializable public messages."""
import numpy as np
from .estimator import Estimator
from .planner import HybridAStar, PlanningError, curve_reference
from .controller import MPC
from .geometry import wrap


class Navigation:
    def __init__(self, cfg):
        self.cfg = cfg
        self.estimator = Estimator(cfg)
        self.planner = HybridAStar(cfg)
        self.controller = MPC(cfg)
        self.goal = None
        self.mode = "idle"
        self.status = "等待视觉定位与 IMU 静止标定"
        self.path = None
        self.settle_start = None
        self.curve = False
        self.stop_requested = False
        self.filtered_speed = 0.
        self.best_progress = np.inf
        self.progress_time = 0.
        self.replans = 0
        self.sensor_time = 0.

    def update(self, request):
        self.estimator.consume(request.get("sensors", []))
        e = self.estimator
        # The EKF integration time need not advance when IMUs disappear. Use
        # the newest allowed sensor timestamp to age feedback, including empty
        # camera frames; otherwise a frozen EKF clock would hide an IMU outage.
        self.sensor_time = max(self.sensor_time, e.time,
                               max((p["time"] for p in request.get("sensors", [])), default=0.))
        now = self.sensor_time
        biases = e.x[5:7] if e.x is not None else e.gyro_bias
        self.controller.steering.observe(request.get("sensors", []), biases)
        action = request.get("action")
        if action == "stop":
            self.mode = "stopped"
            self.goal = None
            self.status = "已停车"
            self.settle_start = None
            self.best_progress = np.inf
            self.progress_time = self.estimator.time
            self.replans = 0
        if action in ("goal", "curve"):
            if action == "goal":
                goal = np.asarray(request["goal"], float)
                if goal.shape != (3,) or not np.isfinite(goal).all():
                    raise ValueError("Goal must be finite x,y,front yaw")
                self.goal = goal
            self.curve = action == "curve"
            self.mode = "pending"
            self.path = None
            self.settle_start = None
            self.replans = 0
        state = e.state()
        if state is not None:
            self.filtered_speed += .15*(e.x[4]-self.filtered_speed)
        command = np.zeros(2)
        pos_sigma, yaw_sigma = e.sigma()
        age = now-e.last_visual
        ready = (state is not None and e.calibrated and age < self.cfg["navigation"]["vision_stop_age"]
                 and self.controller.steering.fresh(now))
        if not ready:
            self.status = e.initialization_reason if state is None else "等待有效视觉、双 IMU 观测与静止标定"
        elif pos_sigma > self.cfg["navigation"]["max_position_sigma"] or yaw_sigma > np.deg2rad(self.cfg["navigation"]["max_heading_sigma_deg"]):
            self.status = "定位不确定度过大，停车等待"
        else:
            if self.mode == "pending":
                try:
                    if self.curve:
                        self.path = curve_reference(state, self.cfg)
                        self.goal = self.path[-1, :3].copy()
                    else:
                        self.path = self.planner.plan(state, self.goal, self.cfg["navigation"]["footprint_margin"]+2*pos_sigma)
                    self.controller.set_path(self.path)
                    self.mode = "running"
                    self.progress_time = e.time
                    self.best_progress = np.inf
                except PlanningError as exc:
                    self.mode, self.status = "failed", str(exc)
            if self.mode == "running":
                error = np.linalg.norm(state[:2]-self.goal[:2])
                yaw_error = abs(wrap(state[2]-self.goal[2]))
                n = self.cfg["navigation"]
                progress = error+.15*yaw_error
                if progress < self.best_progress-.004:
                    self.best_progress = progress
                    self.progress_time = e.time
                elif e.time-self.progress_time > 6 and not self.curve:
                    # A saturated joint cannot finish every terminal yaw locally.
                    # Replan a feasible forward/reverse manoeuvre from sensor state.
                    self.mode = "pending"
                    self.replans += 1
                    self.status = "终点姿态调整：重新规划前进／倒车动作"
                    return self.update({"sensors": []})
                if error < n["position_tolerance"] and yaw_error < np.deg2rad(n["heading_tolerance_deg"]):
                    if abs(self.filtered_speed) < .015:
                        if self.settle_start is None:
                            self.settle_start = e.time
                        if e.time-self.settle_start >= n["settle_seconds"]:
                            self.mode = "complete"
                            self.status = "按传感器估计已到达；实际精度见独立评分"
                    else:
                        self.settle_start = None
                else:
                    self.settle_start = None
                    command = self.controller.control(state, self.goal, e.x[4], pos_sigma, now)
                    if age > n["vision_slow_age"]:
                        command *= .35
                if self.mode == "running":
                    self.status = f"导航中 · 距离 {error:.3f} m · 前车朝向误差 {np.rad2deg(yaw_error):.1f}°"
            elif self.mode == "idle":
                self.status = "定位就绪，请点选目标或启动曲线演示"
        # Every issued neutral/scaled command must reach the learning history too;
        # otherwise a pause or visual interruption could be learned as motor gain.
        if not np.any(command):
            self.controller.steering.control(0., now, 0. if state is None else state[3])
        elif self.controller.steering.commands:
            self.controller.steering.commands[-1] = (now, float(command[1]))
        result = {"time": now, "command": command.tolist(), "mode": self.mode, "status": self.status,
                  "estimate": None if state is None else state.tolist(),
                  "speed_estimate": None if state is None else float(e.x[4]),
                  "position_sigma": pos_sigma if np.isfinite(pos_sigma) else None,
                  "heading_sigma": yaw_sigma if np.isfinite(yaw_sigma) else None,
                  "goal": None if self.goal is None else self.goal.tolist(),
                  "vision_age": age if np.isfinite(age) else None,
                  "rejected_observations": e.rejected}
        result["replans"] = self.replans
        result["steering_feedback"] = self.controller.steering.diagnostics()
        if self.path is not None:
            result["path"] = self.path.tolist()
        return result
