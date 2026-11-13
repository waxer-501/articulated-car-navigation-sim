import numpy as np
from scipy.optimize import minimize
from .geometry import advance, footprint, wrap
from .steering import SteeringFeedback


class MPC:
    def __init__(self, cfg):
        self.g, self.n = cfg["geometry"], cfg["navigation"]
        self.path = None
        self.index = 0
        self.warm = np.zeros((4, 2))
        self.last_result = {}
        self.steering = SteeringFeedback(cfg)

    def set_path(self, path):
        self.path = np.asarray(path)
        self.index = 0
        self.warm[:] = 0

    def control(self, state, goal, speed, sigma, now):
        path = self.path
        direction = np.sign(path[min(self.index+1, len(path)-1), 4])
        phase_end = len(path)-1
        for j in range(self.index+2, len(path)):
            if path[j, 4]*direction < 0:
                phase_end = j-1
                break
        if phase_end < len(path)-1 and np.linalg.norm(state[:2]-path[phase_end, :2]) < .035 and abs(speed) < .025:
            self.index = phase_end+1
            direction = np.sign(path[self.index, 4])
            phase_end = len(path)-1
            for j in range(self.index+1, len(path)):
                if path[j, 4]*direction < 0:
                    phase_end = j-1
                    break
        end = min(phase_end+1, self.index+30)
        candidates = path[self.index:end]
        distances = np.sum((candidates[:, :2]-state[:2])**2, axis=1)+.006*wrap(candidates[:, 2]-state[2])**2
        self.index += int(np.argmin(distances))
        horizon, dt = self.n["horizon_steps"], self.n["horizon_dt"]
        maxv, maxr = self.n["max_speed"], np.deg2rad(self.n["max_steer_rate_deg"])
        remaining = phase_end-self.index
        final_phase = phase_end == len(path)-1
        target = goal if final_phase else path[phase_end, :3]
        terminal = remaining < 3 and np.linalg.norm(state[:2]-target[:2]) < .1
        refs = []
        for k in range(horizon):
            idx = min(phase_end, self.index+2+round((k+1)*dt/.1))
            refs.append(path[idx].copy())
        refs = np.asarray(refs)
        if terminal:
            refs[:, :3] = target[:3]
            refs[:, 4:6] = 0
        margin = self.n["footprint_margin"]+min(.06, 2*sigma)
        def rollout(flat, return_path=False):
            controls = flat.reshape(4, 2)
            s = np.array(state)
            value = 0.
            out = []
            for k in range(horizon):
                u = controls[min(3, k*4//horizon)]
                proposed_alpha = s[3]+u[1]*dt
                limit = np.deg2rad(self.g["max_articulation_deg"])
                value += 50*(proposed_alpha-np.clip(proposed_alpha, -limit, limit))**2
                s = advance(s, u[0], u[1], dt, self.g)
                e = s[:2]-refs[k, :2]
                yaw = wrap(s[2]-refs[k, 2])
                value += (90 if terminal else 100)*np.dot(e, e)+(12.0 if terminal else 4.)*yaw*yaw
                if not terminal:
                    value += .5*(s[3]-refs[k, 3])**2+.3*(u[0]-refs[k, 4])**2
                value += .012*u[1]**2+.01*u[0]**2
                excess = np.maximum(np.abs(footprint(s, self.g))-(1-margin), 0)
                value += 20000*np.sum(excess**2)
                out.append(s.copy())
            value += .02*np.sum(np.diff(controls, axis=0)**2)
            return np.asarray(out) if return_path else value
        if np.max(np.abs(self.warm)) < 1e-8 and not terminal:
            self.warm[:, 0] = refs[0, 4]
            self.warm[:, 1] = refs[0, 5]
        v_bounds = (-maxv, maxv) if terminal and final_phase else ((0, maxv) if direction >= 0 else (-maxv, 0))
        result = minimize(rollout, self.warm.ravel(), method="L-BFGS-B",
                          bounds=[v_bounds, (-maxr, maxr)]*4,
                          options={"maxiter": 18, "ftol": 1e-5, "maxls": 12})
        self.warm = result.x.reshape(4, 2)
        desired_v, desired_r = self.warm[0]
        predicted = rollout(result.x, True)
        if np.max(np.abs([footprint(s, self.g) for s in predicted])) > 1-margin+.002:
            self.last_result = {"safe": False, "reason": "预测扫掠范围接近场地边界"}
            return np.zeros(2)
        self.last_result = {"safe": True, "cost": float(result.fun), "optimizer_success": bool(result.success)}
        # Speed feedback is EKF-derived, never wheel/physics velocity.
        drive = (desired_v+.35*(desired_v-speed))/self.n["nominal_drive_speed"]
        if abs(drive) > .02:
            drive += np.sign(drive)*.025
        drive = np.clip(drive, -maxv/self.n["nominal_drive_speed"], maxv/self.n["nominal_drive_speed"])
        steer = self.steering.control(desired_r, now, state[3])
        return np.clip([drive, steer], -1, 1)
