"""Delayed-observation EKF, using only the public sensor contract."""
from collections import deque
import numpy as np
from scipy.optimize import least_squares
from .geometry import wrap, predicted_box, ground_from_pixel


def box_features(box):
    b = np.asarray(box)
    return np.r_[(b[:2]+b[2:])/2, b[2:]-b[:2]]


class Estimator:
    def __init__(self, cfg):
        self.g, self.c, self.n = cfg["geometry"], cfg["camera"], cfg["navigation"]
        # x,y,front yaw,rear yaw,front axle speed,front gyro bias,rear gyro bias
        self.x = None
        self.P = None
        self.time = 0.
        self.last_visual = -np.inf
        self.history = deque(maxlen=400)
        self.calibration = []
        self.accel_bias = 0.
        self.gyro_bias = np.zeros(2)
        self.calibrated = False
        self.rejected = 0
        self.initialization_reason = "等待前后车体检测框"

    def state(self):
        if self.x is None:
            return None
        return np.array([self.x[0], self.x[1], wrap(self.x[2]), wrap(self.x[2]-self.x[3])])

    def sigma(self):
        if self.P is None:
            return np.inf, np.inf
        return float(np.sqrt(max(self.P[0, 0], self.P[1, 1]))), float(np.sqrt(self.P[2, 2]))

    def _measurement(self, x, names):
        state = [x[0], x[1], x[2], wrap(x[2]-x[3])]
        return np.concatenate([box_features(predicted_box(state, self.g, self.c, name)) for name in names])

    def _predict(self, packet):
        dt = packet["time"]-self.time
        self.time = packet["time"]
        if dt <= 0 or self.x is None:
            return
        x = self.x
        wf = packet["front"]["gyro"][2]-x[5]
        wr = packet["rear"]["gyro"][2]-x[6]
        accel = np.clip(packet["front"]["accel"][0]-self.accel_bias, -3, 3)
        th, v, lf = x[2], x[4], self.g["front_length"]
        c, s = np.cos(th), np.sin(th)
        F = np.eye(7)
        F[0, 2] = (-v*s+lf*wf*c)*dt
        F[1, 2] = (v*c+lf*wf*s)*dt
        F[0, 4], F[1, 4] = c*dt, s*dt
        F[0, 5], F[1, 5] = -lf*s*dt, lf*c*dt
        F[2, 5], F[3, 6] = -dt, -dt
        x[0] += (v*c+lf*wf*s)*dt
        x[1] += (v*s-lf*wf*c)*dt
        x[2] = wrap(x[2]+wf*dt)
        x[3] = wrap(x[3]+wr*dt)
        x[4] = np.clip(x[4]+accel*dt, -.5, .5)
        Q = np.diag([.002**2, .002**2, .008**2, .008**2, .08**2, .0001**2, .0001**2])*dt
        self.P = F @ self.P @ F.T + Q

    def imu(self, packet):
        # Startup is held still by Navigation until this calibration completes.
        if not self.calibrated and packet["time"] > .25:
            self.calibration.append([packet["front"]["gyro"][2], packet["rear"]["gyro"][2], packet["front"]["accel"][0]])
            if len(self.calibration) >= 200:
                mean = np.mean(self.calibration, axis=0)
                self.gyro_bias, self.accel_bias = mean[:2], mean[2]
                if self.x is not None:
                    self.x[5:7] = self.gyro_bias
                self.calibrated = True
        self._predict(packet)
        if self.x is not None:
            self.history.append((packet, self.x.copy(), self.P.copy()))

    def _initialize(self, detections, t):
        if set(detections) != {"front", "rear"}:
            return False
        centers = {k: ground_from_pixel(box_features(v)[0:2], self.c, self.g["body_z"]) for k, v in detections.items()}
        delta = centers["front"]-centers["rear"]
        if not .09 < np.linalg.norm(delta) < .23:
            return False
        theta = np.arctan2(delta[1], delta[0])
        midpoint = (centers["front"]+centers["rear"])/2
        obs = np.concatenate([box_features(detections[k]) for k in ("front", "rear")])
        def residual(q):
            state = [q[0], q[1], q[2], q[2]-q[3], 0, 0, 0]
            return (self._measurement(state, ["front", "rear"])-obs)/np.tile([4, 4, 7, 7], 2)
        results = []
        for a in (-.35, 0., .35):
            q = [*midpoint, theta+a/2, a]
            result = least_squares(residual, q, bounds=([-1.3, -1.3, theta-1, -.7], [1.3, 1.3, theta+1, .7]), max_nfev=30)
            results.append(result)
        best = min(results, key=lambda r: np.sum(r.fun**2))
        if np.linalg.norm(best.fun) > 6:
            return False
        for alternative in results:
            separated = (abs(wrap(alternative.x[2]-best.x[2])) > .20
                         or np.linalg.norm(alternative.x[:2]-best.x[:2]) > .025)
            if separated and np.sum(alternative.fun**2)-np.sum(best.fun**2) < 1.0:
                self.initialization_reason = "检测框对应多个可能姿态，停车等待可区分观测"
                return False
        q = best.x
        self.x = np.array([q[0], q[1], q[2], q[2]-q[3], 0., *self.gyro_bias])
        self.P = np.diag([.015**2, .015**2, .10**2, .10**2, .02**2, .005**2, .005**2])
        self.time = t
        self.last_visual = t
        self.history.clear()
        self.initialization_reason = "视觉初始化完成"
        return True

    def camera(self, packet):
        # Candidate association uses innovation, never hidden simulation object IDs.
        detections = {}
        for d in packet["detections"]:
            name = d["class"]
            if name not in ("front", "rear") or d["confidence"] < .6:
                continue
            box = np.asarray(d["box"], float)
            if not np.isfinite(box).all() or np.min(box[2:]-box[:2]) < 3:
                continue
            if name not in detections:
                detections[name] = box
            elif self.x is not None:
                expected = self._measurement(self.x, [name])
                if np.linalg.norm(box_features(box)-expected) < np.linalg.norm(box_features(detections[name])-expected):
                    detections[name] = box
        if not detections:
            return
        if self.x is None:
            self._initialize(detections, packet["time"])
            return
        hist = list(self.history)
        eligible = [i for i, item in enumerate(hist) if item[0]["time"] <= packet["time"]+1e-8]
        if not eligible:
            return
        index = eligible[-1]
        current_x, current_p, current_t = self.x.copy(), self.P.copy(), self.time
        self.x, self.P = hist[index][1].copy(), hist[index][2].copy()
        self.time = hist[index][0]["time"]
        accepted = False
        for name, box in detections.items():
            z = box_features(box)
            h = self._measurement(self.x, [name])
            H = np.zeros((4, 7))
            for j in range(4):
                q = self.x.copy()
                q[j] += 1e-5
                H[:, j] = (self._measurement(q, [name])-h)/1e-5
            R = np.diag([4.0**2, 4.0**2, max(4., z[2]*.07)**2, max(4., z[3]*.07)**2])
            innovation = z-h
            S = H @ self.P @ H.T + R
            if innovation @ np.linalg.solve(S, innovation) > 30:
                self.rejected += 1
                continue
            K = np.linalg.solve(S, H @ self.P).T
            self.x += K @ innovation
            self.x[2:4] = wrap(self.x[2:4])
            # The mechanical limit is known geometry, not a joint-angle sensor.
            alpha = wrap(self.x[2]-self.x[3])
            limit = np.deg2rad(self.g["max_articulation_deg"])
            excess = alpha-np.clip(alpha, -limit, limit)
            self.x[2] = wrap(self.x[2]-.5*excess)
            self.x[3] = wrap(self.x[3]+.5*excess)
            A = np.eye(7)-K@H
            self.P = A @ self.P @ A.T+K@R@K.T
            accepted = True
        if not accepted:
            self.x, self.P, self.time = current_x, current_p, current_t
            return
        self.last_visual = packet["time"]
        hist[index] = (hist[index][0], self.x.copy(), self.P.copy())
        for j in range(index+1, len(hist)):
            self._predict(hist[j][0])
            hist[j] = (hist[j][0], self.x.copy(), self.P.copy())
        self.history = deque(hist, maxlen=400)

    def consume(self, packets):
        for packet in packets:
            if packet["kind"] == "imu":
                self.imu(packet)
            elif packet["kind"] == "camera":
                self.camera(packet)
