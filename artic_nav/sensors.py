"""Privileged observation synthesis; output schemas deliberately exclude truth."""
from collections import deque
import numpy as np
import mujoco
from .geometry import project


class Sensors:
    def __init__(self, world, cfg):
        self.world, self.cfg = world, cfg
        # Separate RNG streams keep observations reproducible regardless of rendering.
        self.rng = np.random.default_rng(cfg["seed"])
        self.imu_rng = np.random.default_rng(cfg["seed"]+100000)
        self.profile = cfg["detector"][cfg["profile"]]
        self.pending = deque()
        self.next_camera = self.next_imu = 0.0
        self.box_bias = self.rng.normal(0, self.profile["bias_std"], (2, 2))
        self.gyro_bias = self.imu_rng.normal(0, cfg["imu"]["gyro_bias_std"], (2, 3))
        self.accel_bias = self.imu_rng.normal(0, cfg["imu"]["accel_bias_std"], (2, 3))
        self.blackout = None

    def _box(self, name):
        geom = self.world.model.geom(name+"_shell")
        gd = self.world.data.geom(name+"_shell")
        half = geom.size
        local = np.array([[a*half[0], b*half[1], c*half[2]]
                          for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)])
        corners = local @ gd.xmat.reshape(3, 3).T + gd.xpos
        camera = self.cfg["camera"]
        if np.any(corners[:, 2] >= camera["z"]):
            return None
        uv = project(corners, camera)
        box = np.r_[uv.min(axis=0), uv.max(axis=0)]
        box[[0, 2]] = np.clip(box[[0, 2]], 0, camera["width"]-1)
        box[[1, 3]] = np.clip(box[[1, 3]], 0, camera["height"]-1)
        if np.min(box[2:]-box[:2]) < 3:
            return None
        # Camera rays test actual geometric visibility (no detection through occluders).
        hits = 0
        origin = np.array([0., 0., camera["z"]])
        f = camera["height"]/(2*np.tan(np.deg2rad(camera["fovy"])/2))
        hit = np.zeros(1, dtype=np.int32)
        for a in (.2, .5, .8):
            for b in (.2, .5, .8):
                pixel = box[:2]+(box[2:]-box[:2])*[a, b]
                direction = np.array([(pixel[0]-camera["width"]/2)/f,
                                      -(pixel[1]-camera["height"]/2)/f, -1.])
                direction /= np.linalg.norm(direction)
                mujoco.mj_ray(self.world.model, self.world.data, origin, direction, None, 1, -1, hit)
                hits += int(hit[0] == geom.id)
        return box if hits >= 2 else None

    def camera_frame(self, rgb=False):
        t = self.world.time
        out = []
        p = self.profile
        blocked = self.blackout is not None and self.blackout[0] <= t < self.blackout[1]
        self.box_bias = .995*self.box_bias + self.rng.normal(0, p["bias_std"]*.1, (2, 2))
        for i, name in enumerate(("front", "rear")):
            box = self._box(name)
            if blocked or box is None or self.rng.random() < p["miss"]:
                continue
            center = (box[:2]+box[2:])/2 + self.box_bias[i]+self.rng.normal(0, p["center_std"], 2)
            size = (box[2:]-box[:2])*np.clip(1+self.rng.normal(0, p["size_std"], 2), .3, 2)
            noisy = np.r_[center-size/2, center+size/2]
            noisy[[0, 2]] = np.clip(noisy[[0, 2]], 0, self.cfg["camera"]["width"]-1)
            noisy[[1, 3]] = np.clip(noisy[[1, 3]], 0, self.cfg["camera"]["height"]-1)
            out.append({"class": name, "box": noisy.tolist(), "confidence": float(self.rng.uniform(.7, .95))})
        if not blocked and self.rng.random() < p["false_positive"]:
            xy = self.rng.uniform(50, 850, 2)
            out.append({"class": self.rng.choice(["front", "rear"]), "box": [*xy, *(xy+50)], "confidence": .65})
        frame = {"time": t, "detections": out}
        if rgb:
            frame["rgb"] = self.world.render("overhead")
        return frame

    def sample(self, rgb=False):
        t, output = self.world.time, []
        if t+1e-8 >= self.next_imu:
            self.next_imu += 1/self.cfg["imu"]["hz"]
            cfg = self.cfg["imu"]
            self.gyro_bias += self.imu_rng.normal(0, cfg["gyro_bias_walk"]/np.sqrt(cfg["hz"]), (2, 3))
            packet = {"kind": "imu", "time": t}
            for i, name in enumerate(("front", "rear")):
                acc = self.world.data.sensor(name+"_accel").data.copy()
                gyro = self.world.data.sensor(name+"_gyro").data.copy()
                packet[name] = {"accel": (acc+self.accel_bias[i]+self.imu_rng.normal(0, cfg["accel_noise"], 3)).tolist(),
                                "gyro": (gyro+self.gyro_bias[i]+self.imu_rng.normal(0, cfg["gyro_noise"], 3)).tolist()}
            output.append(packet)
        if t+1e-8 >= self.next_camera:
            self.next_camera += 1/self.cfg["camera"]["hz"]
            frame = self.camera_frame(rgb)
            frame["kind"] = "camera"
            self.pending.append((t+self.profile["delay"], frame))
        while self.pending and self.pending[0][0] <= t+1e-8:
            output.append(self.pending.popleft()[1])
        return output
