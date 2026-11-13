"""Privileged physics boundary. Never imported by the navigation worker."""
import os
os.environ.setdefault("MUJOCO_GL", "egl")
from collections import deque
import numpy as np
import mujoco
from .geometry import wrap, footprint


def make_xml(cfg):
    g, p, c = cfg["geometry"], cfg["physics"], cfg["camera"]
    def wheels(name, axle):
        out = ""
        for side, sign in (("l", 1), ("r", -1)):
            tag = name+"_"+side
            out += f'''<body name="{tag}" pos="{axle} {sign*g['track']/2} {g['wheel_radius']-g['body_z']}">
              <joint name="{tag}" type="hinge" axis="0 1 0" damping="{p['wheel_damping']}" armature="0.00002"/>
              <geom name="{tag}_tire" type="cylinder" size="{g['wheel_radius']} {g['wheel_width']/2}"
                quat="0.70710678 0.70710678 0 0" mass="{p['mass']*.05}" rgba="0.055 0.065 0.075 1"/>
              <geom type="cylinder" size="0.019 {g['wheel_width']/2+.001}" quat="0.70710678 0.70710678 0 0"
                mass="0" contype="0" conaffinity="0" rgba="0.65 0.72 0.77 1"/>
            </body>'''
        return out
    def chassis(name, color):
        return f'''<geom name="{name}_shell" type="box" pos="{g[name+'_body_center']} 0 0"
          size="{g[name+'_body_half_length']} {g['body_half_width']} {g['body_half_height']}"
          mass="{p['mass']*.4}" rgba="{color}"/>
          <geom type="capsule" fromto="0 0 0 {g[name+'_body_center']} 0 0" size="0.008" mass="0"
            contype="0" conaffinity="0" rgba="0.32 0.37 0.4 1"/>
          <site name="{name}_imu" pos="{g[name+'_body_center']} 0 0.022" size="0.006" rgba="0.9 0.9 0.9 1"/>'''
    motors = "".join(f'<motor name="{n}_{s}" joint="{n}_{s}" ctrlrange="-1 1" gear="{p["drive_torque"]}"/>'
                     for n in ("front", "rear") for s in ("l", "r"))
    return f'''<mujoco model="independent articulated toy">
      <compiler angle="radian"/>
      <option timestep="{p['dt']}" integrator="implicitfast" cone="elliptic" gravity="0 0 -9.81"/>
      <visual><global offwidth="{c['width']}" offheight="{c['height']}"/><quality shadowsize="2048"/></visual>
      <default><geom friction="{p['friction']} 0.005 0.0001" condim="4" solref="0.008 1" solimp="0.95 0.99 0.001"/></default>
      <asset><texture name="floor" type="2d" builtin="checker" width="512" height="512" rgb1="0.18 0.23 0.29" rgb2="0.20 0.25 0.31"/>
        <material name="floor" texture="floor" texrepeat="10 10" reflectance="0.05"/></asset>
      <worldbody>
        <light pos="0 -1 3" dir="0 0 -1" diffuse="0.9 0.9 0.9"/>
        <geom name="ground" type="plane" size="3 3 0.1" material="floor"/>
        <geom type="box" pos="0 1.003 0.001" size="1.006 0.003 0.001" rgba="0.95 0.65 0.2 1" contype="0" conaffinity="0"/>
        <geom type="box" pos="0 -1.003 0.001" size="1.006 0.003 0.001" rgba="0.95 0.65 0.2 1" contype="0" conaffinity="0"/>
        <geom type="box" pos="1.003 0 0.001" size="0.003 1.006 0.001" rgba="0.95 0.65 0.2 1" contype="0" conaffinity="0"/>
        <geom type="box" pos="-1.003 0 0.001" size="0.003 1.006 0.001" rgba="0.95 0.65 0.2 1" contype="0" conaffinity="0"/>
        <camera name="overhead" pos="0 0 {c['z']}" quat="1 0 0 0" fovy="{c['fovy']}"/>
        <camera name="observer" pos="1.8 -2.1 2.1" xyaxes="0.759 0.651 0 -0.395 0.461 0.794" fovy="52"/>
        <body name="front" pos="0 0 {g['body_z']}">
          <freejoint name="base"/>
          {chassis('front', '0.15 0.73 0.75 1')}
          {wheels('front', g['front_length'])}
          <body name="rear">
            <joint name="articulation" type="hinge" axis="0 0 1" range="{-np.deg2rad(g['max_articulation_deg']-.1)} {np.deg2rad(g['max_articulation_deg']-.1)}"
              damping="0.03" frictionloss="{p['steer_friction']}" armature="0.0005"/>
            {chassis('rear', '0.94 0.53 0.23 1')}
            {wheels('rear', -g['rear_length'])}
          </body>
        </body>
      </worldbody>
      <contact><exclude body1="front" body2="rear"/></contact>
      <actuator>{motors}<general name="steering" joint="articulation"
        gainprm="{p['steer_motor_damping']}" biastype="affine" biasprm="0 0 {-p['steer_motor_damping']}"
        ctrllimited="true" ctrlrange="-2 2" forcelimited="true" forcerange="{-p['steer_torque']} {p['steer_torque']}"/></actuator>
      <sensor><accelerometer name="front_accel" site="front_imu"/><gyro name="front_gyro" site="front_imu"/>
        <accelerometer name="rear_accel" site="rear_imu"/><gyro name="rear_gyro" site="rear_imu"/></sensor>
    </mujoco>'''


class World:
    def __init__(self, cfg):
        self.cfg = cfg
        self.model = mujoco.MjModel.from_xml_string(make_xml(cfg))
        self.data = mujoco.MjData(self.model)
        self.renderer = None
        self.steer_joint = self.model.joint("articulation")
        self.command_queue = deque()
        self.filtered = np.zeros(2)
        self.applied = np.zeros(2)
        self.reset(cfg["physics"]["initial_pose"])

    def reset(self, pose):
        mujoco.mj_resetData(self.model, self.data)
        x, y, theta, alpha = pose
        self.data.qpos[:3] = [x, y, self.cfg["geometry"]["body_z"]+0.001]
        self.data.qpos[3:7] = [np.cos(theta/2), 0, 0, np.sin(theta/2)]
        self.data.qpos[self.steer_joint.qposadr[0]] = -alpha
        self.command_queue.clear()
        self.filtered[:] = 0
        self.applied[:] = 0
        mujoco.mj_forward(self.model, self.data)

    @property
    def time(self):
        return float(self.data.time)

    def command(self, command):
        arr = np.asarray(command, float)
        if arr.shape != (2,) or not np.isfinite(arr).all():
            raise ValueError("Command must be two finite normalized values")
        self.command_queue.append((self.time+self.cfg["physics"]["actuator_delay"], np.clip(arr, -1, 1)))

    def emergency_stop(self):
        # Simulator-only stop: remove delayed drive input; wheel damping brakes.
        self.command_queue.clear()
        self.applied[:] = self.filtered[:] = 0

    def step(self):
        p = self.cfg["physics"]
        while self.command_queue and self.command_queue[0][0] <= self.time+1e-9:
            self.applied = self.command_queue.popleft()[1]
        self.filtered += (self.applied-self.filtered)*min(1, p["dt"]/p["actuator_tau"])
        drive, steer = self.filtered
        drive = 0 if abs(drive) < p["drive_deadzone"] else drive
        self.data.ctrl[:4] = drive*np.array([1, p["left_right_gain"], 1, p["left_right_gain"]])
        q = float(self.data.qpos[self.steer_joint.qposadr[0]])
        limit = np.deg2rad(self.cfg["geometry"]["max_articulation_deg"]-.1)
        # Private motor/load surrogate: asymmetric nonlinear no-load speed plus
        # torque/speed drag. It is NOT the navigator's requested angular rate.
        magnitude = max(0., (abs(steer)-p["steer_deadzone"])/(1-p["steer_deadzone"]))
        rate = p["steer_left_rate_deg"] if steer >= 0 else p["steer_right_rate_deg"]
        load = 1-p["steer_angle_loss"]*(q/limit)**2
        load *= 1-p["steer_speed_loss"]*min(1., np.linalg.norm(self.data.qvel[:2])/.15)
        drift = 1+p["steer_drift_fraction"]*np.sin(2*np.pi*self.time/p["steer_drift_period"])
        target = -np.sign(steer)*np.deg2rad(rate)*magnitude**p["steer_exponent"]*load*drift
        target = np.clip(target, (-limit-q)/p["dt"], (limit-q)/p["dt"])
        # MuJoCo integrates the torque/speed slope implicitly; doing the drag
        # explicitly in Python is unstable at this inertia and timestep.
        self.data.ctrl[4] = target
        mujoco.mj_step(self.model, self.data)

    def truth(self):
        front = self.data.body("front")
        mat = front.xmat.reshape(3, 3)
        theta = np.arctan2(mat[1, 0], mat[0, 0])
        alpha = -float(self.data.qpos[self.steer_joint.qposadr[0]])
        state = np.array([*front.xpos[:2], theta, alpha])
        return {"time": self.time, "state": state.tolist(),
                "speed": float(np.linalg.norm(self.data.qvel[:2])),
                "yaw_rate": float(self.data.qvel[5]),
                "extent": float(np.max(np.abs(footprint(state, self.cfg["geometry"])))),
                "height": float(front.xpos[2])}

    def render(self, camera="overhead"):
        if self.renderer is None:
            c = self.cfg["camera"]
            self.renderer = mujoco.Renderer(self.model, height=c["height"], width=c["width"])
        self.renderer.update_scene(self.data, camera=camera)
        return self.renderer.render().copy()

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
