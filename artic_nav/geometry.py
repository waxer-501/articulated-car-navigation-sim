"""Known geometry and prediction model. This module never imports the simulator."""
import math
import numpy as np


def wrap(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


def rotation(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def advance(state, speed, alpha_rate, dt, geom):
    """State is pivot x,y, front yaw, front-minus-rear articulation.

    Front axle longitudinal speed is the prediction input, NOT a measurement.
    No-slip lateral constraints at both axles give the pivot twist below.
    """
    x, y, theta, alpha = state
    lf, lr = geom["front_length"], geom["rear_length"]
    limit = math.radians(geom["max_articulation_deg"])
    alpha_rate = np.clip(alpha_rate, (-limit-alpha)/dt, (limit-alpha)/dt)
    omega = (speed * np.sin(alpha) + lr * alpha_rate) / (lr + lf * np.cos(alpha))
    mid = theta + 0.5 * omega * dt
    vx = speed * np.cos(mid) + lf * omega * np.sin(mid)
    vy = speed * np.sin(mid) - lf * omega * np.cos(mid)
    return np.array([x + dt*vx, y + dt*vy, wrap(theta + omega*dt), alpha + alpha_rate*dt])


def footprint(state, geom):
    p = np.asarray(state[:2])
    theta, alpha = state[2:4]
    points = []
    for name, angle, axle in (("front", theta, geom["front_length"]),
                              ("rear", theta-alpha, -geom["rear_length"])):
        rot = rotation(angle)
        center = geom[name+"_body_center"]
        half = geom[name+"_body_half_length"]
        for dx in (-half, half):
            for dy in (-geom["body_half_width"], geom["body_half_width"]):
                points.append(p + rot @ [center+dx, dy])
        for side in (-1, 1):
            for dx in (-geom["wheel_radius"], geom["wheel_radius"]):
                for dy in (-geom["wheel_width"]/2, geom["wheel_width"]/2):
                    points.append(p + rot @ [axle+dx, side*geom["track"]/2+dy])
    return np.asarray(points)


def inside(state, geom, margin=0.0):
    return bool(np.max(np.abs(footprint(state, geom))) <= 1.0-margin)


def body_corners(state, geom, name):
    theta = state[2] if name == "front" else state[2]-state[3]
    rot = rotation(theta)
    center, half = geom[name+"_body_center"], geom[name+"_body_half_length"]
    return np.array([[*(np.asarray(state[:2]) + rot @ [center+dx, dy]), geom["body_z"]+dz]
                     for dx in (-half, half)
                     for dy in (-geom["body_half_width"], geom["body_half_width"])
                     for dz in (-geom["body_half_height"], geom["body_half_height"])])


def project(points, camera):
    points = np.asarray(points)
    f = camera["height"] / (2*np.tan(np.deg2rad(camera["fovy"])/2))
    depth = camera["z"]-points[:, 2]
    return np.column_stack((camera["width"]/2+f*points[:, 0]/depth,
                            camera["height"]/2-f*points[:, 1]/depth))


def ground_from_pixel(pixel, camera, z=0.0):
    f = camera["height"] / (2*np.tan(np.deg2rad(camera["fovy"])/2))
    return np.array([(pixel[0]-camera["width"]/2)*(camera["z"]-z)/f,
                     -(pixel[1]-camera["height"]/2)*(camera["z"]-z)/f])


def predicted_box(state, geom, camera, name):
    uv = project(body_corners(state, geom, name), camera)
    return np.r_[uv.min(axis=0), uv.max(axis=0)]
