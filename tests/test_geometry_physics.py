import numpy as np
from artic_nav.config import load_config
from artic_nav.geometry import advance, inside, predicted_box, project
from artic_nav.physics import World
from artic_nav.sensors import Sensors


def test_geometry_and_mass():
    cfg = load_config(profile="ideal")
    world = World(cfg)
    g = cfg["geometry"]
    assert np.isclose(world.model.body_mass.sum(), 1.)
    assert np.isclose(g["track"]+g["wheel_width"], .18)
    assert np.isclose(g["track"]-g["wheel_width"], .10)
    assert np.isclose(g["wheel_radius"]*2, .103)
    assert inside([0, 0, .3, .65], g)
    assert not inside([.99, 0, 0, 0], g)
    world.close()


def test_physics_drive_turn_hold_and_sensors():
    cfg = load_config(profile="ideal")
    world = World(cfg)
    sensors = Sensors(world, cfg)
    for _ in range(400):
        world.step()
    start = np.array(world.truth()["state"])
    # Exact projection and actual physical geometry must agree at rest.
    for name in ("front", "rear"):
        np.testing.assert_allclose(sensors._box(name), predicted_box(start, cfg["geometry"], cfg["camera"], name), atol=.1)
    world.command([.4, .4])
    for _ in range(800):
        world.step()
    state = world.truth()["state"]
    assert np.linalg.norm(np.asarray(state)[:2]-start[:2]) > .07
    assert .12 < state[3] < .6
    assert abs(world.data.sensor("front_accel").data[0]) < 1.
    world.command([0, 0])
    for _ in range(400):
        world.step()
    a = world.truth()["state"][3]
    for _ in range(400):
        world.step()
    assert abs(world.truth()["state"][3]-a) < .01
    assert abs(a) > .1  # release does not recenter
    world.close()


def test_prediction_respects_both_axle_lateral_constraints():
    cfg = load_config()
    g = cfg["geometry"]
    s = np.array([0., 0., .4, .3])
    dt = 1e-6
    n = advance(s, -.1, .2, dt, g)
    for name, length, angle, newangle in (("front", g["front_length"], s[2], n[2]),
                                        ("rear", -g["rear_length"], s[2]-s[3], n[2]-n[3])):
        p0 = s[:2]+length*np.array([np.cos(angle), np.sin(angle)])
        p1 = n[:2]+length*np.array([np.cos(newangle), np.sin(newangle)])
        lateral_speed = ((p1-p0)/dt)@[-np.sin(angle), np.cos(angle)]
        assert abs(lateral_speed) < 1e-6
