from copy import deepcopy
import numpy as np
import pytest
from artic_nav.config import load_config, public_config
from artic_nav.physics import World
from artic_nav.steering import SteeringFeedback, remote_to_steer, steer_to_remote


def imu(t, rate, bias=(.013, -.009)):
    return {"kind": "imu", "time": t,
            "front": {"gyro": [0, 0, rate/2+bias[0]], "accel": [0, 0, 9.81]},
            "rear": {"gyro": [0, 0, -rate/2+bias[1]], "accel": [0, 0, 9.81]}}


def test_remote_neutral_and_direction():
    assert remote_to_steer(0) == 1
    assert remote_to_steer(.5) == 0
    assert remote_to_steer(1) == -1
    for u in (0, .1, .5, .9, 1):
        assert steer_to_remote(remote_to_steer(u)) == pytest.approx(u)
    for u in (-.1, 1.1, float("nan")):
        with pytest.raises(ValueError):
            remote_to_steer(u)


def test_plant_does_not_use_navigation_rate_and_is_asymmetric():
    cfg = load_config()
    changed = deepcopy(cfg)
    changed["navigation"]["max_steer_rate_deg"] = 3.
    a, b = World(cfg), World(changed)
    try:
        for w in (a, b):
            w.command([0, .5])
            for _ in range(240):
                w.step()
        np.testing.assert_array_equal(a.data.qpos, b.data.qpos)
        np.testing.assert_array_equal(a.data.qvel, b.data.qvel)
        plus = a.truth()["state"][3]
        a.reset(cfg["physics"]["initial_pose"])
        a.command([0, -.5])
        for _ in range(240):
            a.step()
        minus = a.truth()["state"][3]
        assert plus > .02 and minus < -.02
        assert abs(minus) > 1.25*plus
        assert "steer_left_rate_deg" not in repr(public_config(cfg))
    finally:
        a.close()
        b.close()


def test_bias_correction_neutral_stale_limit_and_reversal():
    feedback = SteeringFeedback(public_config(load_config()))
    bias = [.013, -.009]
    assert feedback.control(.2, 0., 0.) == 0  # no observation, no blind steering
    feedback.observe([imu(.1, .1)], bias)
    assert feedback.rate == pytest.approx(.1)
    assert feedback.control(.2, .1, 0.) > 0
    assert feedback.control(0., .1, 0.) == 0  # neutral holds, no residual integral
    assert feedback.control(.2, .3, 0.) == 0  # stale IMUs
    feedback.observe([imu(.4, 0)], bias)
    assert feedback.control(.2, .4, np.deg2rad(39)) == 0  # outward at stop
    assert feedback.control(-.2, .4, np.deg2rad(39)) < 0  # can leave stop


def test_gain_learning_excludes_mechanical_stops_and_transients():
    cfg = public_config(load_config())
    feedback = SteeringFeedback(cfg)
    for i in range(1, 15):
        t = i*.1
        feedback.observe([imu(t, .15)], [.013, -.009])
        feedback.commands.append((t, .5))
        feedback._learn(t, np.deg2rad(39))
    assert feedback.updates == [0, 0]
    for i in range(15, 35):
        t = i*.1
        feedback.observe([imu(t, .15)], [.013, -.009])
        feedback.commands.append((t, -.5 if i % 2 else .5))
        feedback._learn(t, 0.)
    assert feedback.updates == [0, 0]
    for i in range(35, 65):
        t = i*.1
        feedback.observe([imu(t, .15)], [.013, -.009])
        feedback.commands.append((t, .5))
        feedback._learn(t, 0.)
    assert feedback.updates[0] > 10 and feedback.updates[1] == 0
    assert feedback.gains[0] == pytest.approx(.3, abs=.01)


def tracking_run(closed_loop, plant):
    """Control receives gyro packets only; plant states are scoring-only."""
    cfg = load_config()
    cfg["physics"].update(plant)
    world = World(cfg)
    feedback = SteeringFeedback(public_config(cfg))
    alpha = 0.
    errors = []
    # Balanced normal tracking requests, not a firmware calibration manoeuvre.
    targets = [.14, .22, .10, -.14, -.22, -.10]*3+[0.]
    try:
        for target in targets:
            for tick in range(12):
                u = feedback.control(target, world.time, alpha) if closed_loop else target/np.deg2rad(30)
                world.command([0., u])
                for step in range(40):
                    world.step()
                    wf = world.data.sensor("front_gyro").data[2]
                    wr = world.data.sensor("rear_gyro").data[2]
                    observed = wf-wr
                    alpha += observed*cfg["physics"]["dt"]
                    feedback.observe([imu(world.time, observed)], [.013, -.009])
                if tick >= 6 and target != 0:
                    errors.append((feedback.rate-target)**2)
        return np.sqrt(np.mean(errors)), feedback.diagnostics(), world.truth()
    finally:
        world.close()


@pytest.mark.parametrize("plant", [
    {},
    {"steer_left_rate_deg": 18., "steer_right_rate_deg": 48., "steer_exponent": 1.6, "steer_deadzone": .12},
    {"steer_left_rate_deg": 48., "steer_right_rate_deg": 22., "actuator_delay": .15},
])
def test_sensor_feedback_reduces_unknown_plant_rate_error(plant):
    baseline, _, _ = tracking_run(False, plant)
    error, diagnostics, truth = tracking_run(True, plant)
    assert error < baseline*.65, (error, baseline)
    assert error < np.deg2rad(4.)
    assert all(n > 0 for n in diagnostics["gain_updates"])
    assert abs(truth["state"][3]) < np.deg2rad(40)
