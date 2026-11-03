import sys
import numpy as np
import pytest
from artic_nav.config import load_config, public_config
from artic_nav.physics import World
from artic_nav.sensors import Sensors
from artic_nav.worker import NavigationProcess
from artic_nav.worker import validate_request


def test_truth_fields_are_rejected_at_process_boundary():
    with pytest.raises(ValueError):
        validate_request({"sensors": [], "true_pose": [0, 0, 0]})
    with pytest.raises(ValueError):
        validate_request({"sensors": [{"kind": "camera", "time": 0., "detections": [], "true_yaw": 0.}]})


def test_public_config_excludes_plant_and_initial_state():
    public = public_config(load_config())
    assert set(public) == {"geometry", "camera", "navigation"}
    assert "initial_pose" not in repr(public)
    assert "drive_torque" not in repr(public)


def test_detector_timestamp_noise_delay_and_blackout():
    cfg = load_config()
    world = World(cfg)
    sensors = Sensors(world, cfg)
    frames = []
    for _ in range(160):
        world.step()
        for packet in sensors.sample():
            if packet["kind"] == "camera":
                assert world.time-packet["time"] >= .1-1e-8
                assert set(packet) == {"time", "kind", "detections"}
                for d in packet["detections"]:
                    assert set(d) == {"class", "box", "confidence"}
                frames.append(packet)
    assert len(frames) > 5
    sensors.blackout = (0, 10)
    assert sensors.camera_frame()["detections"] == []
    world.close()


def test_navigation_is_independent_of_evaluation_truth():
    cfg = load_config(profile="ideal")
    world = World(cfg)
    sensors = Sensors(world, cfg)
    messages, packets = [], []
    for i in range(640):
        world.step()
        packets.extend(sensors.sample())
        if i % 40 == 39:
            messages.append(packets)
            packets = []
    first, second = NavigationProcess(cfg), NavigationProcess(cfg)
    try:
        for i, packets in enumerate(messages):
            action = "goal" if i == 0 else None
            goal = [-.25, -.45, 0.] if i == 0 else None
            truth = world.truth()
            result_a = first.request(packets, action, goal)
            # Independently destroy every privileged score-side pose.
            truth["state"] = [999, -999, 2., -.6]
            result_b = second.request(packets, action, goal)
            assert result_a == result_b
        assert result_a["estimate"] is not None
        assert result_a["mode"] == "running"
    finally:
        first.close()
        second.close()
        world.close()
