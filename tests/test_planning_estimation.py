import numpy as np
import pytest
from artic_nav.config import load_config, public_config
from artic_nav.geometry import predicted_box, wrap, inside
from artic_nav.estimator import Estimator
from artic_nav.planner import HybridAStar, PlanningError


def test_visual_initialization_is_not_seeded_with_pose():
    cfg = load_config()
    estimator = Estimator(public_config(cfg))
    state = [.3, -.25, 1.1, .25]
    packet = {"kind": "camera", "time": .1, "detections": [
        {"class": name, "box": predicted_box(state, cfg["geometry"], cfg["camera"], name).tolist(), "confidence": .9}
        for name in ("front", "rear")]}
    estimator.camera(packet)
    np.testing.assert_allclose(estimator.state(), state, atol=.01)


def test_planner_reverse_and_invalid_goal():
    cfg = public_config(load_config())
    planner = HybridAStar(cfg)
    path = planner.plan(np.array([.3, 0., 0., 0.]), [-.2, 0., 0.])
    assert np.any(path[:, 4] < 0)
    assert all(inside(row, cfg["geometry"], cfg["navigation"]["footprint_margin"]) for row in path)
    with pytest.raises(PlanningError):
        planner.plan(np.zeros(4), [.99, .99, 0.])


def test_angle_wrapping():
    assert abs(wrap(np.deg2rad(-179)-np.deg2rad(179))-np.deg2rad(2)) < 1e-10


def test_ambiguous_stationary_boxes_do_not_receive_truth_initialization():
    cfg = load_config()
    estimator = Estimator(public_config(cfg))
    state = [0., 0., .2, .4]
    detections = {name: predicted_box(state, cfg["geometry"], cfg["camera"], name)
                  for name in ("front", "rear")}
    assert not estimator._initialize(detections, .1)
    assert estimator.state() is None
    assert "多个" in estimator.initialization_reason
