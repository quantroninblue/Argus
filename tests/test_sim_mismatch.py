"""A body that is worse than its profile says, late sensors and drifting odometry: Argo must
still never hit anything. Arriving is asked only where the mismatch is small."""

import pytest

from argus import profile as profiles
from argus.sim import world as worlds
from argus.sim.episode import SimOptions, run_episode

GOAL = [{"id": "goal", "position": (5.0, 2.0, 0.0), "tolerance_m": 0.4}]
AERIAL_GOAL = [{"id": "goal", "position": (4.0, 1.0, 1.5), "tolerance_m": 0.4}]


def _run(name, options, world=None):
    doc = profiles.load(profiles.EXAMPLES_DIR / f"{name}.yaml")
    if doc["motion"]["space"] == "aerial":
        w = world or worlds.pillars(1, keep_clear=[(-4.0, 0.0), (4.0, 1.0)])
        return run_episode(doc, w, (-4.0, 0.0, 1.5, 0.0), AERIAL_GOAL, 40.0, options)
    return run_episode(doc, world or worlds.wall_with_gap(), (0.0, 0.0, 0.0, 0.0), GOAL, 40.0, options)


@pytest.mark.parametrize("name", ["planar_differential", "planar_omnidirectional", "aerial_multirotor"])
def test_brakes_weaker_and_slower_than_declared_never_cause_a_collision(name):
    ep = _run(name, SimOptions(brake_scale=0.7, extra_latency_s=0.05))
    assert not ep.collided
    assert ep.complete


@pytest.mark.parametrize("name", ["planar_differential", "aerial_multirotor"])
def test_late_range_frames_never_cause_a_collision(name):
    ep = _run(name, SimOptions(sensor_latency_s=0.1))
    assert not ep.collided
    assert ep.complete


@pytest.mark.parametrize("name", ["planar_differential", "planar_omnidirectional"])
def test_drifting_odometry_never_causes_a_collision(name):
    ep = _run(name, SimOptions(odom_scale=1.03, odom_yaw_drift_radps=0.01), worlds.clutter(7))
    assert not ep.collided
