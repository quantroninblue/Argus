"""Robots modelled on ones people build (tests/bodies/), which must reach their goals: through
a gap for ground robots, a pillar field for the quadcopter, directly and through the hub.
Generated bodies (test_episodes.py) only have to stay safe; these have to arrive as well."""

from pathlib import Path

import pytest

from argus import profile as profiles
from argus.sim import world as worlds
from argus.sim.episode import run_episode
from argus.sim.hub_episode import run_hub_episode

BODIES = sorted((Path(__file__).parent / "bodies").glob("*.yaml"))


def _scenario(doc):
    if doc["motion"]["space"] == "planar":
        return worlds.wall_with_gap(), (0.0, 0.0, 0.0, 0.0), (5.0, 2.0, 0.0)
    world = worlds.pillars(1, keep_clear=[(-4.0, 0.0), (4.0, 1.0)])
    return world, (-4.0, 0.0, 1.5, 0.0), (4.0, 1.0, 1.5)


def _duration(doc) -> float:
    # The slowest of these moves at about 0.26 m/s; the detour is about 7 m.
    top = doc["motion"]["limits"]["x"]["velocity_mps"][1]
    return max(40.0, 12.0 / top)


@pytest.mark.parametrize("path", BODIES, ids=lambda p: p.stem)
def test_reference_robot_reaches_the_goal_safely(path):
    doc = profiles.load(path)
    assert profiles.derive(doc).safety_ready
    world, start, goal = _scenario(doc)
    ep = run_episode(doc, world, start, [{"id": "goal", "position": goal, "tolerance_m": 0.4}], _duration(doc))
    assert not ep.collided and ep.violations == []
    assert ep.complete, (ep.final_pose, ep.flag_counts)


@pytest.mark.parametrize("path", BODIES, ids=lambda p: p.stem)
def test_reference_robot_reaches_the_goal_through_the_hub(path):
    doc = profiles.load(path)
    world, start, goal = _scenario(doc)
    ep = run_hub_episode(doc, world, start, goal, _duration(doc) + 30.0)
    assert not ep.body.collided and ep.body.violations == []
    assert ep.goal_reached, (ep.mission_state, ep.body.final_pose)


def test_quadcopter_climbs_to_a_higher_goal():
    doc = profiles.load(Path(__file__).parent / "bodies" / "x500_class.yaml")
    world, start, _ = _scenario(doc)
    plan = [{"id": "goal", "position": (4.0, 1.0, 2.2), "tolerance_m": 0.4}]
    ep = run_episode(doc, world, start, plan, 40.0)
    assert not ep.collided and ep.violations == []
    assert ep.complete, (ep.final_pose, ep.flag_counts)
