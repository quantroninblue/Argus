"""Argo driving simulated bodies. Collisions, limits and coverage are judged from truth."""

import math

import numpy as np
import pytest

from argus import profile as profiles
from argus.sim import world as worlds
from argus.sim.episode import SimOptions, run_episode
from argus.sim.profile_gen import generate_profiles

GOAL_BEHIND_WALL = [{"id": "goal", "position": (5.0, 2.0, 0.0), "tolerance_m": 0.4}]
# Generated drones may have no sensor looking up or down, so their goal is at the start
# height; the example drone can see both ways and has to climb.
LEVEL_AERIAL_GOAL = [{"id": "goal", "position": (4.0, 1.0, 1.5), "tolerance_m": 0.4}]
CLIMBING_AERIAL_GOAL = [{"id": "goal", "position": (4.0, 1.0, 2.2), "tolerance_m": 0.4}]


def _example(name):
    return profiles.load(profiles.EXAMPLES_DIR / f"{name}.yaml")


def _scenario(doc, climb=False):
    """A world, start and plan for the body: a detour through a gap for ground bodies, a
    pillar field for aerial ones."""
    if doc["motion"]["space"] == "planar":
        return worlds.wall_with_gap(), (0.0, 0.0, 0.0, 0.0), GOAL_BEHIND_WALL
    world = worlds.pillars(1, keep_clear=[(-4.0, 0.0), (4.0, 1.0)])
    return world, (-4.0, 0.0, 1.5, 0.0), CLIMBING_AERIAL_GOAL if climb else LEVEL_AERIAL_GOAL


def _can_travel(doc) -> bool:
    """Whether some direction the body can actually move in is covered at every height."""
    caps = profiles.derive(doc)
    if not caps.safety_ready:
        return False
    m = caps.motion
    level = [b for b in caps.sensing.bins if b.elevation_deg == 0.0]
    covered = [b.azimuth_deg for b in level if b.full_height_reach_m > 0.5]
    if m.space == "planar" and not m.can_strafe:
        reverse = m.can_reverse and m.x.min_mps < 0.0
        return 0.0 in covered or (reverse and 180.0 in covered)
    return bool(covered)


def test_raycast_hits_boxes_and_floor():
    world = worlds.World().add((2.0, -1.0, 0.0), (3.0, 1.0, 1.0))
    dirs = np.array([[1.0, 0.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
    r = world.raycast(np.array([0.0, 0.0, 0.5]), dirs, 10.0)
    assert r[0] == pytest.approx(2.0)
    assert math.isinf(r[1])
    assert r[2] == pytest.approx(0.5)
    assert math.isinf(r[3])
    assert math.isinf(world.raycast(np.array([0.0, 0.0, 0.5]), dirs[:1], 1.5)[0])


@pytest.mark.parametrize("name", ["planar_differential", "planar_omnidirectional", "aerial_multirotor"])
def test_examples_reach_a_goal_that_needs_a_detour(name):
    doc = _example(name)
    world, start, plan = _scenario(doc, climb=True)
    ep = run_episode(doc, world, start, plan, 40.0)
    assert ep.complete, ep.flag_counts
    assert not ep.collided
    assert ep.violations == []
    assert ep.time_s < 20.0


def test_noisy_ranges_still_get_there():
    doc = _example("planar_differential")
    world, start, plan = _scenario(doc)
    ep = run_episode(doc, world, start, plan, 40.0, SimOptions(range_noise_m=0.02, seed=3))
    assert ep.complete and not ep.collided and ep.violations == []


GENERATED = generate_profiles(24, seed=0)


@pytest.mark.parametrize("doc", GENERATED, ids=lambda d: d["id"])
def test_generated_bodies_never_collide_or_break_rules(doc):
    world, start, plan = _scenario(doc)
    ep = run_episode(doc, world, start, plan, 30.0)
    assert not ep.collided
    assert ep.violations == []
    if not profiles.derive(doc).safety_ready:
        assert ep.distance_m == 0.0
        assert any(e["kind"] == "plan_rejected" for e in ep.events)
    if _can_travel(doc):
        assert ep.complete, ep.flag_counts


def test_generated_set_covers_capable_planar_and_aerial_bodies():
    capable = [d for d in GENERATED if _can_travel(d)]
    assert {d["motion"]["space"] for d in capable} == {"planar", "aerial"}
    assert len(capable) >= 8


def _open_run(options):
    doc = _example("planar_differential")
    world = worlds.open_room(size=20.0)
    plan = [{"id": "far", "position": (8.0, 0.0, 0.0), "tolerance_m": 0.3}]
    return doc, run_episode(doc, world, (-8.0, 0.0, 0.0, 0.0), plan, 12.0, options)


def test_stale_sensors_bring_the_body_to_rest():
    doc, ep = _open_run(SimOptions(sensors_stop_at_s=4.0))
    caps = profiles.derive(doc)
    max_age = max(3.0 / s["rate_hz"] for s in doc["sensors"] if "range_m" in s)
    braking = caps.motion.x.max_mps / caps.motion.x.decel_mps2
    assert not ep.complete
    assert ep.rest_after_fault_s is not None
    assert ep.rest_after_fault_s - 4.0 <= max_age + caps.motion.latency_s + braking + 0.1
    causes = [e["cause"] for e in ep.events if e["kind"] == "blocked"]
    assert "sensor_invalid" in causes
    assert ep.violations == [] and not ep.collided


def test_failed_odometry_brings_the_body_to_rest():
    doc, ep = _open_run(SimOptions(odometry_fails_at_s=4.0))
    caps = profiles.derive(doc)
    braking = caps.motion.x.max_mps / caps.motion.x.decel_mps2
    assert ep.rest_after_fault_s is not None
    assert ep.rest_after_fault_s - 4.0 <= caps.motion.latency_s + braking + 0.1
    assert "localization_invalid" in [e["cause"] for e in ep.events if e["kind"] == "blocked"]
