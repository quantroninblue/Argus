"""Scenario worlds that need the hub's judgement: dead ends, detours that lead away first, a
gap blocked mid-run, a sensor failing mid-run, and random link loss over generated bodies."""

import copy

import pytest

from argus import profile as profiles
from argus.hub import MissionState
from argus.hub.exploration import PlaceStatus
from argus.sim import world as worlds
from argus.sim.episode import SimOptions, run_episode
from argus.sim.hub_episode import HubLoop, run_hub_episode
from argus.sim.profile_gen import generate_profiles
from argus.sim.sweep import run_sweep, scenario
from test_episodes import _can_travel


def _example(name):
    return profiles.load(profiles.EXAMPLES_DIR / f"{name}.yaml")


def _clean(ep):
    assert not ep.body.collided
    assert ep.body.violations == []


def _farthest_x(ep, start_x):
    return max(abs(r["pose"][0] - start_x) for r in ep.body.trace)


def test_ground_robot_backs_out_of_a_dead_end():
    ep = run_hub_episode(_example("planar_differential"), worlds.dead_end_branch(), (-5.0, 0.0, 0.0, 0.0),
                         (5.0, 5.5, 0.0), 120.0, options=SimOptions(trace=True))
    _clean(ep)
    assert ep.goal_reached
    # It went into the north corridor (the short way) before taking the south one.
    north = [r["pose"] for r in ep.body.trace if r["pose"][1] > 4.5]
    south = [r["pose"] for r in ep.body.trace if r["pose"][1] < -4.5]
    assert north and south
    assert max(p[0] for p in north) > -2.0
    first_south = next(r["t"] for r in ep.body.trace if r["pose"][1] < -4.5)
    first_north = next(r["t"] for r in ep.body.trace if r["pose"][1] > 4.5)
    assert first_north < first_south


def test_drone_backs_out_of_a_dead_end_under_a_ceiling():
    world = worlds.with_ceiling(worlds.dead_end_branch(height=3.0), 3.0)
    ep = run_hub_episode(_example("aerial_multirotor"), world, (-5.0, 0.0, 1.5, 0.0), (5.0, 5.5, 1.5), 120.0)
    _clean(ep)
    assert ep.goal_reached


def test_goal_inside_a_cup_that_opens_away():
    ep = run_hub_episode(_example("planar_differential"), worlds.deep_cup(), (-4.0, 0.0, 0.0, 0.0),
                         (0.8, 0.0, 0.0), 90.0, options=SimOptions(trace=True))
    _clean(ep)
    assert ep.goal_reached
    assert _farthest_x(ep, -4.0) > 8.5, "had to go round the far end of the cup"
    assert ep.reached_at_s < 45.0


def test_gap_blocked_mid_run_is_routed_around():
    world = worlds.two_gaps()
    loop = HubLoop(_example("planar_differential"), world, (0.0, 0.0, 0.0, 0.0), (5.0, 1.0, 0.0))
    blocked = False
    while loop.sim.t < 60.0:
        loop.step()
        if not blocked and loop.sim.plant.pos[0] > 1.0:
            world.add((2.9, 0.5, 0.0), (3.4, 2.0, 2.0))
            blocked = True
        if loop.mission_state != MissionState.ACTIVE and loop.sim.at_rest:
            break
    ep = loop.finish()
    _clean(ep)
    assert blocked and ep.goal_reached
    # The south gap is the only way left.
    assert min(p[1] for p, _ in ep.places) < -3.0
    assert not any(s == PlaceStatus.EXHAUSTED_FOR_GOAL.value for _, s in ep.places if _[1] < -3.0)


def test_failed_sensor_mid_run_keeps_the_body_safe_and_it_still_arrives():
    ep = run_hub_episode(_example("planar_omnidirectional"), worlds.wall_with_gap(), (0.0, 0.0, 0.0, 0.0),
                         (5.0, 2.0, 0.0), 90.0, options=SimOptions(sensor_fails_at_s={"front_depth": 2.0}))
    _clean(ep)
    assert ep.goal_reached


def test_all_sensors_failing_stops_the_body_without_touching_memory():
    ep = run_hub_episode(_example("planar_differential"), worlds.wall_with_gap(), (0.0, 0.0, 0.0, 0.0),
                         (5.0, 2.0, 0.0), 20.0, options=SimOptions(sensors_stop_at_s=3.0))
    _clean(ep)
    assert not ep.goal_reached
    assert ep.body.rest_after_fault_s is not None
    assert "sensor_invalid" in [e[3] for e in ep.hub_events if e[1] == "step_failed"]
    assert not any(s in (PlaceStatus.EXHAUSTED_FOR_GOAL.value, PlaceStatus.TEMPORARILY_BLOCKED.value)
                   for _, s in ep.places)


SWEEP = [d for d in generate_profiles(24, seed=0) if profiles.derive(d).safety_ready][:8]


@pytest.mark.parametrize("doc", SWEEP, ids=lambda d: d["id"])
def test_random_link_loss_ends_in_each_bodys_failsafe(doc):
    (run,) = run_sweep([doc], seed=int(doc["id"][-4:]), can_travel=_can_travel)
    assert run.problems == [], (run.action, run.lost_after_s, run.stall)


def _generated(seed, gid):
    return next(d for d in generate_profiles(24, seed=seed) if d["id"] == gid)


def test_body_seeing_only_one_side_reaches_the_goal_through_the_hub():
    # Covered from straight ahead round to the right only. It used to drive nose first along
    # the edge of what it sees and stop where the path turned left.
    doc = _generated(1, "generated-0007")
    world, start, goal = scenario(doc)
    ep = run_hub_episode(doc, world, start, goal, 60.0)
    _clean(ep)
    assert ep.goal_reached


def test_drone_seeing_only_level_keeps_a_covered_way_to_a_higher_goal():
    # Two narrow side arcs at level only; below the goal height every sampled direction was
    # uncovered until level flight was offered.
    doc = _generated(1, "generated-0000")
    world, start, goal = scenario(doc)
    ep = run_episode(doc, world, start, [{"id": "goal", "position": goal, "tolerance_m": 0.4}], 60.0)
    assert not ep.collided and ep.violations == []
    assert ep.complete


def test_body_that_sees_only_behind_reverses_at_its_full_speed():
    doc = _generated(1, "generated-0014")
    world, start, goal = scenario(doc)
    ep = run_episode(doc, world, start, [{"id": "goal", "position": goal, "tolerance_m": 0.4}], 90.0)
    assert not ep.collided and ep.violations == []
    assert ep.complete
    assert ep.max_speed_mps > 0.9 * -doc["motion"]["limits"]["x"]["velocity_mps"][0]


@pytest.mark.parametrize("gid", ["generated-0008", "generated-0015"])
def test_creep_is_short_and_a_body_with_no_range_sensor_never_drives(gid):
    # Both may creep where they cannot see. 0008 has no range sensor at all; 0015's sensors
    # miss the level it flies at. Both used to creep across the room into a box.
    doc = next(d for d in generate_profiles(60, seed=0) if d["id"] == gid)
    _, start, goal = scenario(doc)
    world = worlds.clutter(7, keep_clear=[start[:2], goal[:2]])
    ep = run_episode(doc, world, start, [{"id": "goal", "position": goal, "tolerance_m": 0.4}], 60.0)
    assert not ep.collided and ep.violations == []
    if gid == "generated-0008":
        assert ep.distance_m < 0.05


@pytest.mark.parametrize("scale", [1.0, 1.15, 1.25])
def test_car_like_body_reaches_the_goal_when_it_turns_wider_than_declared(scale):
    # Real steering turns wider than the profile says (Gazebo's front-steered model by about
    # 15%). A car that may not reverse used to cut the corner into the wall by the gap, or
    # stop beside its path inside its own turning circle.
    doc = copy.deepcopy(_example("planar_differential"))
    doc["id"] = "test-car-like"
    doc["motion"]["min_turn_radius_m"] = 0.8
    doc["motion"]["limits"]["yaw"]["rate_radps"] = 0.6
    ep = run_hub_episode(doc, worlds.wall_with_gap(), (0.0, 0.0, 0.0, 0.0), (5.0, 2.0, 0.0), 60,
                         options=SimOptions(turn_radius_scale=scale, range_noise_m=0.01))
    _clean(ep)
    assert ep.mission_state == MissionState.REACHED
