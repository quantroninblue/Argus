"""Whole loop in sim time: body, Argo and its session, a faulty link, and the hub planning the
route. Arrival, collisions and rule breaks are judged from simulator truth."""

import copy

import numpy as np
import pytest

from argus import profile as profiles
from argus.sim import world as worlds
from argus.sim.channel import LinkFaults
from argus.sim.episode import SimOptions
from argus.sim.hub_episode import run_hub_episode


def _example(name):
    return profiles.load(profiles.EXAMPLES_DIR / f"{name}.yaml")


def _scenario(doc):
    if doc["motion"]["space"] == "planar":
        return worlds.wall_with_gap(), (0.0, 0.0, 0.0, 0.0), (5.0, 2.0, 0.0)
    world = worlds.pillars(1, keep_clear=[(-4.0, 0.0), (4.0, 1.0)])
    return world, (-4.0, 0.0, 1.5, 0.0), (4.0, 1.0, 2.2)


def _run(doc, faults=None, duration=60.0):
    world, start, goal = _scenario(doc)
    return run_hub_episode(doc, world, start, goal, duration, faults=faults, options=SimOptions(trace=True))


def _clean(ep):
    assert not ep.body.collided
    assert ep.body.violations == []


def _pose_at(ep, t):
    return next(r["pose"] for r in ep.body.trace if r["t"] >= t - 1e-9)


def _event(ep, kind):
    return next((e for e in ep.failsafe_events() if e["kind"] == kind), None)


@pytest.mark.parametrize("name", ["planar_differential", "planar_omnidirectional", "aerial_multirotor"])
def test_examples_reach_a_goal_through_the_hub(name):
    ep = _run(_example(name))
    _clean(ep)
    assert ep.goal_reached and ep.mission_state == "reached"
    assert ep.reached_at_s < 20.0
    # The link comes up once and stays up; plans are not resent for nothing.
    assert [s for _, s in ep.link_states] == ["lost", "connected"]
    assert ep.plans_sent <= 3
    assert not ep.failsafe_events()


STALL = (3.0, 16.0)


# With the hub unreachable Argo only stops and parks (decisions, 2026-11-03).
@pytest.mark.parametrize("name,action,lost_after", [
    ("planar_differential", "stop", 1.0),
    ("planar_omnidirectional", "stop", None),
    ("aerial_multirotor", "hold", None),
    ("aerial_multirotor", "land", None),
])
def test_link_loss_ends_in_the_profiles_failsafe_and_the_mission_resumes(name, action, lost_after):
    doc = copy.deepcopy(_example(name))
    doc["failsafe"]["on_link_lost"] = action
    if lost_after:
        doc["failsafe"]["link_lost_after_s"] = lost_after
    lost_after = doc["failsafe"]["link_lost_after_s"]
    ep = _run(doc, LinkFaults(stalls=[STALL]))
    _clean(ep)

    entered = _event(ep, "failsafe_entered")
    assert entered is not None and entered["detail"] == action
    # Silence counts from the last heartbeat that got through, at most a period or so before
    # the stall began.
    assert lost_after - 0.35 <= entered["stamp"] - STALL[0] <= lost_after + 0.1
    done = _event(ep, "failsafe_done")
    assert done is not None and done["stamp"] < STALL[1]
    rest_from = done["stamp"] + 1.5
    a, b = np.array(_pose_at(ep, rest_from)[:3]), np.array(_pose_at(ep, STALL[1])[:3])
    assert np.linalg.norm(b - a) < 0.05, "the body stays where the failsafe left it"

    pose = _pose_at(ep, done["stamp"])
    if action == "land":
        assert pose[2] < 1.0
    if action in ("stop", "hold"):
        assert done["stamp"] == entered["stamp"]

    cleared = _event(ep, "failsafe_cleared")
    assert cleared is not None and cleared["stamp"] > STALL[1]
    assert ep.goal_reached and ep.mission_state == "reached"


def test_short_disconnect_reconnects_without_a_failsafe():
    doc = _example("planar_differential")
    ep = _run(doc, LinkFaults(cuts=[(3.0, 4.0)]))
    _clean(ep)
    assert ep.connections == 2
    assert not ep.failsafe_events()
    assert ep.goal_reached


def test_long_disconnect_runs_the_failsafe_then_resumes():
    doc = copy.deepcopy(_example("planar_omnidirectional"))
    ep = _run(doc, LinkFaults(cuts=[(3.0, 9.0)]))
    _clean(ep)
    assert ep.connections == 2
    assert _event(ep, "failsafe_entered")["detail"] == "stop"
    assert _event(ep, "failsafe_cleared")["stamp"] > 9.0
    assert ep.goal_reached


def test_lossy_slow_link_still_gets_there():
    doc = _example("planar_differential")
    ep = _run(doc, LinkFaults(latency_s=0.15, jitter_s=0.1, loss=0.1, seed=4))
    _clean(ep)
    assert ep.goal_reached
    assert not ep.failsafe_events()
