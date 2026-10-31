"""Random link loss over generated bodies: does every body end in its own failsafe, stay safe,
and finish the mission once the link is back?"""

from __future__ import annotations

import random
from dataclasses import dataclass, field

from argus import profile as profiles
from argus.sim import world as worlds
from argus.sim.channel import LinkFaults
from argus.sim.hub_episode import HubEpisode, run_hub_episode

# Silence is measured from the last heartbeat that got through, up to about one heartbeat
# period plus latency before the stall began.
DETECT_EARLY_S = 0.35
DETECT_LATE_S = 0.1


def scenario(doc: dict):
    """World, start and goal: a detour through a gap for ground bodies, a pillar field at the
    start height for aerial ones (generated drones may not see up or down)."""
    if doc["motion"]["space"] == "planar":
        return worlds.wall_with_gap(), (0.0, 0.0, 0.0, 0.0), (5.0, 2.0, 0.0)
    world = worlds.pillars(1, keep_clear=[(-4.0, 0.0), (4.0, 1.0)])
    return world, (-4.0, 0.0, 1.5, 0.0), (4.0, 1.0, 1.5)


@dataclass
class SweepRun:
    profile_id: str
    action: str
    lost_after_s: float
    stall: tuple[float, float]
    episode: HubEpisode
    problems: list[str] = field(default_factory=list)


def check(doc: dict, stall: tuple[float, float], ep: HubEpisode, expect_arrival: bool) -> list[str]:
    fs = doc["failsafe"]
    lost_after, action = fs["link_lost_after_s"], fs["on_link_lost"]
    events = ep.failsafe_events()
    entered = [e for e in events if e["kind"] == "failsafe_entered"]
    cleared = [e for e in events if e["kind"] == "failsafe_cleared"]
    problems = []
    if ep.body.collided:
        problems.append("collided")
    problems += ep.body.violations[:3]
    length = stall[1] - stall[0]
    if length > lost_after + DETECT_LATE_S + 0.05:
        if not entered:
            problems.append(f"no failsafe after a {length:.1f} s stall (limit {lost_after} s)")
        else:
            e = entered[0]
            delay = e["stamp"] - stall[0]
            if e["detail"] != action:
                problems.append(f"failsafe {e['detail']}, profile says {action}")
            if not lost_after - DETECT_EARLY_S <= delay <= lost_after + DETECT_LATE_S:
                problems.append(f"failsafe after {delay:.2f} s, limit {lost_after} s")
            if not cleared or cleared[0]["stamp"] < stall[1]:
                problems.append("failsafe not cleared after the link came back")
    elif length < lost_after - DETECT_EARLY_S and entered:
        problems.append(f"failsafe after a {length:.1f} s stall, limit {lost_after} s")
    if expect_arrival and not ep.goal_reached:
        problems.append(f"did not arrive: {ep.mission_state} {ep.mission_cause}")
    return problems


def run_sweep(docs: list[dict], seed: int = 0, duration_s: float = 90.0, can_travel=None) -> list[SweepRun]:
    rng = random.Random(seed)
    runs = []
    for doc in docs:
        lost_after = doc["failsafe"]["link_lost_after_s"]
        start = rng.uniform(1.5, 8.0)
        stall = (start, start + rng.uniform(0.2, 2.0 * lost_after + 4.0))
        world, pose, goal = scenario(doc)
        # Run past the stall and the recovery after it, even if the body arrives first.
        ep = run_hub_episode(doc, world, pose, goal, duration_s, faults=LinkFaults(stalls=[stall]),
                             min_duration_s=stall[1] + 3.0)
        expect = can_travel(doc) if can_travel else False
        runs.append(SweepRun(doc["id"], doc["failsafe"]["on_link_lost"], lost_after, stall, ep,
                             check(doc, stall, ep, expect)))
    return runs
