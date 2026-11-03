"""Argo's local odometry alarm: each range sensor's frame is matched against a keyframe of its
own; a shift or turn that lines them up better than none is what odometry got wrong. When it
keeps happening, odometry is suspect and speed is capped. The hub's site map is where drift
gets corrected; this only notices it, hub or not."""

from pathlib import Path

import pytest

from argus import profile as profiles
from argus.sim import world as worlds
from argus.sim.episode import SimOptions, Simulation

BODIES = sorted((Path(__file__).parent / "bodies").glob("*.yaml"))
CAP = 0.25  # SafetyConfig::odometry_suspect_speed_mps


def _scenario(doc):
    if doc["motion"]["space"] == "planar":
        return worlds.wall_with_gap(), (0.0, 0.0, 0.0, 0.0), (5.0, 2.0, 0.0)
    world = worlds.pillars(1, keep_clear=[(-4.0, 0.0), (4.0, 1.0)])
    return world, (-4.0, 0.0, 1.5, 0.0), (4.0, 1.0, 1.5)


def _run(doc, options, duration=40.0):
    world, start, goal = _scenario(doc)
    sim = Simulation(doc, world, start, options)
    node = sim.node
    suspect_at = None
    raised_at = None
    fast_while_suspect = 0.0

    def tick(t):
        nonlocal suspect_at, raised_at, fast_while_suspect
        if t == 0.0:
            node.set_plan(1, duration + 60.0, [{"id": "goal", "position": goal, "tolerance_m": 0.4}], 0.0)
        out = node.tick(t)
        if node.odometry_check()["suspect"]:
            if suspect_at is None:
                suspect_at = t
            if raised_at is None:
                raised_at = t
            elif t > raised_at + 1.0:  # braking down to the cap takes a moment
                fast_while_suspect = max(fast_while_suspect, sim.speed)
        else:
            raised_at = None  # cleared: checks that agree may lift the cap
        return out

    while sim.t < duration:
        sim.step(tick)
        if sim.result.complete and sim.speed < 0.01:
            break
    return sim, suspect_at, fast_while_suspect


@pytest.mark.parametrize("path", BODIES, ids=lambda p: p.stem)
def test_no_alarm_when_odometry_is_right(path):
    doc = profiles.load(path)
    sim, suspect_at, _ = _run(doc, SimOptions(range_noise_m=0.01))
    assert suspect_at is None, (suspect_at, sim.node.odometry_check())
    assert sim.node.odometry_check()["verdicts"] > 0


@pytest.mark.parametrize("options", [
    SimOptions(odom_scale=0.85, odom_drift_from_s=2.0, range_noise_m=0.01),
    SimOptions(odom_yaw_drift_radps=0.05, odom_drift_from_s=2.0, range_noise_m=0.01),
], ids=["scale_15_percent_short", "yaw_drift_3_deg_per_s"])
def test_drifting_odometry_is_noticed_and_speed_is_capped(options):
    doc = profiles.load(profiles.EXAMPLES_DIR / "planar_differential.yaml")
    sim, suspect_at, fast = _run(doc, options)
    assert not sim.result.collided
    assert suspect_at is not None and suspect_at < 8.0, (suspect_at, sim.node.odometry_check())
    assert fast <= CAP + 0.02, fast
