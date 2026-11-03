"""Argo follows, avoids and reports (decisions, "Argo's job"): when it finds no clear way along
the hub's path it stops and tells the hub at once, and it tells the hub when it pauses for
something that stopped in its way. It does not look for another route itself."""

from pathlib import Path

from argus import profile as profiles
from argus.sim import world as worlds
from argus.sim.episode import SimOptions, run_episode
from argus.sim.hub_episode import run_hub_episode
from argus.sim.movers import Mover

ROOT = Path(__file__).resolve().parents[1]
GOAL = [{"id": "goal", "position": (5.0, 2.0, 0.0), "tolerance_m": 0.4}]


def test_with_no_clear_motion_argo_stops_and_reports_it_within_seconds():
    # The Jackal class just outside its padding at the wall end by the gap (as Gazebo left it
    # after an overshoot): no rollout toward the waypoint is clear.
    doc = profiles.load(ROOT / "tests" / "bodies" / "jackal_class.yaml")
    ep = run_episode(doc, worlds.wall_with_gap(), (2.35, 1.25, 0.0, 0.82), GOAL, 6.0)
    blocked = [e for e in ep.events if e["kind"] == "blocked"]
    assert blocked, ep.events
    first = blocked[0]
    assert first["cause"] == "temporary_obstruction" and first["detail"] == "no clear motion along the path"
    assert first["stamp"] < 3.0  # not the 10 s no-progress wait
    assert len([e for e in blocked if e["detail"] == first["detail"]]) == 1  # once per waypoint
    assert not ep.collided


def test_the_hub_hears_of_a_pause_for_someone_who_stopped_in_the_way():
    doc = profiles.load(ROOT / "schemas" / "examples" / "planar_differential.yaml")
    w = worlds.open_room()
    # Walks across in front of the robot and stops in its way for good.
    w.add_mover(Mover(path=[(2.5, -3.0), (2.5, 0.0)], speed_mps=1.0, start_s=1.0))
    ep = run_hub_episode(doc, w, (0.0, 0.0, 0.0, 0.0), (5.0, 0.0, 0.0), 40.0)
    paused = [e for e in ep.hub_events if e[1] == "safety_stop"]
    assert paused, ep.hub_events
    assert paused[0][3] == "temporary_obstruction" and "stopped on the way at 2." in paused[0][4]
    assert not ep.body.collided


def test_argo_does_not_take_a_way_far_from_the_hubs_leg_it_reports_it():
    # The only way to a waypoint straight across the wall is the gap 6 m to the side, well out of
    # the corridor round the leg; finding that way is the hub's job, so Argo stops and says so.
    doc = profiles.load(ROOT / "schemas" / "examples" / "planar_differential.yaml")
    ep = run_episode(doc, worlds.wall_with_gap(), (0.0, -5.0, 0.0, 0.0),
                     [{"id": "across", "position": (5.0, -5.0, 0.0), "tolerance_m": 0.4}], 30.0,
                     options=SimOptions(trace=True))
    report = [e for e in ep.events if e["kind"] == "blocked" and e["detail"] == "no way around near the hub's path"]
    assert report and report[0]["cause"] == "temporary_obstruction"
    assert max(r["pose"][1] for r in ep.trace) < -2.0  # never went up to the gap at y 1-3
    assert not ep.complete and not ep.collided
