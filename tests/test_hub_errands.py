"""Errands through the whole loop in sim time: body, Argo, a faulty link and the hub, with arrival
judged from simulator truth. Then places and routes from the command line."""

import math

import pytest

from argus import profile as profiles
from argus.__main__ import main
from argus.hub.client import AppClient
from argus.hub.places import Place, Route, Stop
from argus.sim import world as worlds
from argus.sim.channel import LinkFaults
from argus.sim.episode import SimOptions
from argus.sim.hub_episode import NODE_ID, SITE_ID, HubLoop
from test_hub_app import _Loop, running_hub  # noqa: F401

PLACES = {"near": (1.0, 0.0, 0.0), "through": (5.0, 2.0, 0.0), "beyond": (5.0, -3.0, 0.0)}
# Stop tolerance, one site map cell (how far drift correction can sit off the truth, see
# the README's known issues) and the margin hub episodes allow for stopping.
ARRIVED_M = 0.4 + 0.1 + 0.05


def _loop(tmp_path, route, places=PLACES, faults=None, profile="planar_differential"):
    doc = profiles.load(profiles.EXAMPLES_DIR / f"{profile}.yaml")
    loop = HubLoop(doc, worlds.wall_with_gap(), (0.0, 0.0, 0.0, 0.0), next(iter(places.values())), faults=faults,
                   options=SimOptions(trace=True), sites_dir=tmp_path / "sites")
    for name, position in places.items():
        loop.hub.dispatch.add_place(SITE_ID, Place(name, position))
    loop.hub.dispatch.add_route(SITE_ID, route)
    return loop


def _distance(loop, position):
    p = loop.sim.plant.pos
    return math.hypot(p[0] - position[0], p[1] - position[1])


def _run(loop, until, limit_s=300.0, places=PLACES):
    """Steps until `until(event kinds so far)`; returns (t, kind, detail, distance to the leg's
    place at that moment) for each event."""
    log = []
    stops = {}
    while loop.sim.t < limit_s:
        loop.step()
        for e in loop.hub.dispatch.take_events():
            for name, position in places.items():
                if name in e.detail:
                    stops[len(log)] = _distance(loop, position)
            log.append((loop.sim.t, e.kind, e.detail, stops.get(len(log))))
        if until([k for _, k, _, _ in log]):
            return log
    pytest.fail(f"still going after {limit_s} s: {log}")


def _ended(kinds):
    return bool({"errand_done", "errand_failed", "errand_cancelled"} & set(kinds))


def _clean(loop):
    assert not loop.sim.result.collided
    assert loop.sim.result.violations == []


def test_a_loop_of_three_stops_goes_round_twice_through_the_gap(tmp_path):
    route = Route("rounds", (Stop("near"), Stop("through", 1.0), Stop("beyond")), mode="loop", cycles=2)
    loop = _loop(tmp_path, route)
    assert "rounds" in loop.hub.patrol(NODE_ID, "rounds", 0.0)
    log = _run(loop, _ended, limit_s=400.0)
    _clean(loop)
    kinds = [k for _, k, _, _ in log]
    assert kinds[-1] == "errand_done"
    reached = [(detail, away) for _, k, detail, away in log if k == "leg_reached"]
    assert len(reached) == 6
    for name, (detail, away) in zip(["near", "through", "beyond"] * 2, reached):
        assert name in detail and away <= ARRIVED_M, reached
    assert "dwelling" in kinds


def test_an_unreachable_stop_is_skipped_and_the_route_carries_on(tmp_path):
    places = PLACES | {"outside": (12.0, 0.0, 0.0)}
    route = Route("rounds", (Stop("near"), Stop("outside"), Stop("through")))
    loop = _loop(tmp_path, route, places)
    loop.hub.patrol(NODE_ID, "rounds", 0.0)
    log = _run(loop, _ended, places=places)
    _clean(loop)
    kinds = [k for _, k, _, _ in log]
    assert "leg_skipped" in kinds and kinds[-1] == "errand_done"
    assert _distance(loop, PLACES["through"]) <= ARRIVED_M


def test_an_unreachable_stop_ends_a_route_that_says_stop(tmp_path):
    places = PLACES | {"outside": (12.0, 0.0, 0.0)}
    route = Route("rounds", (Stop("near"), Stop("outside"), Stop("through")), on_unreachable="stop")
    loop = _loop(tmp_path, route, places)
    loop.hub.patrol(NODE_ID, "rounds", 0.0)
    log = _run(loop, _ended, places=places)
    _clean(loop)
    assert log[-1][1] == "errand_failed"
    assert _distance(loop, PLACES["through"]) > 1.0


def test_cancelling_stops_the_robot_and_the_route(tmp_path):
    route = Route("rounds", (Stop("near"), Stop("through"), Stop("beyond")))
    loop = _loop(tmp_path, route)
    loop.hub.patrol(NODE_ID, "rounds", 0.0)
    _run(loop, lambda kinds: kinds.count("leg_started") >= 2)
    for _ in range(40):
        loop.step()
    loop.hub.cancel(NODE_ID)
    log = _run(loop, _ended, limit_s=loop.sim.t + 5.0)
    assert log[-1][1] == "errand_cancelled"
    settle = loop.sim.t + 8.0
    while loop.sim.t < settle and not loop.sim.at_rest:
        loop.step()
    assert loop.sim.at_rest and loop.hub.dispatch.errand(NODE_ID) is None
    _clean(loop)
    assert _distance(loop, PLACES["through"]) > 0.5
    assert not loop.hub.dispatch.take_events()


def test_a_route_carries_on_after_the_link_was_lost_on_the_way(tmp_path):
    route = Route("rounds", (Stop("near"), Stop("through")))
    loop = _loop(tmp_path, route, faults=LinkFaults(cuts=[(6.0, 13.0)]), profile="planar_omnidirectional")
    loop.hub.patrol(NODE_ID, "rounds", 0.0)
    log = _run(loop, _ended)
    _clean(loop)
    assert log[-1][1] == "errand_done"
    events = loop.finish().failsafe_events()
    assert [e for e in events if e["kind"] == "failsafe_entered"]
    assert [e for e in events if e["kind"] == "failsafe_cleared" and e["stamp"] > 13.0]
    assert _distance(loop, PLACES["through"]) <= ARRIVED_M


def test_places_and_routes_through_the_command(running_hub, capsys):  # noqa: F811
    home = ["--home", str(running_hub.root)]
    assert main(["place", "add", "lab", "shelf", "6.5", "2", "--tolerance", "0.3"] + home) == 0
    assert main(["place", "add", "lab", "dock", "0", "0"] + home) == 0
    assert main(["place", "add", "lab", "shelf", "1", "1"] + home) == 1
    assert main(["route", "add", "lab", "rounds", "shelf:30", "dock", "--mode", "loop", "--cycles", "4"] + home) == 0
    assert main(["route", "add", "lab", "broken", "nowhere"] + home) == 1
    capsys.readouterr()
    assert main(["place", "list"] + home) == 0
    assert main(["route", "list", "lab"] + home) == 0
    out = capsys.readouterr().out
    assert "shelf" in out and "dock" in out and "rounds" in out
    assert main(["go", "rover-1", "nowhere"] + home) == 1
    assert "nowhere" in capsys.readouterr().err
    assert main(["go", "rover-1", "shelf"] + home) == 0
    with AppClient(running_hub) as hub:
        assert hub.robots().robots[0].mission == "active"
    assert main(["patrol", "rover-1", "rounds"] + home) == 0
    assert "rounds" in capsys.readouterr().out
    assert main(["cancel", "rover-1"] + home) == 0
    with AppClient(running_hub) as hub:
        assert hub.robots().robots[0].mission == "cancelled"
    assert main(["place", "remove", "lab", "shelf"] + home) == 1
    assert main(["route", "remove", "lab", "rounds"] + home) == 0
    assert main(["place", "remove", "lab", "shelf"] + home) == 0
