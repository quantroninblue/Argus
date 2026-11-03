"""Places, routes and errands wired to robots' missions (argus.hub.dispatch)."""

import pytest

from argus.hub.dispatch import Dispatch
from argus.hub.mission import MissionState
from argus.hub.places import Place, PlaceError, Route, Stop


class _Mission:
    """Stands in for a robot's MissionManager: the goals it was given and its state."""

    def __init__(self):
        self.state = MissionState.IDLE
        self.cause = ""
        self.goals = []

    def set_goal(self, position, tolerance_m, now):
        self.goals.append(tuple(position))
        self.state = MissionState.ACTIVE


@pytest.fixture
def dispatch(tmp_path):
    d = Dispatch(tmp_path / "sites")
    for name, x in (("dock", 0.0), ("shelf", 4.0), ("door", 8.0)):
        d.add_place("plant", Place(name, (x, 1.0, 0.0)))
    d.add_route("plant", Route("rounds", (Stop("shelf", 2.0), Stop("door")), mode="loop", cycles=2))
    return d


def test_go_sends_the_robot_to_a_named_place(dispatch):
    m = _Mission()
    assert "shelf" in dispatch.go("robot-1", "plant", "shelf", m, 0.0)
    assert m.goals == [(4.0, 1.0, 0.0)]
    m.state = MissionState.REACHED
    dispatch.step("robot-1", m, 1.0)
    dispatch.step("robot-1", m, 1.1)
    kinds = [e.kind for e in dispatch.take_events()]
    assert kinds[0] == "leg_started" and "leg_reached" in kinds and kinds[-1] == "errand_done"
    assert dispatch.errand("robot-1") is None


def test_a_patrol_dwells_then_goes_round_its_route(dispatch):
    m = _Mission()
    dispatch.patrol("robot-1", "plant", "rounds", m, 0.0)
    m.state = MissionState.REACHED
    dispatch.step("robot-1", m, 10.0)  # at the shelf: dwell 2 s
    dispatch.step("robot-1", m, 11.0)
    assert m.goals == [(4.0, 1.0, 0.0)]
    dispatch.step("robot-1", m, 12.5)  # dwell over: on to the door
    assert m.goals[-1] == (8.0, 1.0, 0.0)
    assert "stop 2 of 2" in dispatch.errand("robot-1").describe()


def test_a_plain_goal_or_cancel_ends_the_errand(dispatch):
    m = _Mission()
    dispatch.patrol("robot-1", "plant", "rounds", m, 0.0)
    dispatch.end("robot-1", "a goal was set by hand")
    assert dispatch.errand("robot-1") is None
    assert dispatch.take_events()[-1].kind == "errand_cancelled"


def test_unknown_places_and_routes_and_duplicates_are_refused(dispatch):
    m = _Mission()
    with pytest.raises(PlaceError, match="no place"):
        dispatch.go("robot-1", "plant", "kitchen", m, 0.0)
    with pytest.raises(PlaceError, match="no route"):
        dispatch.patrol("robot-1", "plant", "night-shift", m, 0.0)
    with pytest.raises(PlaceError, match="already has a place"):
        dispatch.add_place("plant", Place("dock", (1.0, 1.0, 0.0)))
    dispatch.add_place("plant", Place("dock", (1.0, 1.0, 0.0)), replace=True)
    assert m.goals == []
