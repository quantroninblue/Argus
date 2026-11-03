"""Named places, routes and errands on the hub (hub plan N1)."""

import pytest

from argus.hub.errand import Errand, ErrandState
from argus.hub.mission import MissionState
from argus.hub.places import Place, PlaceError, Route, SiteBook, Stop

PLACES = {k: Place(k, (float(i), 0.0, 0.0)) for i, k in enumerate("ABC")}


def _visit(route, outcomes):
    """Runs an errand where each leg ends as `outcomes` says; returns the stops it set out
    for, its final state and its last event."""
    e = Errand(route, PLACES)
    e.start()
    order = [e.place().name]
    t = 0.0
    for outcome in outcomes:
        t += 1.0
        goal = e.update(t, outcome, "dead_end_observed")
        if goal is None and e.state == ErrandState.DWELLING:
            t += 1000.0
            goal = e.update(t, MissionState.ACTIVE)
        if goal is not None:
            order.append(e.place().name)
        if not e.active:
            break
    return order, e.state, e.take_events()[-1].kind


ABC = (Stop("A"), Stop("B"), Stop("C"))


def test_once_visits_each_stop_in_order_then_is_done():
    assert _visit(Route("r", ABC), [MissionState.REACHED] * 5) == (["A", "B", "C"], ErrandState.DONE, "errand_done")


def test_a_loop_goes_round_the_given_number_of_times():
    order, state, _ = _visit(Route("r", ABC, "loop", cycles=2), [MissionState.REACHED] * 10)
    assert order == ["A", "B", "C", "A", "B", "C"] and state == ErrandState.DONE


def test_back_and_forth_turns_at_the_ends():
    order, state, _ = _visit(Route("r", ABC, "back_and_forth", cycles=2), [MissionState.REACHED] * 20)
    assert order == ["A", "B", "C", "B", "A", "B", "C", "B", "A"] and state == ErrandState.DONE


def test_an_unreachable_stop_is_skipped_or_ends_the_errand_as_the_route_says():
    outcomes = [MissionState.REACHED, MissionState.FAILED, MissionState.REACHED]
    order, state, _ = _visit(Route("r", ABC, on_unreachable="skip"), outcomes)
    assert order == ["A", "B", "C"] and state == ErrandState.DONE
    order, state, last = _visit(Route("r", ABC, on_unreachable="stop"), outcomes)
    assert order == ["A", "B"] and state == ErrandState.FAILED and last == "errand_failed"


def test_a_loop_where_nothing_can_be_reached_ends_instead_of_going_round_forever():
    order, state, last = _visit(Route("r", ABC[:2], "loop"), [MissionState.FAILED] * 10)
    assert order == ["A", "B"] and state == ErrandState.FAILED and last == "errand_failed"


def test_the_robot_holds_at_a_stop_for_its_dwell_time():
    e = Errand(Route("r", (Stop("A", dwell_s=5.0), Stop("B"))), PLACES)
    e.start()
    assert e.update(10.0, MissionState.REACHED) is None and e.state == ErrandState.DWELLING
    assert e.update(14.9, MissionState.REACHED) is None
    goal = e.update(15.0, MissionState.REACHED)
    assert goal is not None and goal.position == (1.0, 0.0, 0.0)
    assert [ev.kind for ev in e.take_events()] == ["leg_started", "leg_reached", "dwelling", "leg_started"]


def test_a_goal_replaced_underneath_cancels_the_errand():
    e = Errand(Route("r", ABC), PLACES)
    e.start()
    e.update(1.0, MissionState.CANCELLED)
    assert e.state == ErrandState.CANCELLED and not e.active


def test_places_and_routes_are_kept_per_site_and_checked(tmp_path):
    book = SiteBook(tmp_path, "warehouse")
    book.add_place(Place("dock", (1.0, 2.0, 0.0), 0.3, heading_rad=1.57))
    book.add_place(Place("shelf-4", (6.0, -1.0, 0.0)))
    assert book.places()["dock"] == Place("dock", (1.0, 2.0, 0.0), 0.3, 1.57)
    book.add_route(Route("rounds", (Stop("dock", 10.0), Stop("shelf-4")), "loop", cycles=3))
    assert SiteBook(tmp_path, "warehouse").routes()["rounds"].cycles == 3  # read back from disk
    assert SiteBook(tmp_path, "lab").places() == {}  # another site has its own
    with pytest.raises(PlaceError, match="no place"):
        book.add_route(Route("bad", (Stop("nowhere"),)))
    with pytest.raises(PlaceError, match="on route"):
        book.remove_place("dock")
    with pytest.raises(PlaceError, match="cycles"):
        book.add_route(Route("bad", (Stop("dock"),), "once", cycles=2))
    with pytest.raises(PlaceError, match="name"):
        book.add_place(Place("../escape", (0.0, 0.0, 0.0)))
    book.remove_route("rounds")
    book.remove_place("dock")
    assert list(book.places()) == ["shelf-4"]


def test_site_floor_and_ceiling_come_from_the_hub_home(tmp_path):
    from argus.hub.places import site_heights

    (tmp_path / "warehouse").mkdir()
    (tmp_path / "warehouse" / "site.yaml").write_text("floor_z_m: 0.0\nceiling_z_m: 4.0\n")
    (tmp_path / "yard").mkdir()  # no site.yaml: defaults
    assert site_heights(tmp_path) == {"warehouse": (0.0, 4.0)}
    (tmp_path / "warehouse" / "site.yaml").write_text("floor_z_m: 2.0\nceiling_z_m: 1.0\n")
    with pytest.raises(PlaceError, match="ceiling"):
        site_heights(tmp_path)
