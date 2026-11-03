"""Moving obstacles in the simulator, and how Argo copes with people walking where it drives.

Rule (passive motion safety, Fraichard 2007): a contact is the robot's fault unless the robot
was already at rest. Someone walking into a stopped robot is not a failure; the robot still
moving at contact always is. Movers never react to the robot.
"""

from pathlib import Path

import numpy as np
import pytest

from argus import profile as profiles
from argus.sim import world as worlds
from argus.sim.episode import SimOptions, Simulation, run_episode
from argus.sim.movers import Mover, crossing

GOAL = [{"id": "goal", "position": (6.0, 0.0, 0.0), "tolerance_m": 0.4}]


def _diff():
    return profiles.load(profiles.EXAMPLES_DIR / "planar_differential.yaml")


def test_a_mover_walks_its_path_once_or_back_and_forth():
    m = Mover(path=[(0, 0), (2, 0), (2, 2)], speed_mps=1.0, start_s=1.0)
    assert m.length_m == 4.0
    pos, vel = m.state(0.5)
    assert tuple(pos) == (0.0, 0.0) and tuple(vel) == (0.0, 0.0)  # waiting to start
    pos, vel = m.state(3.5)
    assert np.allclose(pos, (2.0, 0.5)) and np.allclose(vel, (0.0, 1.0))  # round the corner
    pos, vel = m.state(9.0)
    assert np.allclose(pos, (2.0, 2.0)) and tuple(vel) == (0.0, 0.0)  # stays at the end
    b = Mover(path=[(0, 0), (2, 0)], speed_mps=1.0, back_and_forth=True)
    assert np.allclose(b.state(2.5)[0], (1.5, 0.0)) and np.allclose(b.state(2.5)[1], (-1.0, 0.0))
    assert np.allclose(b.box(0.0), (-0.25, -0.25, 0.0, 0.25, 0.25, 1.75))
    with pytest.raises(ValueError):
        Mover(path=[(0, 0)])


def test_crossing_walks_square_to_the_robots_heading():
    c = crossing((3.0, 1.0), np.pi / 2, 2.0)
    assert np.allclose(c.path, [(5.0, 1.0), (1.0, 1.0)])


def test_sensors_see_a_mover_where_it_is_at_that_moment():
    w = worlds.open_room().add_mover(Mover(path=[(3.0, -4.0), (3.0, 4.0)], speed_mps=1.0))
    ray = np.array([[1.0, 0.0, 0.0]])
    assert np.isfinite(w.at(4.0).raycast(np.array([0.0, 0.0, 0.5]), ray, 20.0)[0])  # at (3, 0)
    far = w.at(0.0).raycast(np.array([0.0, 0.0, 0.5]), ray, 20.0)[0]
    assert far > 6.0  # still at (3, -4): the ray reaches the room's wall
    assert w.at(4.0).raycast(np.array([0.0, 0.0, 0.5]), ray, 20.0)[0] == pytest.approx(2.75)
    assert len(w.boxes) == len(worlds.open_room().boxes)  # the static boxes are untouched


def test_walking_into_a_robot_at_rest_is_not_its_fault_but_moving_into_one_is():
    doc = _diff()
    w = worlds.open_room().add_mover(Mover(path=[(3.0, 0.0), (-3.0, 0.0)], speed_mps=1.0))
    sim = Simulation(doc, w, (0.0, 0.0, 0.0, 0.0))
    for k in range(int(4.0 / sim.dt)):
        sim.step(lambda t: sim.node.tick(t))  # no plan: the robot stays where it is
    assert not sim.result.collided
    assert sim.result.contacts and all(c["at_rest"] for c in sim.result.contacts)

    # Driving into someone just ahead is a collision; pulling away from someone behind is not.
    ahead = Simulation(doc, worlds.open_room().add_mover(Mover(path=[(0.3, 0.0), (0.3, 0.1)], speed_mps=0.01)),
                       (0.0, 0.0, 0.0, 0.0))
    ahead.plant.vel = np.array([0.5, 0.0, 0.0])
    ahead._judge_movers(0.0)
    assert ahead.result.collided and ahead.result.contacts[0]["toward"] is True
    behind = Simulation(doc, worlds.open_room().add_mover(Mover(path=[(-0.3, 0.0), (-0.3, 0.1)], speed_mps=0.01)),
                        (0.0, 0.0, 0.0, 0.0))
    behind.plant.vel = np.array([0.5, 0.0, 0.0])
    behind._judge_movers(0.0)
    assert not behind.result.collided and behind.result.contacts[0]["toward"] is False


@pytest.mark.parametrize("start_s", [
    1.0, 1.5, 2.0, 2.5, 3.0, 4.0])
def test_a_person_crossing_ahead_is_never_hit(start_s):
    # They cross 3 m ahead, square to the robot's way; whether the robot gets there first
    # depends on when they set off.
    w = worlds.open_room().add_mover(crossing((3.0, 0.0), 0.0, 2.5, start_s=start_s))
    ep = run_episode(_diff(), w, (0.0, 0.0, 0.0, 0.0), GOAL, 30.0)
    assert not ep.collided, ep.contacts
    assert ep.complete, ep.final_pose


@pytest.mark.parametrize("speed_mps", [0.5, 1.0, 1.4, 2.0])
def test_a_person_walking_head_on_is_never_hit(speed_mps):
    # Straight down the robot's way toward it, not giving way: the robot must be at rest
    # (or out of the way) by the time they meet, and carry on once they have passed.
    w = worlds.open_room().add_mover(Mover(path=[(6.5, 0.0), (-6.0, 0.0)], speed_mps=speed_mps,
                                           name="oncoming person"))
    ep = run_episode(_diff(), w, (0.0, 0.0, 0.0, 0.0), GOAL, 30.0)
    assert not ep.collided, ep.contacts
    assert ep.complete, ep.final_pose


def test_someone_who_stops_in_the_way_is_waited_for_then_gone_around():
    # Someone who comes and stops in the robot's path makes it pause (they may
    # set off again in any direction), then it goes around them.
    w = worlds.open_room().add_mover(Mover(path=[(3.0, -3.0), (3.0, 0.0)], speed_mps=1.4, start_s=0.5,
                                           name="person who stops"))
    doc = _diff()
    ep = run_episode(doc, w, (0.0, 0.0, 0.0, 0.0), GOAL, 40.0)
    assert not ep.collided, ep.contacts
    assert ep.complete, ep.final_pose
    rate = doc["control"]["rate_hz"]
    assert ep.flag_counts.get("yielding", 0) >= 2.5 * rate  # held still for the yield, about 3 s
    assert ep.flag_counts.get("yielding", 0) <= 3.5 * rate


def _speeds_near(ep, x_lo, x_hi):
    """The robot's speed between control ticks while it is between x_lo and x_hi."""
    out = []
    for a, b in zip(ep.trace, ep.trace[1:]):
        if x_lo <= a["pose"][0] <= x_hi:
            out.append(float(np.linalg.norm(np.subtract(b["pose"][:3], a["pose"][:3])) / (b["t"] - a["t"])))
    return out


def test_it_slows_down_passing_someone_who_just_stopped_beside_its_way():
    # They walk in from the side and stop 1.6 m off the robot's line, out of its path: no
    # reason to wait, but it passes them slowly (SafetyConfig::people_*).
    w = worlds.open_room().add_mover(Mover(path=[(3.0, -3.0), (3.0, -1.6)], speed_mps=1.4, start_s=0.3,
                                           name="person beside the way"))
    ep = run_episode(_diff(), w, (0.0, 0.0, 0.0, 0.0), GOAL, 30.0, options=SimOptions(trace=True))
    assert not ep.collided, ep.contacts
    assert ep.complete, ep.final_pose
    assert ep.flag_counts.get("near_people", 0) > 0
    assert ep.flag_counts.get("yielding", 0) == 0
    near = _speeds_near(ep, 2.6, 3.4)
    assert near and max(near) <= 0.5, max(near)  # 0.3 at 0.5 m, rising to full speed at 2 m


def test_it_keeps_out_of_the_way_of_someone_crossing():
    # Someone crossing ahead, square to its way: the robot leaves them room, passing behind
    # rather than cutting in front (PlannerConfig::people_*).
    w = worlds.open_room().add_mover(crossing((3.0, 0.0), 0.0, 2.5, start_s=1.0))
    ep = run_episode(_diff(), w, (0.0, 0.0, 0.0, 0.0), GOAL, 30.0, options=SimOptions(trace=True))
    assert not ep.collided and not ep.contacts, ep.contacts
    assert ep.complete, ep.final_pose
    m = w.movers[0]
    gaps = []
    for r in ep.trace:
        pos, _ = m.state(r["t"])
        gaps.append(float(np.linalg.norm(np.subtract(r["pose"][:2], pos))))
    assert min(gaps) > 0.9, min(gaps)  # centre to centre; touching is about 0.55


def test_a_low_robot_crossing_below_the_cameras_is_tracked_and_never_hit():
    # A robot vacuum, 9 cm tall: every return from it lands in the map layer that holds the
    # ground cut. It is seen moving because the space above it is ever-free.
    # It reaches the robot's line as the robot gets there (about 3.7 s at x 3).
    vacuum = Mover(path=[(3.0, -1.1), (3.0, 3.0)], speed_mps=0.3, size=(0.35, 0.35, 0.09), name="robot vacuum")
    w = worlds.open_room().add_mover(vacuum)
    ep = run_episode(_diff(), w, (0.0, 0.0, 0.0, 0.0), GOAL, 40.0, options=SimOptions(trace=True))
    assert not ep.collided, ep.contacts
    assert ep.complete, ep.final_pose
    assert any(m["moving"] for r in ep.trace for m in r["movers"]), "the vacuum was never tracked as moving"


@pytest.mark.parametrize("turn_x", [2.5, 3.5, 4.5])
def test_someone_walking_alongside_who_turns_across_its_way_is_never_hit(turn_x):
    # The constant velocity prediction says they keep walking beside the robot; they turn
    # square across its way instead. The predicted spot is widened by the tracker's
    # uncertainty (SafetyConfig::mover_sigmas, PANTHER) to cover changes like this.
    w = worlds.open_room().add_mover(Mover(path=[(0.5, 1.1), (turn_x, 1.1), (turn_x, -3.0)], speed_mps=1.2,
                                           name="person turning across"))
    ep = run_episode(_diff(), w, (0.0, 0.0, 0.0, 0.0), GOAL, 40.0)
    assert not ep.collided, ep.contacts
    assert ep.complete, ep.final_pose


def _drone():
    return profiles.load(profiles.EXAMPLES_DIR / "aerial_multirotor.yaml")


DRONE_GOAL = [{"id": "goal", "position": (6.0, 0.0, 1.0), "tolerance_m": 0.4}]


@pytest.mark.parametrize("speed_mps", [0.5, 1.4])
def test_a_drone_is_never_hit_by_someone_walking_at_it(speed_mps):
    w = worlds.open_room().add_mover(Mover(path=[(6.5, 0.0), (-6.0, 0.0)], speed_mps=speed_mps,
                                           name="oncoming person"))
    ep = run_episode(_drone(), w, (0.0, 0.0, 1.0, 0.0), DRONE_GOAL, 30.0)
    assert not ep.collided, ep.contacts
    assert ep.complete, ep.final_pose


def test_a_drone_waits_for_someone_who_stops_in_its_way_then_goes_around():
    w = worlds.open_room().add_mover(Mover(path=[(3.0, -3.0), (3.0, 0.0)], speed_mps=1.4, start_s=0.5,
                                           name="person who stops"))
    doc = _drone()
    ep = run_episode(doc, w, (0.0, 0.0, 1.0, 0.0), DRONE_GOAL, 40.0)
    assert not ep.collided, ep.contacts
    assert ep.complete, ep.final_pose
    rate = doc["control"]["rate_hz"]
    assert 2.5 * rate <= ep.flag_counts.get("yielding", 0) <= 3.5 * rate, ep.flag_counts


def test_a_drone_well_above_someone_walking_is_not_held_up():
    # Flying at 4.5 m over a person 1.75 m tall walking the other way, more than the 2 m slow
    # zone above their head: neither slowed nor held.
    w = worlds.open_room().add_mover(Mover(path=[(6.5, 0.0), (-6.0, 0.0)], speed_mps=1.0))
    goal = [{"id": "goal", "position": (6.0, 0.0, 4.5), "tolerance_m": 0.4}]
    doc = _drone()
    doc["assumptions"]["max_flight_height_m"] = 5.5  # an open hall, no ceiling to speak of
    ep = run_episode(doc, w, (0.0, 0.0, 4.5, 0.0), goal, 30.0)
    assert not ep.collided
    assert ep.complete, ep.final_pose
    assert ep.flag_counts.get("near_people", 0) == 0, ep.flag_counts
    assert ep.flag_counts.get("yielding", 0) == 0, ep.flag_counts


def test_an_x500_class_drone_with_a_forward_camera_is_never_hit_by_someone_crossing():
    # Only a forward depth camera (87 degrees) sees people: someone crossing from the side is
    # seen late, as with a ground robot's narrow camera.
    doc = profiles.load(Path(__file__).parent / "bodies" / "x500_class.yaml")
    w = worlds.open_room().add_mover(crossing((3.0, 0.0), 0.0, 2.5, start_s=1.0))
    ep = run_episode(doc, w, (0.0, 0.0, 1.0, 0.0), DRONE_GOAL, 40.0)
    assert not ep.collided, ep.contacts
    assert ep.complete, ep.final_pose


def test_a_drone_with_a_forward_camera_turns_to_keep_someone_beside_its_way_in_view():
    # The X500 class sees people with one forward camera (87 degrees). Someone walking the
    # same way, ahead and to the left, drifts out of that view as the drone overtakes; it
    # turns to keep them in view while the way ahead stays in view too (PANTHER).
    doc = profiles.load(Path(__file__).parent / "bodies" / "x500_class.yaml")
    w = worlds.open_room().add_mover(Mover(path=[(2.5, 1.4), (6.5, 1.4)], speed_mps=0.5, name="person ahead"))
    ep = run_episode(doc, w, (0.0, 0.0, 1.0, 0.0), DRONE_GOAL, 40.0, options=SimOptions(trace=True))
    assert not ep.collided, ep.contacts
    assert ep.complete, ep.final_pose
    m = w.movers[0]
    looked, kept = 0, 0
    for a, b in zip(ep.trace, ep.trace[1:]):
        pos, _ = m.state(b["t"])
        rel = np.subtract(pos, b["pose"][:2])
        travel = np.subtract(b["pose"][:2], a["pose"][:2])
        if np.linalg.norm(rel) > 4.0 or np.linalg.norm(travel) < 1e-4:
            continue
        off = abs(np.arctan2(travel[0] * rel[1] - travel[1] * rel[0], travel @ rel))
        if 0.8 < off < 1.35:  # beside the way, outside the camera with the nose along it
            looked += 1
            kept += any(np.linalg.norm(np.subtract(t["position"], pos)) < 0.7 for t in b["movers"])
    assert looked > 0 and kept / looked >= 0.6, (kept, looked)
