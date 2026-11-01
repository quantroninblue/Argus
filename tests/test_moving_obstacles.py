"""Moving obstacles in the simulator, and how Argo copes with people walking where it drives.

Rule (passive motion safety, Fraichard 2007): a contact is the robot's fault unless the robot
was already at rest. Someone walking into a stopped robot is not a failure; the robot still
moving at contact always is. Movers never react to the robot.
"""

import numpy as np
import pytest

from argus import profile as profiles
from argus.sim import world as worlds
from argus.sim.episode import Simulation, run_episode
from argus.sim.movers import Mover, crossing

N2 = pytest.mark.xfail(strict=True, reason="Argo treats obstacles as still; tracking and closing-speed "
                                            "margins are hub plan item N2")
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

    moving = Simulation(doc, worlds.open_room().add_mover(Mover(path=[(0.0, 0.0), (0.0, 0.1)], speed_mps=0.01)),
                        (0.0, 0.0, 0.0, 0.0))
    moving.plant.vel = np.array([0.5, 0.0, 0.0])
    moving._judge_movers(0.0)
    assert moving.result.collided and moving.result.contacts[0]["at_rest"] is False


@pytest.mark.parametrize("start_s", [
    1.0, pytest.param(1.5, marks=N2), pytest.param(2.0, marks=N2), pytest.param(2.5, marks=N2), 3.0, 4.0])
def test_a_person_crossing_ahead_is_never_hit(start_s):
    # They cross 3 m ahead, square to the robot's way; whether the robot gets there first
    # depends on when they set off.
    w = worlds.open_room().add_mover(crossing((3.0, 0.0), 0.0, 2.5, start_s=start_s))
    ep = run_episode(_diff(), w, (0.0, 0.0, 0.0, 0.0), GOAL, 30.0)
    assert not ep.collided, ep.contacts
    assert ep.complete, ep.final_pose


@pytest.mark.parametrize("speed_mps", [0.5, 1.0, pytest.param(1.4, marks=N2), pytest.param(2.0, marks=N2)])
def test_a_person_walking_head_on_is_never_hit(speed_mps):
    # Straight down the robot's way toward it, not giving way: the robot must be at rest
    # (or out of the way) by the time they meet, and carry on once they have passed.
    w = worlds.open_room().add_mover(Mover(path=[(6.5, 0.0), (-6.0, 0.0)], speed_mps=speed_mps,
                                           name="oncoming person"))
    ep = run_episode(_diff(), w, (0.0, 0.0, 0.0, 0.0), GOAL, 30.0)
    assert not ep.collided, ep.contacts
    assert ep.complete, ep.final_pose


@N2
def test_someone_who_stops_in_the_way_is_waited_for_then_gone_around():
    # Neel, 2026-11-01: a body that comes and stops in the robot's path makes it pause (it may
    # set off again in any direction), then go around it.
    w = worlds.open_room().add_mover(Mover(path=[(3.0, -3.0), (3.0, 0.0)], speed_mps=1.4, start_s=0.5,
                                           name="person who stops"))
    doc = _diff()
    ep = run_episode(doc, w, (0.0, 0.0, 0.0, 0.0), GOAL, 40.0)
    assert not ep.collided, ep.contacts
    assert ep.complete, ep.final_pose
    rate = doc["control"]["rate_hz"]
    assert ep.flag_counts.get("yielding", 0) >= 2.5 * rate  # held still for the yield, about 3 s
    assert ep.flag_counts.get("yielding", 0) <= 3.5 * rate
