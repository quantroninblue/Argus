"""Drops in the floor (a stairwell, a pit, a dock edge): Argo must see them with sensors that
look at the floor and never drive over the edge."""

from argus import profile as profiles
from argus.sim import world as worlds
from argus.sim.episode import SimOptions, Simulation, run_episode

GOAL = [{"id": "goal", "position": (6.0, 0.0, 0.0), "tolerance_m": 0.4}]


def _diff():
    # Its depth camera looks down at the floor ahead; the lidar's scan plane never sees it.
    return profiles.load(profiles.EXAMPLES_DIR / "planar_differential.yaml")


def test_a_pit_across_the_way_is_seen_and_gone_around():
    w = worlds.open_room().add_hole((3.0, -1.5), (4.0, 1.5), 1.0)
    ep = run_episode(_diff(), w, (0.0, 0.0, 0.0, 0.0), GOAL, 60.0)
    assert not ep.collided, ep.final_pose
    assert ep.complete, ep.final_pose


def test_a_drop_from_wall_to_wall_stops_the_robot_short_of_the_edge():
    w = worlds.open_room().add_hole((3.0, -7.0), (8.0, 7.0), 1.0)
    ep = run_episode(_diff(), w, (0.0, 0.0, 0.0, 0.0), GOAL, 30.0)
    assert not ep.collided, ep.final_pose
    assert not ep.complete
    assert ep.final_pose[0] < 3.0


def test_rays_into_a_pit_end_on_its_bottom_or_sides():
    import numpy as np

    w = worlds.World().add_hole((1.0, -1.0), (2.0, 1.0), 0.5)
    origin = np.array([0.0, 0.0, 1.0])
    dirs = np.array([[1.2, 0.0, -1.0], [1.5, 0.0, -1.0], [0.5, 0.0, -1.0], [3.0, 0.0, -1.0]])
    dirs /= np.linalg.norm(dirs, axis=1)[:, None]
    hit = origin + dirs * w.raycast(origin, dirs, 20.0)[:, None]
    assert np.allclose(hit[0], [1.8, 0.0, -0.5])  # the pit's bottom
    assert np.allclose(hit[1], [2.0, 0.0, -1.0 / 3.0])  # its far side
    assert np.allclose(hit[2], [0.5, 0.0, 0.0])  # floor short of it
    assert np.allclose(hit[3], [3.0, 0.0, 0.0])  # floor beyond it


def test_an_edge_stays_marked_as_the_robot_comes_close():
    # Rays passing low over a pit show the space above it empty, which is true; the edge is
    # missing floor, so they must not wipe it out. Only a return from the floor clears it.
    w = worlds.open_room().add_hole((3.0, -7.0), (8.0, 7.0), 1.0)
    sim = Simulation(_diff(), w, (0.0, 0.0, 0.0, 0.0), SimOptions())
    node = sim.node

    def tick(t):
        if t == 0.0:
            node.set_plan(1, 60.0, GOAL, 0.0)
        return node.tick(t)

    edge = [(3.05, y, 0.15) for y in (-0.45, -0.25, -0.05)]
    marked_at = None
    while sim.t < 8.0:
        sim.step(tick)
        states = [node.cell(p) for p in edge]
        if marked_at is None and all(s == "occupied" for s in states):
            marked_at = sim.t
        elif marked_at is not None:
            assert all(s == "occupied" for s in states), (round(sim.t, 2), sim.pose, states)
    assert marked_at is not None
