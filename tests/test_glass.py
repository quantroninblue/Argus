"""Glass: the depth camera sees it, the lidar does not (its beams pass through and come back
from the wall behind, or not at all). What one sensor saw as solid, another seeing through
it must not wipe from the map."""

import pytest

from argus import profile as profiles
from argus.sim import world as worlds
from argus.sim.episode import run_episode

GOAL = [{"id": "goal", "position": (6.0, 0.0, 0.0), "tolerance_m": 0.4}]


def _diff():
    return profiles.load(profiles.EXAMPLES_DIR / "planar_differential.yaml")


def _with_3d_lidar():
    # A 3D lidar clears every height its beams pass through, so it would wipe out all of
    # what the camera saw of the glass, not one layer of it.
    doc = _diff()
    lidar = next(s for s in doc["sensors"] if s["id"] == "front_lidar")
    lidar["kind"] = "range_3d"
    lidar["fov_deg"] = {"h": 360, "v": 45}
    return doc


def test_a_glass_pane_across_the_way_seen_only_by_the_camera_is_gone_around():
    w = worlds.open_room().add_glass((3.0, -1.5, 0.0), (3.1, 1.5, 2.0))
    ep = run_episode(_diff(), w, (0.0, 0.0, 0.0, 0.0), GOAL, 60.0)
    assert not ep.collided, ep.final_pose
    assert ep.complete, ep.final_pose


def test_glass_a_3d_lidar_sees_through_is_never_hit():
    # The lidar shows the space free at every height, so the camera's returns from the pane
    # read as something standing where nothing was: Argo keeps clear and waits rather than
    # map it. Safe; going around it as well is still to come.
    w = worlds.open_room().add_glass((3.0, -1.5, 0.0), (3.1, 1.5, 2.0))
    ep = run_episode(_with_3d_lidar(), w, (0.0, 0.0, 0.0, 0.0), GOAL, 40.0)
    assert not ep.collided, ep.final_pose


def test_rays_from_a_lidar_pass_through_glass():
    import numpy as np

    w = worlds.World().add_glass((1.0, -1.0, 0.0), (1.1, 1.0, 2.0)).add((3.0, -1.0, 0.0), (3.2, 1.0, 2.0))
    ray = np.array([[1.0, 0.0, 0.0]])
    assert w.raycast(np.array([0.0, 0.0, 0.5]), ray, 10.0)[0] == 1.0
    assert w.raycast(np.array([0.0, 0.0, 0.5]), ray, 10.0, see_glass=False)[0] == 3.0


def _scan_only(no_return=None):
    from pathlib import Path

    doc = profiles.load(Path(__file__).parent / "bodies" / "turtlebot3_waffle_class.yaml")
    if no_return:
        doc["sensors"][0]["no_return"] = no_return
    return doc


@pytest.mark.parametrize("no_return,expected", [(None, "free"), ("free", "free"), ("unknown", "unknown")])
def test_a_sensor_may_declare_that_no_return_says_nothing(no_return, expected):
    import numpy as np

    from argus import node as argo

    doc = _scan_only(no_return)
    assert profiles.problems(doc) == []
    node = argo.Node(profiles.dump(doc), argo.NodeConfig())
    node.on_odometry(0.0, [0.0, 0.0, 0.0], 0.0, [0.0, 0.0, 0.0], 0.0, True)
    ranges = np.full(len(node.self_hits("scan")), np.inf, dtype=np.float32)
    assert node.on_range_frame(0.0, "scan", ranges)
    lidar_z = 0.18
    assert node.cell((1.5, 0.0, lidar_z)) == expected


def test_no_return_takes_only_free_or_unknown():
    doc = _scan_only("maybe")
    assert any("no_return" in p for p in profiles.problems(doc))
