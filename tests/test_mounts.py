"""The setup check for sensor mounts: a sensor mounted a few degrees off its declared pose is
found from the floor it sees and from where it puts surfaces another sensor also sees."""

import copy
import math

import numpy as np
import pytest

from argus import node as argo
from argus import profile as profiles
from argus.setup.mounts import check_mounts
from argus.sim import world as worlds


def _rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _points(declared: dict, actual: dict, world) -> dict:
    """Each range sensor's returns from where it really is (`actual`), placed in the base frame
    through the declared mount, as setup would."""
    node = argo.Node(profiles.dump(declared), argo.NodeConfig())
    out = {}
    for spec in declared["sensors"]:
        if spec["kind"] not in ("range_2d", "range_3d", "depth"):
            continue
        sid = spec["id"]
        dirs = np.asarray(node.ray_directions(sid))
        real = actual["geometry"]["frames"][spec["frame"]]
        told = declared["geometry"]["frames"][spec["frame"]]
        r_real, t_real = _rotation(real.get("rpy_rad", [0, 0, 0])), np.array(real["xyz_m"], float)
        r_told, t_told = _rotation(told.get("rpy_rad", [0, 0, 0])), np.array(told["xyz_m"], float)
        rng = world.raycast(t_real, dirs @ r_real.T, spec["range_m"][1])
        ok = np.isfinite(rng) & (rng >= spec["range_m"][0])
        out[sid] = (dirs[ok] * rng[ok, None]) @ r_told.T + t_told
    return out


def _room():
    w = worlds.open_room(8.0)
    w.add((2.0, -1.5, 0.0), (2.4, -0.5, 1.0)).add((2.5, 0.8, 0.0), (3.0, 1.6, 1.2))
    return w


def _diff():
    return profiles.load(profiles.EXAMPLES_DIR / "planar_differential.yaml")


def test_mounts_as_declared_pass():
    doc = _diff()
    found = check_mounts(doc, _points(doc, doc, _room()))
    assert any("floor" in f.name for f in found) and any("agree" in f.name for f in found), found
    assert all(f.ok for f in found), [f.detail for f in found]


@pytest.mark.parametrize("axis,deg", [(1, 3.0), (1, -3.0), (0, 3.0)], ids=["pitched_down", "pitched_up", "rolled"])
def test_a_camera_tilted_off_its_declared_mount_sees_a_tilted_floor(axis, deg):
    doc = _diff()
    actual = copy.deepcopy(doc)
    rpy = actual["geometry"]["frames"]["front_depth"].setdefault("rpy_rad", [0.0, 0.0, 0.0])
    rpy[axis] += math.radians(deg)
    found = check_mounts(doc, _points(doc, actual, _room()))
    floor = [f for f in found if f.name == "mount front_depth (floor)"]
    assert floor and not floor[0].ok, found


def test_a_camera_turned_off_its_declared_mount_disagrees_with_the_lidar():
    doc = _diff()
    actual = copy.deepcopy(doc)
    rpy = actual["geometry"]["frames"]["front_depth"].setdefault("rpy_rad", [0.0, 0.0, 0.0])
    rpy[2] += math.radians(4.0)
    found = check_mounts(doc, _points(doc, actual, _room()))
    pair = [f for f in found if "agree" in f.name]
    assert pair and not pair[0].ok, found
