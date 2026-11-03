"""Odometry drift correction on the hub (argus.hub.drift): the correction is found by matching
what Argo sees now against the surfaces the site has held for a while, evidence is moved by it,
and the hub keeps track of a robot whose odometry drifts."""

import math

import numpy as np
import pytest

from argus import profile as profiles
from argus import sitemap
from argus.hub import HubConfig, MissionState, SiteService
from argus.hub.drift import DriftConfig, DriftEstimator, SurfaceMemory
from argus.protocol.v1 import node_messages_pb2
from argus.sim import world as worlds
from argus.sim.episode import SimOptions
from argus.sim.hub_episode import NODE_ID, HubLoop

RES = 0.1
Z = 5  # wall cells at 0.5 to 0.6 m


def _room_cells(half_x=3.0, half_y=2.0, box=True):
    """Wall columns of a room, plus a box inside so no direction is unconstrained."""
    pts = []
    for x in np.arange(-half_x, half_x + 1e-9, RES):
        pts += [(x, -half_y), (x, half_y)]
    for y in np.arange(-half_y, half_y + 1e-9, RES):
        pts += [(-half_x, y), (half_x, y)]
    if box:
        for x in np.arange(0.5, 1.3, RES):
            pts += [(x, 0.4), (x, 1.0)]
    xy = np.array(pts)
    cells = np.floor(xy / RES).astype(np.int32)
    cells = np.unique(cells, axis=0)
    return np.column_stack([cells, np.full(len(cells), Z, dtype=np.int32)])


def _site(cells, at=0.0):
    m = sitemap.SiteMap("s", RES)
    deltas = np.full(len(cells), 12, dtype=np.int32)
    assert m.apply_evidence(m.epoch, sitemap.encode_evidence_arrays(cells, deltas), at)
    mem = SurfaceMemory(RES, 0.3, m.epoch)
    mem.add(cells, at)
    return m, mem


def _seen_through(cells, x, y, yaw):
    """The same cells as an odometry frame sees them when map = R(yaw) odom + (x, y)."""
    xy = (cells[:, :2] + 0.5) * RES
    c, s = math.cos(yaw), math.sin(yaw)
    odom = (xy - (x, y)) @ np.array([[c, -s], [s, c]])
    out = cells.copy()
    out[:, :2] = np.floor(odom / RES).astype(np.int32)
    return out


def _converge(est, mem, m, odom_cells, start=10.0, rounds=12):
    deltas = np.full(len(odom_cells), 3, dtype=np.int32)
    t = start
    for _ in range(rounds):
        est.observe(t, odom_cells, deltas)
        est.update(t, mem, m)
        t += est.config.period_s
    return est


@pytest.mark.parametrize("x,y,yaw", [(0.25, 0.0, 0.0), (0.0, -0.2, 0.0), (0.2, 0.15, 0.04), (0.0, 0.0, -0.06)])
def test_the_correction_is_found_from_known_surfaces(x, y, yaw):
    cells = _room_cells()
    m, mem = _site(cells)
    est = _converge(DriftEstimator(DriftConfig(), RES), mem, m, _seen_through(cells, x, y, yaw))
    assert est.corrections >= 1
    # Within the deadband of the true correction.
    assert math.hypot(est.x - x, est.y - y) < 0.08
    assert abs(est.yaw - yaw) < 0.025


def test_no_correction_without_drift():
    cells = _room_cells()
    m, mem = _site(cells)
    est = _converge(DriftEstimator(DriftConfig(), RES), mem, m, cells)
    assert est.corrections == 0
    assert est.identity
    assert est.last is not None and est.last.reason == "within the noise"


def test_each_step_is_capped():
    cells = _room_cells()
    m, mem = _site(cells)
    est = DriftEstimator(DriftConfig(max_step_m=0.1), RES)
    _converge(est, mem, m, _seen_through(cells, 0.25, 0.0, 0.0), rounds=1)
    assert est.corrections == 1
    assert est.x == pytest.approx(0.1, abs=0.02)


def test_new_surfaces_are_not_reference_until_settled():
    cells = _room_cells()
    m, mem = _site(cells, at=9.0)  # first seen 1 s before the match; settle_s is 3
    est = _converge(DriftEstimator(DriftConfig(), RES), mem, m, _seen_through(cells, 0.3, 0.0, 0.0), rounds=1)
    assert est.corrections == 0


def test_unknown_space_gives_no_correction():
    cells = _room_cells()
    m, mem = _site(cells)
    # Seen somewhere the site has never mapped.
    far = cells.copy()
    far[:, 0] += 200
    est = _converge(DriftEstimator(DriftConfig(), RES), mem, m, far)
    assert est.corrections == 0 and est.identity


def test_surfaces_seen_again_close_by_do_not_become_reference():
    cells = _room_cells(box=False)
    mem = SurfaceMemory(RES, 0.3)
    mem.add(cells, 0.0)
    n = len(mem.columns)
    shifted = cells.copy()
    shifted[:, 0] += 2  # 0.2 m off: the same wall seen through drift
    mem.add(shifted, 5.0)
    far = cells.copy()
    far[:, 0] += 100
    assert len(mem.columns) - n == 0
    mem.add(far, 5.0)
    assert len(mem.columns) > n


def _evidence(m, cells, deltas):
    ev = node_messages_pb2.LocalEvidence()
    ev.map_epoch = m.epoch
    ev.resolution_m = RES
    ev.encoding = node_messages_pb2.EVIDENCE_ENCODING_VOXEL_DELTA
    ev.payload = sitemap.encode_evidence_arrays(cells, deltas)
    ev.map_to_odom.orientation.w = 1.0
    return ev


def test_evidence_is_moved_into_the_site_frame():
    sites = SiteService(RES)
    m = sites.map("s")
    est = DriftEstimator(DriftConfig(), RES)
    est.x, est.y = 0.1, -0.2
    cells = np.array([[10, 10, Z], [11, 10, Z], [10, 10, 0]], dtype=np.int32)
    deltas = np.array([12, 12, -4], dtype=np.int32)
    assert sites.apply("s", _evidence(m, cells, deltas), est, now=1.0)
    assert m.cell((11, 8, Z)) == sitemap.OCCUPIED
    assert m.cell((12, 8, Z)) == sitemap.OCCUPIED
    assert m.cell((10, 10, Z)) == sitemap.UNKNOWN
    assert m.cell((11, 8, 0)) == sitemap.FREE
    # Only hits above the floor go into the site's surfaces, in the site frame.
    assert set(sites.surfaces("s").columns) == {(11, 8), (12, 8)}


def test_points_round_trip_between_frames():
    est = DriftEstimator(DriftConfig(), RES)
    est.x, est.y, est.yaw = 0.4, -1.1, 0.3
    p = (2.0, 3.0, 0.5)
    back = est.point_to_odom(est.point_to_map(p))
    assert back == pytest.approx(p)


def test_encoding_arrays_sums_repeated_cells():
    cells = np.array([[1, 2, 3], [1, 2, 3], [4, 5, 6]], dtype=np.int32)
    deltas = np.array([2, 3, -1], dtype=np.int32)
    out_cells, out_deltas = sitemap.decode_evidence(sitemap.encode_evidence_arrays(cells, deltas))
    got = {tuple(c): int(d) for c, d in zip(out_cells.tolist(), out_deltas)}
    assert got == {(1, 2, 3): 5, (4, 5, 6): -1}


def _out_and_back(options, enabled=True):
    """Out through the gap and back to the start: new space cannot be corrected (no loop closure
    yet), the room the robot came from can."""
    doc = profiles.load(profiles.EXAMPLES_DIR / "planar_differential.yaml")
    loop = HubLoop(doc, worlds.wall_with_gap(), (0.0, 0.0, 0.0, 0.0), (5.0, 2.0, 0.0), options=options,
                   hub_config=HubConfig(drift=DriftConfig(enabled=enabled)))
    while loop.mission_state != MissionState.REACHED and loop.sim.t < 40.0:
        loop.step()
    assert loop.mission_state == MissionState.REACHED
    loop.hub.set_goal(NODE_ID, (0.0, 0.0, 0.0), 0.4, loop.sim.t)
    end = loop.sim.t + 40.0
    while loop.mission_state != MissionState.REACHED and loop.sim.t < end:
        loop.step()
    assert loop.mission_state == MissionState.REACHED
    assert not loop.sim.result.collided and loop.sim.result.violations == []
    return loop


def _pose_error(loop):
    x, y = loop.sim.plant.pos[:2]
    px, py = loop.hub_session.pose[:2]
    return math.hypot(px - x, py - y)


def test_the_hub_keeps_track_of_a_drifting_robot():
    # A gyro bias of half a degree per second, from once the first walls are settled.
    drifting = SimOptions(odom_yaw_drift_radps=0.01, odom_drift_from_s=4.0)
    corrected = _out_and_back(drifting)
    plain = _out_and_back(drifting, enabled=False)
    assert corrected.hub_session.drift.corrections >= 1
    assert plain.hub_session.drift is None
    assert _pose_error(corrected) < 0.1
    assert _pose_error(plain) > 0.3


def test_a_run_without_drift_is_left_alone():
    loop = _out_and_back(SimOptions())
    d = loop.hub_session.drift
    # At worst one map cell, from a wall lying on a cell boundary.
    assert math.hypot(d.x, d.y) < 0.15 and abs(d.yaw) < 0.03
    assert _pose_error(loop) < 0.15


def test_a_robot_that_stops_sending_hits_is_not_matched():
    cells = _room_cells()
    m, mem = _site(cells)
    est = DriftEstimator(DriftConfig(), RES)
    est.observe(10.0, cells, np.full(len(cells), 3, dtype=np.int32))
    # Standing still: nothing new, and the last hits are out of the window.
    assert est.update(30.0, mem, m) is False


def _wall(x, y0=-3.0, y1=3.0):
    ys = np.arange(y0, y1, RES)
    cells = np.floor(np.column_stack([np.full(len(ys), x), ys]) / RES).astype(np.int32)
    return np.column_stack([cells, np.full(len(cells), Z, dtype=np.int32)])


def test_the_far_face_of_a_wall_is_not_the_near_face():
    near, far = _wall(2.0), _wall(2.2)
    m = sitemap.SiteMap("s", RES)
    for cells in (near, far):
        assert m.apply_evidence(m.epoch, sitemap.encode_evidence_arrays(cells, np.full(len(cells), 12, dtype=np.int32)), 0.0)
    mem = SurfaceMemory(RES, 0.3, m.epoch)
    mem.add(near, 0.0, (0.0, 0.0))

    est = DriftEstimator(DriftConfig(), RES)
    deltas = np.full(len(far), 3, dtype=np.int32)
    t = 10.0
    for _ in range(6):
        est.observe(t, far, deltas, (4.0, 0.0))
        est.update(t, mem, m)
        t += 1.0
    assert est.corrections == 0 and est.identity

    # Seen from the other side, the far face is a surface of its own.
    mem.add(far, 20.0, (4.0, 0.0))
    assert (22, 0) in mem.columns


def test_a_corridor_is_corrected_across_and_left_alone_along():
    xs = np.arange(-5.0, 5.0, RES)
    pts = np.array([(x, y) for x in xs for y in (-1.0, 1.0)])
    cells = np.floor(pts / RES).astype(np.int32)
    cells = np.column_stack([cells, np.full(len(cells), Z, dtype=np.int32)])
    m, mem = _site(cells)
    est = _converge(DriftEstimator(DriftConfig(), RES), mem, m, _seen_through(cells, 0.3, 0.2, 0.0))
    assert est.y == pytest.approx(0.2, abs=0.06)
    assert est.x == pytest.approx(0.0, abs=0.01)
