"""Loop closure on the hub (argus.hub.loops): recognising a place from far enough off that
drift.py cannot, and above all not recognising one that only looks the same. Whole runs judge the
hub's pose and map against simulator truth."""

import math
import tempfile
from pathlib import Path

import numpy as np
import pytest

from argus import profile as profiles
from argus import sitemap
from argus.hub import HubConfig
from argus.hub.loops import Field, LoopConfig, between, compose, invert, search
from argus.hub.places import Place, Route, Stop
from argus.sim import world as worlds
from argus.sim.episode import SimOptions
from argus.sim.hub_episode import NODE_ID, SITE_ID, HubLoop


def ring():
    """Corridors round a block, with things along them so that no two corners look alike."""
    w = worlds._room(worlds.World(name="ring"), 7.0, 7.0, 2.0)
    w.add((-4, -4, 0), (4, 4, 2.0))
    for lo, hi in [((-6.4, 5.0), (-5.6, 5.8)), ((5.0, 5.2), (5.5, 6.4)), ((4.6, -6.2), (6.2, -5.8)),
                   ((-5.0, -6.5), (-4.6, -5.2)), ((0.0, 5.6), (0.5, 6.6)), ((5.4, 0.0), (6.6, 0.4)),
                   ((-1.0, -6.6), (-0.5, -5.6))]:
        w.add((*lo, 0), (*hi, 2.0))
    return w


def plain_ring():
    """The same corridors with nothing in them: every corner is every other corner."""
    w = worlds._room(worlds.World(name="plain_ring"), 7.0, 7.0, 2.0)
    w.add((-4, -4, 0), (4, 4, 2.0))
    return w


def doors():
    """One long corridor with the same recess every three metres."""
    w = worlds.World(name="doors")
    w.add((-12.2, -1.2, 0), (12.2, -1.0, 2.0))
    x = -12.0
    while x < 12.0:
        w.add((x, 1.0, 0), (x + 2.2, 1.2, 2.0))
        w.add((x + 2.2, 1.0, 0), (x + 2.4, 2.0, 2.0))
        w.add((x + 2.2, 2.0, 0), (x + 3.2, 2.2, 2.0))
        w.add((x + 3.0, 1.0, 0), (x + 3.2, 2.0, 2.0))
        x += 3.0
    w.add((-12.2, -1.2, 0), (-12.0, 2.2, 2.0))
    w.add((12.0, -1.2, 0), (12.2, 2.2, 2.0))
    return w


RING_STOPS = {"a": (-5.3, -3.0, 0), "b": (5.3, -4.8, 0), "c": (4.3, 5.0, 0), "d": (-4.8, 4.9, 0)}
RING_START = (-5.3, -5.0, 0.0, 1.57)


def _patrol(world, start, stops, yaw_drift=0.0, scale=1.0, loops=True, laps=2, limit_s=300.0, config=None):
    doc = profiles.load(profiles.EXAMPLES_DIR / "planar_differential.yaml")
    hub = HubConfig()
    hub.loops = config or LoopConfig(enabled=loops)
    with tempfile.TemporaryDirectory() as home:
        first = next(iter(stops.values()))
        loop = HubLoop(doc, world, start, first, hub_config=hub, sites_dir=Path(home) / "sites",
                       options=SimOptions(odom_yaw_drift_radps=yaw_drift, odom_scale=scale))
        for name, p in stops.items():
            loop.hub.dispatch.add_place(SITE_ID, Place(name, p, tolerance_m=0.6))
        loop.hub.dispatch.add_route(SITE_ID, Route("r", tuple(Stop(k) for k in stops), mode="loop", cycles=laps))
        loop.hub.patrol(NODE_ID, "r", 0.0)
        done = False
        while loop.sim.t < limit_s and not done:
            loop.step()
            done = any(e.kind in ("errand_done", "errand_failed") for e in loop.hub.dispatch.take_events())
    return loop


def _pose_error(loop):
    s, p = loop.hub_session, loop.sim.plant.pos
    return math.hypot(s.pose[0] - p[0], s.pose[1] - p[1])


def _off_walls(loop, world):
    """Share of the map's occupied columns more than 0.25 m from anything real."""
    view = loop.hub.sites.map(SITE_ID).planar_view(0.2, 1.5)
    occupied = np.argwhere(view["cells"] == sitemap.OCCUPIED)
    xy = (occupied + np.array(view["origin"]) + 0.5) * view["resolution"]
    boxes = world.boxes
    dx = np.maximum.reduce([boxes[:, 0][None] - xy[:, 0:1], np.zeros((len(xy), len(boxes))), xy[:, 0:1] - boxes[:, 3][None]])
    dy = np.maximum.reduce([boxes[:, 1][None] - xy[:, 1:2], np.zeros((len(xy), len(boxes))), xy[:, 1:2] - boxes[:, 4][None]])
    return float(np.mean(np.hypot(dx, dy).min(axis=1) > 0.25))


def _closer(loop):
    return loop.hub.sites._closers[SITE_ID]


def test_poses_compose_and_invert():
    a, b = (1.0, 2.0, 0.7), (-0.5, 0.3, -1.2)
    x, y, yaw = between(a, compose(a, b))
    assert (x, y, yaw) == pytest.approx(b)
    assert compose(a, invert(a)) == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


def _wall_points():
    xs = np.arange(0.05, 6.0, 0.1)
    return np.concatenate([np.column_stack([xs, np.zeros_like(xs)]), np.column_stack([np.zeros_like(xs), xs])])


def test_the_search_finds_a_shifted_corner():
    corner = _wall_points()
    target = Field(corner, np.empty((0, 2)), 0.1, 3.0)
    c = LoopConfig()
    fix, score, _ = search(corner + (0.8, -0.6), target, c, 1.5, 0.2)
    assert score > 0.9 and abs(fix[0] + 0.8) <= 0.2 and abs(fix[1] - 0.6) <= 0.2


def test_the_search_sees_that_a_straight_wall_has_no_single_best_fit():
    xs = np.arange(0.05, 8.0, 0.1)
    wall = np.column_stack([xs, np.zeros_like(xs)])
    target = Field(wall, np.empty((0, 2)), 0.1, 3.0)
    query = wall[20:50] + (0.0, 0.5)
    _, score, runner_up = search(query, target, LoopConfig(), 1.5, 0.2)
    assert score - runner_up < LoopConfig().min_peak_gap


def test_a_room_that_repeats_within_the_window_has_no_single_best_fit():
    posts = np.array([(float(x), float(y)) for x in np.arange(0.0, 8.0, 2.0) for y in (0.0, 0.1, 0.2)])
    target = Field(posts, np.empty((0, 2)), 0.1, 3.0)
    query = posts[3:9] + (0.3, 0.0)
    _, score, runner_up = search(query, target, LoopConfig(), 2.5, 0.1)
    assert score - runner_up < LoopConfig().min_peak_gap


def test_hits_in_space_a_target_saw_empty_are_noticed():
    corner = _wall_points()
    xs, ys = np.meshgrid(np.arange(0.5, 5.0, 0.1), np.arange(0.5, 5.0, 0.1))
    empty = np.column_stack([xs.ravel(), ys.ravel()])
    target = Field(corner, empty, 0.1, 3.0)
    inside = np.array([[2.05, 2.05], [3.05, 3.05]])
    assert target.look(target.empty, inside).all()
    assert not target.look(target.empty, corner).any()


def test_clean_odometry_closes_nothing_and_leaves_the_map_alone():
    world = ring()
    plain = _patrol(world, RING_START, RING_STOPS, loops=False)
    closed = _patrol(world, RING_START, RING_STOPS)
    assert _closer(closed).closures == 0
    assert _off_walls(closed, world) == pytest.approx(_off_walls(plain, world), abs=0.01)


def test_a_robot_that_drifted_is_brought_back_and_the_doubled_walls_go():
    world = ring()
    plain = _patrol(world, RING_START, RING_STOPS, yaw_drift=0.005, loops=False)
    closed = _patrol(world, RING_START, RING_STOPS, yaw_drift=0.005)
    assert _closer(closed).closures >= 1
    assert not closed.sim.result.collided
    assert _pose_error(plain) > 1.0 and _pose_error(closed) < 0.5
    assert _off_walls(plain, world) > 0.2 and _off_walls(closed, world) < 0.05


@pytest.mark.parametrize("yaw_drift", [0.0, 0.002, 0.005])
def test_corners_that_all_look_the_same_are_never_taken_for_each_other(yaw_drift):
    """The drift can only do so much here, but a closure must not make it worse."""
    world = plain_ring()
    plain = _patrol(world, RING_START, RING_STOPS, yaw_drift=yaw_drift, loops=False)
    closed = _patrol(world, RING_START, RING_STOPS, yaw_drift=yaw_drift)
    assert _pose_error(closed) <= _pose_error(plain) + 0.3
    assert _off_walls(closed, world) <= _off_walls(plain, world) + 0.05


@pytest.mark.parametrize("yaw_drift", [0.0, 0.002])
def test_a_row_of_identical_doors_is_never_closed_on(yaw_drift):
    world = doors()
    stops = {"west": (-10.0, 0.0, 0), "east": (10.0, 0.0, 0)}
    plain = _patrol(world, (-10.0, 0.0, 0.0, 0.0), stops, yaw_drift=yaw_drift, loops=False, limit_s=200.0)
    closed = _patrol(world, (-10.0, 0.0, 0.0, 0.0), stops, yaw_drift=yaw_drift, limit_s=200.0)
    assert _pose_error(closed) <= _pose_error(plain) + 0.3
