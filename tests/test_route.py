from pathlib import Path

import numpy as np

from argus import sitemap
from argus.sim.world import wall_with_gap

EXAMPLES = Path(__file__).resolve().parents[1] / "schemas" / "examples"


def _profile(name):
    return (EXAMPLES / name).read_text()


def _scan(world, m, origin, stamp, rays=1440, max_range=8.0, elevations=(0.0,)):
    for el in elevations:
        a = np.linspace(-np.pi, np.pi, rays, endpoint=False)
        dirs = np.stack([np.cos(a) * np.cos(el), np.sin(a) * np.cos(el), np.full_like(a, np.sin(el))], axis=1)
        r = world.raycast(np.asarray(origin), dirs, max_range)
        hits = np.isfinite(r)
        ends = np.asarray(origin) + dirs * np.where(hits, r, max_range)[:, None]
        m.integrate(origin, ends, hits.tolist(), stamp)


def _distance_to_boxes_xy(world, p):
    lo, hi = world.boxes[:, 0:2], world.boxes[:, 3:5]
    d = np.maximum(np.maximum(lo - p[:2], p[:2] - hi), 0.0)
    return np.sqrt((d ** 2).sum(axis=1)).min()


def test_planar_route_goes_through_the_gap_and_marks_the_unseen_side():
    world = wall_with_gap()
    m = sitemap.SiteMap("site-test")
    for t, o in enumerate([(-3.0, -2.0, 0.3), (-1.0, 2.0, 0.3), (1.0, 0.0, 0.3)]):
        _scan(world, m, o, float(t))
    planner = sitemap.RoutePlanner(_profile("planar_differential.yaml"))
    # Scans come from 0.3 m; routes start at the base, which this profile puts on the floor.
    route = planner.plan(m, (-3.0, -2.0, 0.0), (5.0, -2.0, 0.0))
    assert route["reached"]
    path = route["path"]
    crossing = path[np.argmin(np.abs(path[:, 0] - 2.6))]
    assert 1.0 < crossing[1] < 3.0, "route must use the gap"
    for p in path:
        assert _distance_to_boxes_xy(world, p) >= planner.min_clearance_m - 0.1
    wps = route["waypoints"]
    legs = np.diff(np.array([(-3.0, -2.0, 0.0)] + [w["position"] for w in wps]), axis=0)
    assert np.linalg.norm(legs, axis=1).max() <= 3.0 + 1e-6
    assert not wps[0]["speculative"], "the first leg is in observed free space"
    assert wps[-1]["speculative"], "the far side of the wall was never seen"
    assert 0.0 < route["unknown_m"] < route["length_m"]


def test_unexplored_site_gives_a_straight_speculative_route():
    m = sitemap.SiteMap("site-test")
    planner = sitemap.RoutePlanner(_profile("planar_omnidirectional.yaml"))
    route = planner.plan(m, (0.0, 0.0, 0.2), (6.0, 0.0, 0.2))
    assert route["reached"]
    assert all(w["speculative"] for w in route["waypoints"])
    assert abs(route["length_m"] - 6.0) < 0.3


def test_goal_behind_a_closed_wall_is_not_reached():
    world = wall_with_gap(gap_y=(20.0, 21.0))  # gap outside the room: the wall is closed
    m = sitemap.SiteMap("site-test")
    for t, o in enumerate([(-3.0, -4.0, 0.3), (-3.0, 0.0, 0.3), (-3.0, 4.0, 0.3), (0.0, 0.0, 0.3)]):
        _scan(world, m, o, float(t))
    planner = sitemap.RoutePlanner(_profile("planar_differential.yaml"))
    route = planner.plan(m, (-3.0, 0.0, 0.0), (5.0, 0.0, 0.0), margin_m=1.0)
    # The outer walls close the room too, so the only ways round are through unknown space
    # outside it, which the scans saw as walls. Nothing reaches the goal.
    assert not route["reached"]
    assert route["path"][-1][0] < 2.5


def test_aerial_route_keeps_its_radius_in_3d():
    world = wall_with_gap()
    m = sitemap.SiteMap("site-test")
    for t, o in enumerate([(-3.0, -2.0, 1.0), (-1.0, 2.0, 1.0), (1.0, 0.0, 1.0)]):
        _scan(world, m, o, float(t), rays=720, elevations=np.radians([-30, -15, 0, 15, 30]))
    planner = sitemap.RoutePlanner(_profile("aerial_multirotor.yaml"))
    assert not planner.planar
    route = planner.plan(m, (-3.0, -2.0, 1.0), (5.0, -2.0, 1.0))
    assert route["reached"]
    for p in route["path"]:
        if p[2] < 2.0:  # below the top of the walls
            assert _distance_to_boxes_xy(world, p) >= planner.min_clearance_m - 0.2


def test_blocked_zone_diverts_the_route_but_never_traps_the_start():
    m = sitemap.SiteMap("site-test")
    planner = sitemap.RoutePlanner(_profile("planar_omnidirectional.yaml"))
    straight = planner.plan(m, (0.0, 0.0, 0.0), (6.0, 0.0, 0.0))
    around = planner.plan(m, (0.0, 0.0, 0.0), (6.0, 0.0, 0.0), blocked=[(3.0, 0.0, 0.0, 1.0)])
    assert around["reached"]
    assert around["length_m"] > straight["length_m"] + 0.3
    assert min(np.hypot(p[0] - 3.0, p[1]) for p in around["path"]) > 1.0
    inside = planner.plan(m, (3.0, 0.0, 0.0), (6.0, 0.0, 0.0), blocked=[(3.0, 0.0, 0.0, 1.0)])
    assert inside["reached"]


def test_waypoints_leave_room_to_turn_where_the_path_has_it():
    world = wall_with_gap()
    m = sitemap.SiteMap("site-test")
    for t, o in enumerate([(-3.0, -2.0, 0.3), (-1.0, 2.0, 0.3), (1.0, 0.0, 0.3), (1.5, 2.0, 0.3)]):
        _scan(world, m, o, float(t))
    planner = sitemap.RoutePlanner(_profile("planar_differential.yaml"))
    route = planner.plan(m, (0.0, 0.0, 0.0), (5.0, 2.0, 0.0))
    assert route["reached"]
    # The gap is 2 m wide, so every waypoint short of the goal can keep turning room, even
    # where the shortest path cuts close to the wall's end.
    for w in route["waypoints"][:-1]:
        assert _distance_to_boxes_xy(world, np.asarray(w["position"])) >= planner.turn_clearance_m - 0.1
