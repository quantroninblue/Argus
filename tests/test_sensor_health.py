"""Sensor data that keeps arriving but cannot be right: a frozen driver, a blinded sensor,
stamps that do not move forward. Argo refuses such frames, so the sensor goes stale and the
supervisor stops motion that only it covers; other sensors carry on."""

from pathlib import Path

import numpy as np

from argus import node as argo
from argus import profile as profiles
from argus.sim import world as worlds
from argus.sim.episode import SimOptions, Simulation

GOAL = [{"id": "goal", "position": (5.0, 2.0, 0.0), "tolerance_m": 0.4}]
BODIES = Path(__file__).parent / "bodies"


def _run(doc, options, duration=30.0):
    sim = Simulation(doc, worlds.wall_with_gap(), (0.0, 0.0, 0.0, 0.0), options)
    node = sim.node
    faults = {}

    def tick(t):
        if t == 0.0:
            node.set_plan(1, duration + 60.0, GOAL, 0.0)
        out = node.tick(t)
        faults.update(node.sensor_faults())
        return out

    trail = []
    while sim.t < duration:
        sim.step(tick)
        trail.append((sim.t, sim.plant.pos.copy()))
        if sim.result.complete and sim.speed < 0.01:
            break
    return sim, faults, trail


def _moved_after(trail, t0):
    pts = [p for t, p in trail if t >= t0]
    return sum(float(np.linalg.norm(b[:2] - a[:2])) for a, b in zip(pts, pts[1:]))


def test_a_frozen_lidar_on_a_lidar_only_robot_stops_it():
    doc = profiles.load(BODIES / "turtlebot3_waffle_class.yaml")
    sim, faults, trail = _run(doc, SimOptions(sensor_freezes_at_s={"scan": 2.0}))
    ep = sim.result
    assert not ep.collided
    assert not ep.complete
    assert "repeat" in faults.get("scan", ""), faults
    assert _moved_after(trail, 2.0) < 0.5, _moved_after(trail, 2.0)
    blocked = [e for e in ep.events if e["kind"] == "blocked" and e["cause"] == "sensor_invalid"]
    assert blocked and "scan" in blocked[0]["detail"], ep.events


def test_a_blinded_lidar_on_a_lidar_only_robot_stops_it():
    doc = profiles.load(BODIES / "turtlebot3_waffle_class.yaml")
    sim, faults, trail = _run(doc, SimOptions(sensor_blinded_at_s={"scan": 2.0}))
    assert not sim.result.collided
    assert not sim.result.complete
    assert "invalid" in faults.get("scan", ""), faults
    assert _moved_after(trail, 2.0) < 0.5


def test_a_frozen_depth_camera_is_refused_and_the_lidar_carries_on():
    # The differential example's lidar covers the way ahead too; its frozen camera's frames
    # are kept out of the map rather than stopping the robot.
    doc = profiles.load(profiles.EXAMPLES_DIR / "planar_differential.yaml")
    sim, faults, _ = _run(doc, SimOptions(sensor_freezes_at_s={"front_depth": 2.0}), 40.0)
    assert not sim.result.collided
    assert sim.result.complete, sim.result.final_pose
    assert "repeat" in faults.get("front_depth", ""), faults


def test_a_still_robot_seeing_identical_frames_is_not_called_frozen():
    # A noiseless sensor on a robot at rest repeats itself exactly, and that is fine.
    doc = profiles.load(BODIES / "turtlebot3_waffle_class.yaml")
    node = argo.Node(profiles.dump(doc), argo.NodeConfig())
    n = len(node.self_hits("scan"))
    ranges = np.full(n, 2.0, dtype=np.float32)
    for k in range(20):
        t = 0.1 * k
        node.on_odometry(t, [0.0, 0.0, 0.0], 0.0, [0.0, 0.0, 0.0], 0.0, True)
        assert node.on_range_frame(t, "scan", ranges)
    assert node.sensor_faults() == {}


def test_frames_whose_stamps_do_not_move_forward_or_run_ahead_are_refused():
    doc = profiles.load(BODIES / "turtlebot3_waffle_class.yaml")
    node = argo.Node(profiles.dump(doc), argo.NodeConfig())
    n = len(node.self_hits("scan"))
    ranges = np.full(n, 2.0, dtype=np.float32)
    node.on_odometry(1.0, [0.0, 0.0, 0.0], 0.0, [0.0, 0.0, 0.0], 0.0, True)
    assert node.on_range_frame(1.0, "scan", ranges)
    assert not node.on_range_frame(1.0, "scan", ranges)
    assert "forward" in node.sensor_faults()["scan"]
    assert not node.on_range_frame(2.0, "scan", ranges)
    assert "ahead" in node.sensor_faults()["scan"]
    node.on_odometry(1.05, [0.0, 0.0, 0.0], 0.0, [0.0, 0.0, 0.0], 0.0, True)
    assert node.on_range_frame(1.05, "scan", ranges)
    assert node.sensor_faults() == {}


def test_returns_are_placed_with_the_body_tilted_as_odometry_says():
    # A multirotor pitches nose down to speed up and its camera with it. Placed as if level,
    # a wall ahead lands far from where it is, and seems to move as the tilt changes.
    import math

    doc = profiles.load(BODIES / "x500_class.yaml")
    node = argo.Node(profiles.dump(doc), argo.NodeConfig())
    world = worlds.open_room().add((3.0, -2.0, 0.0), (3.2, 2.0, 3.0))
    pitch = 0.2  # nose down
    c, s = math.cos(pitch), math.sin(pitch)
    tilt = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    mount = np.asarray(node.sensor_mount("depth"))
    origin = np.array([0.0, 0.0, 1.5]) + tilt @ mount[:3, 3]
    dirs = np.asarray(node.ray_directions("depth")) @ (tilt @ mount[:3, :3]).T
    ranges = world.raycast(origin, dirs, 10.0).astype(np.float32)
    hit = np.isfinite(ranges)
    ends = origin + dirs[hit] * ranges[hit, None]
    on_wall = ends[(np.abs(ends[:, 0] - 3.0) < 1e-6) & (ends[:, 2] > 0.5) & (ends[:, 2] < 2.5)]
    assert len(on_wall)

    def wall_cells(odom_pitch):
        n = argo.Node(profiles.dump(doc), argo.NodeConfig())
        n.on_odometry(0.0, [0.0, 0.0, 1.5], 0.0, [0.0, 0.0, 0.0], 0.0, True, pitch=odom_pitch)
        assert n.on_range_frame(0.0, "depth", ranges)
        # The face is on a cell boundary: either cell beside it counts.
        return sum("occupied" in (n.cell((2.95, p[1], p[2])), n.cell((3.05, p[1], p[2]))) for p in on_wall[::5])

    told = wall_cells(pitch)
    level = wall_cells(0.0)
    assert told >= 0.9 * len(on_wall[::5]), (told, len(on_wall[::5]))
    assert level < 0.5 * told, (level, told)
