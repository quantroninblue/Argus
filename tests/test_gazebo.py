"""Bodies driven through Gazebo by the real Argo binary and hub. Needs ROS 2 Jazzy sourced and a
build with the ROS 2 driver; skipped otherwise."""

import copy
import json
import os
import shutil
from pathlib import Path

import pytest

from argus import profile as profiles
from argus.hub.runlogs import RunLogStore
from argus.sim import world as worlds

ROOT = Path(__file__).resolve().parents[1]
ARGO = Path(os.environ.get("ARGO_BIN", ROOT / "build" / "ros" / "argo_ros" / "argo"))

rclpy = pytest.importorskip("rclpy", reason="ROS 2 is not sourced")
if shutil.which("gz") is None or not ARGO.is_file():
    pytest.skip("Gazebo or the argo binary is missing", allow_module_level=True)

from argus.gazebo.run import run_gazebo_episode  # noqa: E402

GOAL = (5.0, 2.0, 0.0)


def _example(name):
    return profiles.load(ROOT / "schemas" / "examples" / f"{name}.yaml")


def _omnidirectional():
    # The example has no odometry source, which a ROS 2 robot needs; a mecanum base reports
    # wheel odometry.
    doc = _example("planar_omnidirectional")
    doc["sensors"].append({"id": "wheel_odom", "kind": "wheel_odom", "frame": "imu",
                           "source": {"transport": "ros2", "endpoint": "/odom"}, "rate_hz": 50})
    return doc


def _car_like():
    doc = copy.deepcopy(_example("planar_differential"))
    doc["id"] = "test-car-like"
    doc["motion"]["min_turn_radius_m"] = 0.8
    doc["motion"]["limits"]["yaw"]["rate_radps"] = 0.6
    return doc


# Reaches about 3 runs in 4: it sometimes stops before the gap with standing still scoring best.
CAR_LIKE_MISSES = pytest.mark.xfail(reason="car-like tracking under turn mismatch, see needed_fixes", strict=False)


@pytest.mark.parametrize("make", [lambda: _example("planar_differential"), _omnidirectional,
                                  pytest.param(_car_like, marks=CAR_LIKE_MISSES)],
                         ids=["differential", "omnidirectional", "car_like"])
def test_body_reaches_the_goal_behind_the_wall_without_touching_it(make, tmp_path):
    ep = run_gazebo_episode(make(), worlds.wall_with_gap(), (0.0, 0.0, 0.0, 0.0), GOAL, 90.0,
                            workdir=str(tmp_path))
    assert not ep.collided, ep.final_pose
    assert ep.goal_reached, (ep.mission_state, ep.final_pose, ep.time_s)


def test_run_log_holds_the_whole_run_and_no_keys(tmp_path):
    store = RunLogStore(tmp_path / "store")
    doc = _example("planar_differential")
    ep = run_gazebo_episode(doc, worlds.wall_with_gap(), (0.0, 0.0, 0.0, 0.0), GOAL, 90.0, log_store=store,
                            record_bag=True)
    assert ep.goal_reached
    assert store.ids() == [ep.run_id]
    log = store.consume(ep.run_id)
    assert store.ids() == []

    for name in ("run.json", "world.sdf", "model.sdf", "bridge.yaml", "profile.yaml", "trajectory.csv",
                 "commands.csv", "odometry.csv", "argo.log"):
        assert log[name], name
    assert not [n for n in log if n.endswith((".key", ".pem", ".csr"))]
    assert "bag" in log

    run = json.loads(log["run.json"])
    assert run["outcome"]["goal_reached"] and run["outcome"]["collisions"] == []
    assert run["body"]["drivetrain"]["kind"] == "differential"
    scan = run["topics"]["/scan"]
    assert scan["declared_rate_hz"] == 15 and abs(scan["rate_hz"] - 15) < 2
    cmd = run["topics"][doc["control"]["endpoint"]]
    assert cmd["messages"] > 100
    assert len(log["commands.csv"].splitlines()) == cmd["messages"] + 1
    assert run["argo"]["events"]["waypoint_reached"] >= 1


def test_setup_check_passes_on_a_simulated_robot_and_catches_a_missing_timeout(tmp_path):
    import rclpy

    from argus.gazebo.robot import isolated_env, start_argo_setup, start_robot, stop_process
    from argus.setup.live import live_check

    doc = _example("planar_differential")
    results = {}
    for timeout in (True, False):
        work = tmp_path / f"timeout-{timeout}"
        work.mkdir()
        env = isolated_env()
        rclpy.init()
        robot = start_robot(doc, worlds.open_room(), (0.0, 0.0, 0.0, 0.0), work, env, controller_timeout=timeout)
        argo = start_argo_setup(doc, work, env, str(ARGO))
        try:
            results[timeout] = live_check(doc, duration_s=3.0, motion=True, sim_time=True)
        finally:
            stop_process(argo)
            robot.stop()
            rclpy.shutdown()
    good, bad = results[True], results[False]
    assert good.ok, good.text()
    names = {c.name for c in good.checks}
    assert {"front_lidar rate", "imu rate", "wheel_odom rate", "command topic", "controller timeout"} <= names
    timeout_check = next(c for c in bad.checks if c.name == "controller timeout")
    assert not timeout_check.ok and "no timeout" in timeout_check.detail, bad.text()


def _calibrate_in_gazebo(true_doc, declared_doc, work):
    import rclpy

    from argus.gazebo.robot import isolated_env, start_argo_setup, start_robot, stop_process
    from argus.setup.calibrate import calibrate

    env = isolated_env()
    rclpy.init()
    robot = start_robot(true_doc, worlds.open_room(), (-5.0, 0.0, 0.0, 0.0), work, env)
    argo = start_argo_setup(declared_doc, work, env, str(ARGO))
    try:
        return calibrate(declared_doc, sim_time=True)
    finally:
        stop_process(argo)
        robot.stop()
        rclpy.shutdown()


def test_calibration_catches_optimistic_braking_and_a_wider_turn(tmp_path):
    true_doc = _example("planar_differential")
    true_doc["motion"]["limits"]["x"]["decel_mps2"] = 1.0
    declared = copy.deepcopy(true_doc)
    declared["motion"]["limits"]["x"]["decel_mps2"] = 2.0
    (tmp_path / "diff").mkdir()
    cal = _calibrate_in_gazebo(true_doc, declared, tmp_path / "diff")
    assert cal.problems == [], cal.text()
    got = {m.field: m for m in cal.measurements}
    lx = cal.profile["motion"]["limits"]["x"]
    assert 0.8 <= lx["decel_mps2"] <= 1.2, cal.text()
    assert cal.profile["provenance"]["motion.limits.x.decel_mps2"] == "calibrated"
    assert lx["velocity_mps"][1] == pytest.approx(true_doc["motion"]["limits"]["x"]["velocity_mps"][1], rel=0.1)
    assert "motion.limits.yaw.rate_radps" in got
    assert profiles.problems(cal.profile) == []

    car = _car_like()
    (tmp_path / "car").mkdir()
    cal = _calibrate_in_gazebo(car, car, tmp_path / "car")
    assert cal.problems == [], cal.text()
    assert cal.profile["motion"]["min_turn_radius_m"] > 0.85, cal.text()


def test_argo_refuses_a_setup_motion_into_a_wall(tmp_path):
    import rclpy

    from argus.gazebo.robot import isolated_env, start_argo_setup, start_robot, stop_process
    from argus.setup.live import live_check

    doc = _example("planar_differential")
    world = worlds.open_room()
    # The body's front is at 0.30 m and the one second crawl covers 0.10 m, so a wall at 0.38 m
    # would be touched.
    world.add((0.38, -1.0, 0.0), (0.58, 1.0, 1.0))
    env = isolated_env()
    rclpy.init()
    robot = start_robot(doc, world, (0.0, 0.0, 0.0, 0.0), tmp_path, env)
    argo = start_argo_setup(doc, tmp_path, env, str(ARGO))
    try:
        # wheels_up skips the tool's own clearance check, so only Argo stands in the way.
        result = live_check(doc, duration_s=2.0, motion=True, wheels_up=True, sim_time=True)
    finally:
        stop_process(argo)
        robot.stop()
        rclpy.shutdown()
    check = next(c for c in result.checks if c.name == "controller timeout")
    assert not check.ok and "supervisor" in check.detail, result.text()

