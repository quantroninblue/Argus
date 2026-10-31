"""Runs one body in Gazebo with the real Argo binary and a real hub, judged from the true pose.

Everything a robot run has, except the robot: Gazebo simulates the body and its sensors, the
ros_gz bridge carries them to the profile's ROS 2 topics, `argo` drives through its ROS 2
driver, and a hub on this machine plans over gRPC with mutual TLS. Simulated time is the clock
for all of them. Arrival and collisions are judged from Gazebo's true pose against the world's
boxes, the same rules the headless simulator uses.

Needs a sourced ROS 2 Jazzy environment and a build with the ROS 2 driver (tools/check.sh).
"""

from __future__ import annotations

import os
import random
import signal
import subprocess
import tempfile
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock

from argus import certs
from argus import profile as profiles
from argus.hub import Hub, MissionState, Registry
from argus.hub.server import HubServer, TlsFiles
from argus.sim.episode import _Judge
from argus.sim.grpc_episode import _Background
from argus.sim.world import World

from .model import model_sdf
from .world import bridge_yaml, world_sdf

ROOT = Path(__file__).resolve().parents[3]
NODE_ID = "gazebo-robot"
SITE_ID = "gazebo-site"


@dataclass
class GazeboEpisode:
    goal_reached: bool = False
    reached_at_s: float | None = None
    mission_state: str = ""
    collided: bool = False
    final_pose: tuple = ()
    time_s: float = 0.0
    workdir: str = ""
    trace: list = field(default_factory=list)  # (t, x, y, z, yaw) from the true pose


class _Watcher:
    """Simulated time and the true pose, from ROS 2 topics."""

    def __init__(self) -> None:
        self.node = rclpy.create_node("argus_gazebo_watcher")
        self.now = 0.0
        self.pose = None
        self.lock = threading.Lock()
        self.node.create_subscription(Clock, "/clock", self._clock, 10)
        self.node.create_subscription(Odometry, "/argus_truth", self._truth, qos_profile_sensor_data)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _clock(self, m: Clock) -> None:
        self.now = m.clock.sec + 1e-9 * m.clock.nanosec

    def _truth(self, m: Odometry) -> None:
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = float(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        with self.lock:
            self.pose = (m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, p.x, p.y, p.z, yaw)

    def close(self) -> None:
        self.executor.shutdown()
        self.node.destroy_node()


def _start(cmd: list[str], log: Path, env: dict) -> subprocess.Popen:
    return subprocess.Popen(cmd, stdout=log.open("w"), stderr=subprocess.STDOUT, env=env, start_new_session=True)


def _stop(p: subprocess.Popen, sig=signal.SIGINT, wait_s: float = 5.0) -> None:
    if p.poll() is not None:
        return
    try:
        os.killpg(p.pid, sig)
        p.wait(wait_s)
    except (subprocess.TimeoutExpired, ProcessLookupError):
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def run_gazebo_episode(doc: dict, world: World, start, goal, duration_s: float, tolerance_m: float = 0.4,
                       gui: bool = False, workdir: str | None = None) -> GazeboEpisode:
    """Sends the body from `start` (x, y, z, yaw) to `goal` through the hub, `duration_s` of
    simulated time at most. With `gui`, Gazebo's window shows the run."""
    work = Path(workdir or tempfile.mkdtemp(prefix="argus-gazebo-"))
    work.mkdir(parents=True, exist_ok=True)
    out = GazeboEpisode(workdir=str(work))

    model, topics = model_sdf(doc, "robot", start)
    (work / "world.sdf").write_text(world_sdf(world, [model]))
    (work / "bridge.yaml").write_text(bridge_yaml(doc, topics))
    (work / "profile.yaml").write_text(profiles.dump(doc))
    certs.make_ca(work)
    certs.make_hub(work, ["localhost", "127.0.0.1"])
    certs.make_node(work, NODE_ID)

    registry = Registry()
    profile = registry.add_profile(doc)
    registry.add_robot(NODE_ID, profile.id, SITE_ID)
    hub = Hub(registry)

    # Keep this run's topics to itself.
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(random.randint(100, 200))
    env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    env["GZ_PARTITION"] = f"argus-{os.getpid()}"
    os.environ.update({k: env[k] for k in ("ROS_DOMAIN_ID", "ROS_AUTOMATIC_DISCOVERY_RANGE", "GZ_PARTITION")})

    rclpy.init()
    watcher = _Watcher()
    bg = _Background()
    server = HubServer(hub, tls=TlsFiles(work / "ca.pem", work / "hub.pem", work / "hub.key"),
                       clock=lambda: watcher.now)
    port = bg.run(server.start())
    (work / "argo.yaml").write_text(
        f"node_id: {NODE_ID}\nprofile: profile.yaml\nprofile_hash: {profile.hash}\n"
        f"hub:\n  target: 127.0.0.1:{port}\n  server_name: localhost\n"
        f"  ca: ca.pem\n  cert: {NODE_ID}.pem\n  key: {NODE_ID}.key\n")

    argo_bin = os.environ.get("ARGO_BIN", str(ROOT / "build" / "ros" / "argo_ros" / "argo"))
    procs = []
    try:
        procs.append(_start(["gz", "sim", "-s", "-r", "--headless-rendering", str(work / "world.sdf")],
                            work / "gz.log", env))
        if gui:
            procs.append(_start(["gz", "sim", "-g"], work / "gz_gui.log", env))
        procs.append(_start(["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args",
                             "-p", f"config_file:={work / 'bridge.yaml'}", "-p", "use_sim_time:=true"],
                            work / "bridge.log", env))
        procs.append(_start([argo_bin, "--config", str(work / "argo.yaml"), "--ros-args",
                             "-p", "use_sim_time:=true"], work / "argo.log", env))

        judge = _Judge(profiles.derive(doc), doc, 0.02)
        wall_limit = time.monotonic() + 3 * duration_s + 60.0
        goal_set = False
        last = None
        while time.monotonic() < wall_limit:
            time.sleep(0.02)
            if any(p.poll() is not None for p in procs):
                raise RuntimeError(f"a process exited early; logs in {work}")
            with watcher.lock:
                pose = watcher.pose
            if pose is None or pose == last:
                continue
            last = pose
            t, x, y, z, yaw = pose
            out.trace.append(pose)
            if judge.collides(world, np.array([x, y, z]), yaw):
                out.collided = True
            if not goal_set:
                hub.set_goal(NODE_ID, tuple(goal), tolerance_m, watcher.now)
                goal_set = True
            state = hub.mission_state(NODE_ID)
            if out.reached_at_s is None and state == MissionState.REACHED:
                out.reached_at_s = t
            if state in (MissionState.REACHED, MissionState.FAILED) and t > (out.reached_at_s or t) + 1.0:
                break
            if t >= duration_s:
                break
        out.mission_state = hub.mission_state(NODE_ID).value
        if out.trace:
            t, x, y, z, yaw = out.trace[-1]
            out.final_pose = (x, y, z, yaw)
            out.time_s = t
            out.goal_reached = float(np.hypot(x - goal[0], y - goal[1])) <= tolerance_m + 0.05
        return out
    finally:
        for p in reversed(procs):
            _stop(p)
        bg.run(server.stop())
        bg.close()
        watcher.close()
        rclpy.shutdown()
