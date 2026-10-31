"""A simulated robot: Gazebo with the body's model, the ros_gz bridge to the profile's ROS 2
topics, and a motor controller that stops the wheels when commands stop, as robot firmware
does (Gazebo's drive systems hold the last command forever).

What setup sees here is what it would see on a real robot: the profile's sensor, odometry and
command topics, nothing else (plus `/clock` and `/argus_truth` for judging runs).
"""

from __future__ import annotations

import os
import random
import subprocess
import threading
from dataclasses import dataclass, field
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from argus import profile as profiles
from argus.sim.world import World

from . import procs
from .model import Topics, model_sdf
from .procs import start_process, stop_process
from .world import bridge_entries, bridge_yaml, world_sdf

# The bridge carries commands to Gazebo from here; the motor controller feeds it.
DRIVE_TOPIC = "/argus_gazebo/drive"


def isolated_env() -> dict:
    """Environment that keeps one run's ROS 2 and Gazebo traffic to itself, applied to this
    process too. Gazebo's discovery otherwise goes out on the first network interface, and on
    a Wi-Fi laptop the bridge never finds the server."""
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(random.randint(100, 200))
    env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    env["GZ_PARTITION"] = f"argus-{os.getpid()}-{random.randint(0, 1 << 30)}"
    env["GZ_IP"] = "127.0.0.1"
    os.environ.update({k: env[k] for k in ("ROS_DOMAIN_ID", "ROS_AUTOMATIC_DISCOVERY_RANGE", "GZ_PARTITION", "GZ_IP")})
    return env




class MotorController:
    """Forwards the profile's command topic to Gazebo and sends zero once no command has
    arrived for `controller_timeout_s` (simulated time). With `timeout` False it never stops
    the wheels by itself, like a controller with no timeout."""

    def __init__(self, doc: dict, timeout: bool = True) -> None:
        c = doc["control"]
        self.timeout_s = c["controller_timeout_s"] if timeout else None
        self.node = rclpy.create_node("argus_gazebo_motor_controller")
        self.node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.pub = self.node.create_publisher(Twist, DRIVE_TOPIC, 10)
        self.last = None
        self.stopped = True
        self.node.create_subscription(Twist, c["endpoint"], self._on_command, 10)
        self.node.create_timer(1.0 / c["rate_hz"], self._tick)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _on_command(self, m: Twist) -> None:
        self.last = self._now()
        self.stopped = False
        self.pub.publish(m)

    def _tick(self) -> None:
        if self.timeout_s is None or self.last is None:
            return
        if self._now() - self.last > self.timeout_s:
            self.pub.publish(Twist())
            self.stopped = True

    def close(self) -> None:
        self.executor.shutdown()
        self.thread.join(5.0)
        self.node.destroy_node()


@dataclass
class GazeboRobot:
    model: str
    topics: Topics
    entries: list
    procs: list = field(default_factory=list)
    controller: MotorController | None = None
    # Set when the robot was started outside a run that already reaps its processes.
    reaping: object | None = None

    def alive(self) -> bool:
        return all(p.poll() is None for p in self.procs)

    def stop(self) -> None:
        for p in reversed(self.procs):
            stop_process(p)
        self.procs.clear()
        if self.controller is not None:
            self.controller.close()
            self.controller = None
        if self.reaping is not None:
            self.reaping.__exit__(None, None, None)
            self.reaping = None


def start_robot(doc: dict, world: World, start, work: Path, env: dict, gui: bool = False,
                controller_timeout: bool = True, odom_scale: float = 1.0) -> GazeboRobot:
    """Starts Gazebo, the bridge and the motor controller. Needs rclpy initialised."""
    model, topics = model_sdf(doc, "robot", start, odom_scale=odom_scale)
    entries = bridge_entries(doc, topics, command_topic=DRIVE_TOPIC)
    (work / "world.sdf").write_text(world_sdf(world, [model]))
    (work / "bridge.yaml").write_text(bridge_yaml(entries))
    robot = GazeboRobot(model=model, topics=topics, entries=entries)
    if procs._active is None:
        robot.reaping = procs.reaping(work)
        robot.reaping.__enter__()
    robot.procs.append(start_process(["gz", "sim", "-s", "-r", "--headless-rendering", str(work / "world.sdf")],
                                     work / "gz.log", env))
    if gui:
        robot.procs.append(start_process(["gz", "sim", "-g"], work / "gz_gui.log", env))
    robot.procs.append(start_process(["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args",
                                      "-p", f"config_file:={work / 'bridge.yaml'}", "-p", "use_sim_time:=true"],
                                     work / "bridge.log", env))
    robot.controller = MotorController(doc, timeout=controller_timeout)
    return robot


def start_argo_setup(doc: dict, work: Path, env: dict, argo_bin: str) -> subprocess.Popen:
    """Argo in setup mode beside a simulated robot: no hub, motion only on setup requests."""
    (work / "setup_profile.yaml").write_text(profiles.dump(doc))
    (work / "argo_setup.yaml").write_text(f"node_id: setup\nprofile: setup_profile.yaml\n"
                                          f"profile_hash: {profiles.profile_hash(doc)}\n")
    return start_process([argo_bin, "--config", str(work / "argo_setup.yaml"), "--setup", "--ros-args",
                          "-p", "use_sim_time:=true"], work / "argo_setup.log", env)

