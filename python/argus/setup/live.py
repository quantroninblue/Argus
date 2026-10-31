"""Live checks on a running robot (setup step 3 and 4): every declared topic exists with the
right type and publishes at its declared rate, and the robot's controller stops the robot on
its own when commands stop.

The timeout test moves the robot: forward at a crawl for one second, then silence. It refuses
to start unless a range sensor shows clear space ahead, or `wheels_up` says the drive wheels
are off the ground. Explicit stops are always sent afterwards.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

ROS_TYPES = {
    "range_2d": ["sensor_msgs/msg/LaserScan"],
    "range_3d": ["sensor_msgs/msg/PointCloud2"],
    "depth": ["sensor_msgs/msg/Image"],
    "rgb": ["sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"],
    "imu": ["sensor_msgs/msg/Imu"],
    "wheel_odom": ["nav_msgs/msg/Odometry"],
    "odometry": ["nav_msgs/msg/Odometry"],
}
COMMAND_TYPES = ["geometry_msgs/msg/Twist", "geometry_msgs/msg/TwistStamped"]
# A topic passes when it publishes at least this share of its declared rate.
MIN_RATE_SHARE = 0.8
CRAWL_MPS = 0.1
CLEAR_AHEAD_M = 0.6
STOPPED_MPS = 0.02


@dataclass
class Check:
    name: str
    ok: bool
    detail: str


@dataclass
class LiveResult:
    checks: list[Check] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return all(c.ok for c in self.checks)

    def text(self) -> str:
        return "\n".join(f"{'pass' if c.ok else 'FAIL'}  {c.name}: {c.detail}" for c in self.checks)


class _Probe:
    def __init__(self, sim_time: bool) -> None:
        self.node = rclpy.create_node("argus_setup_check")
        if sim_time:
            self.node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.lock = threading.Lock()

    def now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def close(self) -> None:
        self.executor.shutdown()
        self.thread.join(5.0)
        self.node.destroy_node()


def _wait(probe: _Probe, seconds: float) -> None:
    """Waits `seconds` of the node's clock (simulated time when that is in use)."""
    start = probe.now()
    deadline = time.monotonic() + 10 * seconds + 10
    while probe.now() - start < seconds and time.monotonic() < deadline:
        time.sleep(0.02)


def _topic_checks(probe: _Probe, doc: dict, duration_s: float, result: LiveResult) -> dict:
    graph = dict(probe.node.get_topic_names_and_types())
    stats = {}
    for s in doc.get("sensors", []):
        if s["source"]["transport"] != "ros2" or s["kind"] not in ROS_TYPES:
            continue
        topic = s["source"]["endpoint"]
        types = graph.get(topic)
        if not types:
            result.checks.append(Check(f"{s['id']} topic", False, f"{topic} does not exist"))
            continue
        wanted = ROS_TYPES[s["kind"]]
        if not set(types) & set(wanted):
            result.checks.append(Check(f"{s['id']} topic", False, f"{topic} is {', '.join(types)}, "
                                       f"expected {' or '.join(wanted)}"))
            continue
        result.checks.append(Check(f"{s['id']} topic", True, f"{topic} ({types[0]})"))
        st = {"times": [], "lags": [], "spec": s}
        stats[topic] = st

        def on_msg(msg, st=st):
            now = probe.now()
            with probe.lock:
                st["times"].append(now)
                header = getattr(msg, "header", None)
                if header is not None and (header.stamp.sec or header.stamp.nanosec):
                    st["lags"].append(now - (header.stamp.sec + 1e-9 * header.stamp.nanosec))

        probe.node.create_subscription(get_message(next(t for t in types if t in wanted)), topic, on_msg,
                                       qos_profile_sensor_data)

    control = doc["control"]
    topic = control["endpoint"]
    types = graph.get(topic, [])
    subscribers = probe.node.count_subscribers(topic)
    if control["transport"] == "ros2":
        if subscribers == 0:
            result.checks.append(Check("command topic", False, f"nothing subscribes to {topic}"))
        elif types and not set(types) & set(COMMAND_TYPES):
            result.checks.append(Check("command topic", False, f"{topic} is {', '.join(types)}"))
        else:
            result.checks.append(Check("command topic", True, f"{topic}, {subscribers} subscriber(s)"))

    _wait(probe, duration_s)
    for topic, st in stats.items():
        s = st["spec"]
        with probe.lock:
            times = list(st["times"])
            lags = list(st["lags"])
        name = f"{s['id']} rate"
        if len(times) < 2:
            result.checks.append(Check(name, False, f"{len(times)} message(s) in {duration_s:g} s"))
            continue
        rate = (len(times) - 1) / (times[-1] - times[0])
        gap = max(b - a for a, b in zip(times, times[1:]))
        detail = f"{rate:.1f} Hz (declared {s['rate_hz']:g}), longest gap {gap:.3f} s"
        if lags:
            detail += f", stamp lag {sum(lags) / len(lags) * 1000:.0f} ms mean, {max(lags) * 1000:.0f} ms max"
        ok = rate >= MIN_RATE_SHARE * s["rate_hz"]
        result.checks.append(Check(name, ok, detail if ok else detail + ": too slow"))
        max_age = s.get("max_age_s", 3.0 / s["rate_hz"])
        if lags and max(lags) > max_age:
            result.checks.append(Check(f"{s['id']} age", False, f"messages up to {max(lags):.3f} s old, "
                                       f"limit {max_age:.3f} s"))
    return stats


def _clear_ahead(probe: _Probe, doc: dict, stats: dict) -> float | None:
    """Nearest return within 20 degrees of forward on any 2D scan, or None without one."""
    best = None
    for topic, st in stats.items():
        if st["spec"]["kind"] != "range_2d":
            continue
        got = []
        sub = probe.node.create_subscription(get_message("sensor_msgs/msg/LaserScan"), topic, got.append,
                                             qos_profile_sensor_data)
        _wait(probe, 0.5)
        probe.node.destroy_subscription(sub)
        if not got:
            continue
        scan = got[-1]
        frame = doc["geometry"].get("frames", {}).get(st["spec"]["frame"], {})
        mount_yaw = frame.get("rpy_rad", [0, 0, 0])[2]
        for i, r in enumerate(scan.ranges):
            a = mount_yaw + scan.angle_min + i * scan.angle_increment
            if abs(math.remainder(a, 2 * math.pi)) <= math.radians(20) and scan.range_min <= r <= scan.range_max:
                best = r if best is None else min(best, r)
        best = best if best is not None else scan.range_max
    return best


def _timeout_check(probe: _Probe, doc: dict, wheels_up: bool, stats: dict, result: LiveResult) -> None:
    control = doc["control"]
    odom = next((s for s in doc["sensors"] if s["kind"] == "odometry"), None) or next(
        (s for s in doc["sensors"] if s["kind"] == "wheel_odom"), None)
    if odom is None:
        result.checks.append(Check("controller timeout", False, "no odometry to watch the robot stop"))
        return
    if not wheels_up:
        clear = _clear_ahead(probe, doc, stats)
        if clear is None:
            result.checks.append(Check("controller timeout", False, "not run: no 2D scan to confirm space "
                                       "ahead; lift the drive wheels and pass --wheels-up"))
            return
        if clear < CLEAR_AHEAD_M:
            result.checks.append(Check("controller timeout", False, f"not run: only {clear:.2f} m clear ahead, "
                                       f"need {CLEAR_AHEAD_M} m"))
            return

    speeds = []
    sub = probe.node.create_subscription(
        get_message("nav_msgs/msg/Odometry"), odom["source"]["endpoint"],
        lambda m: speeds.append((probe.now(), math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y))),
        qos_profile_sensor_data)
    twist = get_message("geometry_msgs/msg/Twist")
    pub = probe.node.create_publisher(twist, control["endpoint"], 10)
    period = 1.0 / control["rate_hz"]
    timeout = control["controller_timeout_s"]
    decel = doc["motion"]["limits"]["x"].get("decel_mps2", 1.0)
    allowed = timeout + control.get("latency_s", 0.0) + CRAWL_MPS / decel + 0.15
    try:
        cmd = twist()
        cmd.linear.x = CRAWL_MPS
        start = probe.now()
        while probe.now() - start < 1.0:
            pub.publish(cmd)
            _wait(probe, period)
        moving = [v for t, v in speeds if t > start + 0.5]
        if not moving or max(moving) < 0.5 * CRAWL_MPS:
            result.checks.append(Check("controller timeout", False, "the robot did not move on a "
                                       f"{CRAWL_MPS} m/s command; check the command topic and that it is armed"))
            return
        silent_at = probe.now()
        _wait(probe, allowed + 1.0)
        after = [(t, v) for t, v in speeds if t > silent_at]
        stopped_at = next((t for t, v in after if v < STOPPED_MPS), None)
        if stopped_at is None:
            result.checks.append(Check("controller timeout", False, f"still moving {allowed + 1.0:.1f} s after "
                                       "commands stopped: the controller has no timeout, Argo cannot rely on it"))
        else:
            took = stopped_at - silent_at
            ok = took <= allowed
            result.checks.append(Check("controller timeout", ok, f"stopped {took:.2f} s after the last command "
                                       f"(declared timeout {timeout:g} s, allowed {allowed:.2f} s)"))
    finally:
        stop = twist()
        for _ in range(5):
            pub.publish(stop)
            time.sleep(0.02)
        probe.node.destroy_publisher(pub)
        probe.node.destroy_subscription(sub)


def live_check(doc: dict, duration_s: float = 5.0, motion: bool = False, wheels_up: bool = False,
               sim_time: bool = False) -> LiveResult:
    """Runs against the ROS 2 graph this process can see. Needs rclpy initialised."""
    result = LiveResult()
    probe = _Probe(sim_time)
    try:
        time.sleep(1.5)  # discovery, in wall time: simulated time may not be running yet
        stats = _topic_checks(probe, doc, duration_s, result)
        if motion:
            _timeout_check(probe, doc, wheels_up, stats, result)
    finally:
        probe.close()
    return result
