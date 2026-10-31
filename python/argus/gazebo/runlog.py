"""A detailed log of one Gazebo run, for the hub's models and for people debugging a body.

The log is a directory:

- `run.json`: outcome, start and goal, timing, collisions, the drivetrain made up for the body,
  the world's boxes, a summary of every ROS 2 topic (messages, measured and declared rate,
  header stamp lag, longest gap) and Argo's event and flag counts.
- `world.sdf`, `model.sdf`, `bridge.yaml`, `profile.yaml`: exactly what was simulated.
- `trajectory.csv`: the true pose from Gazebo; `odometry.csv`: the robot's own odometry
  (10 Hz), so drift shows against the truth; `commands.csv`: every command Argo published.
- `argo.log`, `gz.log`, `bridge.log`: the processes' own output.
- `bag/` (optional): an mcap recording of every bridged ROS 2 topic.

Certificates and keys from the run's work directory are never copied in.
"""

from __future__ import annotations

import csv
import json
import math
import shutil
import threading
from functools import partial
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

ODOM_LOG_PERIOD_S = 0.1


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class _TopicStats:
    def __init__(self, ros_type: str) -> None:
        self.type = ros_type
        self.count = 0
        self.first = None
        self.last = None
        self.max_gap = 0.0
        self.lag_sum = 0.0
        self.lag_max = 0.0
        self.lag_count = 0

    def add(self, now: float, stamp: float | None) -> None:
        if self.last is not None:
            self.max_gap = max(self.max_gap, now - self.last)
        if self.first is None:
            self.first = now
        self.last = now
        self.count += 1
        if stamp is not None and stamp > 0.0:
            lag = now - stamp
            self.lag_sum += lag
            self.lag_max = max(self.lag_max, lag)
            self.lag_count += 1

    def summary(self) -> dict:
        span = (self.last - self.first) if self.count > 1 else 0.0
        out = {"type": self.type, "messages": self.count,
               "rate_hz": round((self.count - 1) / span, 2) if span > 0 else None,
               "first_s": self.first, "last_s": self.last, "longest_gap_s": round(self.max_gap, 4)}
        if self.lag_count:
            out["stamp_lag_mean_s"] = round(self.lag_sum / self.lag_count, 4)
            out["stamp_lag_max_s"] = round(self.lag_max, 4)
        return out


class RosRecorder:
    """Subscribes to every bridged ROS 2 topic for the length of a run. Commands are kept in
    full, the robot's odometry at 10 Hz, everything else as statistics."""

    def __init__(self, entries, command_topic: str, odom_topic: str, clock) -> None:
        self.clock = clock
        self.command_topic = command_topic
        self.odom_topic = odom_topic
        self.lock = threading.Lock()
        self.stats: dict[str, _TopicStats] = {}
        self.commands: list[tuple] = []
        self.odometry: list[tuple] = []
        self.node = rclpy.create_node("argus_gazebo_recorder")
        for ros, _gz, ros_type, _gz_type, _direction in entries:
            if ros == "/clock" or ros in self.stats:
                continue
            self.stats[ros] = _TopicStats(ros_type)
            self.node.create_subscription(get_message(ros_type), ros, partial(self._on, ros),
                                          qos_profile_sensor_data)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _on(self, topic: str, msg) -> None:
        now = self.clock()
        header = getattr(msg, "header", None)
        stamp = header.stamp.sec + 1e-9 * header.stamp.nanosec if header is not None else None
        with self.lock:
            self.stats[topic].add(now, stamp)
            if topic == self.command_topic:
                tw = getattr(msg, "twist", msg)
                self.commands.append((now, tw.linear.x, tw.linear.y, tw.linear.z, tw.angular.z))
            elif topic == self.odom_topic and (not self.odometry or now - self.odometry[-1][0] >= ODOM_LOG_PERIOD_S):
                p, tw = msg.pose.pose, msg.twist.twist
                self.odometry.append((now, p.position.x, p.position.y, p.position.z, _yaw(p.orientation),
                                      tw.linear.x, tw.linear.y, tw.angular.z))

    def close(self) -> None:
        self.executor.shutdown()
        self.thread.join(5.0)
        self.node.destroy_node()


def _csv(path: Path, header: list[str], rows) -> None:
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        for r in rows:
            w.writerow([f"{v:.6g}" if isinstance(v, float) else v for v in r])


def _argo_counts(log: Path) -> dict:
    events: dict[str, int] = {}
    flags: dict[str, int] = {}
    if log.exists():
        for line in log.read_text().splitlines():
            parts = line.split()
            if len(parts) >= 3 and parts[1] == "event":
                events[parts[2]] = events.get(parts[2], 0) + 1
            elif len(parts) >= 3 and parts[1] == "flags":
                for f in parts[2].split(","):
                    flags[f] = flags.get(f, 0) + 1
    return {"events": events, "flag_changes": flags}


def write_run_log(log_dir: Path, work: Path, *, doc: dict, world, model: str, topics, start, goal,
                  tolerance_m: float, episode, recorder: RosRecorder | None) -> Path:
    log_dir.mkdir(parents=True, exist_ok=True)
    (log_dir / "model.sdf").write_text(model)
    for name in ("world.sdf", "bridge.yaml", "profile.yaml", "argo.log", "gz.log", "bridge.log"):
        if (work / name).exists():
            shutil.copy2(work / name, log_dir / name)
    _csv(log_dir / "trajectory.csv", ["t_s", "x_m", "y_m", "z_m", "yaw_rad"], episode.trace)

    rates = {s["source"]["endpoint"]: s.get("rate_hz") for s in doc.get("sensors", [])}
    topic_summary = {}
    if recorder is not None:
        with recorder.lock:
            _csv(log_dir / "commands.csv", ["t_s", "vx_mps", "vy_mps", "vz_mps", "yaw_rate_radps"],
                 recorder.commands)
            _csv(log_dir / "odometry.csv", ["t_s", "x_m", "y_m", "z_m", "yaw_rad", "vx_mps", "vy_mps",
                                            "yaw_rate_radps"], recorder.odometry)
            for name, st in recorder.stats.items():
                topic_summary[name] = st.summary()
                if name in rates:
                    topic_summary[name]["declared_rate_hz"] = rates[name]
        if doc["control"]["endpoint"] in topic_summary:
            topic_summary[doc["control"]["endpoint"]]["declared_rate_hz"] = doc["control"].get("rate_hz")

    run = {
        "body": {"id": doc.get("id"), "name": doc.get("name"), "drivetrain": topics.drive},
        "world": {"name": world.name, "boxes": [[round(float(v), 4) for v in b] for b in world.boxes]},
        "start": list(start), "goal": list(goal), "tolerance_m": tolerance_m,
        "outcome": {"goal_reached": episode.goal_reached, "reached_at_s": episode.reached_at_s,
                    "mission_state": episode.mission_state, "collided": episode.collided,
                    "collisions": episode.collisions, "final_pose": list(episode.final_pose),
                    "simulated_s": episode.time_s},
        "topics": topic_summary,
        "argo": _argo_counts(work / "argo.log"),
        "files": sorted(p.name for p in log_dir.iterdir()) + ["run.json"],
    }
    (log_dir / "run.json").write_text(json.dumps(run, indent=1, default=float))
    return log_dir
