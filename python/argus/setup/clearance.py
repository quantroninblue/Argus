"""How much room the robot has in a direction, from whatever range sensors it has.

Setup checks the space before every motion test. Each range sensor (2D scan, 3D point cloud,
depth image) gives one message; its returns are placed in the base frame through the
profile's frames, and those inside a cone around the direction and within the body's height
band count. The answer is measured from the body's edge, not from the sensor.
"""

from __future__ import annotations

import math
import threading
import time

import numpy as np
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

from argus import profile as profiles

SENSOR_TYPES = {
    "range_2d": "sensor_msgs/msg/LaserScan",
    "range_3d": "sensor_msgs/msg/PointCloud2",
    "depth": "sensor_msgs/msg/Image",
}


def _rotation(rpy) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _mount(doc: dict, frame: str) -> tuple[np.ndarray, np.ndarray]:
    f = doc["geometry"].get("frames", {}).get(frame, {})
    return _rotation(f.get("rpy_rad", [0.0, 0.0, 0.0])), np.array(f.get("xyz_m", [0.0, 0.0, 0.0]), float)


def scan_points(msg, spec: dict) -> tuple[np.ndarray, np.ndarray]:
    """(hits, open directions), both in the sensor frame. Open directions are unit vectors of
    rays with no return within range."""
    r = np.asarray(msg.ranges, float)
    a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
    lo = max(msg.range_min, spec.get("range_m", [0.0, 0.0])[0])
    dirs = np.stack([np.cos(a), np.sin(a), np.zeros_like(a)], axis=1)
    hit = np.isfinite(r) & (r >= lo) & (r <= msg.range_max)
    open_ = ~np.isnan(r) & ~hit & ~(r < lo)
    return dirs[hit] * r[hit, None], dirs[open_]


def cloud_points(msg, spec: dict) -> tuple[np.ndarray, np.ndarray]:
    fields = {f.name: f.offset for f in msg.fields}
    if not {"x", "y", "z"} <= fields.keys():
        return np.zeros((0, 3)), np.zeros((0, 3))
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
    xyz = np.stack([raw[:, fields[k]:fields[k] + 4].copy().view(np.float32)[:, 0] for k in "xyz"], axis=1)
    xyz = xyz[np.isfinite(xyz).all(axis=1)].astype(float)
    lo, hi = spec.get("range_m", [0.0, math.inf])
    d = np.linalg.norm(xyz, axis=1)
    return xyz[(d >= lo) & (d <= hi)], np.zeros((0, 3))


def depth_points(msg, spec: dict, stride: int = 4) -> tuple[np.ndarray, np.ndarray]:
    """Depth image to points in the sensor frame (x forward, y left, z up). The camera model
    comes from the profile's field of view; close enough to judge room."""
    if msg.encoding == "32FC1":
        z = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width).astype(float)
    elif msg.encoding == "16UC1":
        z = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(msg.height, msg.width).astype(float) / 1000.0
        z[z == 0.0] = np.nan
    else:
        return np.zeros((0, 3)), np.zeros((0, 3))
    fov = spec.get("fov_deg", {})
    fx = (msg.width / 2.0) / math.tan(math.radians(fov.get("h", 90.0)) / 2.0)
    fy = (msg.height / 2.0) / math.tan(math.radians(fov.get("v", 60.0)) / 2.0)
    v, u = np.mgrid[0:msg.height:stride, 0:msg.width:stride]
    z = z[::stride, ::stride]
    right = (u - msg.width / 2.0) / fx
    down = (v - msg.height / 2.0) / fy
    dirs = np.stack([np.ones_like(right), -right, -down], axis=-1)
    lo, hi = spec.get("range_m", [0.0, math.inf])
    ok = np.isfinite(z) & (z > 0.0)
    pts = dirs[ok] * z[ok, None]
    d = np.linalg.norm(pts, axis=1)
    open_dirs = dirs[np.isposinf(z)]
    open_dirs = open_dirs / np.linalg.norm(open_dirs, axis=1, keepdims=True)
    return pts[(d >= lo) & (d <= hi)], open_dirs


_CONVERT = {"range_2d": scan_points, "range_3d": cloud_points, "depth": depth_points}


def _edge(caps, bearing: float) -> float:
    """Distance from the base origin to the padded body's edge along `bearing`."""
    lo, hi = caps.collision.aabb_min, caps.collision.aabb_max
    pad = caps.collision.padding_m
    c, s = math.cos(bearing), math.sin(bearing)
    hx = (hi[0] if c >= 0 else -lo[0]) + pad
    hy = (hi[1] if s >= 0 else -lo[1]) + pad
    return min(hx / abs(c) if abs(c) > 1e-9 else math.inf, hy / abs(s) if abs(s) > 1e-9 else math.inf)


def room(doc: dict, messages: dict, bearing: float, cone_deg: float = 20.0) -> float | None:
    """Room from the body's edge along `bearing` (base frame, radians), from one message per
    range sensor id. None when no sensor looks that way."""
    caps = profiles.derive(doc)
    ground = doc["geometry"].get("ground_z_m", 0.0)
    z_low = ground + 0.05
    z_high = caps.collision.aabb_max[2] + caps.collision.padding_m
    half = math.radians(cone_deg)
    best = None
    for spec in doc["sensors"]:
        msg = messages.get(spec["id"])
        if msg is None or spec["kind"] not in _CONVERT:
            continue
        rot, origin = _mount(doc, spec["frame"])
        hits, open_dirs = _CONVERT[spec["kind"]](msg, spec)
        pts = hits @ rot.T + origin
        heading = np.arctan2(pts[:, 1], pts[:, 0]) if len(pts) else np.zeros(0)
        near = (np.abs(np.remainder(heading - bearing + math.pi, 2 * math.pi) - math.pi) <= half)
        band = (pts[:, 2] >= z_low) & (pts[:, 2] <= z_high) if len(pts) else np.zeros(0, bool)
        sel = pts[near & band]
        dirs = open_dirs @ rot.T
        open_heading = np.arctan2(dirs[:, 1], dirs[:, 0]) if len(dirs) else np.zeros(0)
        open_near = np.abs(np.remainder(open_heading - bearing + math.pi, 2 * math.pi) - math.pi) <= half
        looks = len(sel) > 0 or bool(open_near.any()) or (len(pts) > 0 and near.any())
        if not looks:
            continue
        if len(sel):
            d = float(np.min(np.hypot(sel[:, 0], sel[:, 1]))) - _edge(caps, bearing)
        else:
            d = spec.get("range_m", [0.0, 0.0])[1] - _edge(caps, bearing)
        best = d if best is None else min(best, d)
    return best


def range_toward(doc: dict, messages: dict, bearing: float, cone_deg: float = 3.0) -> float | None:
    """Distance from the base origin to the nearest return in a narrow cone and the body's
    height band (the floor ahead does not count), for watching the range to a wall change.
    None without one."""
    caps = profiles.derive(doc)
    z_low = doc["geometry"].get("ground_z_m", 0.0) + 0.05
    z_high = caps.collision.aabb_max[2] + caps.collision.padding_m
    best = None
    for spec in doc["sensors"]:
        msg = messages.get(spec["id"])
        if msg is None or spec["kind"] not in _CONVERT:
            continue
        rot, origin = _mount(doc, spec["frame"])
        hits, _ = _CONVERT[spec["kind"]](msg, spec)
        pts = hits @ rot.T + origin
        pts = pts[(pts[:, 2] >= z_low) & (pts[:, 2] <= z_high)] if len(pts) else pts
        if not len(pts):
            continue
        heading = np.arctan2(pts[:, 1], pts[:, 0])
        sel = pts[np.abs(np.remainder(heading - bearing + math.pi, 2 * math.pi) - math.pi) <= math.radians(cone_deg)]
        if len(sel):
            d = float(np.median(np.hypot(sel[:, 0], sel[:, 1])))
            best = d if best is None else min(best, d)
    return best


def snapshot(probe, doc: dict, wait_s: float = 0.7) -> dict:
    """The latest message from each ROS 2 range sensor, by sensor id."""
    got: dict = {}
    lock = threading.Lock()
    subs = []
    for spec in doc["sensors"]:
        kind = spec["kind"]
        if kind not in SENSOR_TYPES or spec["source"]["transport"] != "ros2":
            continue

        def on(msg, sid=spec["id"]):
            with lock:
                got[sid] = msg

        subs.append(probe.node.create_subscription(get_message(SENSOR_TYPES[kind]), spec["source"]["endpoint"], on,
                                                   qos_profile_sensor_data))
    deadline = time.monotonic() + 10 * wait_s + 5
    start = probe.now()
    while (probe.now() - start < wait_s or not got) and time.monotonic() < deadline:
        time.sleep(0.02)
    for s in subs:
        probe.node.destroy_subscription(s)
    with lock:
        return dict(got)
