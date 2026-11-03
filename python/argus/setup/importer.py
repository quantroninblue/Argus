"""Builds a body profile from a URDF and a short answers file.

The URDF gives the collision shapes, link frames and any sensors declared in Gazebo tags. The
answers file is a partial profile with what a URDF cannot say: motion limits, the command
endpoint, failsafe, compute, odometry, and topics or rates the URDF lacks. Sensors in the
answers are matched to URDF sensors by id and override their fields.

Provenance: shapes and URDF sensors are `cad`, values from the answers `declared` (unless the
answers set their own provenance).
"""

from __future__ import annotations

import copy
import math
from pathlib import Path

import numpy as np
import yaml

from argus import profile as profiles

from .urdf import UrdfBody, frame_pose, identifier, read_urdf

ANSWERS_HINT = """\
# answers.yaml: what the URDF cannot say. Fill in, then run the import again.
id: my-robot
motion:
  space: planar
  can_reverse: true
  can_strafe: false
  min_turn_radius_m: 0.0
  collision_padding_m: 0.05
  limits:
    x: {velocity_mps: [-0.3, 0.8], accel_mps2: 0.6, decel_mps2: 1.0}
    y: {velocity_mps: [0.0, 0.0]}
    z: {velocity_mps: [0.0, 0.0]}
    yaw: {rate_radps: 1.0, accel_radps2: 2.0}
control: {interface: velocity, transport: ros2, endpoint: /cmd_vel, rate_hz: 50, latency_s: 0.05,
          controller_timeout_s: 0.3}
failsafe: {link_lost_after_s: 3.0, on_link_lost: stop, uncovered_motion: forbid}
compute: {board: vicharak_axon, accelerator: rk3588_npu}
sensors:
  - {id: wheel_odom, kind: wheel_odom, frame: base_link, source: {transport: ros2, endpoint: /odom}, rate_hz: 50}
assumptions:
  clear_at_start_m: 0.5
  # Drones with no range sensor that sees straight up cannot see a ceiling. They must declare
  # the highest they may fly above the floor they start on; you answer for that height.
  # max_flight_height_m: 2.5
"""


def _shape_bottom(shape: dict) -> float:
    pose = shape.get("pose", {})
    z = pose.get("xyz_m", [0, 0, 0])[2]
    r, p, y = pose.get("rpy_rad", [0, 0, 0])
    if shape["type"] == "sphere":
        return z - shape["radius_m"]
    half = (np.array(shape["size_m"]) / 2 if shape["type"] == "box"
            else np.array([shape["radius_m"], shape["radius_m"], shape["length_m"] / 2]))
    cr, sr, cp, sp = math.cos(r), math.sin(r), math.cos(p), math.sin(p)
    # Rows of the rotation's z component: how far each local axis reaches down.
    zrow = np.array([-sp, cp * sr, cp * cr])
    return z - float(np.abs(zrow) @ half)


def _merge(base: dict, over: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in over.items():
        out[k] = _merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else copy.deepcopy(v)
    return out


def _urdf_sensors(body: UrdfBody) -> list[dict]:
    out = []
    used = set()
    for s in body.sensors:
        sid = identifier(s.name or s.link)
        while sid in used:
            sid += "_2"
        used.add(sid)
        d = {"id": sid, "kind": s.kind, "frame": identifier(s.link)}
        if s.topic:
            d["source"] = {"transport": "ros2", "endpoint": s.topic if s.topic.startswith("/") else "/" + s.topic}
        if s.rate_hz:
            d["rate_hz"] = s.rate_hz
        if s.range_m and s.kind in ("range_2d", "range_3d", "depth"):
            d["range_m"] = [round(s.range_m[0], 4), round(s.range_m[1], 4)]
        if s.fov_deg and s.kind in ("range_2d", "range_3d", "depth", "rgb"):
            d["fov_deg"] = {"h": round(s.fov_deg[0], 3), "v": round(s.fov_deg[1], 3)}
        if s.angular_resolution_deg and s.kind in ("range_2d", "range_3d", "depth"):
            d["angular_resolution_deg"] = round(s.angular_resolution_deg, 4)
        out.append(d)
    return out


# Rotation from a camera body frame (x forward, y left, z up) to its optical frame (z forward,
# x right, y down), as ROS camera drivers stamp their images (REP 103).
_BODY_TO_OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def _is_optical(rotation: np.ndarray) -> bool:
    """A frame whose z axis points level, outward, and whose y axis points down: an optical
    frame, whatever its name."""
    z_axis, y_axis = rotation[:, 2], rotation[:, 1]
    return abs(z_axis[2]) < 0.5 and y_axis[2] < -0.5


def _camera_frames(doc: dict, body: UrdfBody, sensors: list[dict], notes: list[str]) -> None:
    """Cameras whose frame is an optical frame get the same mount turned to look along x,
    which is how Argus reads every sensor frame; the frame itself is kept as it is."""
    names = {identifier(n): n for n in body.frames}
    for s in sensors:
        if s.get("kind") not in ("depth", "rgb") or s.get("frame") not in names:
            continue
        t = body.frames[names[s["frame"]]]
        if not _is_optical(t[:3, :3]):
            continue
        looking = np.eye(4)
        looking[:3, :3] = t[:3, :3] @ _BODY_TO_OPTICAL.T
        looking[:3, 3] = t[:3, 3]
        name = f"{s['frame']}_x_forward"
        doc["geometry"]["frames"][name] = frame_pose(looking)
        notes.append(f"{s['id']}: {s['frame']} is an optical frame (z forward, y down); Argus uses the same "
                     f"mount looking along x, as frame {name}")
        s["frame"] = name


def build_profile(urdf: str | Path, answers: dict, base_frame: str | None = None,
                  packages: dict[str, Path] | None = None) -> tuple[dict, list[str], list[str]]:
    """(profile, problems, notes). The profile is only usable when `problems` is empty."""
    body = read_urdf(urdf, base_frame, packages)
    notes = list(body.notes)
    answers = copy.deepcopy(answers or {})
    answer_sensors = {s.get("id"): s for s in answers.pop("sensors", []) or []}
    answer_provenance = answers.pop("provenance", {}) or {}

    doc = {"argus_profile": 1, "id": identifier(body.name).lower().replace("_", "-"), "name": body.name}
    doc = _merge(doc, answers)
    doc["geometry"] = {
        "ground_z_m": round(min(_shape_bottom(s) for s in body.shapes), 6),
        "shapes": body.shapes,
        "frames": {identifier(name): frame_pose(t) for name, t in sorted(body.frames.items())},
    }
    provenance = {"geometry.shapes": "cad"}
    for section in ("motion", "control", "failsafe", "assumptions"):
        if section in answers:
            provenance[section] = "declared"

    sensors = []
    for s in _urdf_sensors(body):
        over = answer_sensors.pop(s["id"], None)
        provenance[f"sensors.{s['id']}"] = "cad"
        if over:
            s = _merge(s, over)
            provenance[f"sensors.{s['id']}"] = "declared"
        sensors.append(s)
    for sid, s in answer_sensors.items():
        sensors.append(s)
        if sid:
            provenance[f"sensors.{sid}"] = "declared"
    _camera_frames(doc, body, sensors, notes)
    doc["sensors"] = sensors
    provenance.update(answer_provenance)
    doc["provenance"] = provenance

    problems = []
    if not any(s.get("kind") in ("odometry", "wheel_odom") for s in sensors):
        problems.append("sensors: no odometry source; Argo needs a wheel_odom or odometry sensor "
                        "(add one to the answers file)")
    for s in sensors:
        if s.get("frame") and s["frame"] not in doc["geometry"]["frames"]:
            problems.append(f"sensors.{s.get('id')}.frame: no link {s['frame']!r} in the URDF")
    problems += [p for p in profiles.problems(doc) if p not in problems]
    if not problems:
        caps = profiles.derive(doc)
        for field in caps.missing_fields:
            problems.append(f"{field}; add it under assumptions in the answers file")
    return doc, problems, notes


def load_answers(path: str | Path | None) -> dict:
    if path is None:
        return {}
    data = yaml.safe_load(Path(path).read_text()) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a mapping at the top level")
    return data
