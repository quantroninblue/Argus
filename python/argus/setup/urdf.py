"""Reads what a body profile needs from a URDF: collision shapes and link frames in the base
frame (joints at zero), and sensors declared in Gazebo tags.

Meshes become their axis-aligned bounding boxes in the mesh's own frame, which is
conservative: the box always contains the mesh.
"""

from __future__ import annotations

import math
import os
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .meshes import mesh_bounds


class UrdfError(ValueError):
    pass


def _rot(rpy) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _rpy(m: np.ndarray) -> tuple[float, float, float]:
    pitch = math.asin(max(-1.0, min(1.0, -m[2, 0])))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(m[2, 1], m[2, 2])
        yaw = math.atan2(m[1, 0], m[0, 0])
    else:  # gimbal lock: put everything in yaw
        roll = 0.0
        yaw = math.atan2(-m[0, 1], m[1, 1])
    return roll, pitch, yaw


def _transform(origin: ET.Element | None) -> np.ndarray:
    t = np.eye(4)
    if origin is None:
        return t
    xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
    t[:3, :3] = _rot(rpy)
    t[:3, 3] = xyz
    return t


def _floats(text: str | None, n: int, what: str) -> list[float]:
    try:
        vals = [float(v) for v in (text or "").split()]
    except ValueError:
        vals = []
    if len(vals) != n:
        raise UrdfError(f"{what}: expected {n} numbers, got {text!r}")
    return vals


def identifier(name: str) -> str:
    """A link name made safe for a profile identifier."""
    s = re.sub(r"[^A-Za-z0-9_]", "_", name)
    return s if s and s[0].isalpha() else "f_" + s


@dataclass
class UrdfSensor:
    link: str
    kind: str  # profile sensor kind
    rate_hz: float | None = None
    range_m: tuple[float, float] | None = None
    fov_deg: tuple[float, float] | None = None
    topic: str | None = None
    name: str = ""


@dataclass
class UrdfBody:
    name: str
    base: str
    frames: dict[str, np.ndarray] = field(default_factory=dict)  # link -> 4x4 in base frame
    shapes: list[dict] = field(default_factory=list)  # profile shapes, base frame
    sensors: list[UrdfSensor] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)


def expand(path: Path) -> str:
    """URDF text; `.xacro` files go through the xacro tool."""
    if path.suffix == ".xacro":
        if shutil.which("xacro") is None:
            raise UrdfError(f"{path} is a xacro file and the xacro tool is not on PATH; "
                            "expand it first (xacro robot.urdf.xacro > robot.urdf)")
        out = subprocess.run(["xacro", str(path)], capture_output=True, text=True)
        if out.returncode != 0:
            raise UrdfError(f"xacro failed on {path}: {out.stderr.strip()}")
        return out.stdout
    return path.read_text()


def resolve_mesh(uri: str, urdf_dir: Path, packages: dict[str, Path]) -> Path:
    if uri.startswith("file://"):
        p = Path(uri[len("file://"):])
    elif uri.startswith("package://"):
        pkg, _, rest = uri[len("package://"):].partition("/")
        roots = [packages[pkg]] if pkg in packages else []
        for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
            if prefix:
                roots.append(Path(prefix) / "share" / pkg)
        p = next((r / rest for r in roots if (r / rest).is_file()), None)
        if p is None:
            found = [r for r in roots if r.is_dir()]
            if found:
                raise UrdfError(f"mesh {uri}: package {pkg!r} has no file {rest} (looked in {found[0]})")
            raise UrdfError(f"mesh {uri}: package {pkg!r} not found; pass --package {pkg}=<directory>")
    else:
        p = Path(uri)
        if not p.is_absolute():
            p = urdf_dir / p
    if not p.is_file():
        raise UrdfError(f"mesh {uri}: no file at {p}")
    return p


def _geometry_shape(geo: ET.Element, where: str, urdf_dir: Path, packages: dict[str, Path]):
    """(shape dict without pose, extra transform in the collision frame)."""
    child = next(iter(geo), None)
    if child is None:
        raise UrdfError(f"{where}: empty geometry")
    if child.tag == "box":
        return {"type": "box", "size_m": _floats(child.get("size"), 3, f"{where} box size")}, np.eye(4)
    if child.tag == "cylinder":
        return {"type": "cylinder", "radius_m": float(child.get("radius")),
                "length_m": float(child.get("length"))}, np.eye(4)
    if child.tag == "sphere":
        return {"type": "sphere", "radius_m": float(child.get("radius"))}, np.eye(4)
    if child.tag == "mesh":
        path = resolve_mesh(child.get("filename", ""), urdf_dir, packages)
        scale = np.array(_floats(child.get("scale", "1 1 1"), 3, f"{where} mesh scale"))
        lo, hi = mesh_bounds(path)
        lo, hi = np.minimum(lo * scale, hi * scale), np.maximum(lo * scale, hi * scale)
        size = hi - lo
        if np.any(size <= 0):
            raise UrdfError(f"{where}: mesh {path.name} is flat or empty")
        t = np.eye(4)
        t[:3, 3] = (lo + hi) / 2
        return {"type": "box", "size_m": [float(v) for v in size]}, t
    raise UrdfError(f"{where}: unsupported geometry <{child.tag}>")


_GZ_KINDS = {"ray": "range_2d", "gpu_ray": "range_2d", "lidar": "range_2d", "gpu_lidar": "range_2d",
             "depth": "depth", "depth_camera": "depth", "rgbd_camera": "depth", "camera": "rgb", "imu": "imu"}


def _gazebo_sensors(root: ET.Element) -> list[UrdfSensor]:
    out = []
    for gz in root.findall("gazebo"):
        link = gz.get("reference")
        for s in gz.findall("sensor"):
            kind = _GZ_KINDS.get(s.get("type", ""))
            if not link or kind is None:
                continue
            sensor = UrdfSensor(link=link, kind=kind, name=s.get("name", ""))
            rate = s.findtext("update_rate")
            sensor.rate_hz = float(rate) if rate else None
            topic = s.findtext("topic")
            for plugin in s.findall("plugin"):
                topic = topic or plugin.findtext("topicName") or plugin.findtext("topic_name")
                remap = plugin.findtext("ros/remapping") or ""
                if ":=" in remap:
                    topic = topic or remap.split(":=", 1)[1].strip()
            sensor.topic = topic.strip() if topic else None
            scan = s.find("ray/scan")
            if scan is None:
                scan = s.find("lidar/scan")
            rng = s.find("ray/range")
            if rng is None:
                rng = s.find("lidar/range")
            if scan is not None:
                h = scan.find("horizontal")
                v = scan.find("vertical")
                if h is not None:
                    span = float(h.findtext("max_angle", "0")) - float(h.findtext("min_angle", "0"))
                    vspan = 0.0
                    if v is not None and int(float(v.findtext("samples", "1"))) > 1:
                        vspan = float(v.findtext("max_angle", "0")) - float(v.findtext("min_angle", "0"))
                        sensor.kind = "range_3d"
                    sensor.fov_deg = (min(360.0, math.degrees(span)), math.degrees(vspan) or 1.0)
            if rng is not None:
                sensor.range_m = (float(rng.findtext("min", "0.1")), float(rng.findtext("max", "10")))
            cam = s.find("camera")
            if cam is not None and kind in ("depth", "rgb"):
                hfov = float(cam.findtext("horizontal_fov", "1.047"))
                w = float(cam.findtext("image/width", "640"))
                hgt = float(cam.findtext("image/height", "480"))
                vfov = 2 * math.atan(math.tan(hfov / 2) * hgt / w)
                sensor.fov_deg = (math.degrees(hfov), math.degrees(vfov))
                near, far = cam.findtext("clip/near"), cam.findtext("clip/far")
                if near and far:
                    sensor.range_m = (float(near), float(far))
            out.append(sensor)
    return out


def read_urdf(path: str | Path, base: str | None = None, packages: dict[str, Path] | None = None) -> UrdfBody:
    path = Path(path)
    try:
        root = ET.fromstring(expand(path))
    except ET.ParseError as e:
        raise UrdfError(f"{path}: not valid XML ({e})") from None
    if root.tag != "robot":
        raise UrdfError(f"{path}: the root element is <{root.tag}>, not <robot>")
    links = {link.get("name"): link for link in root.findall("link")}
    children: dict[str, list[tuple[str, np.ndarray]]] = {}
    has_parent = set()
    for j in root.findall("joint"):
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        children.setdefault(parent, []).append((child, _transform(j.find("origin"))))
        has_parent.add(child)
    roots = [n for n in links if n not in has_parent]
    if base is None:
        base = "base_link" if "base_link" in links else (roots[0] if len(roots) == 1 else None)
        if base is None:
            raise UrdfError(f"{path}: cannot tell the base link; pass --base-frame")
    if base not in links:
        raise UrdfError(f"{path}: no link named {base!r}")

    body = UrdfBody(name=root.get("name", path.stem), base=base)
    # Frames below the base, then any root above it (base_footprint -> base_link).
    stack = [(base, np.eye(4))]
    while stack:
        name, t = stack.pop()
        body.frames[name] = t
        for child, jt in children.get(name, []):
            stack.append((child, t @ jt))
    for parent, kids in children.items():
        for child, jt in kids:
            if child in body.frames and parent not in body.frames:
                body.frames[parent] = body.frames[child] @ np.linalg.inv(jt)
    missing = sorted(set(links) - set(body.frames))
    if missing:
        body.notes.append(f"links not connected to {base} are ignored: {', '.join(missing)}")

    urdf_dir = path.parent
    for name in links:  # in file order, so the profile reads like the URDF
        if name not in body.frames:
            continue
        t = body.frames[name]
        for i, col in enumerate(links[name].findall("collision")):
            geo = col.find("geometry")
            if geo is None:
                continue
            where = f"link {name} collision {i}"
            shape, extra = _geometry_shape(geo, where, urdf_dir, packages or {})
            m = t @ _transform(col.find("origin")) @ extra
            shape["pose"] = {"xyz_m": [round(float(v), 6) for v in m[:3, 3]],
                             "rpy_rad": [round(v, 6) for v in _rpy(m[:3, :3])]}
            body.shapes.append(shape)
    if not body.shapes:
        raise UrdfError(f"{path}: no collision geometry on any link connected to {base}")
    body.sensors = [s for s in _gazebo_sensors(root) if s.link in body.frames]
    return body


def frame_pose(t: np.ndarray) -> dict:
    return {"xyz_m": [round(float(v), 6) for v in t[:3, 3]], "rpy_rad": [round(v, 6) for v in _rpy(t[:3, :3])]}
