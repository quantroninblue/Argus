"""The user's own robot in Gazebo, from a stored design (hub plan H3).

What runs is what the user designed, never a stand-in: their Gazebo model as given when they
supplied one, otherwise their URDF through Gazebo's own conversion (meshes, inertias, joints,
limits and every <gazebo> tag kept). Argus adds only what a run needs and the model lacks,
and lists each addition for the run log:

- a drive system on the wheel joints, when the model has none (differential bodies only;
  other drivetrains must bring their own Gazebo plugin);
- a Gazebo sensor at the declared frame for each declared sensor the model does not simulate;
- a topic name for a sensor that has none, so the bridge can reach it;
- an odometry publisher when the drive system publishes none;
- a true-pose publisher, used only to judge the run, never seen by Argo.

The command timeout sits outside the model, in the motor controller (robot.py).
"""

from __future__ import annotations

import math
import os
import subprocess
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from argus.hub.designs import GAZEBO_NAME, URDF_NAME
from argus.setup.urdf import UrdfError, read_urdf

from .model import Topics, _sensor

DRIVES = {
    "diff-drive": "differential", "diffdrive": "differential",
    "ackermann": "front-steered",
    "mecanum": "mecanum",
    "multicopter-control": "multirotor", "multicoptervelocitycontrol": "multirotor",
}
LIDARS = ("gpu_lidar", "lidar", "gpu_ray", "ray")
DEPTHS = ("depth", "depth_camera", "rgbd_camera", "rgbd")


class DesignModelError(Exception):
    """The design cannot run as it is; the message says what to add to it."""


@dataclass
class DesignModel:
    sdf: str
    topics: Topics
    added: list[str] = field(default_factory=list)
    resource_path: str = ""
    # Where the model's own sensors are not where the profile says they are (design faults).
    mount_findings: list[str] = field(default_factory=list)


# How far a sensor of the user's Gazebo model may sit from the mount the profile declares
# before Argo, which places every return by the declared mount, would see the world wrong.
MOUNT_TOLERANCE_M = 0.05
MOUNT_TOLERANCE_DEG = 3.0


def _source_model(version_dir: Path, env: dict | None) -> tuple[ET.Element, Path]:
    """The <model> element to run and the directory its relative paths start from."""
    gz_model = version_dir / GAZEBO_NAME
    if gz_model.exists():
        root = ET.fromstring(gz_model.read_text())
        base = gz_model.parent
    else:
        try:
            r = subprocess.run(["gz", "sdf", "-p", str(version_dir / URDF_NAME)], capture_output=True, text=True,
                               timeout=120, env=env)
        except FileNotFoundError as e:
            raise DesignModelError("Gazebo (gz) is not installed or not on PATH; source ROS 2 first") from e
        if r.returncode != 0 or "<model" not in r.stdout:
            raise DesignModelError(f"Gazebo could not convert the URDF: {(r.stderr or r.stdout).strip()[:500]}")
        root = ET.fromstring(r.stdout)
        base = version_dir
    model = root if root.tag == "model" else root.find("model")
    if model is None:
        raise DesignModelError("the Gazebo model has no <model> element")
    return model, base


def _resolve_uris(model: ET.Element, base: Path) -> None:
    """Mesh paths Gazebo can open from anywhere: packages through the resource path, files as
    absolute paths (the model is placed into a world written elsewhere)."""
    for el in model.iter("uri"):
        uri = (el.text or "").strip()
        if uri.startswith("package://"):
            el.text = "model://" + uri[len("package://"):]
        elif uri and "://" not in uri and not uri.startswith("/"):
            el.text = "file://" + str((base / uri).resolve())


def _mesh_exists(uri: str, packages: Path) -> bool:
    if uri.startswith("model://"):
        return (packages / uri[len("model://"):]).is_file()
    if uri.startswith("file://"):
        return Path(uri[len("file://"):]).is_file()
    return True  # anything else is Gazebo's to find (Fuel URLs, built-in names)


def _drop_missing_visuals(model: ET.Element, packages: Path) -> list[str]:
    """Visuals whose mesh is not in the design are left out, so Gazebo loads the model; a
    missing collision mesh is a design error and never gets this far (designs.check). Physics
    is unchanged: visuals only change how the robot looks."""
    out = []
    for link in model.iter("link"):
        for visual in list(link.findall("visual")):
            uri = (visual.findtext("geometry/mesh/uri") or "").strip()
            if uri and not _mesh_exists(uri, packages):
                link.remove(visual)
                out.append(f"left out visual {visual.get('name')} of {link.get('name')}: its mesh {uri} is not in "
                           "the design (the robot looks incomplete; its physics is unchanged)")
    return out


def _norm(topic: str) -> str:
    return topic if topic.startswith("/") else "/" + topic


def _text(el: ET.Element, tag: str) -> str:
    child = el.find(tag)
    return (child.text or "").strip() if child is not None else ""


def _drive(model: ET.Element, name: str) -> tuple[str, str, str] | None:
    """(kind, command topic, odometry topic) of the model's drive plugin, if it has one."""
    for p in model.iter("plugin"):
        key = (p.get("filename", "") + " " + p.get("name", "")).lower()
        kind = next((k for frag, k in DRIVES.items() if frag in key), None)
        if kind is None:
            continue
        if kind == "multirotor":
            ns = _text(p, "robotNamespace")
            sub = _text(p, "commandSubTopic") or "cmd_vel"
            return kind, _norm(f"{ns}/{sub}" if ns else sub), ""
        cmd = _text(p, "topic") or f"/model/{name}/cmd_vel"
        odom = _text(p, "odom_topic") or f"/model/{name}/odometry"
        return kind, _norm(cmd), _norm(odom)
    return None


def _sensors(model: ET.Element) -> list[dict]:
    out = []
    for link in model.iter("link"):
        for s in link.findall("sensor"):
            v = s.find("./lidar/scan/vertical/samples")
            if v is None:
                v = s.find("./ray/scan/vertical/samples")
            out.append({"el": s, "type": s.get("type", ""), "link": link.get("name", ""), "link_el": link,
                        "frame": _text(s, "gz_frame_id"), "topic": _text(s, "topic"),
                        "vertical": int(v.text) if v is not None and (v.text or "").strip().isdigit() else 1})
    return out


def _matches(kind: str, s: dict) -> bool:
    if kind == "range_2d":
        return s["type"] in LIDARS and s["vertical"] <= 1
    if kind == "range_3d":
        return s["type"] in LIDARS and s["vertical"] > 1
    if kind == "depth":
        return s["type"] in DEPTHS
    return kind == "imu" and s["type"] == "imu"


def _in_root(doc: dict, version_dir: Path, frame: str) -> dict:
    """A profile frame (base frame) as a pose in the model's root link, which the Gazebo
    conversion makes of the URDF's root with every fixed child merged into it."""
    f = doc["geometry"]["frames"][frame]
    t = np.eye(4)
    r, p, y = f.get("rpy_rad", [0.0, 0.0, 0.0])
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    t[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    t[:3, 3] = f.get("xyz_m", [0.0, 0.0, 0.0])
    try:
        body = read_urdf(version_dir / URDF_NAME)
        roots = [n for n, m in body.frames.items() if n not in _children(version_dir)]
        root_in_base = body.frames[roots[0]] if roots else np.eye(4)
    except (UrdfError, KeyError, IndexError):
        root_in_base = np.eye(4)
    m = np.linalg.inv(root_in_base) @ t
    rpy = (math.atan2(m[2, 1], m[2, 2]), math.asin(max(-1.0, min(1.0, -m[2, 0]))), math.atan2(m[1, 0], m[0, 0]))
    return {"xyz_m": m[:3, 3].tolist(), "rpy_rad": list(rpy)}


def _matrix(xyz, rpy) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    t = np.eye(4)
    t[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    t[:3, 3] = xyz
    return t


def _pose_of(el: ET.Element) -> np.ndarray:
    """An element's own <pose> (a direct child), relative to its parent frame."""
    text = _text(el, "pose")
    v = [float(x) for x in text.split()] if text else []
    return _matrix(v[:3], v[3:6]) if len(v) == 6 else np.eye(4)


def _mount_finding(spec: dict, s: dict, expected: dict) -> str | None:
    """Where Gazebo puts the model's sensor (its link's pose, then its own pose) against the
    mount the profile declares, both in the model's root frame."""
    actual = _pose_of(s["link_el"]) @ _pose_of(s["el"])
    want = _matrix(expected["xyz_m"], expected["rpy_rad"])
    off = actual[:3, 3] - want[:3, 3]
    turn = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(want[:3, :3].T @ actual[:3, :3]) - 1.0) / 2.0))))
    if np.linalg.norm(off) <= MOUNT_TOLERANCE_M and turn <= MOUNT_TOLERANCE_DEG:
        return None
    where = ", ".join(f"{abs(d) * 100:.0f} cm {pos if d > 0 else neg}" for d, pos, neg in
                      zip(off, ("forward", "left", "higher"), ("back", "right", "lower")) if abs(d) >= 0.01)
    return (f"{spec['id']}: the Gazebo model's sensor {s['el'].get('name')} is not where the profile says "
            f"(frame {spec['frame']}): it sits {where or 'in place'}" + (f", turned {turn:.0f} deg" if turn > 1 else "")
            + ". Argo places what it sees by the declared mount, so it will see the world wrong; make the URDF "
              "and the Gazebo model agree")


def mount_findings(version_dir: Path, doc: dict) -> list[str]:
    """Sensors of the user's own Gazebo model that sit away from their declared mounts. Only
    for a design with its own Gazebo model; sensors Argus adds are placed at the mount."""
    version_dir = Path(version_dir)
    if not (version_dir / GAZEBO_NAME).exists():
        return []
    model, _ = _source_model(version_dir, None)
    have = _sensors(model)
    out, used = [], set()
    for spec in doc["sensors"]:
        if spec["kind"] not in ("range_2d", "range_3d", "depth"):
            continue
        pick = _pick(spec, have, used)
        if pick is not None:
            used.add(pick)
            f = _mount_finding(spec, have[pick], _in_root(doc, version_dir, spec["frame"]))
            if f:
                out.append(f)
    return out


def _pick(spec: dict, have: list[dict], used: set[int]) -> int | None:
    cands = [i for i, s in enumerate(have) if i not in used and _matches(spec["kind"], s)]
    by_frame = [i for i in cands if spec["frame"] in (have[i]["frame"], have[i]["link"])]
    return (by_frame or cands or [None])[0]


def _children(version_dir: Path) -> set[str]:
    root = ET.fromstring((version_dir / URDF_NAME).read_text())
    return {c.get("link") for j in root.findall("joint") for c in j.findall("child")}


def _wheels(version_dir: Path) -> tuple[str, str, float, float] | None:
    """(left joint, right joint, separation, radius) from the URDF's wheel joints."""
    root = ET.fromstring((version_dir / URDF_NAME).read_text())
    links = {lk.get("name"): lk for lk in root.findall("link")}
    found = []
    for j in root.findall("joint"):
        if j.get("type") not in ("continuous", "revolute"):
            continue
        child = j.find("child").get("link")
        if "wheel" not in (j.get("name", "") + child).lower():
            continue
        origin = j.find("origin")
        y = float((origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()[1])
        cyl = links.get(child).find("./collision/geometry/cylinder") if child in links else None
        sph = links.get(child).find("./collision/geometry/sphere") if child in links else None
        radius = float((cyl if cyl is not None else sph).get("radius")) if (cyl is not None or sph is not None) else 0.0
        found.append((y, j.get("name"), radius))
    left = [w for w in found if w[0] > 0.0]
    right = [w for w in found if w[0] < 0.0]
    if len(left) != 1 or len(right) != 1 or left[0][2] <= 0.0:
        return None
    return left[0][1], right[0][1], left[0][0] - right[0][0], left[0][2]


def design_model(version_dir: Path, doc: dict, name: str = "robot", pose=(0.0, 0.0, 0.0, 0.0),
                 env: dict | None = None) -> DesignModel:
    """The design's model ready to place in a world, the Gazebo topics the bridge needs, and
    what Argus added. Raises DesignModelError when the design cannot run as it is."""
    version_dir = Path(version_dir)
    model, base = _source_model(version_dir, env)
    _resolve_uris(model, base)
    left_out = _drop_missing_visuals(model, version_dir / "packages")
    model.set("name", name)
    for old in model.findall("pose"):
        model.remove(old)
    x, y, z, heading = pose
    p = ET.Element("pose")
    p.text = f"{x:.6g} {y:.6g} {z:.6g} 0 0 {heading:.6g}"
    model.insert(0, p)
    added: list[str] = left_out
    topics = Topics(model=name, truth=f"/model/{name}/truth")
    links = model.findall("link")
    if not links:
        raise DesignModelError("the model has no links")
    root_link = links[0]

    drive = _drive(model, name)
    if drive is None:
        m = doc["motion"]
        if m["space"] != "planar" or m.get("can_strafe") or m.get("min_turn_radius_m", 0.0) > 0.0:
            raise DesignModelError("the model has no Gazebo drive system, and Argus adds one only for a "
                                   "differential base; add your drivetrain's Gazebo plugin to the model")
        wheels = _wheels(version_dir)
        if wheels is None:
            raise DesignModelError("the model has no Gazebo drive system and its wheel joints could not be "
                                   "found (one continuous joint with 'wheel' in its name on each side)")
        left, right, sep, radius = wheels
        topics.cmd_vel, topics.odom = f"/model/{name}/cmd_vel", f"/model/{name}/odometry"
        model.append(ET.fromstring(
            '<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">'
            f"<left_joint>{left}</left_joint><right_joint>{right}</right_joint>"
            f"<wheel_separation>{sep:.6g}</wheel_separation><wheel_radius>{radius:.6g}</wheel_radius>"
            f"<topic>{topics.cmd_vel}</topic><odom_topic>{topics.odom}</odom_topic>"
            "<frame_id>odom</frame_id><odom_publish_frequency>50</odom_publish_frequency></plugin>"))
        added.append(f"a differential drive on {left} and {right} (separation {sep:.3f} m, wheel radius "
                     f"{radius:.3f} m): the model had no drive system")
        drive_kind = "differential"
    else:
        drive_kind, topics.cmd_vel, odom = drive
        topics.odom = odom
    if not topics.odom:
        topics.odom = f"/model/{name}/odom"
        model.append(ET.fromstring(
            '<plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">'
            f"<odom_topic>{topics.odom}</odom_topic><odom_frame>odom</odom_frame>"
            "<odom_publish_frequency>50</odom_publish_frequency><dimensions>3</dimensions>"
            "<gaussian_noise>0.01</gaussian_noise></plugin>"))
        added.append("an odometry publisher: the drive system publishes none")
    model.append(ET.fromstring(
        '<plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">'
        f"<odom_topic>{topics.truth}</odom_topic><odom_frame>world</odom_frame>"
        "<odom_publish_frequency>50</odom_publish_frequency><dimensions>3</dimensions></plugin>"))
    added.append("a true-pose publisher, only for judging the run (Argo never sees it)")

    have = _sensors(model)
    used: set[int] = set()
    mounts: list[str] = []
    for spec in doc["sensors"]:
        kind = spec["kind"]
        if kind not in ("range_2d", "range_3d", "depth", "imu"):
            continue
        sid = spec["id"]
        pick = _pick(spec, have, used)
        if pick is None:
            topic = f"/model/{name}/{sid}" + ("/depth_image" if kind == "depth" else "")
            root_link.append(ET.fromstring(_sensor(spec, _in_root(doc, version_dir, spec["frame"]), topic)))
            added.append(f"sensor {sid} ({kind}) at frame {spec['frame']}: the model does not simulate it")
        else:
            used.add(pick)
            s = have[pick]
            if kind != "imu":
                f = _mount_finding(spec, s, _in_root(doc, version_dir, spec["frame"]))
                if f:
                    mounts.append(f)
            topic = s["topic"]
            if not topic:
                topic = f"/model/{name}/{sid}" + ("/depth_image" if kind == "depth" else "")
                el = ET.SubElement(s["el"], "topic")
                el.text = topic
                added.append(f"a topic name ({topic}) for the model's sensor {s['el'].get('name')}: it had none")
            topic = _norm(topic)
            if s["type"] in ("rgbd_camera", "rgbd"):
                topic += "/depth_image"
        topics.sensors[sid] = topic
        topics.kinds[sid] = kind
    topics.drive = {"kind": drive_kind, "from": "the design"}
    packages = version_dir / "packages"
    return DesignModel(sdf=ET.tostring(model, encoding="unicode"), topics=topics, added=added,
                       resource_path=str(packages) if packages.is_dir() else "", mount_findings=mounts)


def resource_env(env: dict, dm: DesignModel) -> dict:
    """`env` with the design's packages on Gazebo's resource path, for model:// meshes."""
    if not dm.resource_path:
        return env
    out = dict(env)
    out["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(p for p in (dm.resource_path, env.get("GZ_SIM_RESOURCE_PATH", "")) if p)
    return out
