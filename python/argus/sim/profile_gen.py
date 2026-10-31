"""Random but valid body profiles for testing the core against bodies nobody designed.

The generator varies motion space, drive constraints, size, limits and sensor layout, and
deliberately produces poor layouts too: no range sensors, rear-only cameras, scan planes
too high to see low obstacles.

    python -m argus.sim.profile_gen --count 20 --seed 3 --out /tmp/profiles
"""

from __future__ import annotations

import argparse
import math
import random
from pathlib import Path
from typing import Any

from argus import profile as profiles


def _r(value: float) -> float:
    return round(value, 4)


def _pose(xyz, rpy=(0.0, 0.0, 0.0)) -> dict[str, Any]:
    pose = {"xyz_m": [_r(v) for v in xyz]}
    if any(rpy):
        pose["rpy_rad"] = [_r(v) for v in rpy]
    return pose


class _Builder:
    def __init__(self, rng: random.Random):
        self.rng = rng
        self.frames: dict[str, dict[str, Any]] = {}
        self.sensors: list[dict[str, Any]] = []

    def add_sensor(self, kind: str, pose: dict[str, Any], **extra) -> None:
        name = f"{kind.replace('_', '')}_{len(self.sensors)}"
        self.frames[name] = pose
        sensor = {
            "id": name,
            "kind": kind,
            "frame": name,
            "source": {"transport": "ros2", "endpoint": f"/{name}"},
            "rate_hz": self.rng.choice([10, 15, 20, 30, 50, 100]),
        }
        sensor.update(extra)
        self.sensors.append(sensor)


def _range_extra(rng: random.Random, kind: str) -> dict[str, Any]:
    if kind == "range_2d":
        return {
            "range_m": [_r(rng.uniform(0.05, 0.2)), _r(rng.uniform(5.0, 30.0))],
            "fov_deg": {"h": rng.choice([180, 240, 270, 360]), "v": _r(rng.uniform(0.5, 2.0))},
        }
    if kind == "range_3d":
        return {
            "range_m": [_r(rng.uniform(0.1, 0.5)), _r(rng.uniform(10.0, 40.0))],
            "fov_deg": {"h": rng.choice([120, 360]), "v": rng.choice([30, 45, 60, 90])},
        }
    return {
        "range_m": [_r(rng.uniform(0.15, 0.4)), _r(rng.uniform(3.0, 10.0))],
        "fov_deg": {"h": rng.choice([70, 87, 90, 110]), "v": rng.choice([45, 58, 60, 70])},
    }


def _face_mount(rng, length, width, z_low, z_high, top) -> tuple[tuple, tuple]:
    """A pose just outside one face of a box body, facing outward."""
    face = rng.choice(["front", "back", "left", "right", "top"])
    pitch = rng.uniform(0.0, 0.5)
    if face == "top":
        xyz = (rng.uniform(-0.3, 0.3) * length, rng.uniform(-0.3, 0.3) * width,
               top + rng.uniform(0.03, 0.15))
        return xyz, (0.0, 0.0, rng.choice([0.0, math.pi / 2, math.pi, -math.pi / 2]))
    z = rng.uniform(z_low, z_high)
    offsets = {
        "front": ((length / 2 + 0.02, 0.0, z), 0.0),
        "back": ((-length / 2 - 0.02, 0.0, z), math.pi),
        "left": ((0.0, width / 2 + 0.02, z), math.pi / 2),
        "right": ((0.0, -width / 2 - 0.02, z), -math.pi / 2),
    }
    xyz, yaw = offsets[face]
    return xyz, (0.0, pitch, yaw)


def _planar(rng: random.Random, b: _Builder) -> tuple[dict, dict, dict]:
    drive = rng.choice(["differential", "omnidirectional", "car_like"])
    length = rng.uniform(0.3, 1.2)
    width = rng.uniform(0.3, 1.0)
    height = rng.uniform(0.15, 0.6)
    clearance = rng.uniform(0.02, 0.15)
    top = clearance + height
    shapes = [{"type": "box", "size_m": [_r(length), _r(width), _r(height)],
               "pose": _pose((0.0, 0.0, clearance + height / 2))}]
    if rng.random() < 0.4:
        mast = rng.uniform(0.2, 0.8)
        shapes.append({"type": "cylinder", "radius_m": _r(rng.uniform(0.03, 0.08)),
                       "length_m": _r(mast),
                       "pose": _pose((rng.uniform(-0.3, 0.3) * length, 0.0, top + mast / 2))})

    for _ in range(rng.choice([0, 1, 1, 2, 2, 3, 4])):
        kind = rng.choice(["range_2d", "range_2d", "range_3d", "depth", "depth"])
        xyz, rpy = _face_mount(rng, length, width, clearance + 0.02, top - 0.02, top)
        b.add_sensor(kind, _pose(xyz, rpy), **_range_extra(rng, kind))
    if rng.random() < 0.3:
        xyz, rpy = _face_mount(rng, length, width, clearance + 0.02, top - 0.02, top)
        b.add_sensor("rgb", _pose(xyz, rpy), fov_deg={"h": 90, "v": 60})
    b.add_sensor("imu", _pose((0.0, 0.0, clearance + height / 2)))
    if rng.random() < 0.8:
        b.add_sensor("wheel_odom", _pose((0.0, 0.0, clearance)))

    top_speed = rng.uniform(0.2, 2.0)
    can_reverse = drive != "car_like" or rng.random() < 0.6
    can_strafe = drive == "omnidirectional"

    def axis(enabled: bool, reverse: bool) -> dict[str, Any]:
        if not enabled:
            return {"velocity_mps": [0.0, 0.0]}
        accel = rng.uniform(0.3, 2.0)
        low = -top_speed * rng.uniform(0.2, 1.0) if reverse else 0.0
        return {"velocity_mps": [_r(low), _r(top_speed)], "accel_mps2": _r(accel),
                "decel_mps2": _r(accel * rng.uniform(1.0, 2.0))}

    motion = {
        "space": "planar",
        "can_reverse": can_reverse,
        "can_strafe": can_strafe,
        "min_turn_radius_m": _r(rng.uniform(0.5, 3.0)) if drive == "car_like" else 0.0,
        "collision_padding_m": _r(rng.uniform(0.0, 0.1)),
        "limits": {
            "x": axis(True, can_reverse),
            "y": axis(can_strafe, True),
            "z": axis(False, False),
            "yaw": {"rate_radps": _r(rng.uniform(0.3, 2.0)), "accel_radps2": _r(rng.uniform(0.5, 4.0))},
        },
    }
    geometry = {"ground_z_m": 0.0, "shapes": shapes}
    failsafe_action = rng.choice(["stop", "hold", "finish_leg_then_stop", "return_home"])
    control = {"interface": "velocity", "transport": "ros2", "endpoint": "/cmd_vel"}
    return geometry, motion, {"failsafe_action": failsafe_action, "control": control,
                              "top_speed": top_speed}


def _aerial(rng: random.Random, b: _Builder) -> tuple[dict, dict, dict]:
    core = rng.uniform(0.12, 0.5)
    core_height = rng.uniform(0.05, 0.2)
    rotors = rng.choice([4, 6, 8])
    arm = core / 2 + rng.uniform(0.05, 0.4)
    rotor_radius = rng.uniform(0.05, 0.25)
    shapes = [{"type": "box", "size_m": [_r(core), _r(core), _r(core_height)]}]
    for i in range(rotors):
        angle = 2 * math.pi * (i + 0.5) / rotors
        shapes.append({"type": "sphere", "radius_m": _r(rotor_radius),
                       "pose": _pose((arm * math.cos(angle), arm * math.sin(angle), 0.0))})

    for _ in range(rng.choice([0, 1, 2, 2, 3])):
        kind = rng.choice(["range_3d", "depth", "depth", "range_2d"])
        mount = rng.choice(["top", "bottom", "front", "back"])
        if mount == "top":
            pose = _pose((0.0, 0.0, core_height / 2 + rng.uniform(0.03, 0.3)))
        elif mount == "bottom":
            pose = _pose((0.0, 0.0, -core_height / 2 - 0.02), (0.0, math.pi / 2, 0.0))
        elif mount == "front":
            pose = _pose((core / 2 + 0.02, 0.0, 0.0), (0.0, rng.uniform(0.0, 0.4), 0.0))
        else:
            pose = _pose((-core / 2 - 0.02, 0.0, 0.0), (0.0, 0.0, math.pi))
        b.add_sensor(kind, pose, **_range_extra(rng, kind))
    b.add_sensor("imu", _pose((0.0, 0.0, 0.0)))
    b.add_sensor("odometry", _pose((0.0, 0.0, 0.0)))

    top_speed = rng.uniform(0.5, 5.0)

    def axis(speed: float) -> dict[str, Any]:
        accel = rng.uniform(0.5, 4.0)
        return {"velocity_mps": [_r(-speed), _r(speed)], "accel_mps2": _r(accel),
                "decel_mps2": _r(accel * rng.uniform(1.0, 2.0))}

    motion = {
        "space": "aerial",
        "can_reverse": True,
        "can_strafe": True,
        "collision_padding_m": _r(rng.uniform(0.05, 0.3)),
        "limits": {
            "x": axis(top_speed),
            "y": axis(top_speed),
            "z": axis(top_speed * rng.uniform(0.3, 0.8)),
            "yaw": {"rate_radps": _r(rng.uniform(0.5, 3.0)), "accel_radps2": _r(rng.uniform(1.0, 6.0))},
        },
    }
    failsafe_action = rng.choice(["hold", "land", "return_home"])
    control = rng.choice([
        {"interface": "pose_setpoint", "transport": "mavlink", "endpoint": "udp://:14540"},
        {"interface": "velocity", "transport": "mavlink", "endpoint": "udp://:14540"},
    ])
    return {"shapes": shapes}, motion, {"failsafe_action": failsafe_action, "control": control,
                                        "top_speed": top_speed}


def generate_profile(rng: random.Random, index: int) -> dict[str, Any]:
    b = _Builder(rng)
    space = "aerial" if rng.random() < 0.35 else "planar"
    geometry, motion, extra = (_aerial if space == "aerial" else _planar)(rng, b)
    geometry["frames"] = b.frames

    uncovered = rng.choice(["forbid", "creep"])
    failsafe = {
        "link_lost_after_s": _r(rng.uniform(0.5, 5.0)),
        "on_link_lost": extra["failsafe_action"],
        "uncovered_motion": uncovered,
    }
    if uncovered == "creep":
        failsafe["creep_speed_mps"] = _r(min(0.1, extra["top_speed"]) * rng.uniform(0.3, 1.0))

    doc: dict[str, Any] = {
        "argus_profile": 1,
        "id": f"generated-{index:04d}",
        "geometry": geometry,
        "motion": motion,
        "control": {
            **extra["control"],
            "rate_hz": rng.choice([10, 20, 30, 50, 100]),
            "latency_s": _r(rng.uniform(0.02, 0.15)),
            "controller_timeout_s": _r(rng.uniform(0.1, 0.6)),
        },
        "failsafe": failsafe,
        "sensors": b.sensors,
        "compute": rng.choice([
            {"board": "vicharak_axon", "accelerator": "rk3588_npu"},
            {"board": "generic", "accelerator": "none"},
        ]),
    }

    provenance = {}
    candidates = ["control.latency_s", "motion.limits.x.velocity_mps", "motion.limits.yaw.rate_radps"]
    candidates += [f"sensors.{s['id']}" for s in b.sensors]
    for field in rng.sample(candidates, k=rng.randint(0, min(3, len(candidates)))):
        provenance[field] = rng.choice(
            ["declared", "cad", "calibrated", "llm_draft_reviewed", "llm_draft"])
    if provenance:
        doc["provenance"] = provenance
    return doc


def generate_profiles(count: int, seed: int = 0) -> list[dict[str, Any]]:
    rng = random.Random(seed)
    return [generate_profile(rng, i) for i in range(count)]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    for doc in generate_profiles(args.count, args.seed):
        (args.out / f"{doc['id']}.yaml").write_text(profiles.dump(doc))
    print(f"wrote {args.count} profiles to {args.out}")


if __name__ == "__main__":
    main()
