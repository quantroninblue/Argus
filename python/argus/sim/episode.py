"""Runs Argo against a simulated body and judges the run from simulator truth.

The simulator owns the body's real pose, the world and the sensors. Argo only sees what a real
robot would give it: odometry, range frames and a plan. Collisions, reached goals and broken
rules are decided here, never from Argo's own estimate.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from argus import node as argo
from argus import profile as profiles
from argus.sim.world import World

EPS = 1e-6


@dataclass
class SimOptions:
    dt: float = 0.01
    odom_rate_hz: float = 50.0
    range_noise_m: float = 0.0
    # Every range sensor stops publishing from this time on.
    sensors_stop_at_s: float | None = None
    # Odometry reports itself unhealthy from this time on.
    odometry_fails_at_s: float | None = None
    seed: int = 0
    # Keep one row per control tick in Episode.trace.
    trace: bool = False


@dataclass
class Episode:
    profile_id: str
    world: str
    reached: list[str] = field(default_factory=list)
    complete: bool = False
    time_s: float = 0.0
    collided: bool = False
    violations: list[str] = field(default_factory=list)
    events: list[dict[str, Any]] = field(default_factory=list)
    distance_m: float = 0.0
    final_pose: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    # When the body came to rest after the fault set in SimOptions, if it did.
    rest_after_fault_s: float | None = None
    max_speed_mps: float = 0.0
    # How many ticks each safety flag was raised.
    flag_counts: dict[str, int] = field(default_factory=dict)
    trace: list[dict[str, Any]] = field(default_factory=list)


def _rot(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _wrap(a: float) -> float:
    return math.remainder(a, 2.0 * math.pi)


def _approach(v: float, target: float, accel: float, decel: float, dt: float) -> float:
    slowing = v * target >= 0.0 and abs(target) < abs(v)
    rate = decel if slowing else accel
    if rate <= 0.0:
        return target if slowing else v
    step = rate * dt
    return v + max(-step, min(step, target - v))


class Plant:
    """The body and its own controller: command latency, watchdog, acceleration limits.

    Velocity interface: tracks the commanded base-frame velocity. Pose setpoint interface: a
    flight-controller-like tracker that follows the setpoint moving at its feed-forward velocity.
    """

    def __init__(self, caps, start, lead_s: float):
        self.m = caps.motion
        self.interface = caps.control["interface"]
        self.timeout = caps.control["controller_timeout_s"]
        self.latency = self.m.latency_s
        self.lead = lead_s
        self.pos = np.array(start[:3], dtype=float)
        self.yaw = float(start[3])
        self.vel = np.zeros(3)
        self.w = 0.0
        self.queue: deque = deque()
        self.cmd: dict | None = None
        self.applied_at = -math.inf

    def send(self, t: float, cmd: dict) -> None:
        self.queue.append((t + self.latency, cmd))

    def _target(self, t: float) -> tuple[np.ndarray, float]:
        while self.queue and self.queue[0][0] <= t + EPS:
            self.applied_at, self.cmd = self.queue.popleft()
        if self.cmd is None or t - self.applied_at > self.timeout:
            return np.zeros(3), 0.0
        if self.interface == "velocity":
            v = self.cmd["velocity"]
            return np.asarray(v["linear"], dtype=float), float(v["yaw_rate"])
        since = t - self.applied_at
        ff = np.asarray(self.cmd["setpoint_velocity"], dtype=float)
        wff = float(self.cmd["setpoint_yaw_rate"])
        lead = 0.0 if self.cmd["stop"] else self.lead
        ref = np.asarray(self.cmd["setpoint_position"], dtype=float) + ff * (since - lead)
        ref_yaw = self.cmd["setpoint_yaw"] + wff * (since - lead)
        desired = ff + 1.5 * (ref - self.pos)
        return _rot(-self.yaw) @ desired, wff + 1.5 * _wrap(ref_yaw - self.yaw)

    def step(self, t: float, dt: float) -> None:
        m = self.m
        lin, w = self._target(t)
        lin = np.array([np.clip(lin[0], m.x.min_mps, m.x.max_mps),
                        np.clip(lin[1], m.y.min_mps, m.y.max_mps),
                        np.clip(lin[2], m.z.min_mps, m.z.max_mps)])
        w = float(np.clip(w, -m.yaw_rate_max_radps, m.yaw_rate_max_radps))
        axes = (m.x, m.y, m.z)
        self.vel = np.array([_approach(self.vel[i], lin[i], axes[i].accel_mps2, axes[i].decel_mps2, dt)
                             for i in range(3)])
        self.w = _approach(self.w, w, m.yaw_accel_radps2, m.yaw_accel_radps2, dt)
        if m.space == "planar":
            self.vel[2] = 0.0
            if not m.can_strafe:
                self.vel[1] = 0.0
            if m.min_turn_radius_m > 0.0:
                limit = abs(self.vel[0]) / m.min_turn_radius_m
                self.w = max(-limit, min(limit, self.w))
        mid = self.yaw + 0.5 * self.w * dt
        self.pos = self.pos + _rot(mid) @ self.vel * dt
        self.yaw = _wrap(self.yaw + self.w * dt)


class _Sensor:
    def __init__(self, spec: dict, node, rng: np.random.Generator):
        self.id = spec["id"]
        self.period = 1.0 / spec["rate_hz"]
        self.min_range, self.max_range = spec["range_m"]
        mount = np.asarray(node.sensor_mount(self.id))
        self.mount_t = mount[:3, 3]
        self.dirs = node.ray_directions(self.id) @ mount[:3, :3].T
        self.self_hits = np.asarray(node.self_hits(self.id))
        self.next = 0.0
        self.rng = rng

    def frame(self, world: World, pos: np.ndarray, yaw: float, noise: float) -> np.ndarray:
        r = _rot(yaw)
        origin = pos + r @ self.mount_t
        ranges = world.raycast(origin, self.dirs @ r.T, self.max_range)
        ranges = np.minimum(ranges, self.self_hits)
        if noise > 0.0:
            finite = np.isfinite(ranges)
            ranges[finite] += self.rng.normal(0.0, noise, finite.sum())
        ranges[ranges < self.min_range] = np.nan
        return ranges.astype(np.float32)


class _Judge:
    """Checks every command against the body's limits and coverage, independently of Argo."""

    def __init__(self, caps, doc: dict, control_period_s: float):
        self.caps = caps
        self.max_age = {s["id"]: s.get("max_age_s", 3.0 / s["rate_hz"]) for s in doc["sensors"]}
        self.period = control_period_s
        self.m = caps.motion
        self.bins = list(caps.sensing.bins)
        self.per_row = sum(1 for b in self.bins if b.elevation_deg == self.bins[0].elevation_deg)
        self.creep = (caps.failsafe["uncovered_motion"] == "creep", caps.failsafe["creep_speed_mps"])
        self.footprint = np.asarray(caps.collision.body_footprint)
        self.z_range = (caps.collision.aabb_min[2], caps.collision.aabb_max[2])
        self.radius = caps.collision.body_radius_m

    def _bin(self, d: np.ndarray):
        az = math.degrees(math.atan2(d[1], d[0])) % 360.0
        if self.m.space == "planar":
            n = len(self.bins)
            return self.bins[int(math.floor(az / (360.0 / n) + 0.5)) % n]
        rows = len(self.bins) // self.per_row
        el0 = self.bins[0].elevation_deg
        step = (self.bins[-1].elevation_deg - el0) / (rows - 1)
        el = math.degrees(math.atan2(d[2], math.hypot(d[0], d[1])))
        row = min(rows - 1, max(0, int(math.floor((el - el0) / step + 0.5))))
        col = int(math.floor(az / (360.0 / self.per_row) + 0.5)) % self.per_row
        return self.bins[row * self.per_row + col]

    def sight_limit(self, d: np.ndarray) -> float:
        """Fastest speed along `d` that stops within the distance every height band is seen."""
        b = self._bin(d)
        if b.full_height_reach_m <= 0.0:
            return 0.0
        decel = min(a.decel_mps2 for a, c in zip((self.m.x, self.m.y, self.m.z), d) if abs(c) > 0.1)
        delay = max(self.max_age[s] for s in b.sensors) + self.period + self.m.latency_s
        return decel * (-delay + math.sqrt(delay * delay + 2.0 * b.full_height_reach_m / decel))

    def command(self, t: float, cmd: dict, yaw: float) -> list[str]:
        m = self.m
        if self.caps.control["interface"] == "velocity":
            lin = np.asarray(cmd["velocity"]["linear"], dtype=float)
            w = cmd["velocity"]["yaw_rate"]
        else:
            lin = _rot(-yaw) @ np.asarray(cmd["setpoint_velocity"], dtype=float)
            w = cmd["setpoint_yaw_rate"]
        problems = []
        axes = {"x": m.x, "y": m.y, "z": m.z}
        for i, (name, a) in enumerate(axes.items()):
            if not a.min_mps - EPS <= lin[i] <= a.max_mps + EPS:
                problems.append(f"{t:.2f}s {name} speed {lin[i]:.3f} outside limits")
        if abs(w) > m.yaw_rate_max_radps + EPS:
            problems.append(f"{t:.2f}s yaw rate {w:.3f} over limit")
        if m.min_turn_radius_m > 0.0 and abs(w) > abs(lin[0]) / m.min_turn_radius_m + EPS:
            problems.append(f"{t:.2f}s turn tighter than the minimum radius")
        speed = float(np.linalg.norm(lin))
        moving = speed > EPS or abs(w) > EPS
        if moving and not self.caps.safety_ready:
            problems.append(f"{t:.2f}s moved with an unreviewed profile")
        if speed > EPS:
            limit = self.sight_limit(lin / speed)
            allowed, creep = self.creep
            if limit < 0.02 and (not allowed or speed > creep + EPS):
                problems.append(f"{t:.2f}s moved {speed:.3f} m/s into an uncovered direction")
            elif limit >= 0.02 and speed > limit + EPS:
                problems.append(f"{t:.2f}s moved {speed:.3f} m/s where sensors only allow {limit:.3f}")
        return problems

    def collides(self, world: World, pos: np.ndarray, yaw: float) -> bool:
        if self.m.space == "aerial":
            if pos[2] + self.z_range[0] < 0.0:
                return True
            for box in world.boxes:
                nearest = np.clip(pos, box[:3], box[3:])
                if np.linalg.norm(nearest - pos) < self.radius:
                    return True
            return False
        poly = self.footprint @ _rot(yaw)[:2, :2].T + pos[:2]
        lo_z, hi_z = pos[2] + self.z_range[0], pos[2] + self.z_range[1]
        for box in world.boxes:
            if box[5] <= lo_z + 1e-3 or box[2] >= hi_z:
                continue
            if _polygon_meets_rect(poly, box[0:2], box[3:5]):
                return True
        return False


def _polygon_meets_rect(poly: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> bool:
    if (poly.max(axis=0) < lo).any() or (poly.min(axis=0) > hi).any():
        return False
    rect = np.array([lo, [hi[0], lo[1]], hi, [lo[0], hi[1]]])
    edges = np.roll(poly, -1, axis=0) - poly
    for n in np.stack([-edges[:, 1], edges[:, 0]], axis=1):
        p, r = poly @ n, rect @ n
        if p.max() < r.min() or r.max() < p.min():
            return False
    return True


def run_episode(doc: dict, world: World, start, waypoints: list[dict], duration_s: float,
                options: SimOptions | None = None, config=None) -> Episode:
    """Drives one body from `start` (x, y, z, yaw) through `waypoints` in `world`."""
    options = options or SimOptions()
    caps = profiles.derive(doc)
    node = argo.Node(profiles.dump(doc), config or argo.NodeConfig())
    plant = Plant(caps, start, node.control_period_s + caps.motion.latency_s)
    judge = _Judge(caps, doc, node.control_period_s)
    rng = np.random.default_rng(options.seed)
    specs = {s["id"]: s for s in doc["sensors"]}
    sensors = [_Sensor(specs[i], node, rng) for i in node.range_sensor_ids()]
    result = Episode(profile_id=doc["id"], world=world.name)

    dt = min(options.dt, node.control_period_s / 2)
    fault_at = min(x for x in (options.sensors_stop_at_s, options.odometry_fails_at_s, math.inf)
                   if x is not None)
    next_odom = next_tick = 0.0
    steps = int(math.ceil(duration_s / dt))
    for k in range(steps + 1):
        t = k * dt
        if t >= next_odom - EPS:
            healthy = options.odometry_fails_at_s is None or t < options.odometry_fails_at_s
            node.on_odometry(t, plant.pos.tolist(), plant.yaw, plant.vel.tolist(), plant.w, healthy)
            # Setpoint velocities are in odom; judge them in the base frame Argo last knew.
            odom_yaw = plant.yaw
            next_odom += 1.0 / options.odom_rate_hz
        for s in sensors:
            if t >= s.next - EPS:
                s.next += s.period
                if options.sensors_stop_at_s is None or t < options.sensors_stop_at_s:
                    node.on_range_frame(t, s.id, s.frame(world, plant.pos, plant.yaw, options.range_noise_m))
        if k == 0:
            node.set_plan(1, duration_s + 60.0, waypoints, 0.0)
        if t >= next_tick - EPS:
            tick = node.tick(t)
            next_tick += node.control_period_s
            result.events += tick["events"]
            for e in tick["events"]:
                if e["kind"] == "waypoint_reached":
                    result.reached.append(e["waypoint_id"])
                if e["kind"] == "plan_complete":
                    result.complete = True
            result.violations += judge.command(t, tick["command"], odom_yaw)
            for f in tick["flags"]:
                result.flag_counts[f] = result.flag_counts.get(f, 0) + 1
            if options.trace:
                result.trace.append({"t": t, "pose": (*plant.pos.tolist(), plant.yaw),
                                     "flags": tick["flags"], "proposed": tick["proposed"],
                                     "planner": tick["planner"],
                                     "path": len(node.path())})
            plant.send(t, tick["command"])

        before = plant.pos.copy()
        plant.step(t, dt)
        result.distance_m += float(np.linalg.norm(plant.pos - before))
        speed = float(np.linalg.norm(plant.vel))
        result.max_speed_mps = max(result.max_speed_mps, speed)
        if judge.collides(world, plant.pos, plant.yaw):
            result.collided = True
        if t >= fault_at and result.rest_after_fault_s is None and speed < 0.01 and abs(plant.w) < 0.01:
            result.rest_after_fault_s = t
        result.time_s = t
        if result.complete and speed < 0.01:
            break

    result.final_pose = (*plant.pos.tolist(), plant.yaw)
    return result
