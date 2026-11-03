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
    # Single range sensors that stop publishing, by id, from the given time on.
    sensor_fails_at_s: dict[str, float] = field(default_factory=dict)
    # Faulty sensors that keep publishing, by id, from the given time on: a frozen driver
    # resends its last frame with new stamps; a blinded one sends every ray invalid.
    sensor_freezes_at_s: dict[str, float] = field(default_factory=dict)
    sensor_blinded_at_s: dict[str, float] = field(default_factory=dict)
    seed: int = 0
    # Bodies with a minimum turn radius turn this much wider than commanded and than the
    # profile allows, the way real steering does (Gazebo's front-steered model: about 1.15).
    turn_radius_scale: float = 1.0
    # A profile more optimistic than the body: the plant brakes at this share of the declared
    # deceleration and takes this much longer than declared to act on a command. Safety
    # margins must hold anyway (collisions); arriving may take longer.
    brake_scale: float = 1.0
    extra_latency_s: float = 0.0
    # Range frames reach Argo this long after they were captured (stamped at capture).
    sensor_latency_s: float = 0.0
    # Odometry that drifts: distance reported times this scale, and a yaw rate error, from
    # `odom_drift_from_s` on (from the start when None).
    odom_scale: float = 1.0
    odom_yaw_drift_radps: float = 0.0
    odom_drift_from_s: float | None = None
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
    # Each time a moving obstacle came into contact: {"t", "with", "robot_speed_mps", "at_rest",
    # "toward"}. The robot driving (or turning) into it sets `collided`; a mover walking into a
    # robot at rest, or the robot pulling away from it, does not (passive motion safety,
    # Fraichard 2007).
    contacts: list[dict[str, Any]] = field(default_factory=list)
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

    def __init__(self, caps, start, lead_s: float, turn_radius_scale: float = 1.0,
                 brake_scale: float = 1.0, extra_latency_s: float = 0.0):
        self.m = caps.motion
        self.interface = caps.control["interface"]
        self.timeout = caps.control["controller_timeout_s"]
        self.latency = self.m.latency_s + extra_latency_s
        self.brake_scale = brake_scale
        self.lead = lead_s
        self.turn_scale = turn_radius_scale
        self.pos = np.array(start[:3], dtype=float)
        self.yaw = float(start[3])
        self.vel = np.zeros(3)
        self.w = 0.0
        # The yaw rate the drive is working toward; with a turn mismatch the body achieves only
        # a share of it. Kept apart from `w` so the share is applied once, not every step.
        self.w_drive = 0.0
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
        self.vel = np.array([_approach(self.vel[i], lin[i], axes[i].accel_mps2,
                                       axes[i].decel_mps2 * self.brake_scale, dt) for i in range(3)])
        self.w_drive = _approach(self.w_drive, w, m.yaw_accel_radps2, m.yaw_accel_radps2, dt)
        self.w = self.w_drive
        if m.space == "planar":
            self.vel[2] = 0.0
            if not m.can_strafe:
                self.vel[1] = 0.0
            if m.min_turn_radius_m > 0.0:
                self.w = self.w_drive / self.turn_scale
                limit = abs(self.vel[0]) / (m.min_turn_radius_m * self.turn_scale)
                self.w = max(-limit, min(limit, self.w))
        mid = self.yaw + 0.5 * self.w * dt
        self.pos = self.pos + _rot(mid) @ self.vel * dt
        self.yaw = _wrap(self.yaw + self.w * dt)


class _Sensor:
    def __init__(self, spec: dict, node, rng: np.random.Generator):
        self.id = spec["id"]
        self.sees_glass = spec["kind"] not in ("range_2d", "range_3d")
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
        ranges = world.raycast(origin, self.dirs @ r.T, self.max_range, self.sees_glass)
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
            for box in world.solid():
                nearest = np.clip(pos, box[:3], box[3:])
                if np.linalg.norm(nearest - pos) < self.radius:
                    return True
            return False
        poly = self.footprint @ _rot(yaw)[:2, :2].T + pos[:2]
        lo_z, hi_z = pos[2] + self.z_range[0], pos[2] + self.z_range[1]
        for box in world.solid():
            if box[5] <= lo_z + 1e-3 or box[2] >= hi_z:
                continue
            if _polygon_meets_rect(poly, box[0:2], box[3:5]):
                return True
        # Any part of the footprint over a pit: it has gone over the edge.
        for hole in world.holes:
            if _polygon_meets_rect(poly, hole[0:2], hole[2:4]):
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


class Simulation:
    """One body in one world, stepped at a fixed time step.

    Each step feeds Argo the odometry and range frames due at that time, calls `tick` (which
    returns Argo's tick result) when a control tick is due, sends the command to the plant and
    moves the body. Whoever drives the simulation decides what a tick is: the Node directly, or
    Argo's session with a hub behind it.
    """

    def __init__(self, doc: dict, world: World, start, options: SimOptions | None = None, config=None):
        self.options = options or SimOptions()
        self.doc = doc
        self.world = world
        self.caps = profiles.derive(doc)
        self.node = argo.Node(profiles.dump(doc), config or argo.NodeConfig())
        self.plant = Plant(self.caps, start, self.node.control_period_s + self.caps.motion.latency_s,
                           self.options.turn_radius_scale, self.options.brake_scale,
                           self.options.extra_latency_s)
        # What odometry reports; equal to the truth unless the options make it drift.
        self._est_pos = np.array(start[:3], dtype=float)
        self._est_yaw = float(start[3])
        self._frames: deque = deque()  # (deliver at, captured at, sensor id, ranges)
        self.judge = _Judge(self.caps, doc, self.node.control_period_s)
        rng = np.random.default_rng(self.options.seed)
        specs = {s["id"]: s for s in doc["sensors"]}
        self.sensors = [_Sensor(specs[i], self.node, rng) for i in self.node.range_sensor_ids()]
        self.result = Episode(profile_id=doc["id"], world=world.name)
        self.dt = min(self.options.dt, self.node.control_period_s / 2)
        o = self.options
        self.fault_at = min(x for x in (o.sensors_stop_at_s, o.odometry_fails_at_s, math.inf) if x is not None)
        self.k = 0
        self._next_odom = 0.0
        self._next_tick = 0.0
        self._odom_yaw = self.plant.yaw
        self._touch: dict[int, str] = {}  # mover index -> fault or clear, while in contact
        self._last_ranges: dict[str, np.ndarray] = {}

    def _judge_movers(self, t: float, velocity: np.ndarray | None = None) -> None:
        """A contact is recorded when it starts, and again if the robot starts driving into the
        mover while still touching. The robot is at fault when it moves toward the mover (or
        turns, sweeping its corners) during contact; pulling away from someone pressed against
        it, or standing still while they walk into it, is not."""
        plant, result = self.plant, self.result
        v = plant_velocity(plant) if velocity is None else velocity
        for i, m in enumerate(self.world.movers):
            box = m.box(t)
            alone = type(self.world)(boxes=box[None, :], name=self.world.name)
            if not self.judge.collides(alone, plant.pos, plant.yaw):
                self._touch.pop(i, None)
                continue
            centre = np.array([(box[0] + box[3]) / 2, (box[1] + box[4]) / 2])
            toward = float(np.dot(v[:2], centre - plant.pos[:2])) > 1e-4 or abs(plant.w) > 0.05
            at_fault = not self.at_rest and toward
            before = self._touch.get(i)
            if before is None or (at_fault and before != "fault"):
                result.contacts.append({"t": t, "with": m.name, "robot_speed_mps": self.speed,
                                        "at_rest": self.at_rest, "toward": toward})
            if at_fault:
                result.collided = True
            self._touch[i] = "fault" if at_fault else "clear"

    @property
    def t(self) -> float:
        return self.k * self.dt

    @property
    def speed(self) -> float:
        return float(np.linalg.norm(self.plant.vel))

    @property
    def at_rest(self) -> bool:
        return self.speed < 0.01 and abs(self.plant.w) < 0.01

    @property
    def pose(self) -> tuple[float, float, float, float]:
        return (*self.plant.pos.tolist(), self.plant.yaw)

    def step(self, tick) -> dict | None:
        """Advances one time step. Returns the tick result if a control tick was due."""
        t, o, node, plant, result = self.t, self.options, self.node, self.plant, self.result
        drifting = (o.odom_scale != 1.0 or o.odom_yaw_drift_radps != 0.0) and (
            o.odom_drift_from_s is None or t >= o.odom_drift_from_s)
        if not drifting:
            # Until the drift starts, odometry is the truth.
            self._est_pos = plant.pos.copy()
            self._est_yaw = float(plant.yaw)
        if t >= self._next_odom - EPS:
            healthy = o.odometry_fails_at_s is None or t < o.odometry_fails_at_s
            if drifting:
                node.on_odometry(t, self._est_pos.tolist(), self._est_yaw, (plant.vel * o.odom_scale).tolist(),
                                 plant.w + o.odom_yaw_drift_radps, healthy)
            else:
                node.on_odometry(t, plant.pos.tolist(), plant.yaw, plant.vel.tolist(), plant.w, healthy)
            # Setpoint velocities are in odom; judge them in the base frame Argo last knew.
            self._odom_yaw = plant.yaw
            self._next_odom += 1.0 / o.odom_rate_hz
        for s in self.sensors:
            if t >= s.next - EPS:
                s.next += s.period
                failed = s.id in o.sensor_fails_at_s and t >= o.sensor_fails_at_s[s.id]
                if not failed and (o.sensors_stop_at_s is None or t < o.sensors_stop_at_s):
                    frozen = s.id in o.sensor_freezes_at_s and t >= o.sensor_freezes_at_s[s.id]
                    if frozen and s.id in self._last_ranges:
                        ranges = self._last_ranges[s.id]
                    else:
                        ranges = s.frame(self.world.at(t), plant.pos, plant.yaw, o.range_noise_m)
                    if s.id in o.sensor_blinded_at_s and t >= o.sensor_blinded_at_s[s.id]:
                        ranges = np.full_like(ranges, np.nan)
                    self._last_ranges[s.id] = ranges
                    self._frames.append((t + o.sensor_latency_s, t, s.id, ranges))
        while self._frames and self._frames[0][0] <= t + EPS:
            _, captured, sid, ranges = self._frames.popleft()
            node.on_range_frame(captured, sid, ranges)
        out = None
        if t >= self._next_tick - EPS:
            out = tick(t)
            self._next_tick += node.control_period_s
            result.events += out["events"]
            for e in out["events"]:
                if e["kind"] == "waypoint_reached":
                    result.reached.append(e["waypoint_id"])
                if e["kind"] == "plan_complete":
                    result.complete = True
            result.violations += self.judge.command(t, out["command"], self._odom_yaw)
            for f in out["flags"]:
                result.flag_counts[f] = result.flag_counts.get(f, 0) + 1
            if o.trace:
                result.trace.append({"t": t, "pose": self.pose, "flags": out["flags"],
                                     "proposed": out["proposed"], "planner": out["planner"],
                                     "path": len(node.path()), "movers": node.movers()})
            plant.send(t, out["command"])

        before = plant.pos.copy()
        before_yaw = plant.yaw
        plant.step(t, self.dt)
        if drifting:
            # Dead reckoning with a scale error and a yaw rate error, from the true motion.
            moved = _rot(-before_yaw) @ (plant.pos - before)
            self._est_yaw = _wrap(self._est_yaw + _wrap(plant.yaw - before_yaw) + o.odom_yaw_drift_radps * self.dt)
            self._est_pos = self._est_pos + _rot(self._est_yaw) @ (moved * o.odom_scale)
        result.distance_m += float(np.linalg.norm(plant.pos - before))
        result.max_speed_mps = max(result.max_speed_mps, self.speed)
        if self.judge.collides(self.world, plant.pos, plant.yaw):
            result.collided = True
        if self.world.movers:
            self._judge_movers(t)
        if t >= self.fault_at and result.rest_after_fault_s is None and self.at_rest:
            result.rest_after_fault_s = t
        result.time_s = t
        result.final_pose = self.pose
        self.k += 1
        return out


def plant_velocity(plant) -> np.ndarray:
    """The plant's velocity in the world frame (its state is in the body frame)."""
    c, sn = math.cos(plant.yaw), math.sin(plant.yaw)
    vx, vy = plant.vel[0], plant.vel[1]
    return np.array([c * vx - sn * vy, sn * vx + c * vy, plant.vel[2]])


def run_episode(doc: dict, world: World, start, waypoints: list[dict], duration_s: float,
                options: SimOptions | None = None, config=None) -> Episode:
    """Drives one body from `start` (x, y, z, yaw) through `waypoints` in `world`."""
    sim = Simulation(doc, world, start, options, config)
    node = sim.node

    def tick(t: float) -> dict:
        if t == 0.0:
            node.set_plan(1, duration_s + 60.0, waypoints, 0.0)
        return node.tick(t)

    steps = int(math.ceil(duration_s / sim.dt))
    for _ in range(steps + 1):
        sim.step(tick)
        if sim.result.complete and sim.speed < 0.01:
            break
    return sim.result
