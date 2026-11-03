"""Calibration by stepped motion tests (setup step 5), watched through the robot's odometry.

Straight runs at half and then full declared speed along every axis the body moves on
(forward; backward if it may reverse; sideways, toward the side with more room, if it can
strafe), each ending in an explicit stop: command latency, acceleration, top speed and
braking per axis. Then turning: in place at the declared yaw rate, or for a body with a turn
radius, a full-lock arc to measure the real radius.

The forward runs also check the odometry scale: the distance odometry reports against the
change in range to whatever is straight ahead. Every speed above is measured in odometry
units, so a scale off by more than 5% is reported as a problem to fix in the robot's driver
rather than written back.

Results are written back conservatively, with provenance `calibrated`: the lower of declared
and measured for speeds, acceleration, braking and yaw rate; the higher for latency and turn
radius. A measurement that confirms the declared value marks it verified. What the data does
not determine keeps its declared value.

Every run first checks the room it needs in its direction with whatever range sensors the
robot has (see clearance.py). Motion goes through Argo in setup mode unless `direct` (see
live.py); a top speed Argo's supervisor slowed is not written back, since it measures the
supervisor, not the robot.
"""

from __future__ import annotations

import copy
import math
import threading
import time
from dataclasses import dataclass, field

from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

from .clearance import range_toward, room, snapshot
from .live import _Probe, _wait, argo_in_setup_mode

MARGIN_M = 0.5


@dataclass
class Measurement:
    field: str  # provenance path
    declared: float
    measured: float | None
    value: float  # what the profile now says
    note: str = ""


@dataclass
class Calibration:
    profile: dict
    measurements: list[Measurement] = field(default_factory=list)
    problems: list[str] = field(default_factory=list)
    # Optional runs that could not be made here (no room, or no sensor looking that way);
    # their values stay declared.
    skipped: list[str] = field(default_factory=list)

    def text(self) -> str:
        rows = []
        for m in self.measurements:
            meas = "not measured" if m.measured is None else f"{m.measured:.3g}"
            rows.append(f"{m.field:34s} declared {m.declared:<8.3g} measured {meas:<12s} now {m.value:.3g}"
                        + (f"  ({m.note})" if m.note else ""))
        rows += [f"skipped: {s}" for s in self.skipped]
        rows += [f"problem: {p}" for p in self.problems]
        return "\n".join(rows)


class _Odom:
    def __init__(self, probe: _Probe, topic: str) -> None:
        self.probe = probe
        self.samples: list[tuple] = []  # (t, vx, vy, wz, x, y, vz, z)
        self.lock = threading.Lock()
        self.sub = probe.node.create_subscription(get_message("nav_msgs/msg/Odometry"), topic, self._on,
                                                  qos_profile_sensor_data)

    def _on(self, m) -> None:
        tw, p = m.twist.twist, m.pose.pose.position
        with self.lock:
            self.samples.append((self.probe.now(), tw.linear.x, tw.linear.y, tw.angular.z, p.x, p.y,
                                 tw.linear.z, p.z))

    def window(self, t0: float, t1: float) -> list[tuple]:
        with self.lock:
            return [s for s in self.samples if t0 <= s[0] <= t1]


def _drive(probe: _Probe, pub, msg_type, period: float, vx: float, wz: float, seconds: float,
           vy: float = 0.0, vz: float = 0.0) -> float:
    """Publishes the command for `seconds` (node clock); returns when it started."""
    cmd = msg_type()
    cmd.linear.x = vx
    cmd.linear.y = vy
    cmd.linear.z = vz
    cmd.angular.z = wz
    start = probe.now()
    while probe.now() - start < seconds:
        pub.publish(cmd)
        _wait(probe, period)
    return start


def _crossing(samples: list[tuple], index: int, level: float, rising: bool) -> float | None:
    for a, b in zip(samples, samples[1:]):
        va, vb = a[index], b[index]
        if (rising and va < level <= vb) or (not rising and va > level >= vb):
            return a[0] + (b[0] - a[0]) * ((level - va) / (vb - va) if vb != va else 0.0)
    return None


def _straight_run(probe, pub, twist, period, odom, axis: str, sign: float, speed, accel, decel):
    """One run along `axis` ("x", "y" or, for drones, "z") in direction `sign`: step up to
    `speed`, hold, explicit stop. Returns (latency, accel, steady speed, braking) or None."""
    ramp = speed / max(accel, 0.05)
    v = {"x": (sign * speed, 0.0, 0.0), "y": (0.0, sign * speed, 0.0), "z": (0.0, 0.0, sign * speed)}[axis]
    t0 = _drive(probe, pub, twist, period, v[0], 0.0, 1.5 * ramp + 1.5, v[1], v[2])
    t1 = _drive(probe, pub, twist, period, 0.0, 0.0, 1.5 * speed / max(decel, 0.05) + 1.0)
    k = {"x": 1, "y": 2, "z": 6}[axis]
    along = lambda w: [(smp[0], sign * smp[k]) for smp in w]
    up = along(odom.window(t0, t1))
    down = along(odom.window(t1, probe.now()))
    hold = [v for t, v in up if t > t1 - 0.5]
    if len(hold) < 3:
        return None
    steady = sum(hold) / len(hold)
    t20 = _crossing(up, 1, 0.2 * steady, True)
    t80 = _crossing(up, 1, 0.8 * steady, True)
    b80 = _crossing(down, 1, 0.8 * steady, False)
    b20 = _crossing(down, 1, 0.2 * steady, False)
    if None in (t20, t80, b80, b20) or t80 <= t20 or b20 <= b80:
        return None
    a = 0.6 * steady / (t80 - t20)
    b = 0.6 * steady / (b20 - b80)
    latency = max(0.0, t20 - t0 - 0.2 * steady / a)
    return latency, a, steady, b


def _plan(doc: dict, probe) -> list[tuple[str, float, str, float]]:
    """(axis, sign, name, bearing) for every straight run the body can make."""
    m = doc["motion"]
    runs = [("x", 1.0, "forward", 0.0)]
    if m.get("can_reverse", False) and m["limits"]["x"]["velocity_mps"][0] < 0.0:
        runs.append(("x", -1.0, "backward", math.pi))
    y = m["limits"].get("y", {}).get("velocity_mps", [0.0, 0.0])
    if m.get("can_strafe", False) and (y[0] < 0.0 or y[1] > 0.0):
        seen = snapshot(probe, doc)
        left = room(doc, seen, math.pi / 2) if y[1] > 0.0 else None
        right = room(doc, seen, -math.pi / 2) if y[0] < 0.0 else None
        if (left or -1.0) >= (right or -1.0) and y[1] > 0.0:
            runs.append(("y", 1.0, "left", math.pi / 2))
        elif y[0] < 0.0:
            runs.append(("y", -1.0, "right", -math.pi / 2))
    z = m["limits"].get("z", {}).get("velocity_mps", [0.0, 0.0])
    if m.get("space") == "aerial":
        # Vertical runs: the bearing is None and the room comes from heights (_vertical_room).
        if z[1] > 0.0:
            runs.append(("z", 1.0, "up", None))
        if z[0] < 0.0:
            runs.append(("z", -1.0, "down", None))
    return runs


def _vertical_room(doc: dict, odom, sign: float, t: float) -> tuple[float | None, str]:
    """Room above (sign > 0) or below the drone, from its odometry height. The floor is where
    it started landed (ground_z_m in odom); above, the declared highest flight height."""
    with odom.lock:
        before = [smp for smp in odom.samples if smp[0] <= t]
    if not before:
        return None, "no odometry height"
    z = before[-1][7]
    floor = doc["geometry"].get("ground_z_m", 0.0)
    if sign > 0:
        top = (doc.get("assumptions") or {}).get("max_flight_height_m")
        if not top:
            return None, "no declared max_flight_height_m to measure climbing against"
        return floor + float(top) - z, ""
    return z - floor - 0.5, ""


def _latest_pose(odom, t: float):
    with odom.lock:
        before = [smp for smp in odom.samples if smp[0] <= t]
    return (before[-1][4], before[-1][5]) if before else None


def calibrate(doc: dict, sim_time: bool = False, direct: bool = False) -> Calibration:
    """Needs rclpy initialised and clear space ahead of the robot. A drone must already be
    hovering (taken off by its flight controller) with room above and below for the vertical
    runs."""
    out = Calibration(profile=copy.deepcopy(doc))
    new = out.profile
    m, control = new["motion"], new["control"]
    odom_spec = next((s for s in new["sensors"] if s["kind"] == "odometry"), None) or next(
        (s for s in new["sensors"] if s["kind"] == "wheel_odom"), None)
    if odom_spec is None:
        out.problems.append("no odometry source to measure with")
        return out
    top = m["limits"]["x"]["velocity_mps"][1]
    yaw = m["limits"]["yaw"]
    prov = new.setdefault("provenance", {})

    probe = _Probe(sim_time)
    probe.watch_flags()
    twist = get_message("geometry_msgs/msg/Twist")
    pub = probe.node.create_publisher(twist, probe.command_topic(new, direct), 10)
    period = 1.0 / control["rate_hz"]
    try:
        time.sleep(1.5)  # discovery
        if not direct and not argo_in_setup_mode(probe):
            out.problems.append("Argo is not in setup mode; start 'argo --config argo.yaml --setup' on the "
                                "robot (or pass --direct)")
            return out
        odom = _Odom(probe, odom_spec["source"]["endpoint"])
        latencies = []
        any_run = False
        scales = []
        scale_notes = []
        for axis, sign, name, bearing in _plan(new, probe):
            lim = m["limits"][axis]
            end = 1 if sign > 0 else 0
            top_axis = abs(lim["velocity_mps"][end])
            a_decl = lim.get("accel_mps2", 1.0)
            d_decl = lim.get("decel_mps2", a_decl)
            runs = []
            for share in (0.5, 1.0):
                v = share * top_axis
                need = v * v / (2 * a_decl) + 1.5 * v + v * v / (2 * d_decl) + MARGIN_M
                seen = snapshot(probe, new)
                if bearing is None:
                    clear, missing = _vertical_room(new, odom, sign, probe.now())
                else:
                    clear, missing = room(new, seen, bearing), "no range sensor looks that way"
                if clear is None or clear < need:
                    why = (f"{name} run at {v:.2f} m/s needs {need:.1f} m of room, "
                           + (missing if clear is None else f"have {clear:.1f} m"))
                    (out.problems if name == "forward" else out.skipped).append(why)
                    break
                wall_before = range_toward(new, seen, bearing) if axis == "x" and sign > 0 else None
                started = probe.now()
                pose_before = _latest_pose(odom, started)
                r = _straight_run(probe, pub, twist, period, odom, axis, sign, v, a_decl, d_decl)
                slowed = probe.flags_since(started) & {"slowed", "creep", "blocked", "uncovered", "short_sight"}
                if slowed and not direct:
                    if name != "forward" and slowed & {"creep", "uncovered"}:
                        out.skipped.append(f"{name}: the sensors do not cover that direction at every height, "
                                           "so Argo only creeps there; not measured")
                    else:
                        out.problems.append(f"{name} run at {v:.2f} m/s: Argo's supervisor limited it "
                                            f"({' '.join(sorted(slowed))}); calibrate somewhere more open")
                    break
                if r is None:
                    out.problems.append(f"{name} run at {v:.2f} m/s: the robot did not reach a steady speed")
                    break
                runs.append((v, r))
                if axis == "x" and sign > 0:
                    pose_after = _latest_pose(odom, probe.now())
                    wall_after = range_toward(new, snapshot(probe, new), bearing)
                    if wall_before is None or wall_after is None:
                        scale_notes.append("no return straight ahead to measure against")
                    elif pose_before is None or pose_after is None:
                        scale_notes.append("no odometry pose around the run")
                    elif wall_before - wall_after <= 0.3:
                        scale_notes.append(f"range ahead changed only {wall_before - wall_after:.2f} m")
                    else:
                        moved = math.hypot(pose_after[0] - pose_before[0], pose_after[1] - pose_before[1])
                        scales.append(moved / (wall_before - wall_after))
            if not runs:
                continue
            any_run = True
            latencies += [rr[0] for _, rr in runs]
            path = f"motion.limits.{axis}"
            _write(out, prov, f"{path}.accel_mps2", lim, "accel_mps2", a_decl, min(rr[1] for _, rr in runs))
            _write(out, prov, f"{path}.decel_mps2", lim, "decel_mps2", d_decl, min(rr[3] for _, rr in runs))
            v_full, full = runs[-1]
            if v_full >= top_axis - 1e-9:
                reached = full[2]
                measured_top = min(top_axis, reached if reached < 0.95 * top_axis else top_axis)
                note = "" if reached >= 0.95 * top_axis else f"reached {reached:.2f} of {top_axis:.2f} m/s commanded"
                lim["velocity_mps"][end] = round(math.copysign(measured_top, sign), 3)
                prov[f"{path}.velocity_mps"] = "calibrated"
                out.measurements.append(Measurement(f"{path}.velocity_mps ({name})", top_axis, reached,
                                                    abs(lim["velocity_mps"][end]), note))
        if latencies:
            _write(out, prov, "control.latency_s", control, "latency_s", control.get("latency_s", 0.0),
                   max(latencies), higher=True)
        if not scales and scale_notes:
            out.skipped.append(f"odometry scale: {scale_notes[-1]}")
        if scales:
            scale = sum(scales) / len(scales)
            note = "odometry distance / change in range ahead"
            out.measurements.append(Measurement("odometry scale", 1.0, scale, 1.0, note))
            if abs(scale - 1.0) > 0.05:
                out.problems.append(f"odometry reports {abs(scale - 1.0) * 100:.0f}% {'more' if scale > 1 else 'less'} "
                                    "distance than the robot moved toward what is ahead; fix the wheel radius or "
                                    "encoder scale in the robot's driver and calibrate again (the speeds above are "
                                    "in odometry units)")
        runs = any_run

        radius = m.get("min_turn_radius_m", 0.0)
        if radius > 0.0 and runs:
            v = 0.5 * top
            _drive(probe, pub, twist, period, v, v / radius, 1.0)
            t = probe.now()
            _drive(probe, pub, twist, period, v, v / radius, 1.5)
            arc = odom.window(t, probe.now())
            _drive(probe, pub, twist, period, 0.0, 0.0, 1.0)
            vs = [s[1] for s in arc]
            ws = [abs(s[3]) for s in arc]
            if arc and sum(ws) > 1e-6:
                measured = (sum(vs) / len(vs)) / (sum(ws) / len(ws))
                _write(out, prov, "motion.min_turn_radius_m", m, "min_turn_radius_m", radius, measured, higher=True)
        elif radius == 0.0:
            w = yaw["rate_radps"]
            _drive(probe, pub, twist, period, 0.0, w, 1.0)
            t = probe.now()
            _drive(probe, pub, twist, period, 0.0, w, 1.0)
            spin = odom.window(t, probe.now())
            _drive(probe, pub, twist, period, 0.0, 0.0, 1.0)
            if spin:
                measured = sum(abs(s[3]) for s in spin) / len(spin)
                _write(out, prov, "motion.limits.yaw.rate_radps", yaw, "rate_radps", w, measured)
    finally:
        stop = twist()
        for _ in range(5):
            pub.publish(stop)
            time.sleep(0.02)
        probe.close()
    return out


def _write(out: Calibration, prov: dict, path: str, section: dict, key: str, declared: float, measured: float,
           higher: bool = False) -> None:
    """Conservative merge: the higher of the two when `higher`, else the lower. Within 5% of
    declared confirms the declared value."""
    close = abs(measured - declared) <= 0.05 * max(abs(declared), 1e-9) or (declared == 0 and abs(measured) < 0.01)
    if close:
        value, note = declared, "confirmed"
    else:
        value = max(declared, measured) if higher else min(declared, measured)
        worse = (measured > declared) if higher else (measured < declared)
        note = "worse than declared, using measured" if worse else "better than declared, keeping declared"
    section[key] = round(value, 4)
    prov[path] = "calibrated"
    out.measurements.append(Measurement(path, declared, measured, section[key], note))
