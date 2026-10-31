"""Calibration by stepped motion tests (setup step 5), watched through the robot's odometry.

Forward at half and then full declared speed, then an explicit stop: command latency,
acceleration, top speed and braking. Then turning: in place at the declared yaw rate, or for
a body with a turn radius, a full-lock arc to measure the real radius.

Results are written back conservatively, with provenance `calibrated`: the lower of declared
and measured for speeds, acceleration, braking and yaw rate; the higher for latency and turn
radius. A measurement that confirms the declared value marks it verified. What the data does
not determine keeps its declared value.

Every straight run first checks the forward scan for the distance it needs. Motion goes
through Argo in setup mode unless `direct` (see live.py); a top speed Argo's supervisor slowed
is not written back, since it measures the supervisor, not the robot.
"""

from __future__ import annotations

import copy
import math
import threading
import time
from dataclasses import dataclass, field

from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

from .live import ARGO_SETUP_TOPIC, _clear_ahead, _Probe, _wait

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

    def text(self) -> str:
        rows = []
        for m in self.measurements:
            meas = "not measured" if m.measured is None else f"{m.measured:.3g}"
            rows.append(f"{m.field:34s} declared {m.declared:<8.3g} measured {meas:<12s} now {m.value:.3g}"
                        + (f"  ({m.note})" if m.note else ""))
        rows += [f"problem: {p}" for p in self.problems]
        return "\n".join(rows)


class _Odom:
    def __init__(self, probe: _Probe, topic: str) -> None:
        self.probe = probe
        self.samples: list[tuple] = []  # (t, vx, vy, wz, x, y)
        self.lock = threading.Lock()
        self.sub = probe.node.create_subscription(get_message("nav_msgs/msg/Odometry"), topic, self._on,
                                                  qos_profile_sensor_data)

    def _on(self, m) -> None:
        tw, p = m.twist.twist, m.pose.pose.position
        with self.lock:
            self.samples.append((self.probe.now(), tw.linear.x, tw.linear.y, tw.angular.z, p.x, p.y))

    def window(self, t0: float, t1: float) -> list[tuple]:
        with self.lock:
            return [s for s in self.samples if t0 <= s[0] <= t1]


def _drive(probe: _Probe, pub, msg_type, period: float, vx: float, wz: float, seconds: float) -> float:
    """Publishes the command for `seconds` (node clock); returns when it started."""
    cmd = msg_type()
    cmd.linear.x = vx
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


def _straight_run(probe, pub, twist, period, odom, speed, accel, decel):
    """One run: step up to `speed`, hold, explicit stop. (latency, accel, steady, braking)."""
    ramp = speed / max(accel, 0.05)
    t0 = _drive(probe, pub, twist, period, speed, 0.0, 1.5 * ramp + 1.5)
    t1 = _drive(probe, pub, twist, period, 0.0, 0.0, 1.5 * speed / max(decel, 0.05) + 1.0)
    up = odom.window(t0, t1)
    down = odom.window(t1, probe.now())
    hold = [s[1] for s in up if s[0] > t1 - 0.5]
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


def calibrate(doc: dict, sim_time: bool = False, direct: bool = False) -> Calibration:
    """Needs rclpy initialised and clear space ahead of the robot."""
    out = Calibration(profile=copy.deepcopy(doc))
    new = out.profile
    m, control = new["motion"], new["control"]
    odom_spec = next((s for s in new["sensors"] if s["kind"] == "odometry"), None) or next(
        (s for s in new["sensors"] if s["kind"] == "wheel_odom"), None)
    if odom_spec is None:
        out.problems.append("no odometry source to measure with")
        return out
    lx = m["limits"]["x"]
    top = lx["velocity_mps"][1]
    accel = lx.get("accel_mps2", 1.0)
    decel = lx.get("decel_mps2", accel)
    yaw = m["limits"]["yaw"]
    prov = new.setdefault("provenance", {})

    probe = _Probe(sim_time)
    probe.watch_flags()
    twist = get_message("geometry_msgs/msg/Twist")
    pub = probe.node.create_publisher(twist, probe.command_topic(new, direct), 10)
    period = 1.0 / control["rate_hz"]
    try:
        time.sleep(1.5)  # discovery
        if not direct and probe.node.count_subscribers(ARGO_SETUP_TOPIC) == 0:
            out.problems.append("Argo is not in setup mode; start 'argo --config argo.yaml --setup' on the "
                                "robot (or pass --direct)")
            return out
        odom = _Odom(probe, odom_spec["source"]["endpoint"])
        stats = {s["source"]["endpoint"]: {"spec": s} for s in new["sensors"] if s["kind"] == "range_2d"}
        runs = []
        for share in (0.5, 1.0):
            v = share * top
            need = v * v / (2 * accel) + 1.5 * v + v * v / (2 * decel) + MARGIN_M
            clear = _clear_ahead(probe, new, stats)
            if clear is None or clear < need:
                out.problems.append(f"straight run at {v:.2f} m/s needs {need:.1f} m clear ahead, "
                                    + ("no 2D scan to check" if clear is None else f"have {clear:.1f} m"))
                break
            started = probe.now()
            r = _straight_run(probe, pub, twist, period, odom, v, accel, decel)
            slowed = probe.flags_since(started) & {"slowed", "creep", "blocked", "uncovered", "short_sight"}
            if slowed and not direct:
                out.problems.append(f"straight run at {v:.2f} m/s: Argo's supervisor limited it "
                                    f"({' '.join(sorted(slowed))}); calibrate somewhere more open")
                break
            if r is None:
                out.problems.append(f"straight run at {v:.2f} m/s: the robot did not reach a steady speed")
                break
            runs.append((v, r))

        if runs:
            latency = max(r[0] for _, r in runs)
            a = min(r[1] for _, r in runs)
            b = min(r[3] for _, r in runs)
            v_full, full = runs[-1]
            reached = full[2]
            _write(out, prov, "control.latency_s", control, "latency_s", control.get("latency_s", 0.0), latency,
                   higher=True)
            _write(out, prov, "motion.limits.x.accel_mps2", lx, "accel_mps2", accel, a)
            _write(out, prov, "motion.limits.x.decel_mps2", lx, "decel_mps2", decel, b)
            if v_full >= top - 1e-9:
                measured_top = reached if reached < 0.95 * top else top
                note = "" if reached >= 0.95 * top else f"reached {reached:.2f} of {top:.2f} m/s commanded"
                lx["velocity_mps"][1] = round(min(top, measured_top), 3)
                prov["motion.limits.x.velocity_mps"] = "calibrated"
                out.measurements.append(Measurement("motion.limits.x.velocity_mps", top, reached,
                                                    lx["velocity_mps"][1], note))

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
