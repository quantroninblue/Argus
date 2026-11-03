"""Rule-based diagnosis of a validation run, from its run log (argus.gazebo.runlog).

Each finding names who is at fault, what shows it (signals and a time range from the log)
and says so in a sentence:

- design: the user's robot (motors that cannot reach the commanded speed, odometry that
  disagrees with the true pose, a sensor slower than declared or late);
- argus: Argus itself (Argo late on its control period, Argo hitting something, Argo giving
  up while nothing stood in its way, the bridge dropping messages). Every one is a bug;
- world: the goal cannot be reached by this body in this world;
- inconclusive: the run failed and no rule explains it.

These are the classical tier; from M3 the reasoning model explains findings and logs.
"""

from __future__ import annotations

import csv
import json
import math
import re
from dataclasses import asdict, dataclass, field
from pathlib import Path

import yaml

DESIGN = "design"
ARGUS = "argus"
WORLD = "world"
INCONCLUSIVE = "inconclusive"
CATEGORIES = (DESIGN, ARGUS, WORLD, INCONCLUSIVE)

# Thresholds. Commanded motion counts as steady once it has held for SETTLE_S; the body must
# then reach at least SPEED_SHARE of it.
SETTLE_S = 1.0
MIN_COMMAND_MPS = 0.15
SPEED_SHARE = 0.7
ODOM_SCALE_TOLERANCE = 0.05
MIN_PATH_FOR_SCALE_M = 1.0
SCALE_STEP_M = 0.05
RATE_SHARE = 0.8
STAMP_LAG_S = 0.1
GAP_PERIODS = 5.0
OVERRUN_SHARE = 0.01


@dataclass
class Finding:
    category: str
    rule: str
    sentence: str
    signals: list[str] = field(default_factory=list)
    time_s: tuple[float, float] | None = None
    values: dict = field(default_factory=dict)


@dataclass
class Diagnosis:
    run: str
    outcome: str  # "reached", "failed" or "collided"
    findings: list[Finding]

    @property
    def blames(self) -> list[str]:
        return sorted({f.category for f in self.findings})

    def to_dict(self) -> dict:
        return {"run": self.run, "outcome": self.outcome, "findings": [asdict(f) for f in self.findings]}


def _rows(path: Path) -> list[list[float]]:
    if not path.exists():
        return []
    with path.open() as f:
        reader = csv.reader(f)
        next(reader, None)
        return [[float(v) for v in r] for r in reader if r]


def _span(rows: list[list[float]]) -> tuple[float, float] | None:
    return (rows[0][0], rows[-1][0]) if rows else None


def _merge(windows: list[tuple[float, float]], gap: float) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    for a, b in sorted(windows):
        if out and a - out[-1][1] <= gap:
            out[-1] = (out[-1][0], max(out[-1][1], b))
        else:
            out.append((a, b))
    return out


def _speed_at(truth: list[list[float]], t: float) -> float | None:
    """True speed around time t from the trajectory (finite difference over about 0.2 s)."""
    if len(truth) < 2:
        return None
    lo = max((r for r in truth if r[0] <= t - 0.1), key=lambda r: r[0], default=None)
    hi = min((r for r in truth if r[0] >= t + 0.1), key=lambda r: r[0], default=None)
    if lo is None or hi is None or hi[0] - lo[0] < 1e-6:
        return None
    return math.dist(hi[1:4], lo[1:4]) / (hi[0] - lo[0])


def commands_not_achieved(commands, truth) -> list[Finding]:
    """Steady commanded speed the body does not reach (design: motors, gearing, mass)."""
    bad: list[tuple[float, float]] = []
    ratios: list[float] = []
    for i, row in enumerate(commands):
        t, v = row[0], math.hypot(row[1], row[2], row[3])
        if v < MIN_COMMAND_MPS:
            continue
        # Steady: every command in the last SETTLE_S within 10% of this one.
        j = i
        while j > 0 and commands[j - 1][0] >= t - SETTLE_S:
            j -= 1
        if commands[j][0] > t - SETTLE_S + 1e-6:
            continue  # not commanded for long enough yet
        if any(abs(math.hypot(r[1], r[2], r[3]) - v) > 0.1 * v for r in commands[j:i]):
            continue
        achieved = _speed_at(truth, t)
        if achieved is None:
            continue
        if achieved < SPEED_SHARE * v:
            bad.append((t, t))
            ratios.append(achieved / v)
    out = []
    for a, b in _merge(bad, 0.5):
        if b - a < 0.5:
            continue
        out.append(Finding(DESIGN, "commands_not_achieved",
                           f"From {a:.1f} s to {b:.1f} s the body moved at under {SPEED_SHARE:.0%} of the steady "
                           "speed Argo commanded: the drive cannot deliver the declared speed (motors, gearing, "
                           "mass or the declared limits)",
                           ["commands.csv", "trajectory.csv"], (a, b), {"worst_share": round(min(ratios), 3)}))
    return out


def _path_length(rows, cols=(1, 2, 3)) -> float:
    return sum(math.dist([a[c] for c in cols], [b[c] for c in cols]) for a, b in zip(rows, rows[1:]))


def _spaced(rows, step_m: float = SCALE_STEP_M) -> list:
    """The rows at least `step_m` apart, so jitter while standing or turning on the spot does
    not add up to distance."""
    out = rows[:1]
    for r in rows[1:]:
        if math.dist(r[1:4], out[-1][1:4]) >= step_m:
            out.append(r)
    return out


def odometry_off(odometry, truth) -> list[Finding]:
    """Odometry distance against the true distance travelled (design: wheel radius, encoders)."""
    true_len = _path_length(_spaced(truth))
    if true_len < MIN_PATH_FOR_SCALE_M or len(odometry) < 2:
        return []
    scale = _path_length(_spaced(odometry)) / true_len
    if abs(scale - 1.0) <= ODOM_SCALE_TOLERANCE:
        return []
    more = "more" if scale > 1.0 else "less"
    return [Finding(DESIGN, "odometry_scale",
                    f"Odometry reported {abs(scale - 1.0):.0%} {more} distance than the body truly travelled "
                    f"({true_len:.1f} m): check the wheel radius and encoder scale in the robot's driver",
                    ["odometry.csv", "trajectory.csv"], _span(truth), {"scale": round(scale, 3)})]


def topic_health(run: dict, doc: dict) -> list[Finding]:
    """Sensor rates and stamp lag against the profile (design); long gaps (the bridge, Argus)."""
    out = []
    sensors = {s["source"]["endpoint"]: s for s in doc.get("sensors", []) if "source" in s}
    for name, st in sorted((run.get("topics") or {}).items()):
        declared = st.get("declared_rate_hz")
        rate = st.get("rate_hz")
        span = (st["first_s"], st["last_s"]) if st.get("first_s") is not None else None
        if name in sensors and declared and rate is not None and rate < RATE_SHARE * declared:
            out.append(Finding(DESIGN, "sensor_rate",
                               f"{sensors[name]['id']} published at {rate:.1f} Hz against the {declared:g} Hz "
                               "declared: Argo sizes its stopping distances on the declared rate",
                               [name], span, {"rate_hz": rate, "declared_rate_hz": declared}))
        lag = st.get("stamp_lag_mean_s")
        if name in sensors and lag is not None and lag > STAMP_LAG_S:
            out.append(Finding(DESIGN, "stamp_lag",
                               f"{sensors[name]['id']} messages arrived {lag * 1000:.0f} ms after their stamps on "
                               "average: a slow driver or a clock that is off",
                               [name], span, {"stamp_lag_mean_s": lag}))
        gap = st.get("longest_gap_s")
        if declared and rate is not None and rate >= RATE_SHARE * declared and gap and gap > GAP_PERIODS / declared:
            out.append(Finding(ARGUS, "message_gap",
                               f"{name} went {gap:.2f} s without a message though its rate was fine: "
                               "messages were dropped on the way (the bridge, or Argo's driver)",
                               [name], span, {"longest_gap_s": gap, "declared_rate_hz": declared}))
    return out


_STOP_LINE = re.compile(r"stop ticks=(\d+) overruns=(\d+) worst_ms=([0-9.eE+-]+)")


def argo_timing(argo_log: str, doc: dict) -> list[Finding]:
    """Argo late on its control period (Argus)."""
    m = None
    for m in _STOP_LINE.finditer(argo_log):
        pass
    if m is None:
        return []
    ticks, overruns, worst_ms = int(m.group(1)), int(m.group(2)), float(m.group(3))
    period_ms = 1000.0 / float(doc["control"]["rate_hz"])
    if ticks == 0 or (overruns <= OVERRUN_SHARE * ticks and worst_ms <= 2.0 * period_ms):
        return []
    return [Finding(ARGUS, "control_overruns",
                    f"Argo overran its {period_ms:.0f} ms control period on {overruns} of {ticks} ticks "
                    f"(worst {worst_ms:.0f} ms)",
                    ["argo.log"], None, {"ticks": ticks, "overruns": overruns, "worst_ms": worst_ms})]


_BLOCKED = re.compile(r"^([0-9.]+) event blocked\b.*?\bcause=(\w+)", re.M)

# What each of Argo's failure causes blames, and how to say it.
_CAUSES = {
    "dead_end_observed": (WORLD, "Argo found no way to the goal from where it was: out of this body's reach in this "
                                 "world, or the hub's route led into a dead end"),
    "searched_no_goal": (WORLD, "The goal could not be reached from anywhere Argo could get to"),
    "temporary_obstruction": (WORLD, "Something stood in the way and did not move"),
    "controller_no_progress": (ARGUS, "Argo stopped making progress toward the goal without finding a dead end: the "
                                      "planner stalled"),
    "compute_deadline_miss": (ARGUS, "Argo's plan came too late for the control loop, so the supervisor stopped "
                                     "the body"),
    "localization_invalid": (DESIGN, "Odometry failed or went stale, so Argo stopped: check the odometry source"),
    "sensor_invalid": (DESIGN, "A range sensor went stale, so Argo stopped: check the sensor and its driver"),
    "link_lost": (ARGUS, "The link between Argo and the hub dropped during a simulated run, where both run on one "
                         "machine"),
}


# Argo's failures that depend on what it saw.
SEEN_WRONG = ("controller_no_progress", "collision")


def sensor_mounts(run: dict) -> list[Finding]:
    """The design's own sensors away from their declared mounts (design). Argo places every
    return by the declared mount, so it saw the world wrong for the whole run."""
    return [Finding(DESIGN, "sensor_mount", text, ["model.sdf", "profile.yaml"]) for text in run.get("mount_findings") or []]


def outcome_findings(run: dict, argo_log: str) -> tuple[str, list[Finding]]:
    out = run.get("outcome") or {}
    findings = []
    seen: dict[str, list[float]] = {}
    for m in _BLOCKED.finditer(argo_log):
        seen.setdefault(m.group(2), []).append(float(m.group(1)))
    if out.get("collided"):
        what = ", ".join(str(c) for c in (out.get("collisions") or [])[:3]) or "an obstacle"
        findings.append(Finding(ARGUS, "collision",
                                f"Argo drove the body into {what}: its supervisor must stop short of anything it "
                                "can see, so this is a bug (or a blind spot the profile does not declare)",
                                ["trajectory.csv", "run.json"], None, {"collisions": out.get("collisions")}))
    for cause, times in sorted(seen.items()):
        category, sentence = _CAUSES.get(cause, (INCONCLUSIVE, f"Argo reported itself blocked ({cause})"))
        # Where the goal lies is only in question when the run did not get there.
        if category == WORLD and out.get("goal_reached"):
            continue
        findings.append(Finding(category, cause, sentence, ["argo.log"], (min(times), max(times)),
                                {"times": len(times)}))
    if out.get("mission_state") == "reached" and not out.get("goal_reached") and not out.get("collided"):
        away = ""
        pose, goal = out.get("final_pose") or [], run.get("goal") or []
        if len(pose) >= 2 and len(goal) >= 2:
            away = f" ({math.dist(pose[:2], goal[:2]):.2f} m from it)"
        findings.append(Finding(ARGUS, "false_arrival",
                                f"Argo and the hub took the goal as reached while the robot was not there{away}: "
                                "where Argus thought the robot was is wrong",
                                ["run.json", "odometry.csv", "trajectory.csv"], None,
                                {"final_pose": pose[:3], "goal": goal[:3]}))
    outcome = "collided" if out.get("collided") else "reached" if out.get("goal_reached") else "failed"
    return outcome, findings


def diagnose(log_dir: str | Path) -> Diagnosis:
    """All the rules over one run log directory."""
    log_dir = Path(log_dir)
    run = json.loads((log_dir / "run.json").read_text())
    doc = yaml.safe_load((log_dir / "profile.yaml").read_text()) if (log_dir / "profile.yaml").exists() else {}
    argo_log = (log_dir / "argo.log").read_text() if (log_dir / "argo.log").exists() else ""
    truth = _rows(log_dir / "trajectory.csv")
    commands = _rows(log_dir / "commands.csv")
    odometry = _rows(log_dir / "odometry.csv")

    outcome, findings = outcome_findings(run, argo_log)
    mounts = sensor_mounts(run)
    if mounts:
        # Argo saw the world through misplaced sensors, so what it then did is no evidence
        # against Argus until the design is fixed.
        for f in findings:
            if f.category == ARGUS and f.rule in SEEN_WRONG:
                f.category = INCONCLUSIVE
                f.sentence += " (may follow from the design's misplaced sensors; fix those and run again)"
        findings = mounts + findings
    model_findings = commands_not_achieved(commands, truth) + odometry_off(odometry, truth)
    if ((run.get("body") or {}).get("drivetrain") or {}).get("from") != "the design":
        # No user design: Gazebo ran the model Argus builds from the profile, so its drive and
        # odometry are Argus's own work.
        for f in model_findings:
            f.category = ARGUS
            f.sentence = "In the Gazebo model Argus built from this profile: " + f.sentence[0].lower() + f.sentence[1:]
    findings += model_findings
    if doc:
        findings += topic_health(run, doc)
        findings += argo_timing(argo_log, doc)
    if outcome != "reached" and not findings:
        findings.append(Finding(INCONCLUSIVE, "unexplained",
                                "The run did not reach its goal and no rule explains why; read the logs",
                                ["run.json"], None, {"mission_state": (run.get("outcome") or {}).get("mission_state")}))
    return Diagnosis(log_dir.name, outcome, findings)


def job_report(diagnoses: list[Diagnosis]) -> dict:
    """Counts across a job's runs: outcomes, findings per category and per rule."""
    report = {"runs": len(diagnoses), "outcomes": {}, "categories": {c: 0 for c in CATEGORIES}, "rules": {},
              "argus_runs": []}
    for d in diagnoses:
        report["outcomes"][d.outcome] = report["outcomes"].get(d.outcome, 0) + 1
        for f in d.findings:
            report["categories"][f.category] += 1
            report["rules"][f.rule] = report["rules"].get(f.rule, 0) + 1
        if ARGUS in d.blames:
            report["argus_runs"].append(d.run)
    return report
