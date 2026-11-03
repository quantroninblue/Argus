"""Gazebo windows for validation runs, opened by the argus command in the user's own desktop
session; the hub itself never opens one.

- Live: a window attaches to the run Gazebo is simulating now (`gz sim -g` on that run's
  partition). Asked for before a job's first run, it opens as each run starts and closes as it
  ends. Closing it by hand leaves the run going; the next run opens a new one.
- Replay: Gazebo plays back a finished run from the state it recorded (`gz sim --playback`),
  with its own play, pause and seek controls, beside a timeline of what Argo reported and what
  the diagnosis found, so the user knows where to look.
"""

from __future__ import annotations

import os
import random
import re
import shutil
import signal
import subprocess
from dataclasses import dataclass
from pathlib import Path

ATTACH_VARS = ("GZ_PARTITION", "GZ_IP", "GZ_SIM_RESOURCE_PATH")
DESKTOP_VARS = ("DISPLAY", "WAYLAND_DISPLAY")


def problems() -> list[str]:
    """What stops this terminal from opening a Gazebo window."""
    out = []
    if not any(os.environ.get(v) for v in DESKTOP_VARS):
        out.append("no desktop session here (no DISPLAY or WAYLAND_DISPLAY): run this from a terminal on "
                   "the hub machine's desktop")
    if shutil.which("gz") is None:
        out.append("Gazebo ('gz') is not on PATH here: source /opt/ros/jazzy/setup.bash first")
    return out


def _env(extra: dict) -> dict:
    env = dict(os.environ)
    env.update({k: v for k, v in extra.items() if k in ATTACH_VARS and v})
    return env


def _start(cmd: list[str], env: dict, log: Path | None) -> subprocess.Popen:
    out = open(log, "wb") if log else subprocess.DEVNULL
    try:
        return subprocess.Popen(cmd, env=env, stdout=out, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                start_new_session=True)
    finally:
        if log:
            out.close()


def close(p: subprocess.Popen | None, wait_s: float = 5.0) -> None:
    """Closes a window and everything it started (Gazebo's GUI runs as a process group)."""
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(p.pid, signal.SIGINT)
        p.wait(wait_s)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        p.wait()


def attach(live_env: dict, log: Path | None = None) -> subprocess.Popen:
    """A Gazebo window on a live run; `live_env` is the run's attach info from the hub."""
    return _start(["gz", "sim", "-g"], _env(live_env), log)


def replay(run_dir: Path, resource_path: str = "", log: Path | None = None) -> subprocess.Popen:
    """Gazebo playing back a finished run, in a partition of its own so it never meets a live run."""
    rec = Path(run_dir) / "replay"
    if not rec.is_dir():
        raise FileNotFoundError(f"{run_dir} has no recording (replay/); runs from before recording was added have none")
    env = _env({"GZ_PARTITION": f"argus-replay-{os.getpid()}-{random.randint(0, 1 << 30)}", "GZ_IP": "127.0.0.1",
                "GZ_SIM_RESOURCE_PATH": resource_path})
    return _start(["gz", "sim", "--playback", str(rec)], env, log)


@dataclass
class Moment:
    t_s: float
    source: str  # argo, finding
    text: str


_LINE = re.compile(r"^([0-9]+(?:\.[0-9]+)?) (event|flags) (.*)$")
# Argo's routine events; the timeline shows what explains a run.
_ROUTINE = {"waypoint_reached", "plan_accepted"}


def timeline(run_dir: Path, findings: list) -> list[Moment]:
    """What happened when, in simulated time: Argo's events from argo.log and each finding's
    time range, oldest first."""
    out: list[Moment] = []
    log = Path(run_dir) / "argo.log"
    if log.is_file():
        for line in log.read_text(errors="replace").splitlines():
            m = _LINE.match(line)
            if m is None or m.group(3).split()[0] in _ROUTINE:
                continue
            text = m.group(3).strip() if m.group(2) == "event" else f"safety flags: {m.group(3).strip()}"
            out.append(Moment(float(m.group(1)), "argo", text))
    for f in findings:
        t = f.get("time_s") if isinstance(f, dict) else None
        if t:
            out.append(Moment(float(t[0]), "finding", f"{f['category']}: {f['sentence']} (to {float(t[1]):.1f} s)"))
    return sorted(out, key=lambda m: m.t_s)


def format_timeline(moments: list[Moment], limit: int = 40) -> str:
    lines = [f"{m.t_s:7.1f} s  {m.source:<7} {m.text}" for m in moments[:limit]]
    if len(moments) > limit:
        lines.append(f"... {len(moments) - limit} more in argo.log")
    return "\n".join(lines) if lines else "nothing of note: no events or timed findings"
