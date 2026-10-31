"""Cleanup of a Gazebo run's processes and keys when the runner is interrupted or killed.
Uses plain shell processes in place of Gazebo, so it needs no ROS."""

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

RUNNER = r"""
import sys, time
from pathlib import Path
from argus.gazebo.procs import reaping, start_process

work = Path(sys.argv[1])
(work / "robot-1.key").write_text("secret")
with reaping(work):
    # The leader starts a child of its own, as `ros2 run` does.
    p = start_process(["sh", "-c", "sleep 60 & echo $! > child.pid; wait"], work / "child.log", {"PATH": "/usr/bin:/bin"})
    print(p.pid, flush=True)
    try:
        time.sleep(60)
    except KeyboardInterrupt:
        print("interrupted", flush=True)
        raise
"""


def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    # A zombie still answers kill(0); it is gone for our purposes.
    stat = Path(f"/proc/{pid}/stat")
    return stat.exists() and stat.read_text().split()[2] != "Z"


def _start_runner(work: Path) -> tuple[subprocess.Popen, int, int]:
    env = dict(os.environ, PYTHONPATH=str(ROOT / "python"))
    runner = subprocess.Popen([sys.executable, "-c", RUNNER, str(work)], cwd=work, env=env,
                              stdout=subprocess.PIPE, text=True)
    leader = int(runner.stdout.readline())
    deadline = time.monotonic() + 5.0
    while not (work / "child.pid").exists() or not (work / "child.pid").read_text().strip():
        assert time.monotonic() < deadline, "child never started"
        time.sleep(0.05)
    child = int((work / "child.pid").read_text())
    return runner, leader, child


def _wait_gone(pids, timeout_s=8.0) -> list[int]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        left = [p for p in pids if _alive(p)]
        if not left:
            return []
        time.sleep(0.1)
    return [p for p in pids if _alive(p)]


@pytest.mark.parametrize("sig", [signal.SIGKILL, signal.SIGTERM, signal.SIGINT])
def test_an_ended_runner_leaves_no_processes_and_no_keys(tmp_path, sig):
    runner, leader, child = _start_runner(tmp_path)
    assert _alive(leader) and _alive(child)
    runner.send_signal(sig)
    runner.wait(10)
    assert _wait_gone([leader, child]) == []
    key = tmp_path / "robot-1.key"
    deadline = time.monotonic() + 3.0
    while key.exists() and time.monotonic() < deadline:
        time.sleep(0.05)
    assert not key.exists()
    assert (tmp_path / "child.log").exists()  # logs stay
