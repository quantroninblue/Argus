"""Child processes of a Gazebo run, and cleanup that happens however the run ends.

Each child (Gazebo, the bridge, Argo, ros2 bag) runs in its own process group, so stopping it
stops whatever it started. A `Reaper` is a small separate process holding a pipe from the
runner. When the pipe closes, whether the runner finished, raised, was killed or crashed, the
reaper stops every group still registered with it and deletes the run's private keys from the
work directory. Logs are kept.
"""

from __future__ import annotations

import contextlib
import os
import signal
import subprocess
import sys
from pathlib import Path

_REAPER = r"""
import os, signal, sys, time
from pathlib import Path

signal.signal(signal.SIGINT, signal.SIG_IGN)
signal.signal(signal.SIGTERM, signal.SIG_IGN)
work = Path(sys.argv[1])
groups = set()
for line in sys.stdin:
    cmd, _, arg = line.strip().partition(" ")
    if cmd == "add":
        groups.add(int(arg))
    elif cmd == "drop":
        groups.discard(int(arg))

def alive(g):
    try:
        os.killpg(g, 0)
        return True
    except (ProcessLookupError, PermissionError):
        return False

for g in groups:
    if alive(g):
        os.killpg(g, signal.SIGINT)
deadline = time.monotonic() + 5.0
while any(alive(g) for g in groups) and time.monotonic() < deadline:
    time.sleep(0.1)
for g in groups:
    if alive(g):
        os.killpg(g, signal.SIGKILL)
for key in work.rglob("*.key"):
    key.unlink(missing_ok=True)
"""

_active: "Reaper | None" = None


class Reaper:
    """Stops registered process groups and removes `*.key` under `work` once closed or once
    the runner dies."""

    def __init__(self, work: Path) -> None:
        self._p = subprocess.Popen([sys.executable, "-I", "-c", _REAPER, str(work)], stdin=subprocess.PIPE,
                                   text=True, start_new_session=True)

    def _send(self, line: str) -> None:
        if self._p.stdin and not self._p.stdin.closed:
            try:
                self._p.stdin.write(line + "\n")
                self._p.stdin.flush()
            except BrokenPipeError:
                pass

    def add(self, p: subprocess.Popen) -> None:
        self._send(f"add {p.pid}")

    def drop(self, p: subprocess.Popen) -> None:
        self._send(f"drop {p.pid}")

    def close(self, wait_s: float = 10.0) -> None:
        if self._p.stdin and not self._p.stdin.closed:
            self._p.stdin.close()
        try:
            self._p.wait(wait_s)
        except subprocess.TimeoutExpired:
            pass


@contextlib.contextmanager
def reaping(work: Path):
    """Within the block, every `start_process` is registered with one reaper for `work`, and
    SIGTERM unwinds the runner like Ctrl-C so its own cleanup runs too."""
    global _active
    outer = _active
    reaper = Reaper(work)
    _active = reaper
    old_term = None
    try:
        old_term = signal.signal(signal.SIGTERM, _raise_interrupt)
    except ValueError:
        pass  # not the main thread; the reaper still covers it
    try:
        yield reaper
    finally:
        _active = outer
        if old_term is not None:
            signal.signal(signal.SIGTERM, old_term)
        reaper.close()


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt(f"signal {signum}")


def start_process(cmd: list[str], log: Path, env: dict) -> subprocess.Popen:
    p = subprocess.Popen(cmd, stdout=log.open("w"), stderr=subprocess.STDOUT, env=env, start_new_session=True)
    if _active is not None:
        _active.add(p)
    return p


def stop_process(p: subprocess.Popen, sig=signal.SIGINT, wait_s: float = 5.0) -> None:
    try:
        if p.poll() is not None:
            # The leader is gone, but what it started (ros2 run's node, say) may not be.
            with contextlib.suppress(ProcessLookupError, PermissionError):
                os.killpg(p.pid, signal.SIGKILL)
            return
        try:
            os.killpg(p.pid, sig)
            p.wait(wait_s)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    finally:
        if _active is not None:
            _active.drop(p)
