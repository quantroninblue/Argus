"""`python3 -m argus hub DIR`: the hub for a trial, with an operator console.

DIR holds `profiles/*.yaml`, `robots.yaml` (node_id: {profile, site}) and `pki/` with
`ca.pem`, `hub.pem` and `hub.key` (see `python3 -m argus certs`). Robots connect with
certificates signed by that CA; a robot's certificate name must be its node id.

Console commands (one per line):
  goal ROBOT X Y [Z] [TOLERANCE]   send a robot to a point in its site frame
  cancel ROBOT                     stop the robot's mission
  status                           robots, link and mission state
  quit
"""

from __future__ import annotations

import asyncio
import sys
import time
from pathlib import Path

from argus.protocol.v1 import node_messages_pb2

from .gateway import Hub
from .intake import IntakeConfig, parse_rate
from .registry import Registry
from .server import HubServer, TlsFiles
from .session import HubConfig

USAGE = "commands: goal ROBOT X Y [Z] [TOLERANCE] | cancel ROBOT | status | quit"


def _status(hub: Hub, now: float) -> str:
    lines = []
    for node_id in sorted(r.node_id for r in hub.registry.robots()):
        s = hub.sessions.get(node_id)
        if s is None or s.connection is None:
            lines.append(f"{node_id}: not connected")
            continue
        link = node_messages_pb2.LinkState.Name(s.link_state).removeprefix("LINK_STATE_").lower()
        pose = getattr(s, "pose", None)
        where = "pose unknown" if pose is None else "at " + " ".join(f"{v:.2f}" for v in pose)
        lines.append(f"{node_id}: {where}, link {link}, mission {s.mission.state.value}, "
                     f"heard {now - s.last_heard:.1f} s ago" + (", FAILSAFE" if s.failsafe else "")
                     + f", evidence {s.evidence_demand / 1000:.0f} of {s.evidence_rate / 1000:.0f} KB/s"
                     + (f", {s.evidence_dropped} messages dropped" if s.evidence_dropped else ""))
    cap = hub.intake
    lines.append(f"evidence intake {cap.capacity / 1000:.0f} of {cap.config.bytes_per_s / 1000:.0f} KB/s")
    return "\n".join(lines)


def command(hub: Hub, line: str, now: float) -> str | None:
    """Runs one console command; returns the reply, or None to quit."""
    parts = line.split()
    if not parts:
        return ""
    cmd, args = parts[0], parts[1:]
    try:
        if cmd == "quit":
            return None
        if cmd == "status":
            return _status(hub, now)
        if cmd == "goal" and 3 <= len(args) <= 5:
            node_id = args[0]
            xyz = [float(v) for v in args[1:4]]
            if len(xyz) == 2:
                xyz.append(0.0)
            tol = float(args[4]) if len(args) == 5 else 0.4
            if hub.registry.robot(node_id) is None:
                return f"no robot {node_id!r} in robots.yaml"
            hub.set_goal(node_id, tuple(xyz), tol, now)
            return f"{node_id}: goal {xyz[0]:g} {xyz[1]:g} {xyz[2]:g}, tolerance {tol:g} m"
        if cmd == "cancel" and len(args) == 1:
            if hub.registry.robot(args[0]) is None:
                return f"no robot {args[0]!r} in robots.yaml"
            hub.cancel(args[0])
            return f"{args[0]}: cancelled"
    except ValueError as e:
        return f"{e}\n{USAGE}"
    return USAGE


async def serve(directory: Path, listen: str, lines=None, intake_bytes_per_s: float | None = None) -> None:
    """Runs until `quit` or end of input. `lines` (an async iterator) replaces the terminal.
    `intake_bytes_per_s` is the bandwidth set aside for map evidence from all robots."""
    registry = Registry.from_dir(directory)
    pki = directory / "pki"
    config = HubConfig()
    if intake_bytes_per_s is not None:
        config.intake = IntakeConfig(bytes_per_s=intake_bytes_per_s)
    server = HubServer(Hub(registry, config=config), address=listen,
                       tls=TlsFiles(pki / "ca.pem", pki / "hub.pem", pki / "hub.key"))
    port = await server.start()
    host = listen.rsplit(":", 1)[0]
    print(f"hub listening on {host}:{port}, {len(registry.robots())} robot(s) registered", flush=True)
    print(USAGE, flush=True)
    loop = asyncio.get_running_loop()

    async def terminal():
        while True:
            line = await loop.run_in_executor(None, sys.stdin.readline)
            if not line:
                return
            yield line

    try:
        async for line in (lines if lines is not None else terminal()):
            reply = command(server.hub, line, time.time())
            if reply is None:
                break
            if reply:
                print(reply, flush=True)
    finally:
        await server.stop()


def main(directory: str, listen: str, intake: str | None = None) -> int:
    d = Path(directory)
    missing = [p for p in ("robots.yaml", "pki/ca.pem", "pki/hub.pem", "pki/hub.key") if not (d / p).exists()]
    if missing or not (d / "profiles").is_dir():
        print(f"{d}: missing {', '.join(missing + ([] if (d / 'profiles').is_dir() else ['profiles/']))}",
              file=sys.stderr)
        return 1
    try:
        rate = parse_rate(intake) if intake else None
    except ValueError as e:
        print(e, file=sys.stderr)
        return 2
    try:
        asyncio.run(serve(d, listen, intake_bytes_per_s=rate))
    except KeyboardInterrupt:
        pass
    return 0

