"""Talks to a running hub over its app interface (app.proto), for the argus command."""

from __future__ import annotations

import time
from collections.abc import Iterator

import grpc

from argus.protocol.v1 import app_pb2, app_pb2_grpc

from .designs import Bundle
from .home import Home

TIMEOUT_S = 5.0
DESIGN_TIMEOUT_S = 120.0  # storing a design runs Gazebo's check of it
MAX_MESSAGE_BYTES = 300 * 1024 * 1024


class HubUnavailable(RuntimeError):
    pass


class AppClient:
    def __init__(self, home: Home) -> None:
        self.home = home
        if not home.socket.exists():
            raise HubUnavailable(f"no hub is running at {home.root} (start one with: python3 -m argus hub start"
                                 + ("" if home.root == Home.at().root else f" --home {home.root}") + ")")
        self._channel = grpc.secure_channel(f"unix:{home.socket}",
                                            grpc.local_channel_credentials(grpc.LocalConnectionType.UDS),
                                            options=[("grpc.max_send_message_length", MAX_MESSAGE_BYTES),
                                                     ("grpc.max_receive_message_length", MAX_MESSAGE_BYTES)])
        self._stub = app_pb2_grpc.HubAppStub(self._channel)

    def close(self) -> None:
        self._channel.close()

    def __enter__(self) -> "AppClient":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def _call(self, method, request, timeout: float = TIMEOUT_S):
        try:
            return method(request, timeout=timeout)
        except grpc.RpcError as e:
            if e.code() == grpc.StatusCode.UNAVAILABLE:
                raise HubUnavailable(f"the hub at {self.home.root} is not answering") from e
            raise HubUnavailable(f"the hub refused: {e.details()}") from e

    def status(self) -> app_pb2.HubStatus:
        return self._call(self._stub.Status, app_pb2.StatusRequest())

    def robots(self) -> app_pb2.RobotsReply:
        return self._call(self._stub.Robots, app_pb2.RobotsRequest())

    def goal(self, node_id: str, x: float, y: float, z: float = 0.0, tolerance_m: float = 0.0) -> app_pb2.Reply:
        return self._call(self._stub.SetGoal, app_pb2.GoalRequest(node_id=node_id, x=x, y=y, z=z,
                                                                  tolerance_m=tolerance_m))

    def cancel(self, node_id: str) -> app_pb2.Reply:
        return self._call(self._stub.Cancel, app_pb2.CancelRequest(node_id=node_id))

    def add_design(self, bundle: Bundle) -> app_pb2.AddDesignReply:
        request = app_pb2.AddDesignRequest(
            name=bundle.name, files=[app_pb2.DesignFile(path=k, data=v) for k, v in sorted(bundle.files.items())],
            collect_findings=[app_pb2.Finding(severity=f.severity, where=f.where, text=f.text) for f in bundle.findings])
        return self._call(self._stub.AddDesign, request, DESIGN_TIMEOUT_S)

    def designs(self) -> app_pb2.DesignsReply:
        return self._call(self._stub.Designs, app_pb2.DesignsRequest())

    def design(self, name: str, version: str = "") -> app_pb2.DesignReply:
        return self._call(self._stub.Design, app_pb2.DesignRequest(name=name, version=version))

    def watch(self) -> Iterator[app_pb2.HubEvent]:
        try:
            yield from self._stub.Watch(app_pb2.WatchRequest())
        except grpc.RpcError as e:
            if e.code() == grpc.StatusCode.CANCELLED:
                return
            raise HubUnavailable(f"the hub at {self.home.root} stopped answering: {e.details()}") from e


def format_status(st: app_pb2.HubStatus, now: float | None = None) -> str:
    now = time.time() if now is None else now
    up = max(0.0, now - st.started_at)
    return (f"Argus {st.version} at {st.home}, up {up / 60:.0f} min\n"
            f"robots connect to {st.robot_address}; {st.robots_connected} of {st.robots_registered} connected\n"
            f"you are {st.caller}")


def format_robots(reply: app_pb2.RobotsReply) -> str:
    lines = []
    for r in reply.robots:
        if not r.connected:
            heard = "never heard" if r.seconds_since_heard < 0 else f"last heard {r.seconds_since_heard:.0f} s ago"
            lines.append(f"{r.node_id}: not connected ({heard}), profile {r.profile_id}, site {r.site_id}")
            continue
        where = "pose unknown" if not r.has_pose else f"at {r.x:.2f} {r.y:.2f} {r.z:.2f}"
        lines.append(f"{r.node_id}: {where}, link {r.link}, mission {r.mission}, "
                     f"heard {r.seconds_since_heard:.1f} s ago" + (", FAILSAFE" if r.failsafe else "")
                     + f", evidence {r.evidence_demand_bytes_per_s / 1000:.0f} of "
                       f"{r.evidence_rate_bytes_per_s / 1000:.0f} KB/s"
                     + (f", {r.evidence_dropped} messages dropped" if r.evidence_dropped else ""))
    if not reply.robots:
        lines.append("no robots registered")
    lines.append(f"evidence intake {reply.intake_capacity_bytes_per_s / 1000:.0f} of "
                 f"{reply.intake_configured_bytes_per_s / 1000:.0f} KB/s")
    return "\n".join(lines)


def format_event(e: app_pb2.HubEvent) -> str:
    stamp = time.strftime("%H:%M:%S", time.localtime(e.stamp))
    return " ".join(x for x in (stamp, e.kind, e.node_id, e.detail) if x)


def format_findings(findings) -> str:
    order = {"error": 0, "warning": 1, "note": 2}
    lines = []
    for f in sorted(findings, key=lambda f: order.get(f.severity, 3)):
        lines.append(f"  {f.severity}: {f.where + ': ' if f.where else ''}{f.text}")
    return "\n".join(lines)


def format_design(v: app_pb2.DesignVersion) -> str:
    when = time.strftime("%Y-%m-%d %H:%M", time.localtime(v.created))
    parts = [f"{v.name} {v.version} ({when})", "runnable" if v.runnable else "cannot run",
             f"profile {v.profile_id}" if v.profile_id else "no profile",
             "own Gazebo model" if v.has_gazebo_model else "no Gazebo model (Argus adds a drive and sensors)"]
    counts = {s: sum(1 for f in v.findings if f.severity == s) for s in ("error", "warning", "note")}
    tally = ", ".join(f"{n} {s}{'s' if n != 1 else ''}" for s, n in counts.items() if n)
    return ", ".join(parts) + (f"; {tally}" if tally else "")
