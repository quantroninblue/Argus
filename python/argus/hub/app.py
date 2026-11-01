"""The hub's app interface (app.proto): what the argus command, and later the GUI, call.

Served on a Unix socket inside the hub's `run/` directory, which only the hub's owner can
enter, so the kernel decides who reaches it. Every call goes through `identify` (who is
calling) and `authorize` (may they do this). Today the only caller is the hub's owner on this
machine and the owner may do everything; accounts and remote clients change those two
functions, not the services.

Runs on the hub's event loop, so it reads and changes hub state directly.
"""

from __future__ import annotations

import asyncio
import os
import pwd
import socket
import time
from dataclasses import dataclass
from pathlib import Path

import grpc

from argus.protocol.v1 import app_pb2, app_pb2_grpc, node_messages_pb2

from .designs import Bundle, DesignStore, DesignVersion, Finding
from .gateway import Hub
from .home import Home

VERSION = "0.1.0"  # kept equal to pyproject.toml by a test
DEFAULT_TOLERANCE_M = 0.4
EVENT_QUEUE = 256  # events kept for a slow watcher; the oldest go first
EVENT_POLL_S = 0.1
# Designs travel whole in one message (their meshes included), up to designs.MAX_BUNDLE_BYTES.
MAX_MESSAGE_BYTES = 300 * 1024 * 1024


@dataclass(frozen=True)
class Caller:
    kind: str  # "local": the hub's owner on this machine
    name: str

    def __str__(self) -> str:
        return f"{self.name} ({self.kind})"


# What each call does, for authorisation: "read" looks, "control" changes what robots do.
# "change" alters what the hub stores.
ACTIONS = {"status": "read", "robots": "read", "watch": "read", "designs": "read", "design": "read",
           "goal": "control", "cancel": "control", "add_design": "change"}


def local_caller() -> Caller:
    return Caller("local", pwd.getpwuid(os.getuid()).pw_name)


def identify(context) -> Caller | None:
    """The caller of a gRPC call. Only local transports exist today: a Unix socket in a
    directory only the hub's owner can open, so its caller is the owner."""
    if context.auth_context().get("transport_security_type") == [b"local"]:
        return local_caller()
    return None


def authorize(caller: Caller | None, action: str) -> str | None:
    """None when `caller` may do `action`, else the reason they may not."""
    if action not in ACTIONS:
        return f"unknown action {action!r}"
    if caller is None:
        return "the caller could not be identified"
    if caller.kind == "local":
        return None
    return f"{caller} may not {action}"


def _link_name(state) -> str:
    name = node_messages_pb2.LinkState.Name(state).removeprefix("LINK_STATE_").lower()
    return "unknown" if name == "unspecified" else name


def robots_reply(hub: Hub, now: float) -> app_pb2.RobotsReply:
    reply = app_pb2.RobotsReply(intake_capacity_bytes_per_s=hub.intake.capacity,
                                intake_configured_bytes_per_s=hub.intake.config.bytes_per_s)
    for robot in sorted(hub.registry.robots(), key=lambda r: r.node_id):
        st = reply.robots.add(node_id=robot.node_id, profile_id=robot.profile_id, site_id=robot.site_id)
        s = hub.sessions.get(robot.node_id)
        if s is None:
            st.link, st.mission, st.seconds_since_heard = "unknown", "idle", -1.0
            continue
        st.connected = s.connection is not None
        st.seconds_since_heard = now - s.last_heard if s.last_heard > -1e300 else -1.0
        if s.pose is not None:
            st.has_pose = True
            st.x, st.y, st.z = s.pose
        st.link = _link_name(s.link_state)
        st.mission = s.mission.state.value
        st.failsafe = s.failsafe
        st.evidence_demand_bytes_per_s = s.evidence_demand
        st.evidence_rate_bytes_per_s = s.evidence_rate
        st.evidence_dropped = s.evidence_dropped
    return reply


def design_message(v: DesignVersion) -> app_pb2.DesignVersion:
    return app_pb2.DesignVersion(name=v.name, version=v.version, created=v.created, has_answers=v.has_answers,
                                 has_gazebo_model=v.has_gazebo_model, profile_id=v.profile_id,
                                 findings=[app_pb2.Finding(severity=f.severity, where=f.where, text=f.text)
                                           for f in v.findings], runnable=v.runnable)


class Events:
    """Hub events for watchers. Robot state is compared with the last look every
    EVENT_POLL_S; each change becomes an event."""

    def __init__(self, hub: Hub, clock=time.time) -> None:
        self.hub = hub
        self.clock = clock
        self._watchers: list[asyncio.Queue] = []
        self._last: dict[str, tuple] = {}

    def subscribe(self) -> asyncio.Queue:
        q: asyncio.Queue = asyncio.Queue(maxsize=EVENT_QUEUE)
        self._watchers.append(q)
        return q

    def unsubscribe(self, q: asyncio.Queue) -> None:
        if q in self._watchers:
            self._watchers.remove(q)

    def publish(self, kind: str, node_id: str = "", detail: str = "") -> None:
        e = app_pb2.HubEvent(stamp=self.clock(), kind=kind, node_id=node_id, detail=detail)
        for q in self._watchers:
            if q.full():
                q.get_nowait()
            q.put_nowait(e)

    def look(self) -> None:
        for node_id, s in self.hub.sessions.items():
            now = (s.connection is not None, _link_name(s.link_state), s.mission.state.value, s.failsafe)
            before = self._last.get(node_id, (False, "unknown", "idle", False))
            self._last[node_id] = now
            if now[0] != before[0]:
                self.publish("robot_connected" if now[0] else "robot_disconnected", node_id)
            if now[1] != before[1]:
                self.publish("link_changed", node_id, now[1])
            if now[2] != before[2]:
                self.publish("mission_changed", node_id, now[2])
            if now[3] != before[3]:
                self.publish("failsafe_changed", node_id, "on" if now[3] else "off")


class AppService(app_pb2_grpc.HubAppServicer):
    def __init__(self, hub: Hub, home: Home, events: Events, robot_address: str = "",
                 clock=time.time) -> None:
        self.hub = hub
        self.home = home
        self.events = events
        self.robot_address = robot_address
        self.clock = clock
        self.started_at = clock()
        self.designs = DesignStore(home.designs)

    async def _caller(self, context, action: str) -> Caller:
        caller = identify(context)
        problem = authorize(caller, action)
        if problem:
            await context.abort(grpc.StatusCode.PERMISSION_DENIED, problem)
        return caller

    def _unknown(self, node_id: str) -> str | None:
        if self.hub.registry.robot(node_id) is None:
            return f"no robot {node_id!r} in {self.home.robots_file}"
        return None

    async def Status(self, request, context):
        caller = await self._caller(context, "status")
        robots = self.hub.registry.robots()
        connected = sum(1 for r in robots if (s := self.hub.sessions.get(r.node_id)) and s.connection is not None)
        return app_pb2.HubStatus(version=VERSION, home=str(self.home.root), robot_address=self.robot_address,
                                 started_at=self.started_at, robots_registered=len(robots),
                                 robots_connected=connected, caller=str(caller))

    async def Robots(self, request, context):
        await self._caller(context, "robots")
        return robots_reply(self.hub, self.clock())

    async def SetGoal(self, request, context):
        await self._caller(context, "goal")
        problem = self._unknown(request.node_id)
        if problem:
            return app_pb2.Reply(ok=False, detail=problem)
        tol = request.tolerance_m or DEFAULT_TOLERANCE_M
        self.hub.set_goal(request.node_id, (request.x, request.y, request.z), tol, self.clock())
        return app_pb2.Reply(ok=True, detail=f"{request.node_id}: goal {request.x:g} {request.y:g} {request.z:g}, "
                                             f"tolerance {tol:g} m")

    async def Cancel(self, request, context):
        await self._caller(context, "cancel")
        problem = self._unknown(request.node_id)
        if problem:
            return app_pb2.Reply(ok=False, detail=problem)
        self.hub.cancel(request.node_id)
        return app_pb2.Reply(ok=True, detail=f"{request.node_id}: cancelled")

    async def AddDesign(self, request, context):
        await self._caller(context, "add_design")
        bundle = Bundle(name=request.name, files={f.path: f.data for f in request.files},
                        findings=[Finding(f.severity, f.where, f.text) for f in request.collect_findings])
        try:
            # Checking a design includes Gazebo's own check and takes seconds, so it runs off
            # the loop that serves robots.
            v, added = await asyncio.to_thread(self.designs.add, bundle)
        except (ValueError, OSError) as e:
            return app_pb2.AddDesignReply(ok=False, detail=str(e))
        state = "runnable" if v.runnable else "cannot run until its errors are fixed"
        return app_pb2.AddDesignReply(ok=True, added=added, design=design_message(v),
                                      detail=f"{v.name} {v.version}{'' if added else ' (already stored)'}: {state}")

    async def Designs(self, request, context):
        await self._caller(context, "designs")
        reply = app_pb2.DesignsReply()
        for name in self.designs.names():
            reply.designs.add(name=name, versions=[design_message(v) for v in self.designs.versions(name)])
        return reply

    async def Design(self, request, context):
        await self._caller(context, "design")
        v = self.designs.get(request.name, request.version or None)
        if v is None:
            what = f"{request.name}@{request.version}" if request.version else request.name
            return app_pb2.DesignReply(found=False, detail=f"no design {what!r}")
        report = ""
        doc = self.designs.profile(v)
        if doc is not None:
            from argus.setup.report import capability_report

            report = capability_report(doc)
        return app_pb2.DesignReply(found=True, design=design_message(v), capability_report=report)

    async def Watch(self, request, context):
        await self._caller(context, "watch")
        q = self.events.subscribe()
        try:
            while True:
                yield await q.get()
        finally:
            self.events.unsubscribe(q)


def _socket_in_use(path: Path) -> bool:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        s.connect(str(path))
        return True
    except OSError:
        return False
    finally:
        s.close()


class HubAlreadyRunning(RuntimeError):
    pass


class AppServer:
    """The app interface on `home.socket`, beside the robots' HubLink server."""

    def __init__(self, hub: Hub, home: Home, robot_address: str = "", clock=time.time) -> None:
        self.home = home
        self.events = Events(hub, clock)
        self.service = AppService(hub, home, self.events, robot_address, clock)
        self._server: grpc.aio.Server | None = None
        self._poller: asyncio.Task | None = None

    async def start(self) -> None:
        self.home.secure_run_dir()
        sock = self.home.socket
        if sock.exists():
            if _socket_in_use(sock):
                raise HubAlreadyRunning(f"a hub is already running at {self.home.root}")
            sock.unlink()  # left behind by a hub that did not stop cleanly
        self._server = grpc.aio.server(options=[("grpc.max_receive_message_length", MAX_MESSAGE_BYTES),
                                                ("grpc.max_send_message_length", MAX_MESSAGE_BYTES)])
        app_pb2_grpc.add_HubAppServicer_to_server(self.service, self._server)
        self._server.add_secure_port(f"unix:{sock}", grpc.local_server_credentials(grpc.LocalConnectionType.UDS))
        await self._server.start()
        self._poller = asyncio.ensure_future(self._poll())

    async def _poll(self) -> None:
        while True:
            self.events.look()
            await asyncio.sleep(EVENT_POLL_S)

    async def stop(self) -> None:
        if self._poller:
            self._poller.cancel()
        if self._server:
            await self._server.stop(0.5)
        self.home.socket.unlink(missing_ok=True)
