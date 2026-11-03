"""Routes connections to session workers, independent of the transport.

A transport (the gRPC server, or the simulator's channel) opens a Connection per stream,
passes in what arrives, sends what `take_outgoing()` returns, and closes it when the stream
ends. The first message on a connection must be Hello; it is checked against the registry and
the connection is attached to that robot's session worker.
"""

from __future__ import annotations

from argus.hub.dispatch import Dispatch
from argus.hub.intake import Intake
from argus.hub.mission import MissionState
from argus.hub.registry import Registry
from argus.hub.session import HubConfig, HubSession
from argus.hub.site import SiteService
from argus.protocol.v1 import link_pb2, session_pb2


class Connection:
    def __init__(self, hub: "Hub", identity: str | None = None) -> None:
        self._hub = hub
        # Node id proven by the transport (the client certificate's common name), if any.
        self.identity = identity
        self.session: HubSession | None = None
        self.outgoing: list[bytes] = []
        self.rejected = False
        self.reject_reason = session_pb2.REJECT_REASON_UNSPECIFIED
        self.closed = False

    def receive(self, data: bytes, now: float) -> bool:
        """False if `data` is not a NodeMessage."""
        m = link_pb2.NodeMessage()
        try:
            m.ParseFromString(data)
        except Exception:
            return False
        if self.closed or self.rejected:
            return True
        if self.session is None:
            if m.WhichOneof("body") == "hello":
                self._hub._hello(self, m.hello, now)
        elif self.session.connection is self:
            self.session.receive(m, now)
        return True

    def take_outgoing(self) -> list[bytes]:
        out, self.outgoing = self.outgoing, []
        return out

    def close(self) -> None:
        self.closed = True
        if self.session is not None and self.session.connection is self:
            self.session.detach()


class Hub:
    def __init__(self, registry: Registry, sites: SiteService | None = None,
                 config: HubConfig | None = None, sites_dir=None) -> None:
        self.registry = registry
        self.config = config or HubConfig()
        self.sites = sites or SiteService(loops=self.config.loops, merge_m=self.config.drift.merge_m,
                                         settle_s=self.config.drift.settle_s)
        self.sessions: dict[str, HubSession] = {}
        self.intake = Intake(self.config.intake)
        # Named places, routes and errands (hub plan N1); places live in `sites_dir`.
        self.dispatch = Dispatch(sites_dir)

    def connect(self, identity: str | None = None) -> Connection:
        return Connection(self, identity)

    def session(self, node_id: str) -> HubSession:
        s = self.sessions.get(node_id)
        if s is None:
            robot = self.registry.robot(node_id)
            if robot is None:
                raise KeyError(f"node {node_id} is not registered")
            s = HubSession(robot, self.registry.profile(robot.profile_id), self.sites, self.config)
            self.sessions[node_id] = s
        return s

    def set_goal(self, node_id: str, position, tolerance_m: float, now: float) -> None:
        self.dispatch.end(node_id, "a goal was set by hand")
        self.session(node_id).mission.set_goal(position, tolerance_m, now)

    def cancel(self, node_id: str) -> None:
        self.dispatch.end(node_id, "cancelled")
        self.session(node_id).mission.cancel()

    def go(self, node_id: str, place: str, now: float) -> str:
        s = self.session(node_id)
        return self.dispatch.go(node_id, s.robot.site_id, place, s.mission, now)

    def patrol(self, node_id: str, route: str, now: float) -> str:
        s = self.session(node_id)
        return self.dispatch.patrol(node_id, s.robot.site_id, route, s.mission, now)

    def mission_state(self, node_id: str) -> MissionState:
        return self.session(node_id).mission.state

    def step(self, now: float) -> None:
        self.intake.allocate([s for s in self.sessions.values() if s.connection is not None], now)
        for node_id, s in self.sessions.items():
            self.dispatch.step(node_id, s.mission, now)
            s.step(now)

    def _hello(self, connection: Connection, hello, now: float) -> None:
        reason, detail = self.registry.check_hello(hello)
        if connection.identity is not None and hello.node_id != connection.identity:
            reason = session_pb2.REJECT_REASON_UNKNOWN_NODE
            detail = f"certificate is for {connection.identity}, not {hello.node_id}"
        if reason != session_pb2.REJECT_REASON_UNSPECIFIED:
            m = link_pb2.HubMessage(seq=1)
            m.session_reject.reason = reason
            m.session_reject.detail = detail
            connection.outgoing.append(m.SerializeToString())
            connection.rejected = True
            connection.reject_reason = reason
            return
        s = self.session(hello.node_id)
        if s.connection is not None and s.connection is not connection:
            s.connection.closed = True  # superseded by the new stream
        connection.session = s
        s.attach(connection, hello, now)
