"""The whole loop in wall time over the real transport: the simulated body and Argo in this
process, Argo's C++ gRPC client, a fault proxy, and the hub's gRPC server with mutual TLS.
Slower than the sim-time channel, so it is used for a few end-to-end runs only."""

from __future__ import annotations

import asyncio
import threading
import time
from dataclasses import dataclass

from argus import node as argo
from argus.hub import MissionState
from argus.hub.server import HubServer, TlsFiles
from argus.sim.channel import LinkFaults
from argus.sim.fault_proxy import FaultProxy
from argus.sim.hub_episode import NODE_ID, HubEpisode, HubLoop


@dataclass(frozen=True)
class NodeTls:
    """PEM text Argo uses: the CA it trusts and its own certificate and key."""

    ca: str
    cert: str
    key: str


class _Background:
    """An asyncio loop on its own thread for the hub server and the proxy."""

    def __init__(self) -> None:
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.thread.start()

    def run(self, coro, timeout: float = 10.0):
        return asyncio.run_coroutine_threadsafe(coro, self.loop).result(timeout)

    def close(self) -> None:
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(5.0)


class GrpcHubLoop(HubLoop):
    def __init__(self, doc, world, start, goal, tolerance_m: float = 0.4, faults: LinkFaults | None = None,
                 hub_tls: TlsFiles | None = None, node_tls: NodeTls | None = None, options=None,
                 hub_config=None, node_config=None, session_config=None) -> None:
        super().__init__(doc, world, start, goal, tolerance_m, None, options, hub_config, node_config, session_config)
        self._t0 = time.monotonic()
        clock = lambda: time.monotonic() - self._t0  # noqa: E731 - shared by hub, proxy and sim
        self.clock = clock
        self.bg = _Background()
        self.server = HubServer(self.hub, tls=hub_tls, clock=clock)
        port = self.bg.run(self.server.start())
        self.proxy = FaultProxy("127.0.0.1", port, faults or LinkFaults(latency_s=0.0), clock=clock)
        proxy_port = self.bg.run(self.proxy.start())
        lc = argo.GrpcLinkConfig()
        lc.target = f"127.0.0.1:{proxy_port}"
        lc.reconnect_period_s = 0.5
        lc.connect_timeout_s = 0.5
        if node_tls:
            lc.ca_pem, lc.cert_pem, lc.key_pem = node_tls.ca, node_tls.cert, node_tls.key
            lc.server_name = "localhost"
        self.link = argo.GrpcLink(lc)

    def _tick(self, t: float) -> dict:
        u = self.link.poll()
        received = u["received"]
        done = 0
        for up, after in u["transitions"]:
            for b in received[done:after]:
                self.session.receive(b, t)
            done = after
            if up:
                self.session.on_connected(t)
            else:
                self.session.on_disconnected(t)
        for b in received[done:]:
            self.session.receive(b, t)
        out = self.session.step(t)
        self.link.send(self.session.take_outgoing())
        state = self.session.link_state
        if not self.result.link_states or self.result.link_states[-1][1] != state:
            self.result.link_states.append((t, state))
        return out

    def step(self) -> None:
        # Keep sim time on the wall clock the hub and proxy use.
        ahead = self.sim.t - self.clock()
        if ahead > 0:
            time.sleep(ahead)
        t = self.sim.t
        self.sim.step(self._tick)
        if self.result.reached_at_s is None and self.mission_state == MissionState.REACHED:
            self.result.reached_at_s = t

    def finish(self) -> HubEpisode:
        r = super().finish()
        r.bytes_up, r.bytes_down = self.proxy.bytes
        r.connections = self.hub.session(NODE_ID).connections
        return r

    def close(self) -> None:
        self.link = None  # joins the link's thread and closes its stream
        self.bg.run(self.proxy.stop())
        self.bg.run(self.server.stop())
        self.bg.close()


def run_grpc_episode(doc, world, start, goal, duration_s: float, tolerance_m: float = 0.4,
                     faults: LinkFaults | None = None, hub_tls: TlsFiles | None = None,
                     node_tls: NodeTls | None = None, options=None, settle_s: float = 1.0,
                     min_duration_s: float = 0.0) -> HubEpisode:
    loop = GrpcHubLoop(doc, world, start, goal, tolerance_m, faults, hub_tls, node_tls, options)
    try:
        rest_since = None
        while loop.sim.t <= duration_s:
            loop.step()
            sim = loop.sim
            done = (loop.mission_state in (MissionState.REACHED, MissionState.FAILED, MissionState.CANCELLED)
                    and not sim.node.failsafe_active)
            if done and sim.at_rest:
                rest_since = sim.t if rest_since is None else rest_since
                if sim.t - rest_since >= settle_s and sim.t >= min_duration_s:
                    break
            else:
                rest_since = None
        return loop.finish()
    finally:
        loop.close()
