"""The hub's gRPC endpoint: the HubLink service over mutual TLS, in front of a Hub.

Robots open the streams; the server never connects to a robot. Each Session stream becomes a
gateway Connection, tagged with the node id from the client certificate, and the Hub is
stepped on a fixed period in wall time.
"""

from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass
from pathlib import Path

import grpc

from argus.hub.gateway import Hub
from argus.protocol.v1 import link_pb2, link_pb2_grpc


@dataclass(frozen=True)
class TlsFiles:
    """PEM files: the CA that signs every robot and the hub, and the hub's own pair."""

    ca: Path
    cert: Path
    key: Path

    def server_credentials(self) -> grpc.ServerCredentials:
        return grpc.ssl_server_credentials([(Path(self.key).read_bytes(), Path(self.cert).read_bytes())],
                                           root_certificates=Path(self.ca).read_bytes(),
                                           require_client_auth=True)


def _peer_identity(context) -> str | None:
    names = context.auth_context().get("x509_common_name")
    return names[0].decode() if names else None


class _Servicer(link_pb2_grpc.HubLinkServicer):
    def __init__(self, server: "HubServer") -> None:
        self._server = server

    async def Session(self, request_iterator, context):
        s = self._server
        conn = s.hub.connect(_peer_identity(context) if s.tls else None)
        s.streams += 1

        async def read() -> None:
            async for m in request_iterator:
                conn.receive(m.SerializeToString(), s.clock())

        reader = asyncio.ensure_future(read())
        try:
            while True:
                for b in conn.take_outgoing():
                    yield link_pb2.HubMessage.FromString(b)
                if reader.done() or conn.closed or conn.rejected:
                    break
                await asyncio.sleep(s.poll_s)
        finally:
            reader.cancel()
            conn.close()

    async def Sensors(self, request_iterator, context):
        count = 0
        async for _ in request_iterator:
            count += 1
        return link_pb2.SensorAck(frames_received=count)


class HubServer:
    def __init__(self, hub: Hub, address: str = "127.0.0.1:0", tls: TlsFiles | None = None,
                 clock=time.time, step_period_s: float = 0.05, poll_s: float = 0.005) -> None:
        self.hub = hub
        self.address = address
        self.tls = tls
        self.clock = clock
        self.step_period_s = step_period_s
        self.poll_s = poll_s
        self.port = 0
        self.streams = 0
        self._server: grpc.aio.Server | None = None
        self._stepper: asyncio.Task | None = None

    async def start(self) -> int:
        self._server = grpc.aio.server(options=[
            # Robots send keepalive pings every couple of seconds even when idle.
            ("grpc.http2.min_ping_interval_without_data_ms", 1000),
            ("grpc.keepalive_permit_without_calls", 1),
        ])
        link_pb2_grpc.add_HubLinkServicer_to_server(_Servicer(self), self._server)
        if self.tls:
            self.port = self._server.add_secure_port(self.address, self.tls.server_credentials())
        else:
            self.port = self._server.add_insecure_port(self.address)
        await self._server.start()
        self._stepper = asyncio.ensure_future(self._step())
        return self.port

    async def _step(self) -> None:
        while True:
            self.hub.step(self.clock())
            await asyncio.sleep(self.step_period_s)

    async def stop(self) -> None:
        if self._stepper:
            self._stepper.cancel()
        if self._server:
            await self._server.stop(0.5)
