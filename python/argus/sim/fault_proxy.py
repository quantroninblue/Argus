"""A TCP proxy between Argo and the hub that injects link faults in wall time.

It sits under TLS, so it only sees encrypted bytes. Latency and jitter delay each chunk (in
order, as TCP would). During a stall nothing is forwarded and what was sent waits, as on a
congested link; the connection stays open unless the endpoints give up on it. During a cut
every connection is closed and new ones are refused until the cut is over.
"""

from __future__ import annotations

import asyncio
import random
import time

from argus.sim.channel import LinkFaults


class FaultProxy:
    def __init__(self, upstream_host: str, upstream_port: int, faults: LinkFaults | None = None,
                 clock=None, listen_host: str = "127.0.0.1") -> None:
        self.upstream = (upstream_host, upstream_port)
        self.faults = faults or LinkFaults(latency_s=0.0)
        self.listen_host = listen_host
        self._t0 = time.monotonic()
        self.clock = clock or (lambda: time.monotonic() - self._t0)
        self._rng = random.Random(self.faults.seed)
        self.port = 0
        self.accepted = 0
        self.refused = 0
        self.bytes = [0, 0]  # client to upstream, upstream to client
        self._server: asyncio.base_events.Server | None = None
        self._open: set[asyncio.StreamWriter] = set()
        self._watch: asyncio.Task | None = None

    async def start(self) -> int:
        self._server = await asyncio.start_server(self._client, self.listen_host, 0)
        self.port = self._server.sockets[0].getsockname()[1]
        self._watch = asyncio.ensure_future(self._watch_cuts())
        return self.port

    async def stop(self) -> None:
        if self._watch:
            self._watch.cancel()
        if self._server:
            self._server.close()
            await self._server.wait_closed()
        for w in list(self._open):
            w.close()

    async def _watch_cuts(self) -> None:
        while True:
            if self.faults.cut(self.clock()):
                for w in list(self._open):
                    w.close()
                self._open.clear()
            await asyncio.sleep(0.02)

    async def _client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        if self.faults.cut(self.clock()):
            self.refused += 1
            writer.close()
            return
        try:
            up_reader, up_writer = await asyncio.open_connection(*self.upstream)
        except OSError:
            writer.close()
            return
        self.accepted += 1
        self._open.update((writer, up_writer))
        pumps = [asyncio.ensure_future(self._pump(reader, up_writer, 0)),
                 asyncio.ensure_future(self._pump(up_reader, writer, 1))]
        await asyncio.wait(pumps, return_when=asyncio.FIRST_COMPLETED)
        for p in pumps:
            p.cancel()
        for w in (writer, up_writer):
            self._open.discard(w)
            w.close()

    def _release_time(self, now: float) -> float:
        f = self.faults
        t = now + f.latency_s + (self._rng.uniform(0.0, f.jitter_s) if f.jitter_s > 0.0 else 0.0)
        for a, b in f.stalls:
            if a <= t < b:
                t = b
        return t

    async def _pump(self, src: asyncio.StreamReader, dst: asyncio.StreamWriter, direction: int) -> None:
        queue: asyncio.Queue = asyncio.Queue()

        async def deliver() -> None:
            last = 0.0
            while True:
                at, data = await queue.get()
                at = max(at, last)
                last = at
                wait = at - self.clock()
                if wait > 0:
                    await asyncio.sleep(wait)
                dst.write(data)
                await dst.drain()
                self.bytes[direction] += len(data)

        sender = asyncio.ensure_future(deliver())
        try:
            while True:
                data = await src.read(65536)
                if not data:
                    break
                queue.put_nowait((self._release_time(self.clock()), data))
        finally:
            # Let what is already queued go out unless the connection is being torn down.
            while not queue.empty() and not dst.is_closing():
                await asyncio.sleep(0.01)
            sender.cancel()
