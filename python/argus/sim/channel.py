"""An in-memory link between Argo and the hub, stepped in sim time, with injected faults.

It behaves like one ordered stream per direction (as gRPC over TCP does): messages arrive in
the order sent, after latency and jitter. On top of that it can lose single messages, stall
(everything sent during a window is lost, the stream stays open) and cut (the stream closes;
Argo reconnects once the cut is over).
"""

from __future__ import annotations

import random
from collections import deque
from dataclasses import dataclass, field

UP = 0  # Argo to hub
DOWN = 1  # hub to Argo


@dataclass
class LinkFaults:
    latency_s: float = 0.02
    jitter_s: float = 0.0
    loss: float = 0.0
    stalls: list[tuple[float, float]] = field(default_factory=list)
    cuts: list[tuple[float, float]] = field(default_factory=list)
    # Argo's transport retries a closed stream this often.
    reconnect_period_s: float = 1.0
    seed: int = 0

    def stalled(self, t: float) -> bool:
        return any(a <= t < b for a, b in self.stalls)

    def cut(self, t: float) -> bool:
        return any(a <= t < b for a, b in self.cuts)


class SimChannel:
    def __init__(self, faults: LinkFaults | None = None) -> None:
        self.faults = faults or LinkFaults()
        self._rng = random.Random(self.faults.seed)
        self._queues: tuple[deque, deque] = (deque(), deque())
        self._last_arrival = [0.0, 0.0]
        self.open = False
        self.sent = [0, 0]
        self.lost = [0, 0]
        self.bytes = [0, 0]

    def send(self, direction: int, payload: bytes, t: float) -> None:
        self.sent[direction] += 1
        self.bytes[direction] += len(payload)
        f = self.faults
        if not self.open or f.stalled(t) or self._rng.random() < f.loss:
            self.lost[direction] += 1
            return
        arrival = t + f.latency_s + (self._rng.uniform(0.0, f.jitter_s) if f.jitter_s > 0.0 else 0.0)
        arrival = max(arrival, self._last_arrival[direction])
        self._last_arrival[direction] = arrival
        self._queues[direction].append((arrival, payload))

    def receive(self, direction: int, t: float) -> list[bytes]:
        q = self._queues[direction]
        out = []
        while q and q[0][0] <= t + 1e-9:
            out.append(q.popleft()[1])
        return out

    def close(self) -> None:
        self.open = False
        for q in self._queues:
            q.clear()
