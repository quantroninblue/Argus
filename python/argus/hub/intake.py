"""How much map evidence the hub takes in, so no robot, and no number of robots, can overload it.

The way large servers and routers stay up under load, applied to the hub:

- Criticality (Google SRE book, "Handling Overload"): heartbeats, telemetry and events are
  always processed; map evidence is sheddable. Dropped evidence only leaves gaps the hub plans
  through speculatively.
- Fair sharing (max-min fairness, as in deficit round robin and FQ-CoDel): the intake capacity
  is split between robots by water-filling on their recent demand, so a quiet robot's unused
  share goes to busy ones and one robot cannot starve the rest. Capacity left over is spread
  evenly, so a robot that starts sending is not refused while its demand estimate catches up.
- Adaptive capacity (AIMD, as in TCP congestion control and Netflix's concurrency limits): the
  capacity starts at the configured bandwidth; when applying evidence takes more than its share
  of the hub's time it is cut by a fifth, otherwise it grows back by a twentieth of the
  configured value per step.

Each session enforces its allotted rate with a token bucket.
"""

from __future__ import annotations

import re
from dataclasses import dataclass


@dataclass
class IntakeConfig:
    # Total evidence the hub accepts from all robots, bytes per second: the network bandwidth
    # set aside for it (default 16 Mbit/s).
    bytes_per_s: float = 2_000_000.0
    # Share of the hub's time that applying evidence may take before capacity is cut.
    cpu_share: float = 0.5
    # Bucket depth in seconds of a robot's rate, and never below one large message.
    burst_s: float = 2.0
    min_burst_bytes: float = 256_000.0
    # Weight of the newest step in each robot's demand estimate.
    demand_weight: float = 0.3
    # Seconds over which a robot's reported evidence backlog counts as demand.
    backlog_s: float = 3.0


def parse_rate(text: str) -> float:
    """Bytes per second from '20Mbit', '2.5MB', '500KB', '800kbit' or a plain number of bytes."""
    m = re.fullmatch(r"\s*([0-9]*\.?[0-9]+)\s*([kKmMgG]?)(bit|b|B|byte|bytes)?(/s)?\s*", text)
    if not m:
        raise ValueError(f"cannot read {text!r} as a rate (examples: 20Mbit, 2MB, 500KB)")
    value = float(m.group(1)) * {"": 1, "k": 1e3, "m": 1e6, "g": 1e9}[m.group(2).lower()]
    unit = m.group(3) or "B"
    return value / 8 if unit in ("bit", "b") else value


def max_min_shares(capacity: float, demands: list[float]) -> list[float]:
    """Water-filling: everyone gets up to an equal share; what low-demand robots leave goes to
    the rest; whatever is still left is split evenly as headroom."""
    n = len(demands)
    if n == 0:
        return []
    shares = [0.0] * n
    left = capacity
    pending = sorted(range(n), key=lambda i: demands[i])
    while pending:
        fair = left / len(pending)
        i = pending[0]
        if demands[i] <= fair:
            shares[i] = demands[i]
            left -= demands[i]
            pending.pop(0)
        else:
            for j in pending:
                shares[j] = fair
            left = 0.0
            pending = []
    if left > 0:
        for i in range(n):
            shares[i] += left / n
    return shares


class Intake:
    def __init__(self, config: IntakeConfig | None = None) -> None:
        self.config = config or IntakeConfig()
        self.capacity = self.config.bytes_per_s
        self._last = None

    def allocate(self, sessions: list, now: float) -> None:
        """Called every hub step with the connected sessions; sets each one's evidence rate."""
        c = self.config
        dt = (now - self._last) if self._last is not None else 0.0
        self._last = now
        if dt <= 0.0:
            for s in sessions:
                s.set_evidence_rate(self.capacity / max(1, len(sessions)), c)
            return
        cpu = 0.0
        for s in sessions:
            offered, spent = s.take_evidence_counters()
            # What the robot sent plus the backlog it reports holding back (cleared over a few
            # seconds), so a robot that obeys its budget still shows what it wants.
            wanted = offered / dt + getattr(s, "evidence_backlog", 0) / c.backlog_s
            s.evidence_demand = (1 - c.demand_weight) * s.evidence_demand + c.demand_weight * wanted
            cpu += spent
        if cpu / dt > c.cpu_share:
            self.capacity = max(0.05 * c.bytes_per_s, 0.8 * self.capacity)
        else:
            self.capacity = min(c.bytes_per_s, self.capacity + 0.05 * c.bytes_per_s)
        for s, share in zip(sessions, max_min_shares(self.capacity, [s.evidence_demand for s in sessions])):
            s.set_evidence_rate(share, c)
