"""Errands: a robot working through a route of named places.

The errand is the one owner of which stop comes next; the robot's mission manager stays the
one owner of how to get there. Each stop becomes one mission goal. When the goal is reached
the robot holds for the stop's dwell time, then the errand moves on. When a goal fails, the
route says whether to skip the stop or end the errand; a whole cycle in which every stop was
skipped ends it too, so an unreachable route cannot loop forever.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from argus.hub.mission import MissionState
from argus.hub.places import Place, Route


class ErrandState(str, Enum):
    RUNNING = "running"    # on the way to a stop
    DWELLING = "dwelling"  # at a stop, holding for its dwell time
    DONE = "done"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass
class Goal:
    position: tuple[float, float, float]
    tolerance_m: float


@dataclass
class ErrandEvent:
    kind: str    # leg_started, leg_reached, leg_skipped, dwelling, errand_done, errand_failed, errand_cancelled
    detail: str


@dataclass
class Errand:
    route: Route
    # Positions are taken when the errand starts; editing a place affects later errands.
    places: dict[str, Place]
    state: ErrandState = ErrandState.RUNNING
    stop: int = 0
    cycle: int = 1
    detail: str = ""
    _direction: int = 1
    _dwell_until: float = 0.0
    _legs_in_cycle: int = 0
    _skipped_in_cycle: int = 0
    events: list[ErrandEvent] = field(default_factory=list)

    @property
    def active(self) -> bool:
        return self.state in (ErrandState.RUNNING, ErrandState.DWELLING)

    def place(self) -> Place:
        return self.places[self.route.stops[self.stop].place]

    def describe(self) -> str:
        n = len(self.route.stops)
        cycles = "" if self.route.mode == "once" else f", cycle {self.cycle}" + (
            f" of {self.route.cycles}" if self.route.cycles else "")
        return f"{self.route.name}: stop {self.stop + 1} of {n} ({self.place().name}){cycles}"

    def start(self) -> Goal:
        return self._begin_leg()

    def cancel(self) -> None:
        if self.active:
            self._finish(ErrandState.CANCELLED, "errand_cancelled", "cancelled")

    def update(self, now: float, mission_state: MissionState, cause: str = "") -> Goal | None:
        """Called every hub step with the mission's state. Returns the next goal to set."""
        if not self.active:
            return None
        if self.state == ErrandState.RUNNING:
            if mission_state == MissionState.REACHED:
                self._emit("leg_reached", self.describe())
                dwell = self.route.stops[self.stop].dwell_s
                self.state = ErrandState.DWELLING
                self._dwell_until = now + dwell
                if dwell > 0.0:
                    self._emit("dwelling", f"{self.place().name} for {dwell:g} s")
            elif mission_state == MissionState.FAILED:
                why = cause or "unreachable"
                if self.route.on_unreachable == "stop":
                    self._finish(ErrandState.FAILED, "errand_failed", f"{self.place().name}: {why}")
                    return None
                self._emit("leg_skipped", f"{self.describe()}: {why}")
                self._skipped_in_cycle += 1
                return self._advance()
            elif mission_state in (MissionState.CANCELLED, MissionState.IDLE):
                # Someone replaced or cancelled the goal underneath the errand.
                self._finish(ErrandState.CANCELLED, "errand_cancelled", "the robot's goal was replaced")
            return None
        if now >= self._dwell_until:
            return self._advance()
        return None

    def take_events(self) -> list[ErrandEvent]:
        out, self.events = self.events, []
        return out

    # Internals.

    def _emit(self, kind: str, detail: str) -> None:
        self.events.append(ErrandEvent(kind, detail))

    def _finish(self, state: ErrandState, kind: str, detail: str) -> None:
        self.state = state
        self.detail = detail
        self._emit(kind, f"{self.route.name}: {detail}")

    def _begin_leg(self) -> Goal:
        self.state = ErrandState.RUNNING
        self._legs_in_cycle += 1
        self._emit("leg_started", self.describe())
        p = self.place()
        return Goal(p.position, p.tolerance_m)

    def _advance(self) -> Goal | None:
        """The leg at the current stop is over (reached and dwelt, or skipped): next stop."""
        n = len(self.route.stops)
        mode = self.route.mode
        if mode == "back_and_forth":
            # Out to the last stop and back to the first is one cycle.
            end_of_cycle = self._direction < 0 and self.stop == 0
        else:
            end_of_cycle = self.stop == n - 1
        if end_of_cycle:
            if self._skipped_in_cycle >= self._legs_in_cycle:
                self._finish(ErrandState.FAILED, "errand_failed", "no stop on the route could be reached")
                return None
            if mode == "once" or (self.route.cycles is not None and self.cycle >= self.route.cycles):
                self._finish(ErrandState.DONE, "errand_done", f"{self.cycle} cycle(s)")
                return None
            self.cycle += 1
            self._legs_in_cycle = self._skipped_in_cycle = 0
            self._direction = 1
            self.stop = 0 if mode == "loop" else 1  # back and forth sets out from the first stop
            return self._begin_leg()
        if mode == "back_and_forth":
            if self._direction > 0 and self.stop == n - 1:
                self._direction = -1
            self.stop += self._direction
        else:
            self.stop += 1
        return self._begin_leg()
