"""Branch memory for one robot: places it was sent to and what happened there.

Routes come from the planner, which already prefers observed space and marks the first step
into unknown space. This memory only records outcomes, so that a branch Argo found closed is
not tried again for the same goal. Only two failure causes may change it: a dead end Argo saw
in its own map, and a goal that could not be found from where the search ended. A temporary
obstruction keeps a place out of routes for a while and is then retried. Timing, sensor,
localization and link problems change nothing, so they can never teach the robot that a
corridor is bad.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum


class PlaceStatus(str, Enum):
    UNVISITED = "unvisited"
    ACTIVE = "active"
    TRAVERSED = "traversed"
    EXHAUSTED_FOR_GOAL = "exhausted_for_goal"
    TEMPORARILY_BLOCKED = "temporarily_blocked"


# Failure causes (protocol names without the prefix) and the status they lead to.
MEMORY_CAUSES = {
    "dead_end_observed": PlaceStatus.EXHAUSTED_FOR_GOAL,
    "searched_no_goal": PlaceStatus.EXHAUSTED_FOR_GOAL,
    "temporary_obstruction": PlaceStatus.TEMPORARILY_BLOCKED,
}


@dataclass
class Place:
    position: tuple[float, float, float]
    status: PlaceStatus = PlaceStatus.UNVISITED
    since: float = 0.0
    retry_at: float = math.inf


class ExplorationMemory:
    def __init__(self, planar: bool, zone_radius_m: float = 0.6, merge_m: float = 0.5,
                 retry_s: float = 20.0) -> None:
        self.planar = planar
        self.zone_radius_m = zone_radius_m
        self.merge_m = merge_m
        self.retry_s = retry_s
        self.places: list[Place] = []

    def _distance(self, a, b) -> float:
        dz = 0.0 if self.planar else a[2] - b[2]
        return math.hypot(a[0] - b[0], a[1] - b[1], dz)

    def place_at(self, position) -> Place:
        position = tuple(float(v) for v in position)
        for p in self.places:
            if self._distance(p.position, position) <= self.merge_m:
                return p
        p = Place(position)
        self.places.append(p)
        return p

    def _set(self, place: Place, status: PlaceStatus, now: float) -> None:
        place.status = status
        place.since = now
        place.retry_at = now + self.retry_s if status == PlaceStatus.TEMPORARILY_BLOCKED else math.inf

    def mark_active(self, position, now: float) -> None:
        p = self.place_at(position)
        if p.status in (PlaceStatus.UNVISITED, PlaceStatus.TRAVERSED):
            self._set(p, PlaceStatus.ACTIVE, now)

    def mark_traversed(self, position, now: float) -> None:
        self._set(self.place_at(position), PlaceStatus.TRAVERSED, now)

    def record_failure(self, position, cause: str, now: float) -> bool:
        """Applies a failure at `position`. True if the memory changed."""
        status = MEMORY_CAUSES.get(cause)
        if status is None:
            return False
        self._set(self.place_at(position), status, now)
        return True

    def new_goal(self) -> None:
        """Exhausted means exhausted for one goal; a new goal may need those places again."""
        for p in self.places:
            if p.status == PlaceStatus.EXHAUSTED_FOR_GOAL:
                p.status = PlaceStatus.UNVISITED

    def blocked_zones(self, now: float) -> list[tuple[float, float, float, float]]:
        """Zones routes must avoid, as (x, y, z, radius). The route planner ignores one that
        contains the robot, so memory can never trap it."""
        zones = []
        for p in self.places:
            if p.status == PlaceStatus.TEMPORARILY_BLOCKED and now >= p.retry_at:
                p.status = PlaceStatus.UNVISITED
            if p.status in (PlaceStatus.EXHAUSTED_FOR_GOAL, PlaceStatus.TEMPORARILY_BLOCKED):
                zones.append((*p.position, self.zone_radius_m))
        return zones
