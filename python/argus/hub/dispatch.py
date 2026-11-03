"""Named places, routes and errands for the hub's robots (hub plan N1).

Places and routes are kept per site in the hub's home (argus.hub.places). An errand runs one
robot along a route (argus.hub.errand): it owns which stop comes next, and the robot's mission
owns how to get there. `go` is an errand of one stop. A plain goal or a cancel for the robot
ends its errand.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from .errand import Errand
from .places import Place, PlaceError, Route, SiteBook, Stop


@dataclass
class DispatchEvent:
    node_id: str
    kind: str  # the errand's event kinds (errand.ErrandEvent)
    detail: str


class Dispatch:
    def __init__(self, sites_dir: Path | None) -> None:
        self.sites_dir = Path(sites_dir) if sites_dir else None
        self.errands: dict[str, Errand] = {}
        self.events: list[DispatchEvent] = []

    def book(self, site: str) -> SiteBook:
        if self.sites_dir is None:
            raise PlaceError("this hub keeps no places (it has no home directory)")
        return SiteBook(self.sites_dir, site)

    def sites(self) -> list[str]:
        if self.sites_dir is None or not self.sites_dir.is_dir():
            return []
        return sorted(p.name for p in self.sites_dir.iterdir() if p.is_dir())

    def add_place(self, site: str, place: Place, replace: bool = False) -> None:
        book = self.book(site)
        if not replace and place.name in book.places():
            raise PlaceError(f"site {site!r} already has a place {place.name!r} (replace it with --replace)")
        book.add_place(place)

    def add_route(self, site: str, route: Route, replace: bool = False) -> None:
        book = self.book(site)
        if not replace and route.name in book.routes():
            raise PlaceError(f"site {site!r} already has a route {route.name!r} (replace it with --replace)")
        book.add_route(route)

    def errand(self, node_id: str) -> Errand | None:
        e = self.errands.get(node_id)
        return e if e is not None and e.active else None

    def start(self, node_id: str, site: str, route: Route, mission, now: float) -> str:
        """Starts `route` for a robot, replacing any errand it had. Returns what it is doing."""
        book = self.book(site)
        places = book.places()
        missing = [s.place for s in route.stops if s.place not in places]
        if missing:
            raise PlaceError(f"no place(s) {', '.join(missing)} at site {site!r}")
        self.end(node_id, "replaced by a new errand")
        errand = Errand(route, {n: places[n] for n in route.places_used})
        goal = errand.start()
        mission.set_goal(goal.position, goal.tolerance_m, now)
        self.errands[node_id] = errand
        self._collect(node_id, errand)
        return errand.describe()

    def go(self, node_id: str, site: str, place: str, mission, now: float) -> str:
        return self.start(node_id, site, Route(f"go to {place}", (Stop(place),)), mission, now)

    def patrol(self, node_id: str, site: str, route: str, mission, now: float) -> str:
        routes = self.book(site).routes()
        if route not in routes:
            raise PlaceError(f"no route {route!r} at site {site!r}")
        return self.start(node_id, site, routes[route], mission, now)

    def end(self, node_id: str, why: str = "cancelled") -> None:
        """Ends a robot's errand, if it has one (a plain goal or cancel replaced it)."""
        errand = self.errand(node_id)
        if errand is not None:
            errand.cancel()
            errand.detail = why
            self._collect(node_id, errand)

    def step(self, node_id: str, mission, now: float) -> None:
        """Called every hub step for each robot: moves its errand on with its mission."""
        errand = self.errand(node_id)
        if errand is None:
            return
        goal = errand.update(now, mission.state, mission.cause)
        if goal is not None:
            mission.set_goal(goal.position, goal.tolerance_m, now)
        self._collect(node_id, errand)

    def take_events(self) -> list[DispatchEvent]:
        out, self.events = self.events, []
        return out

    def _collect(self, node_id: str, errand: Errand) -> None:
        self.events += [DispatchEvent(node_id, e.kind, e.detail) for e in errand.take_events()]
