"""Named places and routes, per site, in the hub home.

    sites/<site>/places.yaml   name: {position: [x, y, z], tolerance_m, heading_rad}
    sites/<site>/routes.yaml   name: {stops: [{place, dwell_s}], mode, on_unreachable, cycles}

Routes name their places rather than copy positions, so moving a place moves every route
through it. Files are replaced atomically, so a crash never leaves half a file behind.
"""

from __future__ import annotations

import math
import os
import re
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

import yaml

NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")
MODES = ("once", "loop", "back_and_forth")
ON_UNREACHABLE = ("skip", "stop")
DEFAULT_TOLERANCE_M = 0.4


class PlaceError(ValueError):
    pass


@dataclass(frozen=True)
class Place:
    name: str
    position: tuple[float, float, float]
    tolerance_m: float = DEFAULT_TOLERANCE_M
    # Heading to arrive with (docking spots need one, most places do not). Kept for when
    # Argo's waypoints take a heading; today every waypoint is yaw-free.
    heading_rad: float | None = None


@dataclass(frozen=True)
class Stop:
    place: str
    dwell_s: float = 0.0


@dataclass(frozen=True)
class Route:
    name: str
    stops: tuple[Stop, ...]
    mode: str = "once"
    on_unreachable: str = "skip"
    # For loop and back_and_forth: how many times round, or None to run until cancelled.
    cycles: int | None = None
    places_used: tuple[str, ...] = field(init=False, default=())

    def __post_init__(self) -> None:
        object.__setattr__(self, "places_used", tuple(dict.fromkeys(s.place for s in self.stops)))


def check_name(kind: str, name: str) -> None:
    if not NAME.match(name or ""):
        raise PlaceError(f"{kind} name {name!r}: use letters, digits, '_', '-' or '.', up to 64")


def _finite(values, what: str) -> None:
    if not all(math.isfinite(float(v)) for v in values):
        raise PlaceError(f"{what} must be finite numbers")


def _write_yaml(path: Path, doc: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.")
    try:
        with os.fdopen(fd, "w") as f:
            yaml.safe_dump(doc, f, sort_keys=True)
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise


def _read_yaml(path: Path) -> dict:
    if not path.exists():
        return {}
    doc = yaml.safe_load(path.read_text()) or {}
    if not isinstance(doc, dict):
        raise PlaceError(f"{path}: expected a mapping")
    return doc


def site_heights(root: Path) -> dict[str, tuple[float, float | None]]:
    """Each site's floor and optional ceiling (site frame z) from `sites/<site>/site.yaml`.
    Aerial routes keep between them (SiteService.set_heights); a site with no file has its
    floor at 0 and no ceiling."""
    out = {}
    root = Path(root)
    if not root.is_dir():
        return out
    for d in sorted(root.iterdir()):
        f = d / "site.yaml"
        if not f.is_file():
            continue
        doc = _read_yaml(f)
        floor = float(doc.get("floor_z_m", 0.0))
        ceiling = doc.get("ceiling_z_m")
        if ceiling is not None and float(ceiling) <= floor:
            raise PlaceError(f"{f}: ceiling_z_m must be above floor_z_m")
        out[d.name] = (floor, None if ceiling is None else float(ceiling))
    return out


class SiteBook:
    """The places and routes of one site."""

    def __init__(self, root: Path, site: str) -> None:
        check_name("site", site)
        self.site = site
        self.dir = Path(root) / site
        self.places_file = self.dir / "places.yaml"
        self.routes_file = self.dir / "routes.yaml"

    # Places.

    def places(self) -> dict[str, Place]:
        out = {}
        for name, d in _read_yaml(self.places_file).items():
            pos = tuple(float(v) for v in d["position"])
            heading = d.get("heading_rad")
            out[name] = Place(name, pos, float(d.get("tolerance_m", DEFAULT_TOLERANCE_M)),
                              None if heading is None else float(heading))
        return out

    def add_place(self, place: Place) -> None:
        check_name("place", place.name)
        if len(place.position) != 3:
            raise PlaceError("a place needs x, y and z")
        _finite(place.position, "position")
        if not (place.tolerance_m > 0.0 and math.isfinite(place.tolerance_m)):
            raise PlaceError("tolerance_m must be above zero")
        if place.heading_rad is not None:
            _finite([place.heading_rad], "heading")
        doc = _read_yaml(self.places_file)
        entry = {"position": [float(v) for v in place.position], "tolerance_m": float(place.tolerance_m)}
        if place.heading_rad is not None:
            entry["heading_rad"] = float(place.heading_rad)
        doc[place.name] = entry
        _write_yaml(self.places_file, doc)

    def remove_place(self, name: str) -> None:
        users = [r.name for r in self.routes().values() if name in r.places_used]
        if users:
            raise PlaceError(f"place {name!r} is on route(s) {', '.join(sorted(users))}; remove them first")
        doc = _read_yaml(self.places_file)
        if name not in doc:
            raise PlaceError(f"no place {name!r} at site {self.site!r}")
        del doc[name]
        _write_yaml(self.places_file, doc)

    # Routes.

    def routes(self) -> dict[str, Route]:
        out = {}
        for name, d in _read_yaml(self.routes_file).items():
            stops = tuple(Stop(s["place"], float(s.get("dwell_s", 0.0))) for s in d["stops"])
            cycles = d.get("cycles")
            out[name] = Route(name, stops, d.get("mode", "once"), d.get("on_unreachable", "skip"),
                              None if cycles is None else int(cycles))
        return out

    def add_route(self, route: Route) -> None:
        check_name("route", route.name)
        if not route.stops:
            raise PlaceError("a route needs at least one stop")
        if route.mode not in MODES:
            raise PlaceError(f"mode must be one of {', '.join(MODES)}")
        if route.on_unreachable not in ON_UNREACHABLE:
            raise PlaceError(f"on_unreachable must be one of {', '.join(ON_UNREACHABLE)}")
        if route.cycles is not None and (route.cycles < 1 or route.mode == "once"):
            raise PlaceError("cycles is a positive count, for loop and back_and_forth routes")
        if route.mode == "back_and_forth" and len(route.stops) < 2:
            raise PlaceError("back_and_forth needs at least two stops")
        places = self.places()
        missing = [p for p in route.places_used if p not in places]
        if missing:
            raise PlaceError(f"no place(s) {', '.join(missing)} at site {self.site!r}")
        for s in route.stops:
            if not (s.dwell_s >= 0.0 and math.isfinite(s.dwell_s)):
                raise PlaceError("dwell_s must be zero or more")
        doc = _read_yaml(self.routes_file)
        entry = {"stops": [{"place": s.place, "dwell_s": float(s.dwell_s)} for s in route.stops],
                 "mode": route.mode, "on_unreachable": route.on_unreachable}
        if route.cycles is not None:
            entry["cycles"] = int(route.cycles)
        doc[route.name] = entry
        _write_yaml(self.routes_file, doc)

    def remove_route(self, name: str) -> None:
        doc = _read_yaml(self.routes_file)
        if name not in doc:
            raise PlaceError(f"no route {name!r} at site {self.site!r}")
        del doc[name]
        _write_yaml(self.routes_file, doc)
