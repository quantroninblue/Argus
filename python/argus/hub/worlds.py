"""The worlds a validation job can run in (hub plan H4).

Each is one of the simulator's box worlds with a start and a goal for ground robots and for
drones (drones fly at 1.5 m under a 3 m ceiling), a rough length for the way from start to
goal, used to size the run's time, and people who walk through it when a job asks for
moving bodies.

Each world is moved so the run's start is at the origin, facing along x. A robot's odometry
starts at zero where the robot starts, and until the hub corrects odometry against its map
(drift correction, planned) the site frame is the odometry frame, as on a real robot started
in a new site.

What a world holds is the user's choice per job (decisions, 2026-11-01):
- "objects": the world as shipped, its walls and boxes;
- "movers": the same plus people walking set paths that cross or meet the robot's way;
- "empty": nothing but the outer walls of the 14 m room. Every world is the same room then,
  so a job asking for "empty" gets one run in it, not one per world.
"""

from __future__ import annotations

import dataclasses
import math
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

from argus.sim import world as boxes
from argus.sim.movers import Mover
from argus.sim.world import World

PLANAR = "planar"
AERIAL = "aerial"
FLIGHT_Z = 1.5
CEILING_Z = 3.0
CONTENTS = ("objects", "movers", "empty")
EMPTY_WORLD = "open_room"


def _walk(*path: tuple[float, float], speed: float = 1.0, start_s: float = 0.0, back_and_forth: bool = True,
          name: str = "person") -> Mover:
    return Mover(path=list(path), speed_mps=speed, start_s=start_s, back_and_forth=back_and_forth, name=name)


@dataclass(frozen=True)
class WorldSpec:
    name: str
    description: str
    build: Callable[[float], World]  # the world for a given wall height
    start: tuple[float, float, float]  # x, y, yaw on the floor
    goal: tuple[float, float]
    route_m: float  # about how far the body travels from start to goal
    movers: Callable[[], list[Mover]] = field(default=lambda: [])

    def world(self, kind: str, contents: str) -> World:
        """The world for one run, moved so the start is at the origin."""
        if kind == AERIAL:
            w = boxes.with_ceiling(self.build(CEILING_Z), CEILING_Z) if self.name != "pillars" else self.build(5.0)
        else:
            w = self.build(2.0)
        if contents == "movers":
            for m in self.movers():
                w.add_mover(m)
        return _shifted(w, -self.start[0], -self.start[1])

    def start_pose(self, kind: str) -> tuple[float, float, float, float]:
        return (0.0, 0.0, FLIGHT_Z if kind == AERIAL else 0.0, self.start[2])

    def goal_point(self, kind: str) -> tuple[float, float, float]:
        return (self.goal[0] - self.start[0], self.goal[1] - self.start[1], FLIGHT_Z if kind == AERIAL else 0.0)

    def duration_s(self, top_speed_mps: float) -> float:
        """Simulated time allowed: three times the route at the body's top speed, at least 90 s."""
        return max(90.0, 3.0 * self.route_m / max(top_speed_mps, 0.05))


def _shifted(w: World, dx: float, dy: float) -> World:
    """`w` moved by (dx, dy) on the floor: its boxes, pits, glass and movers' paths."""
    def move(rows: np.ndarray, xs: tuple[int, ...], ys: tuple[int, ...]) -> np.ndarray:
        rows = np.array(rows, dtype=float, copy=True)
        if len(rows):
            rows[:, list(xs)] += dx
            rows[:, list(ys)] += dy
        return rows

    movers = [dataclasses.replace(m, path=[(x + dx, y + dy) for x, y in m.path]) for m in w.movers]
    return World(boxes=move(w.boxes, (0, 3), (1, 4)), name=w.name, movers=movers,
                 holes=move(w.holes, (0, 2), (1, 3)), glass=move(w.glass, (0, 3), (1, 4)))


_CLUTTER_CLEAR = [(-5.0, 0.0), (5.0, 1.0)]
_PILLARS_CLEAR = [(-4.0, 0.0), (4.0, 1.0)]

WORLDS: dict[str, WorldSpec] = {s.name: s for s in [
    WorldSpec("open_room", "a 14 m room with nothing in it",
              lambda h: boxes.open_room(height=h), (-4.0, 0.0, 0.0), (4.0, 1.0), 8.1,
              lambda: [_walk((0.0, -3.0), (0.0, 3.0)),
                       _walk((5.5, 1.2), (-5.5, -0.2), speed=0.8, start_s=4.0, back_and_forth=False)]),
    WorldSpec("wall_with_gap", "a wall across the room with one 2 m gap; the goal is behind it",
              lambda h: boxes.wall_with_gap(height=h), (0.0, 0.0, 0.0), (5.0, 2.0), 7.0,
              lambda: [_walk((1.2, -2.5), (1.2, 3.5)),
                       _walk((5.5, 2.0), (2.6, 2.0), (-1.0, 2.5), speed=0.8, start_s=6.0, back_and_forth=False)]),
    WorldSpec("clutter", "twelve boxes, some of them low, scattered across the room",
              lambda h: boxes.clutter(7, keep_clear=_CLUTTER_CLEAR, height=h), (-5.0, 0.0, 0.0), (5.0, 1.0), 12.0,
              lambda: [_walk((-1.25, -3.0), (-1.25, 3.0), speed=0.8), _walk((3.0, -3.0), (3.0, 3.0), speed=0.8)]),
    WorldSpec("pillars", "floor-to-ceiling pillars in a 5 m tall room",
              lambda h: boxes.pillars(1, keep_clear=_PILLARS_CLEAR, height=max(h, 5.0)), (-4.0, 0.0, 0.0),
              (4.0, 1.0), 9.0,
              lambda: [_walk((0.0, -3.0), (0.0, 3.0), speed=0.8)]),
    WorldSpec("dead_end", "the short way ends in a dead end out of sight; the way through is the long one",
              lambda h: boxes.dead_end_branch(height=h), (-5.0, 0.0, 0.0), (5.0, 5.5), 30.0,
              lambda: [_walk((-4.5, -3.0), (-4.5, 3.0)), _walk((-2.0, -5.6), (2.5, -5.6), speed=0.8)]),
    WorldSpec("deep_cup", "a cup that opens away from the start, with the goal inside it",
              lambda h: boxes.deep_cup(height=h), (-4.0, 0.0, 0.0), (0.8, 0.0), 16.0,
              lambda: [_walk((-3.0, 3.5), (4.0, 3.5), speed=0.8), _walk((6.2, -3.0), (6.2, 3.0))]),
    WorldSpec("two_gaps", "a wall with two gaps, one near the middle and one near the south wall",
              lambda h: boxes.two_gaps(height=h), (0.0, 0.0, 0.0), (5.0, 1.0), 7.0,
              lambda: [_walk((1.2, -3.0), (1.2, 3.0)), _walk((4.0, -3.0), (4.0, 4.0), speed=0.8)]),
]}


def body_kind(doc: dict) -> str:
    return AERIAL if doc["motion"]["space"] == "aerial" else PLANAR


def top_speed(doc: dict) -> float:
    return float(doc["motion"]["limits"]["x"]["velocity_mps"][1])


@dataclass(frozen=True)
class RunPlan:
    world: str
    contents: str
    repeat: int  # 1-based


def plan_runs(worlds: list[str], contents: list[str], repeats: int) -> list[RunPlan]:
    """Every run of a job, in order: each world with each contents, `repeats` times. "empty"
    is one run per repeat in the empty room, whatever the worlds."""
    unknown = [w for w in worlds if w not in WORLDS]
    if unknown:
        raise ValueError(f"no world {', '.join(map(repr, unknown))}; worlds: {', '.join(WORLDS)}")
    bad = [c for c in contents if c not in CONTENTS]
    if bad:
        raise ValueError(f"contents {', '.join(map(repr, bad))}: use {', '.join(CONTENTS)}")
    if repeats < 1:
        raise ValueError("repeats must be at least 1")
    runs = []
    for r in range(1, repeats + 1):
        if "empty" in contents:
            runs.append(RunPlan(EMPTY_WORLD, "empty", r))
        for w in worlds:
            for c in contents:
                if c != "empty":
                    runs.append(RunPlan(w, c, r))
    return runs


def mover_clear_of_walls(world: World, mover: Mover, step_s: float = 0.1) -> bool:
    """True when the mover's box never overlaps the world's static boxes along its path."""
    solid = world.boxes
    t_end = mover.start_s + 2.0 * mover.length_m / mover.speed_mps + step_s
    for i in range(int(math.ceil(t_end / step_s)) + 1):
        b = mover.box(i * step_s)
        overlap = ((solid[:, 0] < b[3]) & (solid[:, 3] > b[0]) & (solid[:, 1] < b[4]) & (solid[:, 4] > b[1])
                   & (solid[:, 2] < b[5]) & (solid[:, 5] > b[2]))
        if overlap.any():
            return False
    return True
