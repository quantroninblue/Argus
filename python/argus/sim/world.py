"""Worlds made of axis-aligned boxes over a floor at z = 0, with optional pits cut into the
floor, and ray casting into them."""

from __future__ import annotations

import random
from dataclasses import dataclass, field

import numpy as np


@dataclass
class World:
    # One row per box: min x, min y, min z, max x, max y, max z.
    boxes: np.ndarray = field(default_factory=lambda: np.zeros((0, 6)))
    name: str = "empty"
    # Boxes that walk a path (argus.sim.movers.Mover); `boxes` never move.
    movers: list = field(default_factory=list)
    # Pits cut into the floor (a drop, a stairwell, a loading bay), one row per pit: min x,
    # min y, max x, max y, depth. Straight sides; the floor around them is solid.
    holes: np.ndarray = field(default_factory=lambda: np.zeros((0, 5)))
    # Glass: boxes that depth cameras see and lidars do not (their beams pass through, or
    # come back from whatever is behind). Solid for the robot all the same.
    glass: np.ndarray = field(default_factory=lambda: np.zeros((0, 6)))

    def add(self, lo, hi) -> "World":
        self.boxes = np.vstack([self.boxes, np.asarray([*lo, *hi], dtype=float)])
        return self

    def add_hole(self, lo, hi, depth: float) -> "World":
        self.holes = np.vstack([self.holes, np.asarray([*lo, *hi, depth], dtype=float)])
        return self

    def add_glass(self, lo, hi) -> "World":
        self.glass = np.vstack([self.glass, np.asarray([*lo, *hi], dtype=float)])
        return self

    def solid(self) -> np.ndarray:
        """Every box the robot can run into, glass included."""
        return np.vstack([self.boxes, self.glass]) if len(self.glass) else self.boxes

    def add_mover(self, mover) -> "World":
        self.movers.append(mover)
        return self

    def moving_at(self, t: float) -> "World":
        """Only the movers, where they are at time `t`."""
        boxes = np.array([m.box(t) for m in self.movers]) if self.movers else np.zeros((0, 6))
        return World(boxes=boxes, name=self.name, holes=self.holes, glass=self.glass)

    def at(self, t: float) -> "World":
        """Everything, movers where they are at time `t`, for ray casting."""
        if not self.movers:
            return self
        return World(boxes=np.vstack([self.boxes, self.moving_at(t).boxes]), name=self.name, holes=self.holes,
                     glass=self.glass)

    def raycast(self, origin: np.ndarray, dirs: np.ndarray, max_range: float, see_glass: bool = True) -> np.ndarray:
        """Distance along each unit direction to the first box face or the floor; inf if none.
        With `see_glass` false (a lidar), glass boxes are not there."""
        origin = np.asarray(origin, dtype=float)
        boxes = self.solid() if see_glass else self.boxes
        n = len(dirs)
        best = np.full(n, np.inf)
        down = dirs[:, 2] < -1e-9
        if origin[2] > 0.0:
            best[down] = -origin[2] / dirs[down, 2]
            for x0, y0, x1, y1, depth in self.holes:
                # Into the pit through its open top: the ray ends where it leaves the pit's
                # volume, on its bottom or one of its sides.
                p = origin[:2] + best[:, None] * dirs[:, :2]
                inside = down & (p[:, 0] > x0) & (p[:, 0] < x1) & (p[:, 1] > y0) & (p[:, 1] < y1)
                if not inside.any():
                    continue
                d = dirs[inside]
                with np.errstate(divide="ignore", invalid="ignore"):
                    inv = 1.0 / d
                lo = (np.array([x0, y0, -depth]) - origin) * inv
                hi = (np.array([x1, y1, 0.0]) - origin) * inv
                lo = np.where(np.isnan(lo), -np.inf, lo)
                hi = np.where(np.isnan(hi), np.inf, hi)
                best[inside] = np.maximum(lo, hi).min(axis=1)
        if len(boxes):
            with np.errstate(divide="ignore", invalid="ignore"):
                inv = 1.0 / dirs
            lo = (boxes[None, :, 0:3] - origin) * inv[:, None, :]
            hi = (boxes[None, :, 3:6] - origin) * inv[:, None, :]
            # Rays parallel to a slab: inside it the slab never limits, outside it never hits.
            lo = np.where(np.isnan(lo), -np.inf, lo)
            hi = np.where(np.isnan(hi), np.inf, hi)
            t_near = np.minimum(lo, hi).max(axis=2)
            t_far = np.maximum(lo, hi).min(axis=2)
            hit = (t_far >= t_near) & (t_near > 0.0)
            t = np.where(hit, t_near, np.inf).min(axis=1)
            best = np.minimum(best, t)
        best[best > max_range] = np.inf
        return best


def _room(world: World, half_x: float, half_y: float, height: float, wall: float = 0.2) -> World:
    world.add((-half_x - wall, -half_y - wall, 0.0), (half_x + wall, -half_y, height))
    world.add((-half_x - wall, half_y, 0.0), (half_x + wall, half_y + wall, height))
    world.add((-half_x - wall, -half_y, 0.0), (-half_x, half_y, height))
    world.add((half_x, -half_y, 0.0), (half_x + wall, half_y, height))
    return world


def open_room(size: float = 14.0, height: float = 2.0) -> World:
    return _room(World(name="open_room"), size / 2, size / 2, height)


def wall_with_gap(gap_y: tuple[float, float] = (1.0, 3.0), wall_x: float = 2.5,
                  height: float = 2.0) -> World:
    """A room split by a wall across x = wall_x with one opening. Goals behind it need a detour."""
    world = _room(World(name="wall_with_gap"), 7.0, 7.0, height)
    world.add((wall_x, -7.0, 0.0), (wall_x + 0.2, gap_y[0], height))
    world.add((wall_x, gap_y[1], 0.0), (wall_x + 0.2, 7.0, height))
    return world


def clutter(seed: int, count: int = 12, keep_clear: list[tuple[float, float]] | None = None,
            clear_radius: float = 1.2, low_share: float = 0.3, height: float = 2.0) -> World:
    """A room with random boxes, some of them low. Areas around `keep_clear` points stay empty."""
    rng = random.Random(seed)
    world = _room(World(name=f"clutter_{seed}"), 7.0, 7.0, height)
    keep_clear = keep_clear or []
    placed = 0
    while placed < count:
        cx, cy = rng.uniform(-6.0, 6.0), rng.uniform(-6.0, 6.0)
        sx, sy = rng.uniform(0.2, 0.8), rng.uniform(0.2, 0.8)
        if any(np.hypot(cx - x, cy - y) < clear_radius + max(sx, sy) for x, y in keep_clear):
            continue
        top = rng.uniform(0.15, 0.3) if rng.random() < low_share else rng.uniform(0.8, height)
        world.add((cx - sx / 2, cy - sy / 2, 0.0), (cx + sx / 2, cy + sy / 2, top))
        placed += 1
    return world


def pillars(seed: int, count: int = 10, keep_clear: list[tuple[float, float]] | None = None,
            clear_radius: float = 1.5, height: float = 5.0) -> World:
    """A tall room with floor-to-ceiling pillars and a ceiling, for aerial bodies."""
    rng = random.Random(seed)
    world = _room(World(name=f"pillars_{seed}"), 7.0, 7.0, height)
    world.add((-7.2, -7.2, height), (7.2, 7.2, height + 0.2))
    keep_clear = keep_clear or []
    placed = 0
    while placed < count:
        cx, cy = rng.uniform(-6.0, 6.0), rng.uniform(-6.0, 6.0)
        s = rng.uniform(0.3, 0.8)
        if any(np.hypot(cx - x, cy - y) < clear_radius + s for x, y in keep_clear):
            continue
        world.add((cx - s / 2, cy - s / 2, 0.0), (cx + s / 2, cy + s / 2, height))
        placed += 1
    return world


def dead_end_branch(height: float = 2.0) -> World:
    """Start in the west, goal in the north-east. A corridor along the north wall is the short
    way, but a wall closes it at x = 3, out of sight from the start. The way through is the
    long corridor along the south wall."""
    world = _room(World(name="dead_end_branch"), 7.0, 7.0, height)
    world.add((-3.2, -4.0, 0.0), (-3.0, 4.0, height))  # hides the corridors' far ends
    world.add((-3.2, 4.0, 0.0), (3.2, 4.2, height))  # north corridor floor
    world.add((3.0, 4.2, 0.0), (3.2, 7.0, height))  # closes the north corridor
    world.add((-3.2, -4.2, 0.0), (3.2, -4.0, height))  # south corridor ceiling, open at its east end
    world.add((3.0, -4.0, 0.0), (3.2, 4.0, height))  # east side of the block between them
    return world


def deep_cup(height: float = 2.0) -> World:
    """A cup that opens east, away from a start in the west. A goal near its closed end means
    going round it, farther from the goal at first."""
    world = _room(World(name="deep_cup"), 7.0, 7.0, height)
    world.add((0.0, -2.0, 0.0), (0.2, 2.0, height))
    world.add((0.0, 2.0, 0.0), (5.0, 2.2, height))
    world.add((0.0, -2.2, 0.0), (5.0, -2.0, height))
    return world


def two_gaps(wall_x: float = 2.5, height: float = 2.0) -> World:
    """A wall across the room with a gap near the middle and another near the south wall."""
    world = _room(World(name="two_gaps"), 7.0, 7.0, height)
    world.add((wall_x, -7.0, 0.0), (wall_x + 0.2, -6.0, height))
    world.add((wall_x, -4.5, 0.0), (wall_x + 0.2, 0.5, height))
    world.add((wall_x, 2.0, 0.0), (wall_x + 0.2, 7.0, height))
    return world


def with_ceiling(world: World, z: float) -> World:
    """Closes a room from above so aerial bodies cannot fly over its walls."""
    lo, hi = world.boxes[:, 0:2].min(axis=0), world.boxes[:, 3:5].max(axis=0)
    return world.add((lo[0], lo[1], z), (hi[0], hi[1], z + 0.2))
