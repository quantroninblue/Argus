"""Worlds made of axis-aligned boxes over a floor at z = 0, and ray casting into them."""

from __future__ import annotations

import random
from dataclasses import dataclass, field

import numpy as np


@dataclass
class World:
    # One row per box: min x, min y, min z, max x, max y, max z.
    boxes: np.ndarray = field(default_factory=lambda: np.zeros((0, 6)))
    name: str = "empty"

    def add(self, lo, hi) -> "World":
        self.boxes = np.vstack([self.boxes, np.asarray([*lo, *hi], dtype=float)])
        return self

    def raycast(self, origin: np.ndarray, dirs: np.ndarray, max_range: float) -> np.ndarray:
        """Distance along each unit direction to the first box face or the floor; inf if none."""
        origin = np.asarray(origin, dtype=float)
        n = len(dirs)
        best = np.full(n, np.inf)
        down = dirs[:, 2] < -1e-9
        if origin[2] > 0.0:
            best[down] = -origin[2] / dirs[down, 2]
        if len(self.boxes):
            with np.errstate(divide="ignore", invalid="ignore"):
                inv = 1.0 / dirs
            lo = (self.boxes[None, :, 0:3] - origin) * inv[:, None, :]
            hi = (self.boxes[None, :, 3:6] - origin) * inv[:, None, :]
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
