"""Moving obstacles for the simulator: boxes that follow a path at a set speed.

A mover is an upright box (a person, a cart) that walks a polyline of points, once and then
stands still, or back and forth for as long as the episode lasts. It may wait before it
starts, so a scenario can have someone step out at the worst moment. Movers do not react to
the robot: they keep walking whatever it does, which is the case Argo has to be safe in.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# An adult seen from above and their height, and a brisk walk.
PERSON_SIZE = (0.5, 0.5, 1.75)
WALKING_MPS = 1.4


@dataclass
class Mover:
    path: list[tuple[float, float]]  # points on the floor the box's centre walks through
    speed_mps: float = WALKING_MPS
    size: tuple[float, float, float] = PERSON_SIZE  # x, y extent and height
    start_s: float = 0.0  # stands at the first point until then
    back_and_forth: bool = False  # walk the path back and forth instead of once
    name: str = "person"
    _lengths: list[float] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        if len(self.path) < 2:
            raise ValueError("a mover needs at least two points")
        if self.speed_mps <= 0:
            raise ValueError("a mover needs a positive speed")
        pts = np.asarray(self.path, dtype=float)
        self._lengths = [float(v) for v in np.linalg.norm(np.diff(pts, axis=0), axis=1)]

    @property
    def length_m(self) -> float:
        return sum(self._lengths)

    def _along(self, s: float) -> tuple[np.ndarray, np.ndarray]:
        """Position and unit direction at distance `s` along the path (clamped to its ends)."""
        pts = np.asarray(self.path, dtype=float)
        s = min(max(s, 0.0), self.length_m)
        for i, seg in enumerate(self._lengths):
            if s <= seg or i == len(self._lengths) - 1:
                d = (pts[i + 1] - pts[i]) / seg if seg > 0 else np.zeros(2)
                return pts[i] + d * min(s, seg), d
            s -= seg
        raise AssertionError("unreachable")

    def state(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        """(centre x, y) and velocity (x, y) at time `t`."""
        walked = max(0.0, t - self.start_s) * self.speed_mps
        total = self.length_m
        if not self.back_and_forth:
            pos, d = self._along(walked)
            moving = self.start_s <= t and walked < total
            return pos, d * self.speed_mps if moving else np.zeros(2)
        lap = walked % (2 * total) if total > 0 else 0.0
        if lap <= total:
            pos, d = self._along(lap)
            sign = 1.0
        else:
            pos, d = self._along(2 * total - lap)
            sign = -1.0
        moving = self.start_s <= t
        return pos, d * self.speed_mps * sign if moving else np.zeros(2)

    def box(self, t: float) -> np.ndarray:
        """min x, min y, min z, max x, max y, max z at time `t`, as the world's boxes are."""
        (x, y), _ = self.state(t)
        hx, hy, h = self.size[0] / 2, self.size[1] / 2, self.size[2]
        return np.array([x - hx, y - hy, 0.0, x + hx, y + hy, h])


def crossing(at: tuple[float, float], heading_rad: float, half_width_m: float, start_s: float = 0.0,
             speed_mps: float = WALKING_MPS, back_and_forth: bool = False) -> Mover:
    """A person walking across a line through `at`: from one side to the other, square to
    `heading_rad` (the direction the robot travels there), `half_width_m` either side."""
    nx, ny = -math.sin(heading_rad), math.cos(heading_rad)
    a = (at[0] - nx * half_width_m, at[1] - ny * half_width_m)
    b = (at[0] + nx * half_width_m, at[1] + ny * half_width_m)
    return Mover(path=[a, b], speed_mps=speed_mps, start_s=start_s, back_and_forth=back_and_forth,
                 name="crossing person")
