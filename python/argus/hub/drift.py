"""Odometry drift correction on the hub.

Argo reports evidence and its pose in the robot's odometry frame, and odometry drifts. The hub
keeps a 2D correction per robot (x, y, yaw) from that frame to the site frame, estimated by
matching the surfaces Argo sees now against the surfaces the site has held for a while. Evidence
and poses from Argo are moved into the site frame with it, and waypoints back into odometry
before they are sent. Argo does not take part; its odometry alarm stays the fallback.

The reference is each surface as first seen (SurfaceMemory). A hit close to a settled surface
is that surface seen again and never becomes reference itself, so drift slow enough to be
corrected cannot creep into the reference. Space first seen while drifted, and drift too large
to match here, are the work of loop closure (loops.py).
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from argus import sitemap


@dataclass
class DriftConfig:
    enabled: bool = True
    # Only hits above this height take part: floor returns say nothing about position.
    min_height_m: float = 0.1
    # One match uses the hits of this much recent evidence, and runs this often.
    window_s: float = 1.0
    period_s: float = 1.0
    # Surfaces count as reference once the site has held them this long; newer ones would
    # mostly match themselves.
    settle_s: float = 3.0
    # A hit this close to a settled surface is that surface seen again.
    merge_m: float = 0.3
    # A hit is matched to the nearest reference surface within this distance.
    match_m: float = 0.3
    # A column's neighbours form a surface if their spread across the line is at most this
    # share of their spread along it.
    max_line_spread: float = 0.3
    # A match is believed only with this many matched hits, this share of all hits matched,
    # and this residual.
    min_matches: int = 40
    min_match_share: float = 0.2
    max_rms_m: float = 0.1
    # A direction is corrected only if surfaces across it hold this share of the matched hits.
    min_constraint_share: float = 0.1
    # Smaller changes are within the noise of the map's cells and are ignored. A surface lying
    # on a cell boundary can still read one cell off, so expect about that much at rest.
    deadband_m: float = 0.05
    deadband_rad: float = 0.02
    # Largest change applied per match.
    max_step_m: float = 0.15
    max_step_rad: float = 0.05
    iterations: int = 20
    # The plan is resent once the correction has moved this far since it was sent.
    resend_after_m: float = 0.2


def _rot(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


class SurfaceMemory:
    """Surfaces of one site as first seen, in the site frame, by map column."""

    def __init__(self, resolution_m: float, merge_m: float, epoch: int = 0,
                 settle_s: float = 3.0) -> None:
        self.res = resolution_m
        self.epoch = epoch
        self.settle_s = settle_s
        self._reach = max(0, int(round(merge_m / resolution_m)))
        # (ix, iy) -> (first seen, iz of the first occupied cell, unit vector toward the robot
        # that saw it). Zero when not known; a wall's two faces are different surfaces.
        self.columns: dict[tuple[int, int], tuple[float, int, float, float]] = {}
        # Every column ever hit, merged or not, to skip the neighbourhood check quickly.
        self._seen: set[tuple[int, int]] = set()
        self._arrays: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None = None

    def add(self, cells: np.ndarray, now: float, seen_from=None) -> None:
        """Adds occupied hits, cells [N, 3] in the site frame, seen from `seen_from` (x, y)."""
        r = self._reach
        settled = now - self.settle_s
        added = False
        for ix, iy, iz in cells.tolist():
            key = (ix, iy)
            if key in self._seen:
                continue
            self._seen.add(key)
            vx, vy = 0.0, 0.0
            if seen_from is not None:
                vx, vy = (seen_from[0] - (ix + 0.5) * self.res, seen_from[1] - (iy + 0.5) * self.res)
                length = math.hypot(vx, vy)
                vx, vy = (vx / length, vy / length) if length > 0.0 else (0.0, 0.0)
            # Only settled surfaces seen from the same side count: the rest of a wall seen in
            # the same pass belongs to the reference as well, or walls end up as one point
            # every merge_m.
            near = False
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    known = self.columns.get((ix + dx, iy + dy))
                    if (known is not None and known[0] <= settled
                            and (vx * dx + vy * dy) * (known[2] * dx + known[3] * dy) >= 0.0):
                        near = True
            if not near:
                self.columns[key] = (now, iz, vx, vy)
                added = True
        if added:
            self._arrays = None

    def reference(self, before: float, lo: np.ndarray, hi: np.ndarray,
                  site_map) -> tuple[np.ndarray, np.ndarray]:
        """Columns [M, 2] (int) first seen at or before `before`, inside [lo, hi] (metres), and
        still occupied in the site map, with the unit vector [M, 2] toward whoever saw each."""
        if self._arrays is None:
            n = len(self.columns)
            cols = np.empty((n, 2), dtype=np.int64)
            seen = np.empty(n)
            iz = np.empty(n, dtype=np.int64)
            views = np.empty((n, 2))
            for k, ((x, y), (t, z, vx, vy)) in enumerate(self.columns.items()):
                cols[k] = (x, y)
                seen[k] = t
                iz[k] = z
                views[k] = (vx, vy)
            self._arrays = (cols, seen, iz, views)
        cols, seen, iz, views = self._arrays
        if len(cols) == 0:
            return cols, views
        centres = (cols + 0.5) * self.res
        keep = (seen <= before) & np.all(centres >= lo, axis=1) & np.all(centres <= hi, axis=1)
        idx = np.nonzero(keep)[0]
        occupied = [k for k in idx.tolist()
                    if site_map.cell((int(cols[k, 0]), int(cols[k, 1]), int(iz[k]))) == sitemap.OCCUPIED]
        return cols[occupied], views[occupied]


@dataclass
class Match:
    at: float
    matches: int
    share: float
    rms_m: float
    step_m: float
    step_rad: float
    applied: bool
    reason: str = ""


class DriftEstimator:
    """One robot's correction from its odometry frame to the site frame."""

    def __init__(self, config: DriftConfig, resolution_m: float) -> None:
        self.config = config
        self.res = resolution_m
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._hits: deque[tuple[float, np.ndarray, np.ndarray]] = deque()
        self._next = -math.inf
        self.corrections = 0
        self.last: Match | None = None
        self.rank = 0  # directions the last match constrained, of three

    @property
    def identity(self) -> bool:
        return self.x == 0.0 and self.y == 0.0 and self.yaw == 0.0

    def to_map(self, xy: np.ndarray) -> np.ndarray:
        return xy @ _rot(self.yaw).T + (self.x, self.y)

    def to_odom(self, xy: np.ndarray) -> np.ndarray:
        return (xy - (self.x, self.y)) @ _rot(self.yaw)

    def point_to_map(self, p):
        x, y = self.to_map(np.array([[p[0], p[1]]]))[0]
        return (float(x), float(y), p[2])

    def point_to_odom(self, p):
        x, y = self.to_odom(np.array([[p[0], p[1]]]))[0]
        return (float(x), float(y), p[2])

    def evidence_to_map(self, cells: np.ndarray) -> np.ndarray:
        """Cells [N, 3] of the odometry frame, re-binned into the site frame (z unchanged)."""
        if self.identity:
            return cells
        xy = self.to_map((cells[:, :2] + 0.5) * self.res)
        out = cells.copy()
        out[:, :2] = np.floor(xy / self.res).astype(cells.dtype)
        return out

    def observe(self, now: float, cells: np.ndarray, deltas: np.ndarray, seen_from=None) -> None:
        """Keeps the occupied hits of one evidence message for matching, with where the robot
        was when they arrived (x, y), both in the odometry frame."""
        floor_cell = math.floor(self.config.min_height_m / self.res)
        hit = (deltas > 0) & (cells[:, 2] >= floor_cell)
        if np.any(hit):
            pts = (np.unique(cells[hit, :2], axis=0) + 0.5) * self.res
            origin = pts if seen_from is None else np.tile(np.array(seen_from[:2], dtype=float), (len(pts), 1))
            self._hits.append((now, pts, origin))
        while self._hits and self._hits[0][0] < now - self.config.window_s:
            self._hits.popleft()

    def update(self, now: float, memory: SurfaceMemory, site_map) -> bool:
        """Matches recent hits against the site's settled surfaces. True if the correction
        changed."""
        c = self.config
        if now < self._next or not self._hits:
            return False
        self._next = now + c.period_s
        recent = [(h, o) for t, h, o in self._hits if t >= now - c.window_s]
        if not recent:
            return False
        pts, first = np.unique(np.concatenate([h for h, _ in recent]), axis=0, return_index=True)
        if len(pts) < c.min_matches:
            return False
        p = self.to_map(pts)
        origins = self.to_map(np.concatenate([o for _, o in recent])[first])
        lo, hi = p.min(axis=0) - 1.0, p.max(axis=0) + 1.0
        ref_cols, ref_views = memory.reference(now - c.settle_s, lo, hi, site_map)
        if len(ref_cols) < c.min_matches:
            return False
        result = self._icp(p, ref_cols, ref_views, origins)
        result.at = now
        self.last = result
        if result.applied:
            self.corrections += 1
        return result.applied

    def _nearest(self, q: np.ndarray, grid: np.ndarray, origin: np.ndarray, ref: np.ndarray,
                 offsets: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        cells = np.floor(q / self.res).astype(np.int64) - origin
        cand = cells[:, None, :] + offsets[None, :, :]
        inside = ((cand[..., 0] >= 0) & (cand[..., 0] < grid.shape[0])
                  & (cand[..., 1] >= 0) & (cand[..., 1] < grid.shape[1]))
        ids = np.full(inside.shape, -1, dtype=np.int64)
        ids[inside] = grid[cand[..., 0][inside], cand[..., 1][inside]]
        d = np.linalg.norm(ref[np.maximum(ids, 0)] - q[:, None, :], axis=2)
        d[ids < 0] = np.inf
        best = np.argmin(d, axis=1)
        rows = np.arange(len(q))
        return ids[rows, best], d[rows, best]

    def _normals(self, ref_cols: np.ndarray, grid: np.ndarray, origin: np.ndarray,
                 offsets: np.ndarray) -> np.ndarray:
        """Surface normal per reference column from its neighbours, zero where they do not form
        a line (corners, lone columns)."""
        cand = (ref_cols - origin)[:, None, :] + offsets[None, :, :]
        inside = ((cand[..., 0] >= 0) & (cand[..., 0] < grid.shape[0])
                  & (cand[..., 1] >= 0) & (cand[..., 1] < grid.shape[1]))
        used = np.zeros(inside.shape, dtype=bool)
        used[inside] = grid[cand[..., 0][inside], cand[..., 1][inside]] >= 0
        n = used.sum(axis=1).astype(float)
        w = used.astype(float)
        mx = (w * offsets[:, 0]).sum(axis=1) / n
        my = (w * offsets[:, 1]).sum(axis=1) / n
        dx = offsets[:, 0][None, :] - mx[:, None]
        dy = offsets[:, 1][None, :] - my[:, None]
        cxx = (w * dx * dx).sum(axis=1)
        cyy = (w * dy * dy).sum(axis=1)
        cxy = (w * dx * dy).sum(axis=1)
        along = 0.5 * np.arctan2(2.0 * cxy, cxx - cyy)
        normals = np.column_stack([-np.sin(along), np.cos(along)])
        spread = np.hypot(cxx - cyy, 2.0 * cxy)
        small = 0.5 * (cxx + cyy - spread)
        large = 0.5 * (cxx + cyy + spread)
        lined = (n >= 3) & (small <= self.config.max_line_spread * large)
        normals[~lined] = 0.0
        return normals

    def _solve(self, a: np.ndarray, b: np.ndarray, n: np.ndarray) -> tuple[float, np.ndarray]:
        """One step of point-to-line ICP (Censi, PLICP, ICRA 2008): the turn about the hits'
        centre and the shift that bring `a` onto the lines through `b` with normals `n`. A
        direction the matched surfaces do not constrain, like along a corridor, is left out
        (Zhang et al., ICRA 2016)."""
        centre = a.mean(axis=0)
        arm = a - centre
        reach = max(float(np.sqrt(np.mean(np.sum(arm * arm, axis=1)))), self.res)
        # The turn is scaled by the hits' spread, so its row compares with the shift's.
        J = np.column_stack([n[:, 0], n[:, 1], (arm[:, 0] * n[:, 1] - arm[:, 1] * n[:, 0]) / reach])
        r = np.sum((a - b) * n, axis=1)
        values, vectors = np.linalg.eigh(J.T @ J)
        rhs = vectors.T @ (J.T @ -r)
        keep = values >= self.config.min_constraint_share * len(a)
        self.rank = int(keep.sum())
        step = vectors[:, keep] @ (rhs[keep] / values[keep])
        th = float(step[2]) / reach
        R = _rot(th)
        return th, centre + step[:2] - R @ centre

    def _icp(self, p: np.ndarray, ref_cols: np.ndarray, ref_views: np.ndarray,
             origins: np.ndarray) -> Match:
        c = self.config
        ref = (ref_cols + 0.5) * self.res
        origin = ref_cols.min(axis=0) - 1
        size = ref_cols.max(axis=0) - origin + 2
        grid = np.full(tuple(size.tolist()), -1, dtype=np.int64)
        grid[ref_cols[:, 0] - origin[0], ref_cols[:, 1] - origin[1]] = np.arange(len(ref_cols))
        r = int(math.ceil(c.match_m / self.res))
        offsets = np.array([(dx, dy) for dx in range(-r, r + 1) for dy in range(-r, r + 1)])
        normals = self._normals(ref_cols, grid, origin, offsets)
        lined = np.any(normals != 0.0, axis=1)
        # A hit may only match a surface seen from the side it is seen from now: the far face of
        # a wall lies within match_m of the near face, and would pull the robot across it.
        views = origins - p
        ref_side = np.sum(ref_views * normals, axis=1)

        def same_side(hits: np.ndarray, ids: np.ndarray) -> np.ndarray:
            return np.sum(views[hits] * normals[ids[hits]], axis=1) * ref_side[ids[hits]] >= 0.0

        rot = np.eye(2)
        trans = np.zeros(2)
        q = p.copy()
        for _ in range(c.iterations):
            ids, dist = self._nearest(q, grid, origin, ref, offsets)
            m = dist <= c.match_m
            m[m] = lined[ids[m]] & same_side(m, ids)
            if m.sum() < c.min_matches:
                return Match(0.0, int(m.sum()), float(m.mean()), math.nan, 0.0, 0.0, False, "too few matches")
            th, t = self._solve(q[m], ref[ids[m]], normals[ids[m]])
            R = _rot(th)
            q = q @ R.T + t
            rot = R @ rot
            trans = R @ trans + t
            if abs(th) < 1e-5 and float(np.hypot(*t)) < 1e-5:
                break

        ids, dist = self._nearest(q, grid, origin, ref, offsets)
        m = dist <= c.match_m
        m[m] = same_side(m, ids)
        share = float(m.mean())
        m[m] = lined[ids[m]]
        matches = int(m.sum())
        off = np.sum((q[m] - ref[ids[m]]) * normals[ids[m]], axis=1)
        rms = float(np.sqrt(np.mean(off ** 2))) if matches else math.nan
        # The change as a turn about the hits' centre plus a shift, so it can be scaled down.
        centre = p.mean(axis=0)
        th = math.atan2(rot[1, 0], rot[0, 0])
        shift = rot @ centre + trans - centre
        step_m = float(np.hypot(*shift))
        match = Match(0.0, matches, share, rms, step_m, th, False)
        if matches < c.min_matches:
            match.reason = "too few matches"
        elif share < c.min_match_share:
            match.reason = "too few hits on known surfaces"
        elif rms > c.max_rms_m:
            match.reason = "residual too high"
        elif step_m < c.deadband_m and abs(th) < c.deadband_rad:
            match.reason = "within the noise"
        if match.reason:
            return match
        scale = min(1.0, c.max_step_m / max(step_m, 1e-9), c.max_step_rad / max(abs(th), 1e-9))
        th *= scale
        shift = shift * scale
        R = _rot(th)
        t = centre + shift - R @ centre
        # The step is applied after the current correction.
        self.yaw = math.atan2(math.sin(self.yaw + th), math.cos(self.yaw + th))
        self.x, self.y = (R @ np.array([self.x, self.y]) + t).tolist()
        match.applied = True
        return match
