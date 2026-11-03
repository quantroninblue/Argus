"""Loop closure on the hub.

drift.py keeps a robot on surfaces it can still match, within a fraction of a metre. Past that it
cannot tell it is back somewhere, and space first seen while drifted keeps its error. Here each
robot's evidence is kept in submaps, in the odometry frame it arrived in, joined in a pose graph
per site. When a robot's recent surfaces fit older ones the graph gets a loop edge, the graph is
solved, and the map is rebuilt from the submaps where they now belong. Argo takes no part.

A wrong closure bends the map for every robot at the site, so a loop edge has to pass all of:
overlap, a single clear best fit (a repeated room or a row of doors has several), all three
directions constrained (a corridor has one), no hits in space the older submaps saw empty, and a
second match that agrees. The solver can still switch an edge off, and a solution that leaves a
loop edge unsatisfied or stretches odometry far past what it can be off by is thrown away.

Search after Olson, Real-Time Correlative Scan Matching (ICRA 2009) and Hess et al., Real-Time
Loop Closure in 2D LIDAR SLAM (ICRA 2016). Robust solving after Agarwal et al. (ICRA 2013).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

import numpy as np

from argus import sitemap
from argus.hub.drift import DriftConfig, DriftEstimator, SurfaceMemory

Pose = tuple[float, float, float]


@dataclass
class LoopConfig:
    enabled: bool = True
    # A submap ends after this much travel or turning.
    submap_m: float = 2.0
    submap_rad: float = 0.8
    # Recent submaps searched together: more overlap, but drift bends a longer stretch and a true
    # fit stops fitting. And how far back along the chain a target must lie.
    query_submaps: int = 2
    min_gap_m: float = 8.0
    # Submaps either side of the nearest one that form the target.
    target_reach: int = 3
    min_points: int = 80
    # Points searched with, at most: more costs time and adds little.
    search_points: int = 300
    # Search window: a base, plus this much per metre since the robot last matched anything.
    window_m: float = 1.0
    window_per_m: float = 0.1
    max_window_m: float = 3.0
    window_rad: float = 0.15
    window_rad_per_m: float = 0.02
    max_window_rad: float = 0.5
    coarse_m: float = 0.2
    coarse_rad: float = 0.03
    # Acceptance: share of query points on the target, and how far the next separate fit is below.
    min_score: float = 0.5
    min_peak_gap: float = 0.1
    # Fits this close together are one fit.
    same_fit_m: float = 0.5
    same_fit_rad: float = 0.15
    max_free_share: float = 0.05
    max_rms_m: float = 0.1
    # Two matches from different submaps must agree this well, within this much travel.
    agree_m: float = 0.25
    agree_rad: float = 0.06
    confirm_within_m: float = 12.0
    # Smaller than this is left to drift.py.
    min_move_m: float = 0.15
    min_move_rad: float = 0.03
    # Edges. Odometry gets weaker with the distance it covers.
    odom_sigma_m: float = 0.02
    odom_sigma_per_m: float = 0.05
    odom_sigma_rad: float = 0.01
    odom_sigma_rad_per_m: float = 0.03
    match_sigma_m: float = 0.05
    match_sigma_rad: float = 0.02
    # A solution is thrown away if a loop edge ends below this weight, or odometry is stretched
    # past this many sigmas.
    min_edge_weight: float = 0.5
    max_odom_sigmas: float = 6.0
    min_height_m: float = 0.1


def compose(a: Pose, b: Pose) -> Pose:
    c, s = math.cos(a[2]), math.sin(a[2])
    yaw = a[2] + b[2]
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], math.atan2(math.sin(yaw), math.cos(yaw)))


def invert(a: Pose) -> Pose:
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def between(a: Pose, b: Pose) -> Pose:
    return compose(invert(a), b)


def _apply(t: Pose, xy: np.ndarray) -> np.ndarray:
    c, s = math.cos(t[2]), math.sin(t[2])
    return np.column_stack([c * xy[:, 0] - s * xy[:, 1] + t[0], s * xy[:, 0] + c * xy[:, 1] + t[1]])


@dataclass
class Submap:
    """Part of one robot's run in its odometry frame. `correction` takes it to the site frame."""

    robot: str
    index: int
    anchor: Pose
    correction: Pose
    started: float
    start_m: float
    resolution: float
    cells: np.ndarray | None = None
    values: np.ndarray | None = None
    hits: np.ndarray | None = None  # occupied columns [N, 2], metres
    hit_cells: np.ndarray | None = None  # the same as cells [N, 3]
    free: np.ndarray | None = None  # columns seen empty and never occupied [N, 2], metres
    map: object = None
    stamp: float = 0.0

    @property
    def pose(self) -> Pose:
        return compose(self.correction, self.anchor)

    def close(self, correction: Pose, floor_cell: int) -> None:
        self.correction = correction
        cells, values = self.map.observed_cells()
        self.map = None
        self.cells, self.values = cells, values
        above = cells[:, 2] >= floor_cell
        hit = above & (values > 0)
        self.hit_cells = cells[hit]
        columns = np.unique(cells[hit, :2], axis=0) if np.any(hit) else np.empty((0, 2), dtype=np.int32)
        self.hits = (columns + 0.5) * self.resolution
        empty = above & (values < 0)
        free = np.unique(cells[empty, :2], axis=0) if np.any(empty) else np.empty((0, 2), dtype=np.int32)
        if len(free) and len(columns):
            occupied = {tuple(c) for c in columns.tolist()}
            free = free[[tuple(c) not in occupied for c in free.tolist()]]
        self.free = (free + 0.5) * self.resolution

    def site_hits(self) -> np.ndarray:
        return _apply(self.correction, self.hits)

    def site_free(self) -> np.ndarray:
        return _apply(self.correction, self.free)

    def cells_in_site(self, cells: np.ndarray) -> np.ndarray:
        out = cells.copy()
        out[:, :2] = np.floor(_apply(self.correction, (cells[:, :2] + 0.5) * self.resolution)
                              / self.resolution).astype(cells.dtype)
        return out


@dataclass
class Edge:
    a: tuple[str, int]
    b: tuple[str, int]
    z: Pose
    sigma_m: float
    sigma_rad: float
    kind: str  # odometry, match, loop


@dataclass
class Chain:
    estimator: DriftEstimator
    closed: list[Submap] = field(default_factory=list)
    open: Submap | None = None
    travelled_m: float = 0.0
    last_position: tuple[float, float] | None = None
    matched_at_m: float = 0.0


@dataclass
class Candidate:
    target: tuple[str, int]
    query: tuple[str, int]
    fix: Pose  # moves the query from where it is estimated to where it fits
    score: float
    at_m: float


class Field:
    """Where a target's surfaces and empty space are, as grids over the site frame."""

    def __init__(self, hits: np.ndarray, free: np.ndarray, resolution: float, pad_m: float) -> None:
        self.res = resolution
        lo = np.floor(hits.min(axis=0) / resolution).astype(int) - int(pad_m / resolution)
        hi = np.ceil(hits.max(axis=0) / resolution).astype(int) + int(pad_m / resolution)
        self.origin = lo
        size = (hi - lo + 1).tolist()
        occupied = np.zeros(size, dtype=bool)
        cells = np.floor(hits / resolution).astype(int) - lo
        occupied[cells[:, 0], cells[:, 1]] = True
        self.near = self._grow(occupied, 1)
        self.wide = self._grow(occupied, 2)
        empty = np.zeros(size, dtype=bool)
        if len(free):
            cells = np.floor(free / resolution).astype(int) - lo
            inside = np.all((cells >= 0) & (cells < np.array(size)), axis=1)
            empty[cells[inside, 0], cells[inside, 1]] = True
        # Only space well away from a surface counts: a hit a cell off a wall proves nothing.
        self.empty = empty & ~self.wide

    @staticmethod
    def _grow(grid: np.ndarray, cells: int) -> np.ndarray:
        out = grid.copy()
        for _ in range(cells):
            grown = out.copy()
            grown[1:, :] |= out[:-1, :]
            grown[:-1, :] |= out[1:, :]
            grown[:, 1:] |= out[:, :-1]
            grown[:, :-1] |= out[:, 1:]
            out = grown
        return out

    def look(self, grid: np.ndarray, xy: np.ndarray) -> np.ndarray:
        cells = np.floor(xy / self.res).astype(np.int64) - self.origin
        inside = ((cells[..., 0] >= 0) & (cells[..., 0] < grid.shape[0])
                  & (cells[..., 1] >= 0) & (cells[..., 1] < grid.shape[1]))
        out = np.zeros(inside.shape, dtype=bool)
        out[inside] = grid[cells[..., 0][inside], cells[..., 1][inside]]
        return out


def search(points: np.ndarray, target: Field, c: LoopConfig, window_m: float, window_rad: float):
    """Best fit of `points` on `target` within the window, as (fix, score, next separate score)."""
    pivot = points.mean(axis=0)
    steps = int(math.ceil(window_m / c.coarse_m))
    shifts = np.arange(-steps, steps + 1) * c.coarse_m
    dx, dy = np.meshgrid(shifts, shifts, indexing="ij")
    offsets = np.column_stack([dx.ravel(), dy.ravel()])
    turns = np.arange(-int(math.ceil(window_rad / c.coarse_rad)), int(math.ceil(window_rad / c.coarse_rad)) + 1) * c.coarse_rad
    fits = []
    for th in turns.tolist():
        turned = _apply((0.0, 0.0, th), points - pivot) + pivot
        scores = target.look(target.wide, turned[None, :, :] + offsets[:, None, :]).mean(axis=1)
        fits.append(scores)
    fits = np.array(fits)
    best = np.unravel_index(int(np.argmax(fits)), fits.shape)
    th = float(turns[best[0]])
    shift = offsets[best[1]]
    score = float(fits[best])
    distance = np.hypot(offsets[:, 0] - shift[0], offsets[:, 1] - shift[1])
    same = (distance[None, :] <= c.same_fit_m) & (np.abs(turns - th)[:, None] <= c.same_fit_rad)
    others = fits[~same]
    runner_up = float(others.max()) if others.size else 0.0
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    t = pivot + shift - R @ pivot
    return (float(t[0]), float(t[1]), th), score, runner_up


class LoopCloser:
    """One site's submaps and pose graph."""

    def __init__(self, config: LoopConfig, resolution_m: float, epoch: int) -> None:
        self.config = config
        self.res = resolution_m
        self.epoch = epoch
        self.chains: dict[str, Chain] = {}
        self.edges: list[Edge] = []
        self.pending: Candidate | None = None
        self.closures = 0
        self.refused: dict[str, int] = {}
        self._floor = int(math.floor(config.min_height_m / resolution_m))
        self._probe = DriftEstimator(replace(DriftConfig(), deadband_m=0.0, deadband_rad=0.0, max_step_m=math.inf,
                                             max_step_rad=math.inf, min_constraint_share=0.1,
                                             match_m=0.3), resolution_m)

    def submaps(self) -> list[Submap]:
        """Every submap holding evidence, oldest first."""
        found = [s for chain in self.chains.values() for s in chain.closed]
        return sorted(found, key=lambda s: s.started)

    def observe(self, robot: str, estimator: DriftEstimator, payload: bytes, stamp: float, odom: Pose,
                now: float) -> bool:
        """Takes one evidence message. True if a closure moved the site's frame."""
        chain = self.chains.get(robot)
        if chain is None:
            chain = self.chains[robot] = Chain(estimator)
        chain.estimator = estimator
        if chain.last_position is not None:
            chain.travelled_m += math.hypot(odom[0] - chain.last_position[0], odom[1] - chain.last_position[1])
        chain.last_position = (odom[0], odom[1])
        if chain.open is None:
            chain.open = self._begin(robot, chain, odom, now)
        s = chain.open
        s.map.apply_evidence(1, payload, stamp)
        s.stamp = stamp
        turned = abs(math.atan2(math.sin(odom[2] - s.anchor[2]), math.cos(odom[2] - s.anchor[2])))
        if chain.travelled_m - s.start_m < self.config.submap_m and turned < self.config.submap_rad:
            return False
        return self._finish_submap(robot, chain, odom, now)

    def _begin(self, robot: str, chain: Chain, odom: Pose, now: float) -> Submap:
        e = chain.estimator
        s = Submap(robot, len(chain.closed), odom, (e.x, e.y, e.yaw), now, chain.travelled_m, self.res)
        s.map = sitemap.SiteMap(f"{robot}-{s.index}", self.res)
        return s

    def _finish_submap(self, robot: str, chain: Chain, odom: Pose, now: float) -> bool:
        c = self.config
        e = chain.estimator
        s = chain.open
        s.close((e.x, e.y, e.yaw), self._floor)
        chain.open = self._begin(robot, chain, odom, now)
        chain.open.index = s.index + 1
        if chain.closed:
            # The step as drift.py left it, not as raw odometry has it: the drift it took out must
            # stay out when the graph is solved.
            prev = chain.closed[-1]
            length = s.start_m - prev.start_m
            self.edges.append(Edge((robot, prev.index), (robot, s.index), between(prev.pose, s.pose),
                                   c.odom_sigma_m + c.odom_sigma_per_m * length,
                                   c.odom_sigma_rad + c.odom_sigma_rad_per_m * length, "odometry"))
        chain.closed.append(s)
        self._link_to_matched(robot, chain, s)
        found = self._detect(robot, chain)
        if found is None:
            return False
        return self._consider(found, chain)

    def _node(self, key: tuple[str, int]) -> Submap:
        return self.chains[key[0]].closed[key[1]]

    def _older(self, robot: str, chain: Chain) -> list[Submap]:
        """Submaps far enough back to be a loop: other robots', and this robot's own beyond the gap."""
        out = []
        for name, other in self.chains.items():
            if name != robot:
                out += other.closed
            else:
                out += [s for s in other.closed if chain.travelled_m - s.start_m >= self.config.min_gap_m]
        return out

    def _link_to_matched(self, robot: str, chain: Chain, s: Submap) -> None:
        """drift.py keeps a robot on known surfaces in small steps. Where it did, this submap's
        place relative to the older one it overlaps is known, and a closure must keep it."""
        c = self.config
        last = chain.estimator.last
        if last is None or last.reason not in ("", "within the noise") or len(s.hits) == 0:
            return
        here = np.floor(s.site_hits() / self.res).astype(np.int64)
        mine = {tuple(v) for v in here.tolist()}
        best, overlap = None, 0
        x, y, _ = s.pose
        for other in self._older(robot, chain):
            ox, oy, _ = other.pose
            if math.hypot(ox - x, oy - y) > 6.0 or len(other.hits) == 0:
                continue
            cells = np.floor(other.site_hits() / self.res).astype(np.int64)
            shared = sum(1 for v in cells.tolist() if tuple(v) in mine)
            if shared > overlap:
                best, overlap = other, shared
        if best is None or overlap < 0.3 * len(s.hits):
            return
        self.edges.append(Edge((best.robot, best.index), (robot, s.index), between(best.pose, s.pose),
                               c.match_sigma_m, c.match_sigma_rad, "match"))
        chain.matched_at_m = chain.travelled_m

    def _refuse(self, why: str) -> None:
        self.refused[why] = self.refused.get(why, 0) + 1

    def _detect(self, robot: str, chain: Chain) -> Candidate | None:
        c = self.config
        recent = chain.closed[-c.query_submaps:]
        points = np.concatenate([s.site_hits() for s in recent])
        if len(points) < c.min_points:
            return None
        points = np.unique(np.floor(points / self.res).astype(np.int64), axis=0)
        points = (points + 0.5) * self.res
        since = chain.travelled_m - chain.matched_at_m
        window_m = min(c.max_window_m, c.window_m + c.window_per_m * since)
        window_rad = min(c.max_window_rad, c.window_rad + c.window_rad_per_m * since)
        latest = recent[-1]
        x, y, _ = latest.pose
        older = self._older(robot, chain)
        if not older:
            return None
        near = sorted(older, key=lambda s: math.hypot(s.pose[0] - x, s.pose[1] - y))
        if math.hypot(near[0].pose[0] - x, near[0].pose[1] - y) > window_m + 4.0:
            return None
        hub = near[0]
        lane = self.chains[hub.robot].closed
        group = [s for s in lane[max(0, hub.index - c.target_reach):hub.index + c.target_reach + 1]
                 if s in older and len(s.hits)]
        if not group:
            return None
        hits = np.concatenate([s.site_hits() for s in group])
        free = np.concatenate([s.site_free() for s in group])
        target = Field(hits, free, self.res, window_m + 1.0)
        stride = max(1, len(points) // c.search_points)
        fix, score, runner_up = search(points[::stride], target, c, window_m, window_rad)
        if score < c.min_score:
            self._refuse("overlap")
            return None
        if score - runner_up < c.min_peak_gap:
            self._refuse("no clear best fit")
            return None
        fix = self._refine(points, fix, hits, latest)
        if fix is None:
            return None
        placed = _apply(fix, points)
        if float(target.look(target.empty, placed).mean()) > c.max_free_share:
            self._refuse("hits in empty space")
            return None
        return Candidate((hub.robot, hub.index), (robot, latest.index), fix, score, chain.travelled_m)

    def _refine(self, points: np.ndarray, fix: Pose, hits: np.ndarray, latest: Submap) -> Pose | None:
        c = self.config
        columns = np.unique(np.floor(hits / self.res).astype(np.int64), axis=0)
        placed = _apply(fix, points)
        probe = self._probe
        probe.x, probe.y, probe.yaw = 0.0, 0.0, 0.0
        # Sides do not separate surfaces here: the search has already placed the points.
        views = np.zeros((len(columns), 2))
        origins = np.tile(np.array(latest.pose[:2]), (len(placed), 1))
        match = probe._icp(placed, columns, views, origins)
        if not match.applied:
            self._refuse("no refinement")
            return None
        if probe.rank < 3:
            self._refuse("not constrained in every direction")
            return None
        if match.rms_m > c.max_rms_m:
            self._refuse("residual")
            return None
        return compose((probe.x, probe.y, probe.yaw), fix)

    def _consider(self, found: Candidate, chain: Chain) -> bool:
        c = self.config
        q = self._node(found.query)
        moved = between(q.pose, compose(found.fix, q.pose))
        if math.hypot(moved[0], moved[1]) < c.min_move_m and abs(moved[2]) < c.min_move_rad:
            self.pending = None
            return False
        first = self.pending
        if first is None or found.query[0] != first.query[0] or found.query == first.query \
                or found.at_m - first.at_m > c.confirm_within_m or not self._agree(first, found, q):
            self.pending = found
            return False
        self.pending = None
        return self._close([first, found], chain)

    def _agree(self, a: Candidate, b: Candidate, q: Submap) -> bool:
        c = self.config
        point = np.array([q.pose[:2]])
        apart = float(np.hypot(*(_apply(a.fix, point) - _apply(b.fix, point))[0]))
        turn = math.atan2(math.sin(a.fix[2] - b.fix[2]), math.cos(a.fix[2] - b.fix[2]))
        return apart <= c.agree_m and abs(turn) <= c.agree_rad

    def _close(self, found: list[Candidate], chain: Chain) -> bool:
        c = self.config
        new = [Edge(f.target, f.query, between(self._node(f.target).pose, compose(f.fix, self._node(f.query).pose)),
                    c.match_sigma_m, c.match_sigma_rad, "loop") for f in found]
        edges = self.edges + new
        order = [(s.robot, s.index) for s in self.submaps()]
        column = {key: k for k, key in enumerate(order)}
        nodes = [(*self._node(key).pose, key[1] == 0) for key in order]
        wire = [(column[e.a], column[e.b], *e.z, e.sigma_m, e.sigma_rad, e.kind != "odometry") for e in edges]
        poses, weights, _, _, converged = sitemap.optimize_pose_graph(nodes, wire)
        if not converged:
            self._refuse("no solution")
            return False
        if min(weights[len(self.edges):]) < c.min_edge_weight:
            self._refuse("loop edge switched off")
            return False
        if self._stretch(edges, poses, column) > c.max_odom_sigmas:
            self._refuse("odometry stretched too far")
            return False
        for key, pose in zip(order, poses):
            s = self._node(key)
            s.correction = compose(pose, invert(s.anchor))
        for ch in self.chains.values():
            if not ch.closed:
                continue
            t = ch.closed[-1].correction
            ch.estimator.x, ch.estimator.y, ch.estimator.yaw = t
            if ch.open is not None:
                ch.open.correction = t
        self.edges = edges
        chain.matched_at_m = chain.travelled_m
        self.closures += 1
        return True

    def _stretch(self, edges: list[Edge], poses, column) -> float:
        worst = 0.0
        for e in edges:
            if e.kind != "odometry":
                continue
            d = between(poses[column[e.a]], poses[column[e.b]])
            off = between(e.z, d)
            worst = max(worst, math.hypot(off[0], off[1]) / e.sigma_m, abs(off[2]) / e.sigma_rad)
        return worst

    def rebuild_map(self, site_id: str) -> sitemap.SiteMap:
        out = sitemap.SiteMap(site_id, self.res, self.epoch)
        parts = self.submaps()
        for s in parts:
            out.apply_evidence(self.epoch, sitemap.encode_evidence_arrays(s.cells_in_site(s.cells), s.values), s.stamp)
        for chain in self.chains.values():
            s = chain.open
            if s is not None:
                cells, values = s.map.observed_cells()
                if len(cells):
                    out.apply_evidence(self.epoch, sitemap.encode_evidence_arrays(s.cells_in_site(cells), values), s.stamp)
        return out

    def rebuild_surfaces(self, merge_m: float, settle_s: float) -> SurfaceMemory:
        memory = SurfaceMemory(self.res, merge_m, self.epoch, settle_s)
        for s in self.submaps():
            if len(s.hit_cells):
                memory.add(s.cells_in_site(s.hit_cells), s.started, s.pose[:2])
        return memory
