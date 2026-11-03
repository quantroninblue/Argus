"""Whether each sensor is mounted where the profile says (setup check).

Every range return is placed in the base frame through the declared mount, so a camera
pitched or turned a few degrees from its declared pose puts every obstacle in the wrong
place, and nothing at runtime can tell. With the robot standing on a flat floor:

- a sensor that sees the floor near the robot must see it flat and at floor height;
- two sensors that see the same surfaces at the same heights must place them together.

Works on points already in the base frame, one array per sensor id, so it runs on a live
robot's messages (live.py) and on simulated frames alike.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# Beyond these, the declared mount is wrong enough to matter: 2 degrees puts a wall at 3 m
# about 10 cm off; 4 cm is more than the floor cut Argo uses (5 cm).
MAX_TILT_DEG = 2.0
MAX_FLOOR_OFFSET_M = 0.04
MAX_DISAGREEMENT_M = 0.06
FLOOR_BAND_M = 0.15
FLOOR_REACH_M = 4.0
MIN_POINTS = 50
MIN_PAIRS = 30
PAIR_RADIUS_M = 0.3


@dataclass
class MountFinding:
    name: str
    ok: bool
    detail: str


def _floor(doc: dict, sid: str, pts: np.ndarray, ground: float, mount_xy) -> MountFinding | None:
    near = pts[(np.abs(pts[:, 2] - ground) < FLOOR_BAND_M) & (np.hypot(pts[:, 0], pts[:, 1]) < FLOOR_REACH_M)]
    if len(near) < MIN_POINTS:
        return None
    # z = a x + b y + c, refit once without the points more than 3 cm off (an object's foot).
    sel = near
    for _ in range(2):
        A = np.column_stack([sel[:, 0], sel[:, 1], np.ones(len(sel))])
        (a, b, c), *_ = np.linalg.lstsq(A, sel[:, 2], rcond=None)
        resid = np.abs(near[:, 0] * a + near[:, 1] * b + c - near[:, 2])
        keep = near[resid < 0.03]
        if len(keep) < MIN_POINTS:
            break
        sel = keep
    tilt = math.degrees(math.atan(math.hypot(a, b)))
    offset = float(a * mount_xy[0] + b * mount_xy[1] + c - ground)
    ok = tilt <= MAX_TILT_DEG and abs(offset) <= MAX_FLOOR_OFFSET_M
    detail = f"sees the floor tilted {tilt:.1f} deg and {offset * 100:+.0f} cm from floor height"
    if not ok:
        detail += f": its declared mount (frame {next(s['frame'] for s in doc['sensors'] if s['id'] == sid)}) is probably off"
    return MountFinding(f"mount {sid} (floor)", ok, detail)


def _slice(doc: dict, sid: str, pts: np.ndarray) -> tuple[float, float]:
    """Heights a sensor's structure returns span; a scan plane is one thin layer."""
    spec = next(s for s in doc["sensors"] if s["id"] == sid)
    if spec["kind"] == "range_2d" and len(pts):
        z = float(np.median(pts[:, 2]))
        return z - 0.05, z + 0.05
    return (float(pts[:, 2].min()), float(pts[:, 2].max())) if len(pts) else (0.0, -1.0)


def _nearest(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """For each point of `a` (x, y), the distance to the nearest point of `b`."""
    out = np.empty(len(a))
    for i in range(0, len(a), 256):
        d = np.linalg.norm(a[i:i + 256, None, :] - b[None, :, :], axis=2)
        out[i:i + 256] = d.min(axis=1)
    return out


def _subsample(pts: np.ndarray, n: int = 1500) -> np.ndarray:
    return pts if len(pts) <= n else pts[np.linspace(0, len(pts) - 1, n).astype(int)]


def _pair(doc: dict, a_id: str, a: np.ndarray, b_id: str, b: np.ndarray) -> MountFinding | None:
    lo_a, hi_a = _slice(doc, a_id, a)
    lo_b, hi_b = _slice(doc, b_id, b)
    lo, hi = max(lo_a, lo_b), min(hi_a, hi_b)
    if hi <= lo:
        return None
    pa = _subsample(a[(a[:, 2] >= lo) & (a[:, 2] <= hi)][:, :2])
    pb = _subsample(b[(b[:, 2] >= lo) & (b[:, 2] <= hi)][:, :2])
    if len(pa) < MIN_PAIRS or len(pb) < MIN_PAIRS:
        return None
    d = _nearest(pb, pa)
    d = d[d < PAIR_RADIUS_M]
    if len(d) < MIN_PAIRS:
        return None
    gap = float(np.median(d))
    ok = gap <= MAX_DISAGREEMENT_M
    detail = f"place the same surfaces {gap * 100:.0f} cm apart (median of {len(d)} returns)"
    if not ok:
        detail += ": one of their declared mounts is off"
    return MountFinding(f"mounts {a_id} and {b_id} agree", ok, detail)


def check_mounts(doc: dict, points: dict[str, np.ndarray]) -> list[MountFinding]:
    """`points`: each range sensor's returns in the base frame (N x 3). Sensors with too few
    returns on the floor or on shared surfaces are left out rather than passed."""
    ground = doc["geometry"].get("ground_z_m", 0.0)
    frames = doc["geometry"].get("frames", {})
    out: list[MountFinding] = []
    structure = {}
    for spec in doc["sensors"]:
        sid = spec["id"]
        pts = np.asarray(points.get(sid, np.zeros((0, 3))), float).reshape(-1, 3)
        if not len(pts):
            continue
        mount_xy = frames.get(spec["frame"], {}).get("xyz_m", [0.0, 0.0, 0.0])[:2]
        f = _floor(doc, sid, pts, ground, mount_xy)
        if f:
            out.append(f)
        structure[sid] = pts[pts[:, 2] > ground + FLOOR_BAND_M]
    ids = list(structure)
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            f = _pair(doc, ids[i], structure[ids[i]], ids[j], structure[ids[j]])
            if f:
                out.append(f)
    return out
