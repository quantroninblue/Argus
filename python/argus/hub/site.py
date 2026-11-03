"""The site service: one persistent map per site, fed by evidence from every robot there."""

from __future__ import annotations

import math
from collections import Counter

import numpy as np

from argus import sitemap
from argus.hub.drift import DriftEstimator, SurfaceMemory
from argus.hub.loops import LoopCloser, LoopConfig
from argus.protocol.v1 import node_messages_pb2


def _is_identity(pose) -> bool:
    p, q = pose.position, pose.orientation
    if math.hypot(p.x, p.y, p.z) > 1e-6:
        return False
    # An unset orientation (all zero) also means identity.
    return abs(q.x) < 1e-6 and abs(q.y) < 1e-6 and abs(q.z) < 1e-6


class SiteService:
    def __init__(self, resolution_m: float = 0.1, merge_m: float = 0.3,
                 settle_s: float = 3.0, loops: LoopConfig | None = None) -> None:
        self.resolution_m = resolution_m
        self.merge_m = merge_m
        self.settle_s = settle_s
        self._maps: dict[str, sitemap.SiteMap] = {}
        # Surfaces as first seen per site, the reference for drift correction (drift.py).
        self._surfaces: dict[str, SurfaceMemory] = {}
        self.loops = loops or LoopConfig()
        self._closers: dict[str, LoopCloser] = {}
        # Floor and ceiling heights per site, in the site frame. A site's floor is at z = 0
        # unless set otherwise; a ceiling is only known if set. Aerial routes stay between.
        self._heights: dict[str, tuple[float, float | None]] = {}
        # Evidence messages applied, and dropped by reason.
        self.applied = 0
        self.dropped: Counter[str] = Counter()

    def map(self, site_id: str) -> sitemap.SiteMap:
        m = self._maps.get(site_id)
        if m is None:
            m = self._maps[site_id] = sitemap.SiteMap(site_id, self.resolution_m)
        return m

    def surfaces(self, site_id: str) -> SurfaceMemory:
        epoch = self.map(site_id).epoch
        mem = self._surfaces.get(site_id)
        if mem is None or mem.epoch != epoch:
            mem = self._surfaces[site_id] = SurfaceMemory(self.resolution_m, self.merge_m, epoch, self.settle_s)
        return mem

    def closer(self, site_id: str) -> LoopCloser:
        epoch = self.map(site_id).epoch
        c = self._closers.get(site_id)
        if c is None or c.epoch != epoch:
            c = self._closers[site_id] = LoopCloser(self.loops, self.resolution_m, epoch)
        return c

    def closures(self, site_id: str) -> int:
        c = self._closers.get(site_id)
        return c.closures if c is not None else 0

    def _rebuild(self, site_id: str, closer: LoopCloser) -> None:
        """Replaces a site's map and surfaces with its submaps laid out where they now belong.
        The epoch stays: evidence from Argo is in odometry, which did not change."""
        self._maps[site_id] = closer.rebuild_map(site_id)
        self._surfaces[site_id] = closer.rebuild_surfaces(self.merge_m, self.settle_s)

    def set_heights(self, site_id: str, floor_z: float = 0.0, ceiling_z: float | None = None) -> None:
        self._heights[site_id] = (floor_z, ceiling_z)

    def heights(self, site_id: str) -> tuple[float, float | None]:
        return self._heights.get(site_id, (0.0, None))

    def apply(self, site_id: str, evidence: node_messages_pb2.LocalEvidence,
              drift: DriftEstimator | None = None, now: float = 0.0, odom_pose=None,
              node_id: str = "", odom_yaw: float = 0.0) -> bool:
        """Adds one robot's local evidence to the site map. False if it was dropped. With
        `drift`, the evidence (odometry frame) is moved into the site frame by the robot's
        correction, and its hits feed the correction and the site's surface memory. `odom_pose`
        is where the robot last said it was, which tells which side of a surface it sees."""
        m = self.map(site_id)
        reason = ""
        if evidence.encoding != node_messages_pb2.EVIDENCE_ENCODING_VOXEL_DELTA:
            reason = "encoding"
        elif abs(evidence.resolution_m - m.resolution) > 1e-9:
            reason = "resolution"
        elif evidence.map_epoch != m.epoch:
            reason = "epoch"
        elif not _is_identity(evidence.map_to_odom):
            # Without relocalization (M4) the hub cannot place cells recorded in a shifted frame.
            reason = "frame"
        if not reason:
            try:
                payload = evidence.payload
                hits = None
                if drift is not None:
                    cells, deltas = sitemap.decode_evidence(payload)
                    drift.observe(now, cells, deltas, odom_pose)
                    moved = drift.evidence_to_map(cells)
                    if moved is not cells:
                        payload = sitemap.encode_evidence_arrays(moved, deltas)
                    floor_cell = int(np.floor(drift.config.min_height_m / self.resolution_m))
                    hits = moved[(deltas > 0) & (moved[:, 2] >= floor_cell)]
                if m.apply_evidence(evidence.map_epoch, payload, evidence.stamp.unix_nanos * 1e-9):
                    self.applied += 1
                    if hits is not None and len(hits):
                        seen_from = None if odom_pose is None else drift.point_to_map(odom_pose)
                        self.surfaces(site_id).add(hits, now, seen_from)
                    if drift is not None and node_id and self.loops.enabled:
                        closer = self.closer(site_id)
                        pose = (0.0, 0.0, 0.0) if odom_pose is None else (odom_pose[0], odom_pose[1], odom_yaw)
                        if closer.observe(node_id, drift, evidence.payload, evidence.stamp.unix_nanos * 1e-9,
                                          pose, now):
                            self._rebuild(site_id, closer)
                    return True
                reason = "epoch"
            except ValueError:
                reason = "malformed"
        self.dropped[reason] += 1
        return False
