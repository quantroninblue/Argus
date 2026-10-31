"""The site service: one persistent map per site, fed by evidence from every robot there."""

from __future__ import annotations

import math
from collections import Counter

from argus import sitemap
from argus.protocol.v1 import node_messages_pb2


def _is_identity(pose) -> bool:
    p, q = pose.position, pose.orientation
    if math.hypot(p.x, p.y, p.z) > 1e-6:
        return False
    # An unset orientation (all zero) also means identity.
    return abs(q.x) < 1e-6 and abs(q.y) < 1e-6 and abs(q.z) < 1e-6


class SiteService:
    def __init__(self, resolution_m: float = 0.1) -> None:
        self.resolution_m = resolution_m
        self._maps: dict[str, sitemap.SiteMap] = {}
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

    def set_heights(self, site_id: str, floor_z: float = 0.0, ceiling_z: float | None = None) -> None:
        self._heights[site_id] = (floor_z, ceiling_z)

    def heights(self, site_id: str) -> tuple[float, float | None]:
        return self._heights.get(site_id, (0.0, None))

    def apply(self, site_id: str, evidence: node_messages_pb2.LocalEvidence) -> bool:
        """Adds one robot's local evidence to the site map. False if it was dropped."""
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
                if m.apply_evidence(evidence.map_epoch, evidence.payload, evidence.stamp.unix_nanos * 1e-9):
                    self.applied += 1
                    return True
                reason = "epoch"
            except ValueError:
                reason = "malformed"
        self.dropped[reason] += 1
        return False
