import numpy as np
import pytest

from argus import sitemap
from argus.sim.world import wall_with_gap


def _scan(world, origin, rays=360, max_range=8.0):
    a = np.linspace(-np.pi, np.pi, rays, endpoint=False)
    dirs = np.stack([np.cos(a), np.sin(a), np.zeros_like(a)], axis=1)
    r = world.raycast(np.asarray(origin), dirs, max_range)
    hits = np.isfinite(r)
    ends = np.asarray(origin) + dirs * np.where(hits, r, max_range)[:, None]
    return ends, hits.tolist()


def test_scans_build_a_planar_view_with_the_wall_and_its_gap():
    world = wall_with_gap()
    m = sitemap.SiteMap("site-test")
    for t, origin in enumerate([(-2.0, 0.0, 0.3), (0.0, 2.0, 0.3), (1.5, 2.0, 0.3)]):
        ends, hits = _scan(world, origin)
        m.integrate(origin, ends, hits, float(t))
    view = m.planar_view(0.05, 0.6)
    cells, (ox, oy), res = view["cells"], view["origin"], view["resolution"]

    def at(x, y):
        return cells[int(np.floor(x / res)) - ox, int(np.floor(y / res)) - oy]

    assert at(-1.0, 0.0) == sitemap.FREE
    # The dividing wall's face is on the grid line x = 2.5. Rays reach it at a slant and
    # sparse rays can miss a cell, so check the strip along the face.
    face = [at(2.55, y) for y in np.arange(-3.0, 0.0, 0.1)]
    assert face.count(sitemap.OCCUPIED) >= 0.8 * len(face)
    assert at(2.45, 2.0) == sitemap.FREE and at(2.55, 2.0) == sitemap.FREE  # the gap in it
    assert at(-6.95, 0.0) == sitemap.OCCUPIED  # the outer wall, inner face at x = -7
    assert m.at(-1.0, 0.0, 0.3) == sitemap.FREE


def test_evidence_payload_and_epochs():
    m = sitemap.SiteMap("site-test")
    payload = sitemap.encode_evidence([((3, 4, 2), 6), ((-1, 0, 2), -2)])
    assert m.apply_evidence(m.epoch, payload, 1.0)
    assert m.cell((3, 4, 2)) == sitemap.OCCUPIED
    assert m.cell((-1, 0, 2)) == sitemap.FREE
    old = m.epoch
    m.reset()
    assert not m.apply_evidence(old, payload, 2.0)
    assert m.tile_count == 0
    with pytest.raises(ValueError):
        m.apply_evidence(m.epoch, payload[:-1], 3.0)


def test_tiles_round_trip_and_changed_since():
    m = sitemap.SiteMap("site-test")
    m.apply_evidence(m.epoch, sitemap.encode_evidence([((1, 1, 1), 6)]), 1.0)
    seen = m.version
    m.apply_evidence(m.epoch, sitemap.encode_evidence([((50, 1, 1), 6)]), 2.0)
    assert m.changed_since(seen) == [(3, 0, 0)]
    copy = sitemap.SiteMap("site-test")
    copy.load_tiles(m.encode_all())
    assert copy.cell((50, 1, 1)) == sitemap.OCCUPIED
    assert copy.cell((1, 1, 1)) == sitemap.OCCUPIED
