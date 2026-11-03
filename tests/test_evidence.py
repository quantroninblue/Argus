"""Map evidence reaches the hub through stalls, cuts and a tight hub budget: Argo keeps it,
numbers it, resends what the hub has not acknowledged and sends no more than its budget."""

import numpy as np

from argus import profile as profiles
from argus.hub.drift import DriftConfig
from argus.hub.intake import IntakeConfig
from argus.hub.session import HubConfig
from argus.protocol.v1 import link_pb2
from argus.sim import world as worlds
from argus.sim.channel import LinkFaults
from argus.sim.hub_episode import NODE_ID, HubLoop


def _example(name):
    return profiles.load(profiles.EXAMPLES_DIR / f"{name}.yaml")


def _run(loop: HubLoop, until_s: float) -> None:
    while loop.sim.t <= until_s:
        loop.step()


def _hub_knows_what_argo_knows(loop: HubLoop, radius: float = 4.0) -> float:
    """Share of the cells Argo knows near the robot (planar band) that the hub knows too."""
    node = loop.sim.node
    site = loop.hub.session(NODE_ID).site_map
    x0, y0 = loop.sim.plant.pos[:2]
    known = both = 0
    for x in np.arange(x0 - radius, x0 + radius, 0.1):
        for y in np.arange(y0 - radius, y0 + radius, 0.1):
            for z in (0.15, 0.35, 0.55):
                if node.cell((x, y, z)) == "unknown":
                    continue
                known += 1
                both += site.at(x, y, z) != 0
    assert known > 500
    return both / known


def test_evidence_seen_during_a_stall_reaches_the_hub_afterwards():
    doc = _example("planar_differential")
    loop = HubLoop(doc, worlds.wall_with_gap(), (0.0, 0.0, 0.0, 0.0), (5.0, 2.0, 0.0), 0.4,
                   LinkFaults(stalls=[(2.0, 6.0)]))
    _run(loop, 30.0)
    assert loop.session.evidence_unacked == 0
    assert _hub_knows_what_argo_knows(loop) > 0.99


def test_evidence_seen_while_disconnected_reaches_the_hub_afterwards():
    doc = _example("planar_differential")
    loop = HubLoop(doc, worlds.wall_with_gap(), (0.0, 0.0, 0.0, 0.0), (5.0, 2.0, 0.0), 0.4,
                   LinkFaults(cuts=[(2.0, 6.0)]))
    _run(loop, 30.0)
    assert loop.session.evidence_unacked == 0
    assert _hub_knows_what_argo_knows(loop) > 0.99


def test_argo_keeps_to_the_hub_budget_and_the_backlog_drains():
    doc = _example("planar_differential")
    # Drift correction off: this compares the hub's cells with Argo's one for one.
    config = HubConfig(intake=IntakeConfig(bytes_per_s=12_000.0, min_burst_bytes=8_000.0),
                       drift=DriftConfig(enabled=False))
    loop = HubLoop(doc, worlds.wall_with_gap(), (0.0, 0.0, 0.0, 0.0), (5.0, 2.0, 0.0), 0.4,
                   hub_config=config)
    _run(loop, 60.0)
    session = loop.hub.session(NODE_ID)
    assert session.evidence_dropped == 0
    assert 0 < loop.session.evidence_budget <= 12_000.0
    assert loop.session.evidence_backlog_bytes < 2_000
    assert _hub_knows_what_argo_knows(loop) > 0.99


def test_a_restarted_argo_is_not_taken_for_a_resend():
    doc = _example("planar_differential")
    loop = HubLoop(doc, worlds.wall_with_gap(), (0.0, 0.0, 0.0, 0.0), (5.0, 2.0, 0.0), 0.4)
    _run(loop, 3.0)
    session = loop.hub.session(NODE_ID)
    m = link_pb2.NodeMessage()
    ev = m.local_evidence
    ev.map_epoch = session.site_map.epoch
    ev.resolution_m = 0.1
    ev.encoding = 1
    ev.payload = b"\x00\x00\x00\x00"
    ev.seq, ev.stream = 1, 12345
    duplicates = session.evidence_duplicates
    session.receive(m, 3.0)
    assert session.evidence_acked == 1 and session.evidence_duplicates == duplicates
    session.receive(m, 3.1)
    assert session.evidence_duplicates == duplicates + 1
