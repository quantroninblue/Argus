"""The hub's evidence intake: fair shares, adaptive capacity, and no robot able to overload it."""

import pytest

from argus import profile as profiles
from argus.hub import Hub, Registry
from argus.hub.intake import Intake, IntakeConfig, max_min_shares, parse_rate
from argus.hub.session import HubConfig
from argus.protocol.v1 import link_pb2, node_messages_pb2, session_pb2


def test_rates_read_like_network_bandwidths():
    assert parse_rate("16Mbit") == pytest.approx(2e6)
    assert parse_rate("2MB") == pytest.approx(2e6)
    assert parse_rate("500KB") == pytest.approx(5e5)
    assert parse_rate("800kbit/s") == pytest.approx(1e5)
    assert parse_rate("1000") == 1000
    with pytest.raises(ValueError):
        parse_rate("fast")


def test_max_min_shares_give_quiet_robots_what_they_ask_and_split_the_rest():
    assert max_min_shares(900, [100, 1000, 1000]) == pytest.approx([100, 400, 400])
    assert max_min_shares(900, [1000]) == pytest.approx([900])
    # Spare capacity is spread evenly so a robot that starts sending is not refused.
    assert max_min_shares(900, [0, 0, 300]) == pytest.approx([200, 200, 500])
    assert sum(max_min_shares(1000, [5000, 7000, 10, 0])) == pytest.approx(1000)


class _Fake:
    def __init__(self, offered, cpu):
        self.offered, self.cpu = offered, cpu
        self.evidence_demand = 0.0
        self.rate = None

    def take_evidence_counters(self):
        return self.offered, self.cpu

    def set_evidence_rate(self, rate, config):
        self.rate = rate


def test_capacity_backs_off_when_applying_evidence_eats_the_hub_and_recovers_after():
    intake = Intake(IntakeConfig(bytes_per_s=1e6, cpu_share=0.5))
    busy = [_Fake(50_000, 0.04)]  # 80% of each 50 ms step spent on evidence
    for k in range(20):
        intake.allocate(busy, k * 0.05)
    assert intake.capacity < 0.1e6
    idle = [_Fake(50_000, 0.001)]
    for k in range(20, 60):
        intake.allocate(idle, k * 0.05)
    assert intake.capacity == pytest.approx(1e6)


def _evidence(size):
    m = link_pb2.NodeMessage()
    m.local_evidence.encoding = node_messages_pb2.EVIDENCE_ENCODING_VOXEL_DELTA
    m.local_evidence.payload = b"\0" * size  # malformed on purpose: counted, never applied
    return m


def test_a_flooding_robot_cannot_crowd_out_another_or_the_heartbeats():
    doc = profiles.load(profiles.EXAMPLES_DIR / "planar_omnidirectional.yaml")
    r = Registry()
    p = r.add_profile(doc)
    for node in ("flood", "quiet"):
        r.add_robot(node, p.id, "site-1")
    hub = Hub(r, config=HubConfig(intake=IntakeConfig(bytes_per_s=400_000)))
    conns = {}
    for node in ("flood", "quiet"):
        c = hub.connect()
        h = link_pb2.NodeMessage(seq=1)
        h.hello.CopyFrom(session_pb2.Hello(node_id=node, protocol_version=1, profile_id=p.id, profile_hash=p.hash))
        c.receive(h.SerializeToString(), 0.0)
        c.take_outgoing()
        conns[node] = c

    beats = 0
    echoes = 0
    for k in range(200):  # 10 s
        t = k * 0.05
        hub.step(t)
        for _ in range(10):  # 50 KB every 5 ms: 10 MB/s
            conns["flood"].receive(_evidence(50_000).SerializeToString(), t)
        conns["quiet"].receive(_evidence(1_000).SerializeToString(), t)  # 20 KB/s
        hb = link_pb2.NodeMessage(seq=k + 2)
        hb.heartbeat.seq = k + 1
        conns["flood"].receive(hb.SerializeToString(), t)
        beats += 1
        echoes += sum(1 for b in conns["flood"].take_outgoing() if link_pb2.HubMessage.FromString(b).HasField("heartbeat"))
        conns["quiet"].take_outgoing()

    flood, quiet = hub.sessions["flood"], hub.sessions["quiet"]
    assert echoes == beats  # never shed
    assert quiet.evidence_dropped == 0
    assert flood.evidence_dropped > 1500
    # Of the 400 KB/s, the quiet robot keeps what it needs; the flooding one gets the rest.
    assert quiet.evidence_rate >= 20_000
    assert flood.evidence_rate == pytest.approx(400_000 - quiet.evidence_rate, rel=0.05)
