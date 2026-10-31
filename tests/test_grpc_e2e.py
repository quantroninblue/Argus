"""End to end over the real transport, in wall time: Argo's C++ gRPC client, a TCP fault
proxy, and the hub's gRPC server, with mutual TLS."""

import copy
from pathlib import Path

import pytest

from argus import profile as profiles
from argus.hub.server import TlsFiles
from argus.sim import world as worlds
from argus.sim.channel import LinkFaults
from argus.sim.grpc_episode import GrpcHubLoop, NodeTls, run_grpc_episode
from argus.sim.hub_episode import NODE_ID
from certs import make_ca, make_hub, make_node

@pytest.fixture(scope="module")
def pki(tmp_path_factory):
    d = tmp_path_factory.mktemp("pki")
    make_ca(d)
    make_hub(d)
    make_node(d, NODE_ID)
    make_node(d, "intruder")
    make_ca(d, "other")
    make_node(d, NODE_ID, ca="other", name="forged")
    return d


def _hub(d):
    return TlsFiles(d / "ca.pem", d / "hub.pem", d / "hub.key")


def _node(d, name=NODE_ID):
    return NodeTls((d / "ca.pem").read_text(), (d / f"{name}.pem").read_text(), (d / f"{name}.key").read_text())


def _doc(name="planar_differential"):
    return copy.deepcopy(profiles.load(profiles.EXAMPLES_DIR / f"{name}.yaml"))


SCENE = (worlds.wall_with_gap(), (0.0, 0.0, 0.0, 0.0), (5.0, 2.0, 0.0))


def _clean(ep):
    assert not ep.body.collided
    assert ep.body.violations == []


def test_robot_reaches_the_goal_over_grpc_with_mutual_tls(pki):
    ep = run_grpc_episode(_doc(), *SCENE, 30.0, faults=LinkFaults(latency_s=0.02, jitter_s=0.01),
                          hub_tls=_hub(pki), node_tls=_node(pki))
    _clean(ep)
    assert ep.goal_reached and ep.mission_state == "reached"
    assert ep.connections == 1
    assert not ep.failsafe_events()


def _run_briefly(pki, node_tls, seconds=3.0):
    loop = GrpcHubLoop(_doc(), *SCENE, hub_tls=_hub(pki), node_tls=node_tls)
    try:
        while loop.sim.t < seconds:
            loop.step()
        return loop.session, loop.hub.session(NODE_ID), loop.server
    finally:
        loop.close()


def test_a_certificate_for_another_node_is_rejected(pki):
    session, worker, _ = _run_briefly(pki, _node(pki, "intruder"))
    assert session.rejected and "certificate is for intruder" in session.reject_detail
    assert worker.connections == 0


def test_a_certificate_from_another_ca_never_gets_a_stream(pki):
    forged = NodeTls((pki / "ca.pem").read_text(), (pki / "forged.pem").read_text(), (pki / "forged.key").read_text())
    session, worker, server = _run_briefly(pki, forged)
    assert not session.connected and not session.accepted
    assert server.streams == 0 and worker.connections == 0


def test_stall_through_the_proxy_runs_the_failsafe_then_resumes(pki):
    doc = _doc("planar_omnidirectional")  # stop on link loss after 3 s
    ep = run_grpc_episode(doc, *SCENE, 40.0, faults=LinkFaults(latency_s=0.02, stalls=[(3.0, 8.0)]),
                          hub_tls=_hub(pki), node_tls=_node(pki))
    _clean(ep)
    kinds = [e["kind"] for e in ep.failsafe_events()]
    assert kinds[:3] == ["failsafe_entered", "failsafe_done", "failsafe_cleared"]
    entered = ep.failsafe_events()[0]["stamp"]
    assert 3.0 + 3.0 - 0.4 <= entered <= 3.0 + 3.0 + 0.3
    assert ep.goal_reached


def test_cut_connection_reconnects_and_the_mission_continues(pki):
    ep = run_grpc_episode(_doc(), *SCENE, 40.0, faults=LinkFaults(latency_s=0.02, cuts=[(3.0, 4.0)]),
                          hub_tls=_hub(pki), node_tls=_node(pki))
    _clean(ep)
    assert ep.connections == 2
    assert not ep.failsafe_events(), "back within the 3 s the profile allows"
    assert ep.goal_reached
