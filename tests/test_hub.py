"""Argus hub pieces on their own: registry, site service, exploration memory, mission manager,
and the session handshake with Argo's side."""

import copy

import numpy as np
import pytest

from argus import node as argo
from argus import profile as profiles
from argus import sitemap
from argus.hub import Hub, MissionState, Registry, SiteService
from argus.hub.exploration import ExplorationMemory, PlaceStatus
from argus.hub.mission import GOAL_ID, MissionManager
from argus.protocol.v1 import link_pb2, node_messages_pb2, session_pb2
from argus.sim import world as worlds


def _example(name="planar_omnidirectional"):
    return profiles.load(profiles.EXAMPLES_DIR / f"{name}.yaml")


def _registry(doc):
    r = Registry()
    p = r.add_profile(doc)
    r.add_robot("argo-1", p.id, "site-1")
    return r, p


def _hello(p, **changes):
    h = session_pb2.Hello(node_id="argo-1", protocol_version=1, profile_id=p.id, profile_hash=p.hash)
    for k, v in changes.items():
        setattr(h, k, v)
    return h


def test_registry_accepts_the_registered_profile_and_nothing_else():
    doc = _example()
    r, p = _registry(doc)
    assert r.check_hello(_hello(p))[0] == session_pb2.REJECT_REASON_UNSPECIFIED
    assert r.check_hello(_hello(p, node_id="argo-2"))[0] == session_pb2.REJECT_REASON_UNKNOWN_NODE
    assert r.check_hello(_hello(p, protocol_version=2))[0] == session_pb2.REJECT_REASON_PROTOCOL_VERSION
    edited = copy.deepcopy(doc)
    edited["motion"]["limits"]["x"]["velocity_mps"][1] *= 2
    reason, detail = r.check_hello(_hello(p, profile_hash=profiles.profile_hash(edited)))
    assert reason == session_pb2.REJECT_REASON_PROFILE_MISMATCH and p.id in detail


def test_registry_refuses_sessions_for_unreviewed_profiles():
    doc = _example()
    doc.setdefault("provenance", {})["control.latency_s"] = "llm_draft"
    r, p = _registry(doc)
    assert not p.safety_ready
    assert r.check_hello(_hello(p))[0] == session_pb2.REJECT_REASON_PROFILE_NOT_SAFETY_READY


def test_registry_rejects_invalid_profiles():
    doc = _example()
    del doc["motion"]
    with pytest.raises(ValueError):
        Registry().add_profile(doc)


def test_registry_loads_from_a_directory(tmp_path):
    (tmp_path / "profiles").mkdir()
    (tmp_path / "profiles" / "omni.yaml").write_text(profiles.dump(_example()))
    (tmp_path / "robots.yaml").write_text("argo-7: {profile: example-planar-omnidirectional, site: hall}\n")
    r = Registry.from_dir(tmp_path)
    robot = r.robot("argo-7")
    assert robot.site_id == "hall" and r.profile(robot.profile_id).safety_ready


def _evidence(m, **changes):
    ev = node_messages_pb2.LocalEvidence(map_epoch=m.epoch, resolution_m=m.resolution,
                                         encoding=node_messages_pb2.EVIDENCE_ENCODING_VOXEL_DELTA,
                                         payload=sitemap.encode_evidence([((10, 10, 2), 6)]))
    ev.map_to_odom.orientation.w = 1.0
    for k, v in changes.items():
        setattr(ev, k, v)
    return ev


def test_site_service_applies_good_evidence_and_drops_the_rest():
    sites = SiteService()
    m = sites.map("site-1")
    assert sites.apply("site-1", _evidence(m))
    assert m.cell((10, 10, 2)) == sitemap.OCCUPIED
    assert not sites.apply("site-1", _evidence(m, map_epoch=m.epoch + 1))
    assert not sites.apply("site-1", _evidence(m, resolution_m=0.2))
    assert not sites.apply("site-1", _evidence(m, payload=b"\x05\x00"))
    shifted = _evidence(m)
    shifted.map_to_odom.position.x = 1.0
    assert not sites.apply("site-1", shifted)
    assert dict(sites.dropped) == {"epoch": 1, "resolution": 1, "malformed": 1, "frame": 1}


def test_only_dead_ends_and_searched_places_change_memory_for_good():
    mem = ExplorationMemory(planar=True)
    for cause in ("controller_no_progress", "localization_invalid", "sensor_invalid",
                  "compute_deadline_miss", "link_lost"):
        assert not mem.record_failure((1.0, 0.0, 0.0), cause, 0.0)
    assert mem.blocked_zones(0.0) == []
    assert mem.record_failure((3.0, 0.0, 0.0), "dead_end_observed", 0.0)
    assert mem.record_failure((0.0, 3.0, 0.0), "temporary_obstruction", 0.0)
    assert len(mem.blocked_zones(1.0)) == 2
    # The obstruction is retried after a while; the dead end stays until the goal changes.
    assert mem.blocked_zones(25.0) == [(3.0, 0.0, 0.0, mem.zone_radius_m)]
    mem.new_goal()
    assert mem.blocked_zones(25.0) == []
    # Nearby reports merge into one place.
    mem.record_failure((3.2, 0.1, 0.0), "dead_end_observed", 30.0)
    assert len(mem.places) == 2 and mem.places[0].status == PlaceStatus.EXHAUSTED_FOR_GOAL


def _mission(doc=None):
    doc = doc or _example()
    planner = sitemap.RoutePlanner(profiles.dump(doc))
    return MissionManager(planner), sitemap.SiteMap("site-1")


def test_mission_sends_a_short_plan_and_keeps_it_while_it_is_still_good():
    mission, m = _mission()
    mission.set_goal((14.0, 0.0, 0.0), 0.3, 0.0)
    plan = mission.update(0.0, (0.0, 0.0, 0.0), m)
    assert len(plan) == mission.config.max_waypoints and plan[-1].id != GOAL_ID
    assert all(w.speculative for w in plan), "nothing has been seen yet"
    mission.sent_as(1, 0.0)
    mission.on_telemetry(1, plan[0].id)
    # A second later, a little further along: the plan Argo holds is still the way to go.
    assert mission.update(1.0, (0.5, 0.0, 0.0), m) is None
    for k in range(3):
        mission.on_event("step_reached", plan[k].id, "unspecified", 1, plan[k].position, 2.0 + k)
    # One waypoint left and it is not the goal: extend before Argo slows for it.
    plan2 = mission.update(5.0, plan[2].position, m)
    assert plan2 and plan2[-1].id == GOAL_ID and plan2[-1].position == (14.0, 0.0, 0.0)
    mission.sent_as(2, 5.0)
    mission.on_event("step_reached", GOAL_ID, "unspecified", 2, (13.9, 0.0, 0.0), 9.0)
    assert mission.state == MissionState.REACHED


def test_mission_resends_when_argo_never_took_up_the_plan():
    mission, m = _mission()
    mission.set_goal((5.0, 0.0, 0.0), 0.3, 0.0)
    assert mission.update(0.0, (0.0, 0.0, 0.0), m)
    mission.sent_as(1, 0.0)
    assert mission.update(1.0, (0.0, 0.0, 0.0), m) is None
    assert mission.update(2.5, (0.0, 0.0, 0.0), m), "telemetry still shows the old generation"


def test_dead_end_moves_the_route_and_noise_does_not():
    mission, m = _mission()
    mission.set_goal((8.0, 0.0, 0.0), 0.3, 0.0)
    plan = mission.update(0.0, (0.0, 0.0, 0.0), m)
    mission.sent_as(1, 0.0)
    mission.on_event("step_failed", plan[0].id, "controller_no_progress", 1, (0.5, 0.0, 0.0), 5.0)
    assert mission.memory.places[0].status == PlaceStatus.ACTIVE
    resent = mission.update(5.0, (0.5, 0.0, 0.0), m)
    assert resent, "no progress gets a fresh plan but no memory change"
    mission.sent_as(2, 5.0)
    mission.on_event("step_failed", resent[0].id, "dead_end_observed", 2, (0.5, 0.0, 0.0), 6.0)
    blocked = mission.memory.blocked_zones(6.0)
    assert len(blocked) == 1
    around = mission.update(6.0, (0.5, 0.0, 0.0), m)
    zx, zy, _, zr = blocked[0]
    assert all((w.position[0] - zx) ** 2 + (w.position[1] - zy) ** 2 > zr ** 2 for w in around)


def test_events_about_replaced_plans_are_ignored():
    mission, m = _mission()
    mission.set_goal((8.0, 0.0, 0.0), 0.3, 0.0)
    plan = mission.update(0.0, (0.0, 0.0, 0.0), m)
    mission.sent_as(5, 0.0)
    mission.on_event("step_failed", plan[0].id, "dead_end_observed", 4, (0.5, 0.0, 0.0), 1.0)
    assert mission.memory.blocked_zones(1.0) == []


def test_unreachable_goal_ends_as_searched_once_the_robot_stands_at_the_closest_point():
    mission, m = _mission()
    # A closed box around the goal, seen from every side.
    box = worlds.World()
    box.add((3.0, -1.2, 0.0), (5.0, -1.0, 1.0)).add((3.0, 1.0, 0.0), (5.0, 1.2, 1.0))
    box.add((3.0, -1.0, 0.0), (3.2, 1.0, 1.0)).add((4.8, -1.0, 0.0), (5.0, 1.0, 1.0))
    a = np.linspace(-np.pi, np.pi, 2880, endpoint=False)
    dirs = np.stack([np.cos(a), np.sin(a), np.zeros_like(a)], axis=1)
    for t, origin in enumerate([(0.0, 0.0, 0.3), (4.0, 3.0, 0.3), (8.0, 0.0, 0.3), (4.0, -3.0, 0.3)]):
        r = box.raycast(np.asarray(origin), dirs, 8.0)
        hits = np.isfinite(r)
        m.integrate(list(origin), np.asarray(origin) + dirs * np.where(hits, r, 8.0)[:, None], hits.tolist(), t)
    mission.set_goal((4.0, 0.0, 0.0), 0.3, 0.0)
    plan = mission.update(0.0, (0.0, 0.0, 0.0), m)
    assert plan and not mission.last_route["reached"]
    end = mission.last_route["path"][-1]
    for k in range(1, 4):
        mission.update(float(k), tuple(end), m)
    assert mission.state == MissionState.FAILED and mission.cause == "searched_no_goal"


def _argo(doc, p, node_id="argo-1"):
    node = argo.Node(profiles.dump(doc))
    cfg = argo.SessionConfig()
    cfg.node_id = node_id
    cfg.profile_hash = p.hash
    return node, argo.Session(node, cfg)


def _pump(session, conn, hub, t):
    for b in session.take_outgoing():
        conn.receive(b, t)
    hub.step(t)
    for b in conn.take_outgoing():
        session.receive(b, t)


def test_session_handshake_and_heartbeats_and_no_blind_plan():
    doc = _example()
    r, p = _registry(doc)
    hub = Hub(r)
    hub.set_goal("argo-1", (4.0, 0.0, 0.0), 0.3, 0.0)
    node, session = _argo(doc, p)
    conn = hub.connect()
    session.on_connected(0.0)
    t = 0.0
    while t < 2.0:
        node.on_odometry(t, [0.0, 0.0, 0.0], 0.0, [0.0, 0.0, 0.0], 0.0)
        session.step(t)
        _pump(session, conn, hub, t)
        t += 0.02
    assert session.accepted and session.map_epoch == 1
    assert session.link_state == "connected"
    assert 0.0 <= session.round_trip_s < 0.05
    s = hub.session("argo-1")
    assert s.pose == (0.0, 0.0, 0.0) and s.connections == 1
    # This Argo has no range sensors feeding it, so the hub has seen nothing around it and
    # must not plan blind.
    assert not s.evidence_seen and node.plan_generation == 0


def test_wrong_profile_is_rejected_and_argo_stops_asking():
    doc = _example()
    r, p = _registry(doc)
    hub = Hub(r)
    other = copy.deepcopy(doc)
    other["motion"]["limits"]["x"]["velocity_mps"][1] *= 0.5
    node = argo.Node(profiles.dump(other))
    cfg = argo.SessionConfig()
    cfg.node_id = "argo-1"
    cfg.profile_hash = profiles.profile_hash(other)
    session = argo.Session(node, cfg)
    conn = hub.connect()
    session.on_connected(0.0)
    for k in range(100):
        session.step(k * 0.02)
        _pump(session, conn, hub, k * 0.02)
    assert session.rejected and not session.accepted
    assert "registry has" in session.reject_detail
    assert conn.rejected and conn.reject_reason == session_pb2.REJECT_REASON_PROFILE_MISMATCH
    assert session.take_outgoing() == []


def test_reconnect_keeps_the_worker_and_supersedes_the_old_stream():
    doc = _example()
    r, p = _registry(doc)
    hub = Hub(r)
    node, session = _argo(doc, p)
    first = hub.connect()
    session.on_connected(0.0)
    for k in range(20):
        session.step(k * 0.02)
        _pump(session, first, hub, k * 0.02)
    worker = hub.session("argo-1")
    second = hub.connect()
    session.on_connected(0.5)
    for k in range(25, 45):
        session.step(k * 0.02)
        _pump(session, second, hub, k * 0.02)
    assert hub.session("argo-1") is worker and worker.connections == 2
    assert first.closed and worker.connection is second


def test_hub_never_sends_motion_commands():
    fields = {f.name for f in link_pb2.HubMessage.DESCRIPTOR.oneofs_by_name["body"].fields}
    assert not any(word in name for name in fields for word in ("velocity", "twist", "torque", "setpoint"))
