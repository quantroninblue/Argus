"""The hub's side of one robot's session, without a transport.

A session worker lives as long as the hub knows the robot, not as long as one connection: a
reconnect attaches a new connection to the same worker, which keeps the mission, exploration
memory and plan generation. Like Argo's side, it is fed parsed messages and stepped with an
explicit clock, so the simulator can drive it in sim time and the gRPC gateway in wall time.
"""

from __future__ import annotations

import itertools
import math
import time
from dataclasses import dataclass, field

from argus import sitemap
from argus.hub.intake import IntakeConfig
from argus.hub.mission import MissionConfig, MissionManager, MissionState
from argus.hub.registry import Profile, Robot
from argus.hub.site import SiteService
from argus.protocol.v1 import common_pb2, hub_messages_pb2, link_pb2, node_messages_pb2

_session_ids = itertools.count(1)


@dataclass
class HubConfig:
    # No plans go to a node the hub has not heard from for this long; they would be lost.
    silent_after_s: float = 1.0
    min_obstacle_height_m: float = 0.05
    mission: MissionConfig = field(default_factory=MissionConfig)
    # How much map evidence the hub takes in (intake.py). Measured offers: about 40 KB/s for a
    # ground robot, 620 KB/s for a drone while moving.
    intake: IntakeConfig = field(default_factory=IntakeConfig)


def nanos(seconds: float) -> int:
    return int(round(seconds * 1e9))


def enum_name(enum, value: int, prefix: str) -> str:
    return enum.Name(value)[len(prefix):].lower()


class HubSession:
    def __init__(self, robot: Robot, profile: Profile, sites: SiteService, config: HubConfig) -> None:
        self.robot = robot
        self.profile = profile
        self.sites = sites
        self.config = config
        self.planner = sitemap.RoutePlanner(profile.text, config.min_obstacle_height_m, sites.resolution_m)
        self.mission = MissionManager(self.planner, config.mission)
        if not self.planner.planar:
            floor, ceiling = sites.heights(robot.site_id)
            room = self.planner.min_clearance_m
            self.mission.z_limits = (floor + room, math.nan if ceiling is None else ceiling - room)
        self.connection = None
        self.session_id = ""
        self.generation = 0
        self.node_generation = 0
        self.pose: tuple[float, float, float] | None = None
        self.safety = node_messages_pb2.SAFETY_STATE_UNSPECIFIED
        self.link_state = node_messages_pb2.LINK_STATE_UNSPECIFIED
        self.failsafe = False
        # No routes before the robot's own evidence is in the site map: a plan made blind can
        # put waypoints where the body cannot go, and still look valid later.
        self.evidence_seen = False
        self.last_heard = -math.inf
        # (time, kind, waypoint id, cause, detail) for every event Argo reported.
        self.events: list[tuple[float, str, str, str, str]] = []
        self.connections = 0
        # Evidence intake: the rate the hub allots this robot, a token bucket enforcing it, and
        # what the robot offered and what applying it cost since the last allocation.
        self.evidence_rate = config.intake.bytes_per_s
        self._evidence_burst = max(config.intake.min_burst_bytes, config.intake.burst_s * self.evidence_rate)
        self._evidence_tokens = self._evidence_burst
        self._evidence_at = -math.inf
        self.evidence_demand = 0.0
        self._evidence_offered = 0
        self._evidence_cpu_s = 0.0
        self.evidence_dropped = 0  # messages over the allotted rate
        self._beat_seq = 0
        self._seq = 0
        self._stop_sent_for: MissionState | None = None

    @property
    def site_map(self) -> sitemap.SiteMap:
        return self.sites.map(self.robot.site_id)

    def attach(self, connection, hello, now: float) -> None:
        self.connection = connection
        self.connections += 1
        self.session_id = f"{self.robot.node_id}-{next(_session_ids)}"
        self.last_heard = now
        self.node_generation = hello.last_plan_generation
        resume = (self.generation > 0 and hello.last_plan_generation == self.generation
                  and self.mission.state == MissionState.ACTIVE)
        if not resume:
            self.mission.resend()
        m = link_pb2.HubMessage()
        a = m.session_accept
        a.session_id = self.session_id
        a.site_id = self.robot.site_id
        a.map_epoch = self.site_map.epoch
        a.clock_offset_nanos = 0  # both clocks are disciplined to UTC; M4 measures the offset
        a.resume_plan = resume
        self._send(m)

    def detach(self) -> None:
        self.connection = None

    def set_evidence_rate(self, rate: float, intake: IntakeConfig) -> None:
        self.evidence_rate = rate
        self._evidence_burst = max(intake.min_burst_bytes, intake.burst_s * rate)
        self._evidence_tokens = min(self._evidence_tokens, self._evidence_burst)

    def take_evidence_counters(self) -> tuple[int, float]:
        """Bytes offered and seconds spent applying evidence since the last call."""
        out = (self._evidence_offered, self._evidence_cpu_s)
        self._evidence_offered, self._evidence_cpu_s = 0, 0.0
        return out

    def _evidence_allowed(self, size: int, now: float) -> bool:
        """Token bucket at the allotted rate."""
        self._evidence_offered += size
        if math.isfinite(self._evidence_at):
            self._evidence_tokens = min(self._evidence_burst,
                                        self._evidence_tokens + (now - self._evidence_at) * self.evidence_rate)
        self._evidence_at = now
        if size > self._evidence_tokens:
            return False
        self._evidence_tokens -= size
        return True

    def receive(self, m: link_pb2.NodeMessage, now: float) -> None:
        self.last_heard = now
        body = m.WhichOneof("body")
        if body == "heartbeat":
            # Answered at once, one for one, so Argo measures the real round trip and loss.
            beat = link_pb2.HubMessage()
            self._beat_seq += 1
            beat.heartbeat.seq = self._beat_seq
            beat.heartbeat.sent.unix_nanos = nanos(now)
            beat.heartbeat.echo_seq = m.heartbeat.seq
            beat.heartbeat.echo_sent.unix_nanos = m.heartbeat.sent.unix_nanos
            self._send(beat)
        elif body == "telemetry":
            t = m.telemetry
            p = t.pose_map.position
            self.pose = (p.x, p.y, p.z)
            self.safety = t.safety
            self.link_state = t.link.state
            self.node_generation = max(self.node_generation, t.plan_generation)
            self._set_failsafe(t.safety == node_messages_pb2.SAFETY_STATE_FAILSAFE)
            self.mission.on_telemetry(t.plan_generation, t.active_waypoint_id)
        elif body == "local_evidence":
            if not self._evidence_allowed(m.local_evidence.ByteSize(), now):
                self.evidence_dropped += 1
            else:
                started = time.perf_counter()
                if self.sites.apply(self.robot.site_id, m.local_evidence):
                    self.evidence_seen = True
                self._evidence_cpu_s += time.perf_counter() - started
        elif body == "event":
            self._on_event(m.event, now)

    def _on_event(self, e: node_messages_pb2.Event, now: float) -> None:
        kind = enum_name(node_messages_pb2.EventKind, e.kind, "EVENT_KIND_")
        cause = enum_name(common_pb2.FailureCause, e.cause, "FAILURE_CAUSE_")
        self.events.append((now, kind, e.waypoint_id, cause, e.detail))
        if kind == "failsafe_entered":
            self._set_failsafe(True)
        elif kind == "failsafe_cleared":
            self._set_failsafe(False)
        elif self.pose is not None:
            self.mission.on_event(kind, e.waypoint_id, cause, e.plan_generation, self.pose, now)

    def _set_failsafe(self, active: bool) -> None:
        self.failsafe = active
        self.mission.set_paused(active)

    def step(self, now: float) -> None:
        if self.connection is None:
            return
        if self.pose is None or not self.evidence_seen or now - self.last_heard > self.config.silent_after_s:
            return
        waypoints = self.mission.update(now, self.pose, self.site_map)
        if waypoints:
            self._send_plan(waypoints, now)
        state = self.mission.state
        if state in (MissionState.FAILED, MissionState.CANCELLED) and self._stop_sent_for != state:
            m = link_pb2.HubMessage()
            m.plan_cancel.generation = max(self.generation, self.node_generation)
            m.plan_cancel.then = hub_messages_pb2.PLAN_END_ACTION_STOP
            self._send(m)
            self._stop_sent_for = state
        elif state == MissionState.ACTIVE:
            self._stop_sent_for = None

    def _send_plan(self, waypoints, now: float) -> None:
        self.generation = max(self.generation, self.node_generation) + 1
        m = link_pb2.HubMessage()
        plan = m.mission_plan
        plan.generation = self.generation
        plan.issued.unix_nanos = nanos(now)
        plan.expires.unix_nanos = nanos(now + self.config.mission.plan_expiry_s)
        plan.map_epoch = self.site_map.epoch
        plan.on_complete = hub_messages_pb2.PLAN_END_ACTION_STOP
        for w in waypoints:
            out = plan.waypoints.add()
            out.id = w.id
            out.pose.position.x, out.pose.position.y, out.pose.position.z = w.position
            out.pose.orientation.w = 1.0
            out.position_tolerance_m = w.tolerance_m
            out.yaw_free = True
            out.speculative = w.speculative
        self._send(m)
        self.mission.sent_as(self.generation, now)

    def _send(self, m: link_pb2.HubMessage) -> None:
        self._seq += 1
        m.seq = self._seq
        if self.connection is not None:
            self.connection.outgoing.append(m.SerializeToString())
