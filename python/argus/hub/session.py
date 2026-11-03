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
from argus.hub.drift import DriftConfig, DriftEstimator
from argus.hub.loops import LoopConfig
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
    # Odometry drift correction (drift.py).
    drift: DriftConfig = field(default_factory=DriftConfig)
    loops: LoopConfig = field(default_factory=LoopConfig)


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
            top = math.inf if ceiling is None else ceiling - room
            # A drone that cannot see above itself declares how high it may fly; routes keep
            # under that too (the base's height above the site floor).
            declared = (profile.doc.get("assumptions") or {}).get("max_flight_height_m")
            if declared:
                top = min(top, floor + float(declared))
            self.mission.z_limits = (floor + room, math.nan if math.isinf(top) else top)
        self.connection = None
        self.session_id = ""
        self.generation = 0
        self.node_generation = 0
        # Pose in the site frame; Argo reports it in odometry (`odom_pose`), and the drift
        # correction maps one to the other. Waypoints go back into odometry when sent.
        self.pose: tuple[float, float, float] | None = None
        self.odom_pose: tuple[float, float, float] | None = None
        self.odom_yaw = 0.0
        self._closures_seen = 0
        self.drift = DriftEstimator(config.drift, sites.resolution_m) if config.drift.enabled else None
        self._drift_at_plan = (0.0, 0.0, 0.0)
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
        # Evidence numbering (see EvidenceOutbox in Argo): the stream of the Argo run, the
        # highest seq taken from it, resends ignored, and the backlog Argo reports.
        self._evidence_stream = 0
        self.evidence_acked = 0
        self.evidence_duplicates = 0
        self.evidence_backlog = 0
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
            beat.heartbeat.evidence_ack_seq = self.evidence_acked
            beat.heartbeat.evidence_budget_bytes_per_s = max(1, int(self.evidence_rate))
            self.evidence_backlog = m.heartbeat.evidence_pending_bytes
            self._send(beat)
        elif body == "telemetry":
            t = m.telemetry
            p = t.pose_map.position
            self.odom_pose = (p.x, p.y, p.z)
            q = t.pose_map.orientation
            self.odom_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.pose = self.drift.point_to_map(self.odom_pose) if self.drift else self.odom_pose
            self.safety = t.safety
            self.link_state = t.link.state
            self.node_generation = max(self.node_generation, t.plan_generation)
            self._set_failsafe(t.safety == node_messages_pb2.SAFETY_STATE_FAILSAFE)
            self.mission.on_telemetry(t.plan_generation, t.active_waypoint_id)
        elif body == "local_evidence":
            ev = m.local_evidence
            if ev.stream != self._evidence_stream:
                self._evidence_stream, self.evidence_acked = ev.stream, 0  # Argo started again
            if ev.seq and ev.seq <= self.evidence_acked:
                self.evidence_duplicates += 1  # a resend of something already taken
                return
            # Taken either way: a robot over its budget loses what was dropped.
            self.evidence_acked = max(self.evidence_acked, ev.seq)
            if not self._evidence_allowed(ev.ByteSize(), now):
                self.evidence_dropped += 1
            else:
                started = time.perf_counter()
                if self.sites.apply(self.robot.site_id, m.local_evidence, self.drift, now, self.odom_pose,
                                    self.robot.node_id, self.odom_yaw):
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
        self._update_drift(now)
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

    def _update_drift(self, now: float) -> None:
        d = self.drift
        if d is None:
            return
        closures = self.sites.closures(self.robot.site_id)
        if closures != self._closures_seen:
            # The site's frame moved under this robot: its map, correction and plan all changed.
            self._closures_seen = closures
            if self.odom_pose is not None:
                self.pose = d.point_to_map(self.odom_pose)
            self.mission.resend()
        if not d.update(now, self.sites.surfaces(self.robot.site_id), self.site_map):
            return
        self.pose = d.point_to_map(self.odom_pose)
        # Waypoints already sent were put into odometry with the old correction.
        x0, y0, yaw0 = self._drift_at_plan
        moved = math.hypot(d.x - x0, d.y - y0) + abs(d.yaw - yaw0) * self.config.mission.max_leg_m
        if moved >= self.config.drift.resend_after_m:
            self.mission.resend()

    def _send_plan(self, waypoints, now: float) -> None:
        self.generation = max(self.generation, self.node_generation) + 1
        m = link_pb2.HubMessage()
        plan = m.mission_plan
        plan.generation = self.generation
        plan.issued.unix_nanos = nanos(now)
        plan.expires.unix_nanos = nanos(now + self.config.mission.plan_expiry_s)
        plan.map_epoch = self.site_map.epoch
        plan.on_complete = hub_messages_pb2.PLAN_END_ACTION_STOP
        if self.drift is not None:
            self._drift_at_plan = (self.drift.x, self.drift.y, self.drift.yaw)
        for w in waypoints:
            out = plan.waypoints.add()
            out.id = w.id
            position = self.drift.point_to_odom(w.position) if self.drift else w.position
            out.pose.position.x, out.pose.position.y, out.pose.position.z = position
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
