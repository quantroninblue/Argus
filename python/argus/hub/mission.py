"""Mission manager: the one owner of where a robot goes next.

It turns a goal into a short list of waypoints, keeps that list while it is still a good way
to the goal, and replaces it when it is not. Each step's outcome comes back from Argo as an
event; the manager decides what it means for the mission and for the exploration memory.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from argus.hub.exploration import MEMORY_CAUSES, ExplorationMemory

GOAL_ID = "goal"


@dataclass
class MissionConfig:
    replan_period_s: float = 1.0
    plan_expiry_s: float = 30.0
    # Plans are resent with a fresh expiry once this little time is left.
    refresh_before_s: float = 10.0
    max_waypoints: int = 4
    waypoint_tolerance_m: float = 0.4
    # The current plan is kept unless going through its waypoints costs more than this
    # times the best route plus the margin. Stops the robot flipping between branches.
    switch_ratio: float = 1.3
    switch_margin_m: float = 1.0
    # A remaining leg whose route is longer than this times its straight length plus the
    # margin has been cut by something seen since; the plan is replaced.
    leg_detour_ratio: float = 1.25
    leg_detour_m: float = 0.5
    # Resend when Argo has not taken up a plan this long after it was sent.
    ack_timeout_s: float = 2.0
    # After a sensor, localization or timing failure, wait this long before replanning.
    fault_hold_s: float = 2.0
    # Failed steps in a row, with no waypoint reached in between, before giving up.
    max_failures: int = 6
    # The goal counts as searched when no route reaches it and the robot has stood at the
    # end of the best route for this many replans.
    unreachable_replans: int = 3
    at_end_m: float = 0.5
    margin_m: float = 3.0
    unknown_penalty: float = 2.0
    max_leg_m: float = 3.0


class MissionState(str, Enum):
    IDLE = "idle"
    ACTIVE = "active"
    REACHED = "reached"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass(frozen=True)
class PlannedWaypoint:
    id: str
    position: tuple[float, float, float]
    tolerance_m: float
    speculative: bool


class MissionManager:
    def __init__(self, planner, config: MissionConfig | None = None) -> None:
        self.planner = planner
        self.planar = planner.planar
        self.config = config or MissionConfig()
        self.memory = ExplorationMemory(self.planar)
        # Height band aerial routes keep the body in, from the site's floor and ceiling.
        self.z_limits = (math.nan, math.nan)
        self.state = MissionState.IDLE
        self.goal: tuple[float, float, float] | None = None
        self.goal_tolerance_m = 0.3
        self.cause = ""
        self.detail = ""
        self.sent: list[PlannedWaypoint] = []
        self.sent_generation = 0
        self.sent_at = -math.inf
        self.expires = -math.inf
        self.acked_generation = 0
        self.plans_sent = 0
        self.last_route: dict | None = None
        self.paused = False
        self._next = 0  # index in `sent` of the next waypoint not yet reached
        self._force = False
        self._last_replan = -math.inf
        self._hold_until = -math.inf
        self._failures = 0
        self._unreachable = 0
        self._counter = 0

    # Operator side.

    def set_goal(self, position, tolerance_m: float, now: float) -> None:
        self.goal = tuple(float(v) for v in position)
        self.goal_tolerance_m = tolerance_m
        self.state = MissionState.ACTIVE
        self.cause = self.detail = ""
        self.memory.new_goal()
        self._failures = self._unreachable = 0
        self._force = True

    def cancel(self) -> None:
        if self.state == MissionState.ACTIVE:
            self.state = MissionState.CANCELLED
            self.sent = []

    # Robot side.

    def set_paused(self, paused: bool) -> None:
        """Paused while Argo runs a failsafe. Argo drops its plan when the failsafe clears, so a
        fresh one is needed then."""
        if self.paused and not paused:
            self.resend()
        self.paused = paused

    def resend(self) -> None:
        """Argo no longer holds the last plan; send a new one at the next update."""
        self.sent = []
        self._force = True

    def on_telemetry(self, plan_generation: int, active_waypoint_id: str) -> None:
        self.acked_generation = plan_generation
        if plan_generation == self.sent_generation:
            for i, w in enumerate(self.sent):
                if w.id == active_waypoint_id:
                    self._next = max(self._next, i)

    def on_event(self, kind: str, waypoint_id: str, cause: str, generation: int, pose, now: float) -> None:
        if self.state != MissionState.ACTIVE:
            return
        if kind == "plan_rejected":
            self._hold_until = now + self.config.fault_hold_s
            self._force = True
            return
        if generation < self.sent_generation:
            return  # about a plan that has since been replaced
        index = next((i for i, w in enumerate(self.sent) if w.id == waypoint_id), None)
        if kind == "step_reached":
            if index is None:
                return
            self._next = max(self._next, index + 1)
            self._failures = 0
            w = self.sent[index]
            if w.speculative:
                self.memory.mark_traversed(w.position, now)
            if w.id == GOAL_ID:
                # Argo judged it on a fresher pose than the last telemetry the hub has.
                self.state = MissionState.REACHED
            elif self._next >= len(self.sent):
                self._force = True
        elif kind == "step_failed":
            self._failures += 1
            if self._failures >= self.config.max_failures:
                self._fail(cause, f"{self._failures} failed steps in a row")
                return
            if cause in MEMORY_CAUSES and index is not None:
                place = self._failure_place(pose, self.sent[index])
                if place is not None:
                    self.memory.record_failure(place, cause, now)
            elif cause != "controller_no_progress":
                self._hold_until = now + self.config.fault_hold_s
            self._force = True

    def sent_as(self, generation: int, now: float) -> None:
        """The waypoints from the last update() went out as plan `generation`."""
        self.sent_generation = generation
        self.sent_at = now
        self.expires = now + self.config.plan_expiry_s
        self.plans_sent += 1

    def update(self, now: float, pose, site_map) -> list[PlannedWaypoint] | None:
        """Waypoints to send now, or None when the plan Argo holds is still the right one."""
        c = self.config
        if self.state != MissionState.ACTIVE:
            return None
        if self._distance(pose, self.goal) <= self.goal_tolerance_m:
            self.state = MissionState.REACHED
            return None
        if self.paused or now < self._hold_until:
            return None
        if self.sent and self.acked_generation < self.sent_generation and now - self.sent_at > c.ack_timeout_s:
            self._force = True
        if not self._force and now - self._last_replan < c.replan_period_s:
            return None
        self._last_replan = now

        zones = self.memory.blocked_zones(now)
        route = self._route(site_map, pose, self.goal, zones)
        self.last_route = route
        if not route["reached"]:
            end = route["path"][-1] if len(route["path"]) else pose
            if self._distance(pose, end) <= c.at_end_m:
                self._unreachable += 1
                if self._unreachable >= c.unreachable_replans:
                    self._fail("searched_no_goal", "no route to the goal from anywhere reachable")
                return None
        self._unreachable = 0

        remaining = self.sent[self._next:]
        send = self._force or not remaining or now >= self.expires - c.refresh_before_s
        if not send and len(remaining) == 1 and remaining[0].id != GOAL_ID and len(route["waypoints"]) > 1:
            send = True  # extend before Argo slows down for its last waypoint
        if not send:
            via = self._via_cost(site_map, pose, remaining, zones)
            send = via is None or via > c.switch_ratio * route["length_m"] + c.switch_margin_m
        if not send:
            return None
        waypoints = self._waypoints(route)
        if not waypoints:
            return None
        self._force = False
        self.sent = waypoints
        self._next = 0
        for w in waypoints:
            if w.speculative:
                self.memory.mark_active(w.position, now)
        return waypoints

    # Internals.

    def _distance(self, a, b) -> float:
        dz = 0.0 if self.planar else a[2] - b[2]
        return math.hypot(a[0] - b[0], a[1] - b[1], dz)

    def _route(self, site_map, start, goal, zones) -> dict:
        c = self.config
        return self.planner.plan(site_map, tuple(start), tuple(goal), margin_m=c.margin_m,
                                 unknown_penalty=c.unknown_penalty, max_leg_m=c.max_leg_m, blocked=zones,
                                 z_min_m=self.z_limits[0], z_max_m=self.z_limits[1])

    def _via_cost(self, site_map, pose, remaining: list[PlannedWaypoint], zones) -> float | None:
        """Length of the way to the goal through the waypoints Argo holds, or None if one of
        them can no longer be reached, or a leg that was sent as a short straight hop now
        needs a detour Argo would have to find on its own."""
        c = self.config
        points = [tuple(pose)] + [w.position for w in remaining]
        total = 0.0
        for a, b in zip(points, points[1:]):
            leg = self._route(site_map, a, b, zones)
            if not leg["reached"] or leg["length_m"] > c.leg_detour_ratio * self._distance(a, b) + c.leg_detour_m:
                return None
            total += leg["length_m"]
        if remaining[-1].id != GOAL_ID:
            total += self._route(site_map, remaining[-1].position, self.goal, zones)["length_m"]
        return total

    def _waypoints(self, route: dict) -> list[PlannedWaypoint]:
        out = []
        everything = route["waypoints"]
        # Where a leg changes between seen and unseen space the planner can leave two points a
        # few centimetres apart; keep the later one so Argo does not slow down for nothing.
        kept = [w for i, w in enumerate(everything)
                if i + 1 == len(everything) or self._distance(w["position"], everything[i + 1]["position"]) >= 0.5]
        found = kept[: self.config.max_waypoints]
        for i, w in enumerate(found):
            last = route["reached"] and i == len(kept) - 1
            if last:
                out.append(PlannedWaypoint(GOAL_ID, self.goal, self.goal_tolerance_m, w["speculative"]))
            else:
                self._counter += 1
                out.append(PlannedWaypoint(f"w{self._counter}", tuple(w["position"]),
                                           self.config.waypoint_tolerance_m, w["speculative"]))
        return out

    def _failure_place(self, pose, w: PlannedWaypoint):
        """Where to remember a closed branch. Normally the waypoint Argo could not reach; when
        that is the goal itself, a point on the way to it, so the goal stays reachable from
        other sides."""
        if w.id != GOAL_ID and self._distance(w.position, self.goal) >= 1.0:
            return w.position
        d = self._distance(pose, w.position)
        if d < 1.2:
            return None
        f = min(1.5, d / 2.0) / d
        return tuple(p + (q - p) * f for p, q in zip(pose, w.position))

    def _fail(self, cause: str, detail: str) -> None:
        self.state = MissionState.FAILED
        self.cause = cause
        self.detail = detail
        self.sent = []
