"""Runs one robot through the whole loop: simulated body, Argo with its session, the link, and
the hub planning its route. Success and safety are still judged from simulator truth."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from argus import node as argo
from argus.hub import Hub, HubConfig, MissionState, Registry
from argus.sim.channel import DOWN, UP, LinkFaults, SimChannel
from argus.sim.episode import Episode, SimOptions, Simulation
from argus.sim.world import World

NODE_ID = "argo-sim"
SITE_ID = "site-sim"


@dataclass
class HubEpisode:
    body: Episode
    goal: tuple[float, float, float]
    # From simulator truth: the body ended within the goal tolerance.
    goal_reached: bool = False
    mission_state: str = "idle"
    mission_cause: str = ""
    reached_at_s: float | None = None
    plans_sent: int = 0
    # Argo's link state over time, and everything the hub heard from Argo.
    link_states: list[tuple[float, str]] = field(default_factory=list)
    hub_events: list[tuple[float, str, str, str, str]] = field(default_factory=list)
    connections: int = 0
    evidence_bytes: int = 0
    bytes_up: int = 0
    bytes_down: int = 0
    places: list[tuple[tuple[float, float, float], str]] = field(default_factory=list)

    def failsafe_events(self) -> list[dict]:
        return [e for e in self.body.events if e["kind"].startswith("failsafe")]


class HubLoop:
    """The pieces of a hub episode, steppable, so tests can change things mid-run."""

    def __init__(self, doc: dict, world: World, start, goal, tolerance_m: float = 0.4,
                 faults: LinkFaults | None = None, options: SimOptions | None = None,
                 hub_config: HubConfig | None = None, node_config=None, session_config=None,
                 hub_period_s: float = 0.05) -> None:
        self.registry = Registry()
        profile = self.registry.add_profile(doc)
        self.registry.add_robot(NODE_ID, profile.id, SITE_ID)
        self.hub = Hub(self.registry, config=hub_config)
        self.goal = tuple(float(v) for v in goal)
        self.tolerance_m = tolerance_m
        self.hub.set_goal(NODE_ID, self.goal, tolerance_m, 0.0)

        config = node_config or argo.NodeConfig()
        config.map.record_evidence = True
        self.sim = Simulation(doc, world, start, options, config)
        scfg = session_config or argo.SessionConfig()
        scfg.node_id = NODE_ID
        scfg.profile_hash = profile.hash
        self.session = argo.Session(self.sim.node, scfg)
        self.faults = faults or LinkFaults()
        self.channel = SimChannel(self.faults)
        self.connection = None
        self.hub_period_s = hub_period_s
        self._next_connect = 0.0
        self._next_hub = 0.0
        self.result = HubEpisode(body=self.sim.result, goal=self.goal)
        self.planar = self.sim.caps.motion.space == "planar"

    @property
    def hub_session(self):
        return self.hub.session(NODE_ID)

    def _transport(self, t: float) -> None:
        if self.channel.open and self.faults.cut(t):
            self.channel.close()
            self.connection.close()
            self.session.on_disconnected(t)
        if not self.channel.open and t >= self._next_connect:
            self._next_connect = t + self.faults.reconnect_period_s
            if not self.faults.cut(t):
                self.channel.open = True
                self.connection = self.hub.connect()
                self.session.on_connected(t)

    def _tick(self, t: float) -> dict:
        self._transport(t)
        for b in self.channel.receive(DOWN, t):
            self.session.receive(b, t)
        out = self.session.step(t)
        for b in self.session.take_outgoing():
            self.channel.send(UP, b, t)
        state = self.session.link_state
        if not self.result.link_states or self.result.link_states[-1][1] != state:
            self.result.link_states.append((t, state))
        return out

    def step(self) -> None:
        t = self.sim.t
        self.sim.step(self._tick)
        if t >= self._next_hub - 1e-9:
            self._next_hub += self.hub_period_s
            if self.connection is not None and not self.connection.closed:
                for b in self.channel.receive(UP, t):
                    self.connection.receive(b, t)
            self.hub.step(t)
            if self.connection is not None and not self.connection.closed:
                for b in self.connection.take_outgoing():
                    self.channel.send(DOWN, b, t)
        if self.result.reached_at_s is None and self.mission_state == MissionState.REACHED:
            self.result.reached_at_s = t

    @property
    def mission_state(self) -> MissionState:
        return self.hub.mission_state(NODE_ID)

    def distance_to_goal(self) -> float:
        p = self.sim.plant.pos
        dz = 0.0 if self.planar else p[2] - self.goal[2]
        return math.hypot(p[0] - self.goal[0], p[1] - self.goal[1], dz)

    def finish(self) -> HubEpisode:
        r = self.result
        s = self.hub_session
        r.goal_reached = self.distance_to_goal() <= self.tolerance_m + 0.05
        r.mission_state = s.mission.state.value
        r.mission_cause = s.mission.cause
        r.plans_sent = s.mission.plans_sent
        r.hub_events = list(s.events)
        r.connections = s.connections
        r.evidence_bytes = self.session.evidence_bytes
        r.bytes_up, r.bytes_down = self.channel.bytes
        r.places = [(p.position, p.status.value) for p in s.mission.memory.places]
        return r


def run_hub_episode(doc: dict, world: World, start, goal, duration_s: float, tolerance_m: float = 0.4,
                    faults: LinkFaults | None = None, options: SimOptions | None = None,
                    hub_config: HubConfig | None = None, node_config=None,
                    settle_s: float = 1.0, min_duration_s: float = 0.0) -> HubEpisode:
    """Sends the body from `start` (x, y, z, yaw) to `goal` through the hub. Ends when the
    mission is over, Argo is out of any failsafe and the body has been at rest for
    `settle_s` (but not before `min_duration_s`), or at `duration_s`."""
    loop = HubLoop(doc, world, start, goal, tolerance_m, faults, options, hub_config, node_config)
    sim = loop.sim
    rest_since = None
    while sim.t <= duration_s:
        loop.step()
        done = (loop.mission_state in (MissionState.REACHED, MissionState.FAILED, MissionState.CANCELLED)
                and not sim.node.failsafe_active)
        if done and sim.at_rest:
            rest_since = sim.t if rest_since is None else rest_since
            if sim.t - rest_since >= settle_s and sim.t >= min_duration_s:
                break
        else:
            rest_since = None
    return loop.finish()
