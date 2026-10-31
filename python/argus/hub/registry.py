"""Body profiles and robots the hub knows, and the check every session starts with."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from argus import profile as profiles
from argus.protocol.v1 import session_pb2

PROTOCOL_VERSION = 1


@dataclass(frozen=True)
class Profile:
    id: str
    doc: dict[str, Any]
    text: str
    hash: str
    safety_ready: bool


@dataclass(frozen=True)
class Robot:
    node_id: str
    profile_id: str
    site_id: str


class Registry:
    def __init__(self) -> None:
        self._profiles: dict[str, Profile] = {}
        self._robots: dict[str, Robot] = {}

    @classmethod
    def from_dir(cls, path: str | Path) -> "Registry":
        """Loads `profiles/*.yaml` and `robots.yaml` (node_id: {profile, site}) from `path`."""
        path = Path(path)
        registry = cls()
        for file in sorted((path / "profiles").glob("*.yaml")):
            registry.add_profile(profiles.load(file))
        robots = yaml.safe_load((path / "robots.yaml").read_text()) or {}
        for node_id, entry in robots.items():
            registry.add_robot(node_id, entry["profile"], entry["site"])
        return registry

    def add_profile(self, doc: dict[str, Any]) -> Profile:
        found = profiles.problems(doc)
        if found:
            raise ValueError(f"profile {doc.get('id', '?')} is invalid: " + "; ".join(found))
        p = Profile(id=doc["id"], doc=doc, text=profiles.dump(doc), hash=profiles.profile_hash(doc),
                    safety_ready=profiles.derive(doc).safety_ready)
        self._profiles[p.id] = p
        return p

    def add_robot(self, node_id: str, profile_id: str, site_id: str) -> Robot:
        if profile_id not in self._profiles:
            raise KeyError(f"no profile {profile_id}")
        robot = Robot(node_id, profile_id, site_id)
        self._robots[node_id] = robot
        return robot

    def profile(self, profile_id: str) -> Profile:
        return self._profiles[profile_id]

    def robots(self) -> list[Robot]:
        return list(self._robots.values())

    def robot(self, node_id: str) -> Robot | None:
        return self._robots.get(node_id)

    def check_hello(self, hello: session_pb2.Hello) -> tuple[int, str]:
        """(REJECT_REASON_UNSPECIFIED, "") if the node may start a session, else why not."""
        if hello.protocol_version != PROTOCOL_VERSION:
            return (session_pb2.REJECT_REASON_PROTOCOL_VERSION,
                    f"protocol {hello.protocol_version}, hub speaks {PROTOCOL_VERSION}")
        robot = self._robots.get(hello.node_id)
        if robot is None:
            return session_pb2.REJECT_REASON_UNKNOWN_NODE, f"node {hello.node_id} is not registered"
        p = self._profiles[robot.profile_id]
        if hello.profile_id != p.id or hello.profile_hash != p.hash:
            return (session_pb2.REJECT_REASON_PROFILE_MISMATCH,
                    f"node runs {hello.profile_id} {hello.profile_hash[:19]}, registry has {p.id} {p.hash[:19]}")
        if not p.safety_ready:
            return (session_pb2.REJECT_REASON_PROFILE_NOT_SAFETY_READY,
                    f"profile {p.id} has safety fields that are not reviewed")
        return session_pb2.REJECT_REASON_UNSPECIFIED, ""
