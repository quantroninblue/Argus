#!/usr/bin/env python3
"""Drives Argus's reference bodies (tests/bodies) and the TurtleBot designs that ship with
ROS 2 Jazzy through the validation worlds, the way a user's validation job runs, and lists
every run where the diagnosis blames Argus. Each such run is a bug in Argus.

    source /opt/ros/jazzy/setup.bash
    PYTHONPATH=python:build/python python3 tools/validate_bodies.py --jobs DIR \
        [--bodies husky_class,x500_class] [--worlds clutter,pillars] [--contents objects,movers]

Runs one at a time in Gazebo without a window; about two minutes each. Logs and recordings
stay in DIR/<job>/runs/ (`argus runs show` reads them the same way once a hub has them).
"""

from __future__ import annotations

import argparse
import asyncio
import copy
import os
import sys
import tempfile
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
# The checkout's modules, unless PYTHONPATH already names a build to use.
if "argus" not in os.environ.get("PYTHONPATH", ""):
    sys.path[:0] = [str(ROOT / "python"), str(ROOT / "build" / "python")]

from argus.hub import worlds as world_set  # noqa: E402
from argus.hub.designs import DesignStore, collect  # noqa: E402
from argus.hub.validation import Validator  # noqa: E402

TB3 = Path("/opt/ros/jazzy/share/nav2_minimal_tb3_sim")


def _gazebo_variant(path: Path, out_dir: Path) -> Path:
    """Gazebo's multicopter takes a body velocity over ROS 2; a body commanded and estimated
    over MAVLink is run the same way there (as tests/test_gazebo.py does)."""
    doc = yaml.safe_load(path.read_text())
    if doc["control"].get("transport") == "ros2":
        return path
    doc = copy.deepcopy(doc)
    doc["control"].update({"interface": "velocity", "transport": "ros2", "endpoint": "/cmd_vel"})
    for s in doc["sensors"]:
        if s["kind"] in ("odometry", "imu") and s.get("source", {}).get("transport") != "ros2":
            s["source"] = {"transport": "ros2", "endpoint": "/odom" if s["kind"] == "odometry" else "/imu"}
    out = out_dir / path.name
    out.write_text(yaml.safe_dump(doc, sort_keys=False))
    return out


def _csv(text: str | None) -> list[str]:
    return [t.strip() for t in text.split(",") if t.strip()] if text else []


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--jobs", required=True, help="directory for the jobs, their logs and recordings")
    ap.add_argument("--bodies", help="reference bodies by file stem (default: all of tests/bodies)")
    ap.add_argument("--no-designs", action="store_true", help="leave out the TurtleBot3 design")
    ap.add_argument("--worlds", help="comma separated (default: every world)")
    ap.add_argument("--contents", default="objects", help="comma separated (default objects)")
    ap.add_argument("--repeats", type=int, default=1)
    args = ap.parse_args()

    jobs_dir = Path(args.jobs)
    work = Path(tempfile.mkdtemp(prefix="argus-bodies-"))
    designs = DesignStore(jobs_dir / ".designs")
    v = Validator(jobs_dir, designs, publish=lambda kind, job_id, detail: print(f"  {kind}: {detail}", flush=True))
    worlds, contents = _csv(args.worlds) or list(world_set.WORLDS), _csv(args.contents)

    wanted = set(_csv(args.bodies))
    bodies = [p for p in sorted((ROOT / "tests" / "bodies").glob("*.yaml")) if not wanted or p.stem in wanted]
    jobs = [v.start_profile(_gazebo_variant(p, work), worlds, contents, args.repeats) for p in bodies]
    if not args.no_designs and (TB3 / "urdf").exists():
        d, _ = designs.add(collect(TB3 / "urdf" / "turtlebot3_waffle.urdf",
                                   ROOT / "tests" / "designs" / "turtlebot3_waffle_answers.yaml",
                                   TB3 / "urdf" / "gz_waffle.sdf.xacro"))
        jobs.append(v.start(d.name, d.version, worlds, contents, args.repeats))

    async def run_all() -> None:
        while (nxt := v._next()) is not None:
            await v.run_one(*nxt)

    total = sum(len(j.runs) for j in jobs)
    print(f"{len(jobs)} bodies, {total} runs", flush=True)
    asyncio.run(run_all())

    bad = 0
    for j in jobs:
        j = v.jobs.get(j.id)
        r = j.report
        print(f"{j.design}: {r['outcomes'].get('reached', 0)} of {r['runs']} reached; "
              f"Argus findings in {len(r['argus_runs'])}: {', '.join(r['argus_runs']) or '-'}")
        for run in j.runs:
            for f in run.findings:
                if f["category"] == "argus":
                    print(f"  {run.name}: {f['rule']}: {f['sentence']}")
        bad += len(r["argus_runs"])
    print(f"jobs in {jobs_dir}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
