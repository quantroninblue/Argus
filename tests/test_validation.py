"""Validation jobs (argus.hub.validation, argus.hub.worlds) with a stand-in for the Gazebo run
(tests/validation_child.py), so no Gazebo is needed."""

import asyncio
import json
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path

import pytest

from argus.hub import worlds
from argus.hub.validation import Job, Run, Validator, preflight
from argus.sim.world import World

TESTS = Path(__file__).parent


@dataclass
class _Version:
    name: str = "bot"
    version: str = "abc123"
    profile_id: str = "bot-1"
    runnable: bool = True
    path: Path = Path(".")
    findings: list = field(default_factory=list)


class _Designs:
    def __init__(self, v):
        self.v = v

    def get(self, name, version=None):
        return self.v if name == self.v.name and (not version or self.v.version.startswith(version)) else None

    def profile(self, v):
        return {"motion": {"space": "planar", "limits": {"x": {"velocity_mps": [0.0, 0.5]}}}}


def _validator(tmp_path, monkeypatch, mode="reach", runnable=True, events=None, keep_jobs=10):
    monkeypatch.setenv("FAKE_RUN", mode)
    monkeypatch.setenv("PYTHONPATH", f"{TESTS}{os.pathsep}{os.environ.get('PYTHONPATH', '')}")
    publish = (lambda k, j, d: events.append((k, d))) if events is not None else (lambda *a: None)
    return Validator(tmp_path / "jobs", _Designs(_Version(path=tmp_path, runnable=runnable)), publish,
                     child_module="validation_child", preflight=lambda: [], keep_jobs=keep_jobs)


async def _drain(v: Validator):
    while (nxt := v._next()) is not None:
        await v.run_one(*nxt)


def test_every_world_has_clear_starts_goals_and_walkers():
    for name, spec in worlds.WORLDS.items():
        for kind in (worlds.PLANAR, worlds.AERIAL):
            w = spec.world(kind, "movers")
            assert w.movers, name
            still = World(boxes=w.boxes, name=w.name)
            for m in w.movers:
                assert worlds.mover_clear_of_walls(still, m), (name, kind, m.path)
            for p in (spec.start_pose(kind), spec.goal_point(kind)):
                x, y, z = p[:3]
                b = w.boxes
                near = ((b[:, 0] < x + 0.6) & (b[:, 3] > x - 0.6) & (b[:, 1] < y + 0.6) & (b[:, 4] > y - 0.6)
                        & (b[:, 2] < z + 0.5) & (b[:, 5] > z))
                assert not near.any(), (name, kind, p)
            assert not spec.world(kind, "objects").movers


def test_runs_are_planned_per_world_contents_and_repeat_with_one_empty_room():
    runs = worlds.plan_runs(["wall_with_gap", "clutter"], ["objects", "movers", "empty"], 2)
    assert len(runs) == 2 * (1 + 2 * 2)
    assert [r for r in runs if r.contents == "empty"] == [worlds.RunPlan("open_room", "empty", 1),
                                                          worlds.RunPlan("open_room", "empty", 2)]
    with pytest.raises(ValueError, match="no world"):
        worlds.plan_runs(["moon"], ["objects"], 1)
    with pytest.raises(ValueError, match="contents"):
        worlds.plan_runs(["clutter"], ["lava"], 1)


def test_a_job_runs_each_world_in_turn_and_keeps_the_findings(tmp_path, monkeypatch):
    events = []
    v = _validator(tmp_path, monkeypatch, events=events)
    job = v.start("bot", "", ["open_room", "wall_with_gap"], ["objects"], 1)
    assert [r.status for r in v.jobs.get(job.id).runs] == ["queued", "queued"]
    asyncio.run(_drain(v))

    done = v.jobs.get(job.id)
    assert done.status == "done"
    assert [r.status for r in done.runs] == ["done", "done"]
    assert [r.outcome for r in done.runs] == ["reached", "reached"]
    assert all((tmp_path / "jobs" / job.id / "runs" / r.name / "run.json").is_file() for r in done.runs)
    assert done.runs[0].sim_time_s == 12.5
    assert done.report["runs"] == 2 and done.report["outcomes"] == {"reached": 2}
    assert done.report["argus_runs"] == []
    kinds = [k for k, _ in events]
    assert kinds == ["job_queued", "run_started", "run_finished", "run_started", "run_finished", "job_finished"]
    # Kept on disk, so a new hub sees the same report.
    assert Job.from_dict(json.loads((tmp_path / "jobs" / job.id / "job.json").read_text())).report == done.report


def test_a_collision_is_an_argus_finding_in_the_report(tmp_path, monkeypatch):
    v = _validator(tmp_path, monkeypatch, mode="collide")
    job = v.start("bot", "abc", ["open_room"], ["objects"], 1)
    asyncio.run(_drain(v))
    done = v.jobs.get(job.id)
    assert done.runs[0].outcome == "collided"
    assert "argus" in done.runs[0].blames
    assert done.report["argus_runs"] == [done.runs[0].name]


def test_a_run_that_breaks_is_reported_and_the_job_goes_on(tmp_path, monkeypatch):
    v = _validator(tmp_path, monkeypatch, mode="crash")
    job = v.start("bot", "", ["open_room", "clutter"], ["objects"], 1)
    asyncio.run(_drain(v))
    done = v.jobs.get(job.id)
    assert [r.status for r in done.runs] == ["error", "error"]
    assert done.status == "done"
    f = done.runs[0].findings[-1]
    assert f["rule"] == "run_error" and f["category"] == "inconclusive"
    out = tmp_path / "jobs" / job.id / "runs" / f"{done.runs[0].name}.out"
    assert "gz sim exited early" in out.read_text()


def test_cancelling_stops_the_running_run_and_drops_the_rest(tmp_path, monkeypatch):
    v = _validator(tmp_path, monkeypatch, mode="hang")
    job = v.start("bot", "", ["open_room", "clutter", "pillars"], ["objects"], 1)

    async def go():
        task = asyncio.ensure_future(_drain(v))
        started = tmp_path / "jobs" / job.id / "runs" / (job.runs[0].name + ".started")
        for _ in range(200):
            if started.exists():
                break
            await asyncio.sleep(0.05)
        assert started.exists()
        assert v.cancel(job.id[:12]).endswith("cancelled")
        await asyncio.wait_for(task, 10.0)

    asyncio.run(go())
    done = v.jobs.get(job.id)
    assert done.status == "cancelled"
    assert [r.status for r in done.runs] == ["cancelled"] * 3


def test_a_hub_restart_marks_the_run_it_was_on_and_carries_on(tmp_path, monkeypatch):
    v = _validator(tmp_path, monkeypatch)
    job = v.start("bot", "", ["open_room", "clutter"], ["objects"], 1)
    job.status, job.runs[0].status = "running", "running"
    v.jobs.save(job)

    v2 = _validator(tmp_path, monkeypatch)
    again = v2.jobs.get(job.id)
    assert [r.status for r in again.runs] == ["interrupted", "queued"]
    asyncio.run(_drain(v2))
    assert [r.status for r in v2.jobs.get(job.id).runs] == ["interrupted", "done"]


def test_a_job_is_refused_up_front_with_the_reason(tmp_path, monkeypatch):
    v = _validator(tmp_path, monkeypatch, runnable=False)
    with pytest.raises(ValueError, match="cannot run until its errors are fixed"):
        v.start("bot", "", [], [], 1)
    with pytest.raises(ValueError, match="no design 'other'"):
        v.start("other", "", [], [], 1)
    v.preflight = lambda: ["ROS 2 is not sourced"]
    v.designs.v.runnable = True
    with pytest.raises(ValueError, match="ROS 2 is not sourced"):
        v.start("bot", "", [], [], 1)


def test_preflight_names_what_is_missing():
    problems = preflight(env={"PATH": "/nonexistent"})
    assert any("ROS 2 is not sourced" in p for p in problems)
    assert any("'gz'" in p for p in problems)


def test_a_window_can_attach_to_the_live_run_and_replay_knows_its_meshes(tmp_path, monkeypatch):
    v = _validator(tmp_path, monkeypatch, mode="hang")
    job = v.start("bot", "", ["open_room"], ["objects"], 1)
    assert v.live(v.jobs.get(job.id)) is None  # nothing live before the run starts

    async def go():
        task = asyncio.ensure_future(_drain(v))
        for _ in range(200):
            live = v.live(v.jobs.get(job.id))
            if live:
                break
            await asyncio.sleep(0.05)
        run, env = live
        assert run.index == 1 and env["GZ_PARTITION"] == "fake-run"
        v.cancel(job.id)
        await asyncio.wait_for(task, 10.0)

    asyncio.run(go())
    done = v.jobs.get(job.id)
    assert v.live(done) is None
    assert done.runs[0].resource_path == "/designs/bot/packages"
    assert not v.jobs.live_file(done, done.runs[0]).exists()


def test_old_jobs_make_room_unless_kept(tmp_path, monkeypatch):
    v = _validator(tmp_path, monkeypatch, keep_jobs=2)
    ids = []
    for i in range(4):
        job = v.start("bot", "", ["open_room"], ["objects"], 1)
        asyncio.run(_drain(v))
        ids.append(job.id)
        if i == 0:
            v.keep(job.id)
    v.prune()
    left = [j.id for j in v.jobs.all()]
    assert left == [ids[0], ids[2], ids[3]]  # the kept one, and the newest two
    assert v.delete(ids[2]).endswith("deleted")
    assert [j.id for j in v.jobs.all()] == [ids[0], ids[3]]


def test_the_replay_timeline_lists_what_explains_a_run(tmp_path):
    from argus.hub.viewer import format_timeline, timeline

    (tmp_path / "argo.log").write_text(
        "1.00 event plan_accepted\n"
        "3.20 flags near_people yielding\n"
        "9.50 event blocked cause=controller_no_progress\n"
        "12.00 event waypoint_reached waypoint=w1\n")
    findings = [{"category": "argus", "sentence": "the planner stalled", "time_s": [9.5, 14.0]},
                {"category": "design", "sentence": "untimed", "time_s": None}]
    moments = timeline(tmp_path, findings)
    assert [(m.t_s, m.source) for m in moments] == [(3.2, "argo"), (9.5, "argo"), (9.5, "finding")]
    text = format_timeline(moments)
    assert "safety flags: near_people yielding" in text and "cause=controller_no_progress" in text
    assert "argus: the planner stalled (to 14.0 s)" in text


def test_every_run_starts_at_the_origin_with_the_world_moved_around_it():
    # Odometry starts at zero where the robot starts, and the hub's site frame is the odometry
    # frame until drift correction exists; a start elsewhere sent robots to the wrong place.
    spec = worlds.WORLDS["dead_end"]
    as_built = spec.build(2.0)
    for kind in (worlds.PLANAR, worlds.AERIAL):
        assert spec.start_pose(kind)[:2] == (0.0, 0.0)
    w = spec.world(worlds.PLANAR, "movers")
    assert w.boxes[0][0] == as_built.boxes[0][0] - spec.start[0]
    assert spec.goal_point(worlds.PLANAR)[:2] == (spec.goal[0] - spec.start[0], spec.goal[1] - spec.start[1])
    first = spec.movers()[0]
    assert w.movers[0].path[0] == (first.path[0][0] - spec.start[0], first.path[0][1] - spec.start[1])
