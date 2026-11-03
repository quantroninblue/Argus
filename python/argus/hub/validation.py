"""Validation jobs (hub plan H4): a stored design driven through Gazebo worlds, one run at a time.

A job is a design version, a list of worlds, what they hold (argus.hub.worlds) and how many
times to repeat each. The hub keeps jobs in `jobs/<id>/job.json` with each run's log beside
it in `jobs/<id>/runs/<run>/`, and runs them in the order they were started, one run at a
time.

Gazebo always runs without a window here. A user who wants to watch opens one from the argus
command (argus.hub.viewer), which attaches to the run that is live, before it starts or at any
point during it; closing the window never touches the run. Every run is also recorded by
Gazebo (`replay/` in its log) for playback afterwards. The newest KEEP_JOBS jobs are kept, and
any the user marked to keep; older ones are deleted when a new job starts.

Each run is a child process (argus.gazebo.validate_run) with its own private hub: the real
Argo binary, mutual TLS and the mission path, but a site map of its own, so simulated walls
never reach a real site. After each run the hub diagnoses its log (argus.hub.diagnosis) and
keeps the findings in job.json, so a report needs no Gazebo and survives a hub restart.
"""

from __future__ import annotations

import asyncio
import json
import os
import re
import shutil
import signal
import sys
import tempfile
import time
import uuid
from collections.abc import Callable
from dataclasses import asdict, dataclass, field
from pathlib import Path

from . import worlds as world_set
from .designs import DesignStore
from .diagnosis import ARGUS, INCONCLUSIVE, Diagnosis, Finding, diagnose, job_report

ROOT = Path(__file__).resolve().parents[3]
CHILD_MODULE = "argus.gazebo.validate_run"
CANCEL_GRACE_S = 30.0  # a cancelled run's processes get this long to stop and write its log
_JOB_ID = re.compile(r"^[A-Za-z0-9_-]+$")
KEEP_JOBS = 10

# Run states. A job is "queued", "running", "done" or "cancelled".
QUEUED, RUNNING, DONE, ERROR, CANCELLED, INTERRUPTED = "queued", "running", "done", "error", "cancelled", "interrupted"


@dataclass
class Run:
    index: int
    world: str
    contents: str
    repeat: int
    status: str = QUEUED
    outcome: str = ""  # reached, failed or collided, once done
    detail: str = ""
    started: float = 0.0
    finished: float = 0.0
    sim_time_s: float = 0.0
    findings: list[dict] = field(default_factory=list)  # diagnosis.Finding as dicts
    resource_path: str = ""  # GZ_SIM_RESOURCE_PATH the run used, for playback

    @property
    def name(self) -> str:
        return f"{self.index:02d}-{self.world}-{self.contents}-{self.repeat}"

    @property
    def blames(self) -> list[str]:
        return sorted({f["category"] for f in self.findings})


@dataclass
class Job:
    id: str
    design: str
    version: str
    profile_id: str
    kind: str
    created: float
    status: str = QUEUED
    kept: bool = False  # never deleted to make room
    # Set for a job on a body profile instead of a design (Argus's own tests of reference
    # bodies, hub plan H6): the profile runs on the model Argus builds from it.
    profile_path: str = ""
    runs: list[Run] = field(default_factory=list)
    report: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "Job":
        runs = [Run(**r) for r in d.pop("runs", [])]
        return cls(runs=runs, **d)


class JobStore:
    def __init__(self, root: Path) -> None:
        self.root = Path(root)

    def path(self, job_id: str) -> Path:
        if not _JOB_ID.match(job_id):
            raise ValueError(f"not a job id: {job_id!r}")
        return self.root / job_id

    def run_dir(self, job: Job, run: Run) -> Path:
        return self.path(job.id) / "runs" / run.name

    def live_file(self, job: Job, run: Run) -> Path:
        return self.path(job.id) / "runs" / f"{run.name}.live.json"

    def delete(self, job: Job) -> None:
        shutil.rmtree(self.path(job.id), ignore_errors=True)

    def size(self, job: Job) -> int:
        return sum(p.stat().st_size for p in self.path(job.id).rglob("*") if p.is_file())

    def save(self, job: Job) -> None:
        d = self.path(job.id)
        d.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(prefix=".job-", dir=d)
        with os.fdopen(fd, "w") as f:
            json.dump(job.to_dict(), f, indent=1)
        os.replace(tmp, d / "job.json")

    def all(self) -> list[Job]:
        if not self.root.is_dir():
            return []
        jobs = [Job.from_dict(json.loads((p / "job.json").read_text()))
                for p in self.root.iterdir() if (p / "job.json").is_file()]
        return sorted(jobs, key=lambda j: j.created)

    def get(self, job_id: str) -> Job | None:
        """A job by its id, or by a prefix of it that only one job has."""
        matches = [j for j in self.all() if j.id.startswith(job_id)] if job_id else []
        exact = [j for j in matches if j.id == job_id]
        return exact[0] if exact else matches[0] if len(matches) == 1 else None


def argo_binary() -> Path:
    return Path(os.environ.get("ARGO_BIN", ROOT / "build" / "ros" / "argo_ros" / "argo"))


def preflight(env: dict | None = None) -> list[str]:
    """What stops a validation run on this machine, in words a user can act on. Checked
    before a job is accepted, so a job never queues runs that cannot start."""
    env = os.environ if env is None else env
    problems = []
    if not env.get("ROS_DISTRO") or shutil.which("ros2", path=env.get("PATH")) is None:
        problems.append("ROS 2 is not sourced in the hub's environment: stop the hub, run "
                        "'source /opt/ros/jazzy/setup.bash', then start it again")
    if shutil.which("gz", path=env.get("PATH")) is None:
        problems.append("Gazebo ('gz') is not installed or not on the hub's PATH (ros-jazzy-ros-gz)")
    argo = argo_binary()
    if not argo.is_file():
        problems.append(f"the argo binary is missing at {argo}: build with tools/check.sh while ROS 2 is "
                        "sourced, or set ARGO_BIN")
    return problems


def _child_env() -> dict:
    """The hub's environment, with the argus package findable from any directory."""
    env = dict(os.environ)
    import argus

    paths = [str(Path(p).resolve().parent) for p in argus.__path__]
    env["PYTHONPATH"] = os.pathsep.join(paths + [p for p in env.get("PYTHONPATH", "").split(os.pathsep)
                                                 if p and p not in paths])
    return env


def _finding_dicts(d: Diagnosis) -> list[dict]:
    return d.to_dict()["findings"]


def report(job: Job) -> dict:
    """The job's report from the findings kept in job.json: diagnosis.job_report over the
    finished runs, plus a line per world."""
    finished = [r for r in job.runs if r.status in (DONE, ERROR)]
    diagnoses = [Diagnosis(r.name, r.outcome or "failed", [Finding(**f) for f in r.findings]) for r in finished]
    out = job_report(diagnoses)
    per_world: dict[str, dict] = {}
    for r in finished:
        w = per_world.setdefault(f"{r.world} ({r.contents})", {"runs": 0, "reached": 0, "blames": []})
        w["runs"] += 1
        w["reached"] += r.outcome == "reached"
        w["blames"] = sorted(set(w["blames"]) | set(r.blames))
    out["worlds"] = per_world
    out["unfinished"] = sum(1 for r in job.runs if r.status not in (DONE, ERROR))
    return out


class Validator:
    """Accepts, runs and cancels validation jobs for the hub. `publish(kind, job_id, detail)`
    reports progress (job_queued, run_started, run_finished, job_finished)."""

    def __init__(self, jobs_dir: Path, designs: DesignStore,
                 publish: Callable[[str, str, str], None] = lambda kind, job_id, detail: None,
                 python: str = sys.executable, child_module: str = CHILD_MODULE,
                 preflight: Callable[[], list[str]] = preflight, keep_jobs: int = KEEP_JOBS,
                 clock=time.time) -> None:
        self.jobs = JobStore(jobs_dir)
        self.designs = designs
        self.publish = publish
        self.python = python
        self.child_module = child_module
        self.preflight = preflight
        self.keep_jobs = keep_jobs
        self.clock = clock
        self._wake = asyncio.Event()
        self._child: asyncio.subprocess.Process | None = None
        self._current: tuple[str, int] | None = None
        self._recover()

    def _recover(self) -> None:
        """A hub that stopped mid-job: the run it was on is marked interrupted, the rest stay
        queued and run when this hub gets to them."""
        for job in self.jobs.all():
            if job.status not in (QUEUED, RUNNING):
                continue
            for r in job.runs:
                if r.status == RUNNING:
                    r.status, r.detail, r.finished = INTERRUPTED, "the hub stopped during this run", self.clock()
            job.status = QUEUED if any(r.status == QUEUED for r in job.runs) else DONE
            job.report = report(job)
            self.jobs.save(job)

    def start(self, design: str, version: str, worlds: list[str], contents: list[str], repeats: int) -> Job:
        """A new job, queued. ValueError with the reason when it cannot run."""
        v = self.designs.get(design, version or None)
        if v is None:
            raise ValueError(f"no design {design + ('@' + version if version else '')!r}")
        if not v.runnable:
            raise ValueError(f"{v.name} {v.version} cannot run until its errors are fixed "
                             f"(argus design show {v.name}@{v.version})")
        doc = self.designs.profile(v)
        problems = self.preflight()
        if problems:
            raise ValueError("; ".join(problems))
        plans = world_set.plan_runs(worlds or list(world_set.WORLDS), contents or ["objects"], repeats)
        job = Job(id=f"{time.strftime('%Y%m%d-%H%M%S', time.localtime(self.clock()))}-{uuid.uuid4().hex[:4]}",
                  design=v.name, version=v.version, profile_id=v.profile_id, kind=world_set.body_kind(doc),
                  created=self.clock(),
                  runs=[Run(i + 1, p.world, p.contents, p.repeat) for i, p in enumerate(plans)])
        job.report = report(job)
        self.jobs.save(job)
        self.prune()
        self.publish("job_queued", job.id, f"{job.design} {job.version}, {len(job.runs)} runs")
        self._wake.set()
        return job

    def prune(self) -> list[str]:
        """Deletes finished jobs beyond the newest `keep_jobs`, except kept ones."""
        finished = [j for j in self.jobs.all() if j.status not in (QUEUED, RUNNING) and not j.kept]
        gone = finished[:max(0, len(finished) - self.keep_jobs)]
        for j in gone:
            self.jobs.delete(j)
        return [j.id for j in gone]

    def keep(self, job_id: str, keep: bool = True) -> str:
        job = self._job(job_id)
        job.kept = keep
        self.jobs.save(job)
        return f"{job.id} {'kept until you delete it' if keep else 'may be deleted to make room'}"

    def delete(self, job_id: str) -> str:
        job = self._job(job_id)
        if job.status in (QUEUED, RUNNING):
            raise ValueError(f"{job.id} is {job.status}; cancel it first")
        self.jobs.delete(job)
        return f"{job.id} deleted"

    def _job(self, job_id: str) -> Job:
        job = self.jobs.get(job_id)
        if job is None:
            raise ValueError(f"no job {job_id!r}")
        return job

    def live(self, job: Job) -> tuple[Run, dict] | None:
        """The job's run that is live now and what a Gazebo window needs to attach to it, once
        its Gazebo is up."""
        run = next((r for r in job.runs if r.status == RUNNING), None)
        if run is None:
            return None
        f = self.jobs.live_file(job, run)
        try:
            return run, json.loads(f.read_text())
        except (OSError, ValueError):
            return None

    def start_profile(self, profile_path: str | Path, worlds: list[str], contents: list[str], repeats: int) -> Job:
        """A job on a body profile rather than a design, run on the model Argus builds from the
        profile. For Argus's own tests of reference bodies; users validate designs."""
        from argus import profile as profiles

        doc = profiles.load(profile_path)
        problems = self.preflight()
        if problems:
            raise ValueError("; ".join(problems))
        plans = world_set.plan_runs(worlds or list(world_set.WORLDS), contents or ["objects"], repeats)
        job = Job(id=f"{time.strftime('%Y%m%d-%H%M%S', time.localtime(self.clock()))}-{uuid.uuid4().hex[:4]}",
                  design=doc["id"], version=profiles.profile_hash(doc)[:12], profile_id=doc["id"],
                  kind=world_set.body_kind(doc), created=self.clock(), profile_path=str(Path(profile_path).resolve()),
                  runs=[Run(i + 1, p.world, p.contents, p.repeat) for i, p in enumerate(plans)])
        job.report = report(job)
        self.jobs.save(job)
        self.publish("job_queued", job.id, f"{job.design} (profile), {len(job.runs)} runs")
        self._wake.set()
        return job

    def cancel(self, job_id: str) -> str:
        job = self._job(job_id)
        if job.status not in (QUEUED, RUNNING):
            return f"{job.id} already {job.status}"
        for r in job.runs:
            if r.status == QUEUED:
                r.status = CANCELLED
        job.status = CANCELLED
        job.report = report(job)
        self.jobs.save(job)
        if self._current and self._current[0] == job.id and self._child and self._child.returncode is None:
            self._child.send_signal(signal.SIGTERM)  # the run stops its processes and keeps its log
        self.publish("job_finished", job.id, "cancelled")
        return f"{job.id} cancelled"

    def _next(self) -> tuple[Job, Run] | None:
        for job in self.jobs.all():
            if job.status in (QUEUED, RUNNING):
                run = next((r for r in job.runs if r.status == QUEUED), None)
                if run is not None:
                    return job, run
        return None

    async def run_forever(self) -> None:
        while True:
            nxt = self._next()
            if nxt is None:
                self._wake.clear()
                await self._wake.wait()
                continue
            await self.run_one(*nxt)

    async def run_one(self, job: Job, run: Run) -> None:
        job.status, run.status, run.started = RUNNING, RUNNING, self.clock()
        self.jobs.save(job)
        self.publish("run_started", job.id, f"run {run.index} of {len(job.runs)}: {run.world}, {run.contents}")
        design = None if job.profile_path else self.designs.get(job.design, job.version)
        log_dir = self.jobs.run_dir(job, run)
        work = Path(tempfile.mkdtemp(prefix=f"argus-{job.id}-"))
        spec = {"profile": job.profile_path or (str(design.path / "profile.yaml") if design else ""),
                "design_dir": str(design.path) if design else None,
                "world": run.world, "contents": run.contents, "work": str(work / "run"),
                "log_dir": str(log_dir), "live": str(self.jobs.live_file(job, run)), "result": str(work / "result.json")}
        (work / "spec.json").write_text(json.dumps(spec))
        log_dir.parent.mkdir(parents=True, exist_ok=True)
        child_log = log_dir.parent / f"{run.name}.out"
        result: dict = {"ok": False, "error": "the run did not report back"}
        self._current = (job.id, run.index)
        try:
            if design is None and not job.profile_path:
                raise RuntimeError(f"design {job.design} {job.version} is no longer stored")
            with child_log.open("wb") as out:
                self._child = await asyncio.create_subprocess_exec(
                    self.python, "-m", self.child_module, str(work / "spec.json"), stdout=out,
                    stderr=asyncio.subprocess.STDOUT, env=_child_env(), start_new_session=True)
                try:
                    await self._child.wait()
                except asyncio.CancelledError:  # the hub is stopping
                    await self._stop_child()
                    raise
            if (work / "result.json").is_file():
                result = json.loads((work / "result.json").read_text())
        except asyncio.CancelledError:
            run.status, run.detail = INTERRUPTED, "the hub stopped during this run"
            self.jobs.save(job)
            raise
        except Exception as e:
            result = {"ok": False, "error": f"{type(e).__name__}: {e}"}
        finally:
            self._child, self._current = None, None
            shutil.rmtree(work, ignore_errors=True)
        await asyncio.to_thread(self._finish, job, run, result, log_dir, child_log)

    async def _stop_child(self) -> None:
        if self._child is None or self._child.returncode is not None:
            return
        self._child.send_signal(signal.SIGTERM)
        try:
            await asyncio.wait_for(self._child.wait(), CANCEL_GRACE_S)
        except asyncio.TimeoutError:
            self._child.kill()
            await self._child.wait()

    def _finish(self, job: Job, run: Run, result: dict, log_dir: Path, child_log: Path) -> None:
        """Diagnoses the run, stores the findings and moves the job on."""
        latest = self.jobs.get(job.id) or job  # it may have been cancelled meanwhile
        run = next(r for r in latest.runs if r.index == run.index)
        run.finished = self.clock()
        live = self.jobs.live_file(latest, run)
        try:
            run.resource_path = json.loads(live.read_text()).get("GZ_SIM_RESOURCE_PATH", "")
        except (OSError, ValueError):
            pass
        live.unlink(missing_ok=True)
        findings: list[dict] = []
        if (log_dir / "run.json").is_file():
            d = diagnose(log_dir)
            run.outcome = d.outcome
            findings = _finding_dicts(d)
            run.sim_time_s = float(json.loads((log_dir / "run.json").read_text()).get("outcome", {}).get("simulated_s") or 0.0)
        if result.get("ok"):
            run.status = DONE
        else:
            run.status = CANCELLED if latest.status == CANCELLED else ERROR
            run.detail = result.get("error", "")
            run.outcome = run.outcome or "failed"
            if run.status == ERROR:
                findings.append(asdict(Finding(INCONCLUSIVE, "run_error",
                                               f"The run stopped early ({run.detail}); see {child_log.name} and the "
                                               "processes' logs in the run log", [child_log.name])))
        run.findings = findings
        if latest.status != CANCELLED:
            latest.status = QUEUED if any(r.status == QUEUED for r in latest.runs) else DONE
        latest.report = report(latest)
        self.jobs.save(latest)
        argus = " ARGUS" if ARGUS in run.blames else ""
        self.publish("run_finished", latest.id, f"run {run.index}: {run.world}, {run.contents}: "
                                                f"{run.outcome or run.status}{argus}")
        if latest.status == DONE:
            r = latest.report
            self.publish("job_finished", latest.id,
                         f"{r['outcomes'].get('reached', 0)} of {r['runs']} reached, "
                         f"{len(r['argus_runs'])} with Argus findings")
