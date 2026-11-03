"""Talks to a running hub over its app interface (app.proto), for the argus command."""

from __future__ import annotations

import time
from collections.abc import Iterator

import grpc

from argus.protocol.v1 import app_pb2, app_pb2_grpc

from argus import branding

from .designs import Bundle
from .home import Home

TIMEOUT_S = 5.0
DESIGN_TIMEOUT_S = 120.0  # storing a design runs Gazebo's check of it
CANCEL_TIMEOUT_S = 15.0
FOLLOW_POLL_S = 0.5
MAX_MESSAGE_BYTES = 300 * 1024 * 1024


class HubUnavailable(RuntimeError):
    pass


class AppClient:
    def __init__(self, home: Home) -> None:
        self.home = home
        if not home.socket.exists():
            raise HubUnavailable(f"no hub is running at {home.root} (start one with: argus hub start"
                                 + ("" if home.root == Home.at().root else f" --home {home.root}") + ")")
        self._channel = grpc.secure_channel(f"unix:{home.socket}",
                                            grpc.local_channel_credentials(grpc.LocalConnectionType.UDS),
                                            options=[("grpc.max_send_message_length", MAX_MESSAGE_BYTES),
                                                     ("grpc.max_receive_message_length", MAX_MESSAGE_BYTES)])
        self._stub = app_pb2_grpc.HubAppStub(self._channel)

    def close(self) -> None:
        self._channel.close()

    def __enter__(self) -> "AppClient":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def _call(self, method, request, timeout: float = TIMEOUT_S):
        try:
            return method(request, timeout=timeout)
        except grpc.RpcError as e:
            if e.code() == grpc.StatusCode.UNAVAILABLE:
                raise HubUnavailable(f"the hub at {self.home.root} is not answering") from e
            raise HubUnavailable(f"the hub refused: {e.details()}") from e

    def status(self) -> app_pb2.HubStatus:
        return self._call(self._stub.Status, app_pb2.StatusRequest())

    def robots(self) -> app_pb2.RobotsReply:
        return self._call(self._stub.Robots, app_pb2.RobotsRequest())

    def goal(self, node_id: str, x: float, y: float, z: float = 0.0, tolerance_m: float = 0.0) -> app_pb2.Reply:
        return self._call(self._stub.SetGoal, app_pb2.GoalRequest(node_id=node_id, x=x, y=y, z=z,
                                                                  tolerance_m=tolerance_m))

    def cancel(self, node_id: str) -> app_pb2.Reply:
        return self._call(self._stub.Cancel, app_pb2.CancelRequest(node_id=node_id))

    def add_design(self, bundle: Bundle) -> app_pb2.AddDesignReply:
        request = app_pb2.AddDesignRequest(
            name=bundle.name, files=[app_pb2.DesignFile(path=k, data=v) for k, v in sorted(bundle.files.items())],
            collect_findings=[app_pb2.Finding(severity=f.severity, where=f.where, text=f.text) for f in bundle.findings])
        return self._call(self._stub.AddDesign, request, DESIGN_TIMEOUT_S)

    def designs(self) -> app_pb2.DesignsReply:
        return self._call(self._stub.Designs, app_pb2.DesignsRequest())

    def design(self, name: str, version: str = "") -> app_pb2.DesignReply:
        return self._call(self._stub.Design, app_pb2.DesignRequest(name=name, version=version))

    def worlds(self) -> app_pb2.WorldsReply:
        return self._call(self._stub.Worlds, app_pb2.WorldsRequest())

    def validate(self, design: str, version: str = "", worlds: list[str] | None = None,
                 contents: list[str] | None = None, repeats: int = 1) -> app_pb2.StartValidationReply:
        return self._call(self._stub.StartValidation,
                          app_pb2.StartValidationRequest(design=design, version=version, worlds=worlds or [],
                                                         contents=contents or [], repeats=repeats))

    def jobs(self) -> app_pb2.JobsReply:
        return self._call(self._stub.Jobs, app_pb2.JobsRequest())

    def job(self, job_id: str) -> app_pb2.JobReply:
        return self._call(self._stub.Job, app_pb2.JobRequest(id=job_id))

    def cancel_job(self, job_id: str) -> app_pb2.Reply:
        return self._call(self._stub.CancelJob, app_pb2.JobRequest(id=job_id), CANCEL_TIMEOUT_S)

    def keep_job(self, job_id: str, keep: bool = True) -> app_pb2.Reply:
        return self._call(self._stub.KeepJob, app_pb2.KeepJobRequest(id=job_id, keep=keep))

    def delete_job(self, job_id: str) -> app_pb2.Reply:
        return self._call(self._stub.DeleteJob, app_pb2.JobRequest(id=job_id))

    def add_place(self, site: str, name: str, x: float, y: float, z: float = 0.0, tolerance_m: float = 0.0,
                  heading: float | None = None, replace: bool = False) -> app_pb2.Reply:
        p = app_pb2.PlaceInfo(name=name, x=x, y=y, z=z, tolerance_m=tolerance_m, has_heading=heading is not None,
                              heading_rad=heading or 0.0)
        return self._call(self._stub.AddPlace, app_pb2.AddPlaceRequest(site=site, place=p, replace=replace))

    def place_here(self, node_id: str, name: str, tolerance_m: float = 0.0, with_heading: bool = False,
                   replace: bool = False) -> app_pb2.Reply:
        return self._call(self._stub.PlaceHere, app_pb2.PlaceHereRequest(
            node_id=node_id, name=name, tolerance_m=tolerance_m, with_heading=with_heading, replace=replace))

    def places(self, site: str = "") -> app_pb2.PlacesReply:
        return self._call(self._stub.Places, app_pb2.SiteRequest(site=site))

    def remove_place(self, site: str, name: str) -> app_pb2.Reply:
        return self._call(self._stub.RemovePlace, app_pb2.RemoveRequest(site=site, name=name))

    def add_route(self, site: str, name: str, stops: list[tuple[str, float]], mode: str = "once",
                  on_unreachable: str = "skip", cycles: int = 0, replace: bool = False) -> app_pb2.Reply:
        r = app_pb2.RouteInfo(name=name, stops=[app_pb2.RouteStop(place=p, dwell_s=d) for p, d in stops], mode=mode,
                              on_unreachable=on_unreachable, cycles=cycles)
        return self._call(self._stub.AddRoute, app_pb2.AddRouteRequest(site=site, route=r, replace=replace))

    def routes(self, site: str = "") -> app_pb2.RoutesReply:
        return self._call(self._stub.Routes, app_pb2.SiteRequest(site=site))

    def remove_route(self, site: str, name: str) -> app_pb2.Reply:
        return self._call(self._stub.RemoveRoute, app_pb2.RemoveRequest(site=site, name=name))

    def go(self, node_id: str, place: str) -> app_pb2.Reply:
        return self._call(self._stub.Go, app_pb2.GoRequest(node_id=node_id, place=place))

    def patrol(self, node_id: str, route: str) -> app_pb2.Reply:
        return self._call(self._stub.Patrol, app_pb2.PatrolRequest(node_id=node_id, route=route))

    def watch(self) -> Iterator[app_pb2.HubEvent]:
        try:
            yield from self._stub.Watch(app_pb2.WatchRequest())
        except grpc.RpcError as e:
            if e.code() == grpc.StatusCode.CANCELLED:
                return
            raise HubUnavailable(f"the hub at {self.home.root} stopped answering: {e.details()}") from e


def format_status(st: app_pb2.HubStatus, now: float | None = None) -> str:
    now = time.time() if now is None else now
    up = max(0.0, now - st.started_at)
    return (f"Argus {st.version} at {st.home}, up {up / 60:.0f} min\n"
            f"robots connect to {st.robot_address}; {st.robots_connected} of {st.robots_registered} connected\n"
            f"you are {st.caller}")


def format_robots(reply: app_pb2.RobotsReply) -> str:
    lines = []
    for r in reply.robots:
        if not r.connected:
            heard = "never heard" if r.seconds_since_heard < 0 else f"last heard {r.seconds_since_heard:.0f} s ago"
            lines.append(f"{r.node_id}: not connected ({heard}), profile {r.profile_id}, site {r.site_id}")
            continue
        where = "pose unknown" if not r.has_pose else f"at {r.x:.2f} {r.y:.2f} {r.z:.2f}"
        lines.append(f"{r.node_id}: {where}, link {r.link}, mission {r.mission}"
                     + (f" ({r.errand})" if r.errand else "") + ", "
                     f"heard {r.seconds_since_heard:.1f} s ago" + (", FAILSAFE" if r.failsafe else "")
                     + f", evidence {r.evidence_demand_bytes_per_s / 1000:.0f} of "
                       f"{r.evidence_rate_bytes_per_s / 1000:.0f} KB/s"
                     + (f", {r.evidence_dropped} messages dropped" if r.evidence_dropped else ""))
    if not reply.robots:
        lines.append("no robots registered")
    lines.append(f"evidence intake {reply.intake_capacity_bytes_per_s / 1000:.0f} of "
                 f"{reply.intake_configured_bytes_per_s / 1000:.0f} KB/s")
    return "\n".join(lines)


def format_event(e: app_pb2.HubEvent) -> str:
    stamp = time.strftime("%H:%M:%S", time.localtime(e.stamp))
    return " ".join(x for x in (stamp, e.kind, e.node_id, e.detail) if x)


def format_findings(findings) -> str:
    order = {"error": 0, "warning": 1, "note": 2}
    lines = []
    for f in sorted(findings, key=lambda f: order.get(f.severity, 3)):
        lines.append(f"  {branding.word(f.severity)}: {f.where + ': ' if f.where else ''}{f.text}")
    return "\n".join(lines)


def format_design(v: app_pb2.DesignVersion) -> str:
    when = time.strftime("%Y-%m-%d %H:%M", time.localtime(v.created))
    parts = [f"{branding.paint(v.name, 'brand')} {v.version} ({when})",
             branding.word("runnable" if v.runnable else "cannot run"),
             f"profile {v.profile_id}" if v.profile_id else "no profile",
             "own Gazebo model" if v.has_gazebo_model else "no Gazebo model (Argus adds a drive and sensors)"]
    counts = {s: sum(1 for f in v.findings if f.severity == s) for s in ("error", "warning", "note")}
    tally = ", ".join(f"{n} {s}{'s' if n != 1 else ''}" for s, n in counts.items() if n)
    return ", ".join(parts) + (f"; {tally}" if tally else "")


def _when(stamp: float) -> str:
    return time.strftime("%Y-%m-%d %H:%M", time.localtime(stamp)) if stamp else "-"


def _size(n: int) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return f"{n:.0f} {unit}" if unit == "B" else f"{n:.1f} {unit}"
        n /= 1024
    return ""


def run_line(r: app_pb2.ValidationRun, total: int) -> str:
    """One line per run: where, what happened, and whether Argus was at fault."""
    if r.status == "done":
        state = branding.word(r.outcome)
    elif r.outcome:
        state = f"{branding.word(r.status)}, {branding.word(r.outcome)}"
    else:
        state = branding.word(r.status)
    cats = sorted({f.category for f in r.findings})
    blame = f"; findings: {', '.join(branding.word(c) for c in cats)}" if cats else ""
    took = f" in {r.simulated_s:.0f} s" if r.simulated_s else ""
    return f"run {r.index} of {total}: {r.world}, {r.contents}, repeat {r.repeat}: {state}{took}{blame}"


def format_job_line(j: app_pb2.Job) -> str:
    r = j.report
    reached = r.outcomes.get("reached", 0)
    finished = r.runs
    parts = [j.id, f"{branding.paint(j.design, 'brand')} {j.version}", branding.word(j.status),
             f"{reached} of {finished} reached" if finished else "no runs yet",
             f"{len(r.argus_runs)} with Argus findings" if finished else "",
             f"{len(j.runs) - finished} to go" if j.status in ("queued", "running") else "",
             _size(j.size_bytes), "kept" if j.kept else ""]
    return ", ".join(p for p in parts if p)


def format_report(j: app_pb2.Job) -> str:
    """A job's report: totals, then each run with its findings."""
    r = j.report
    lines = [branding.heading(f"job {j.id}") + f": {branding.paint(j.design, 'brand')} {j.version} ({j.kind}), "
             f"{branding.word(j.status)}, started {_when(j.created)}",
             f"{r.runs} of {len(j.runs)} runs finished: " + (", ".join(f"{n} {k}" for k, n in sorted(r.outcomes.items()))
                                                             or "none yet")]
    cats = ", ".join(f"{n} {branding.word(k)}" for k, n in sorted(r.categories.items()) if n)
    lines.append(f"findings: {cats or 'none'}")
    if r.argus_runs:
        lines.append(branding.word("argus") + f" was at fault in (each one is a bug in Argus): {', '.join(r.argus_runs)}")
    for run in j.runs:
        lines.append("")
        lines.append(run_line(run, len(j.runs)))
        if run.detail:
            lines.append(f"  {run.detail}")
        for f in run.findings:
            when = f" [{f.from_s:.1f}-{f.to_s:.1f} s]" if f.has_time else ""
            lines.append(f"  {branding.word(f.category)}{when}: {f.sentence}")
    lines.append("")
    lines.append(f"a run in full: argus runs show {j.id[:15]} RUN   watch one again: "
                 f"argus runs replay {j.id[:15]} RUN")
    return "\n".join(lines)


def find_run(j: app_pb2.Job, which: str) -> app_pb2.ValidationRun | None:
    """A run by its number or its name (or the start of its name)."""
    if which.isdigit():
        return next((r for r in j.runs if r.index == int(which)), None)
    matches = [r for r in j.runs if r.name.startswith(which)]
    return matches[0] if len(matches) == 1 else None


def _progress(j: app_pb2.Job, live: app_pb2.ValidationRun | None, width: int = 20) -> str:
    """The status line while a job runs: the live run, how long it has run, the job's
    progress, and how it has gone so far."""
    done = sum(1 for r in j.runs if r.status not in ("queued", "running"))
    filled = round(width * done / max(1, len(j.runs)))
    bar = "[" + "=" * filled + (">" if filled < width else "") + " " * max(0, width - filled - 1) + "]"
    reached = sum(1 for r in j.runs if r.outcome == "reached")
    argus = sum(1 for r in j.runs if any(f.category == "argus" for f in r.findings))
    if live is not None:
        took = max(0, int(time.time() - live.started)) if live.started else 0
        where = f"run {live.index} of {len(j.runs)}  {live.world}  {live.contents}  {took // 60}:{took % 60:02d}"
    else:
        where = f"{done} of {len(j.runs)} runs" if j.status == "running" else f"queued, {len(j.runs)} runs"
    return f"{where}  {bar}  {reached} reached, {argus} Argus"


def follow(hub: AppClient, job_id: str, view: bool, say=print, poll_s: float = FOLLOW_POLL_S) -> app_pb2.Job | None:
    """Prints a job's runs as they start and finish until the job ends. With `view`, opens a
    Gazebo window on each run as it goes live and closes it when the run ends; a window the
    user closes stays closed until the next run. Returns the finished job (None when it is
    gone). Ctrl-C stops following; the job carries on."""
    from . import viewer

    window = None
    window_for = None  # the run a window was opened for (closed by the user or not)
    seen: dict[int, str] = {}
    status = branding.StatusLine() if say is print else None
    if status is not None:
        say = status.say
    try:
        while True:
            reply = hub.job(job_id)
            if not reply.found:
                say(reply.detail)
                return None
            j = reply.job
            for r in j.runs:
                if seen.get(r.index) != r.status and r.status != "queued":
                    seen[r.index] = r.status
                    say(f"run {r.index} of {len(j.runs)}: {r.world}, {r.contents}, repeat {r.repeat}: started"
                        if r.status == "running" else run_line(r, len(j.runs)))
            live = next((r for r in j.runs if r.status == "running"), None)
            if window is not None and (live is None or live.index != window_for):
                viewer.close(window)
                window = None
            if view and live is not None and live.attach and window_for != live.index:
                window = viewer.attach(dict(live.attach))
                window_for = live.index
            if status is not None:
                status.update(_progress(j, live))
                branding.set_title(f"Argus: run {live.index} of {len(j.runs)}" if live else f"Argus: job {j.status}")
            if j.status not in ("queued", "running"):
                return j
            time.sleep(poll_s)
    finally:
        viewer.close(window)
        if status is not None:
            status.close()
            branding.set_title("Argus")
