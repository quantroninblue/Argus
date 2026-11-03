"""Command line: argus <command>.

  certs ca DIR                     a new certificate authority in DIR
  certs hub DIR --host H [...]     the hub's certificate, valid for each host name or address
  certs robot DIR NODE_ID          a robot's certificate; NODE_ID must match Argo's node_id
  setup template                   an answers file to fill in for import
  setup import URDF -a ANSWERS -o PROFILE
                                   a body profile from a URDF and the answers, with its report
  setup report PROFILE             the capability report of a profile
  setup check PROFILE [--motion]   live checks on the running robot (needs ROS 2)
  setup calibrate PROFILE -o OUT   stepped motion tests; measured limits written back
  hub init [--host H ...]          a hub home: directories, its CA and certificate
  hub start [--listen HOST:PORT] [--intake RATE]
                                   run the hub until Ctrl-C
  status                           the running hub
  robots                           every registered robot, its link and mission
  goal ROBOT X Y [Z] [--tolerance M]
                                   send a robot to a point in its site frame
  cancel ROBOT                     stop a robot's mission
  watch                            hub events as they happen, until Ctrl-C
  design add URDF [-a ANSWERS] [--gazebo-model FILE] [--package NAME=DIR] [--name NAME]
                                   store a robot design on the hub and check it
  design list                      every design and its versions
  design show NAME[@VERSION]       a design's findings and capability report
  place add SITE NAME X Y [Z] [--tolerance M] [--heading RAD] [--replace]
  place here ROBOT NAME            a place where the robot is now, in its site
  place list [SITE], place remove SITE NAME
  route add SITE NAME PLACE[:DWELL_S] ... [--mode once|loop|back_and_forth] [--cycles N]
          [--on-unreachable skip|stop] [--replace]
  route list [SITE], route remove SITE NAME
  go ROBOT PLACE                   send a robot to a named place in its site
  patrol ROBOT ROUTE               run a robot along a route
  worlds                           the worlds a validation job can use
  validate NAME[@VERSION] [--worlds W,...] [--contents objects,movers,empty] [--repeats N]
          [--view] [--detach]      drive a design through Gazebo worlds and diagnose each run
  jobs list                        every validation job and how it went
  jobs watch JOB [--view]          follow a job's runs; --view opens Gazebo on each live run
  jobs report JOB                  each run's outcome and findings
  jobs cancel|keep|unkeep|delete JOB
  runs show JOB RUN                one run: outcome, findings, timeline, log files
  runs replay JOB RUN              play a finished run back in Gazebo
  runs explain JOB RUN [--ask Q]   why a run went as it did, from the reasoning model
  jobs explain JOB [--ask Q]       what a job's runs show together
  jobs compare A B                 two jobs run by run over the worlds they share
  design review NAME[@VERSION]     suggestions for a design, tied to what was measured
  models connect KIND [URL] [--name N] [--key-prompt]
                                   add a model source: openai-compatible or anthropic
  models list                      connections, their models and the slots
  models use SLOT CONNECTION/MODEL fill the reasoning or vision slot
  models test SLOT [--image FILE]  ask the model in a slot something small
  models disconnect NAME           remove a connection and forget its key
  say ROBOT "TEXT" [--yes]         send a robot somewhere in words, through the site's places

Every hub command takes --home DIR (default ~/.local/share/argus, or ARGUS_HOME).
"""

from __future__ import annotations

import argparse
import sys
import webbrowser
from pathlib import Path

from argus import certs


def _certs(args) -> int:
    d = Path(args.dir)
    name = {"ca": "ca", "hub": "hub", "robot": getattr(args, "node_id", "")}[args.kind]
    if (d / f"{name}.key").exists():
        print(f"{d / name}.key already exists; remove it first to replace it", file=sys.stderr)
        return 1
    if args.kind != "ca" and not (d / "ca.key").exists():
        print(f"no ca.key in {d}; run 'certs ca {d}' first", file=sys.stderr)
        return 1
    d.mkdir(parents=True, exist_ok=True)
    if args.kind == "ca":
        certs.make_ca(d)
    elif args.kind == "hub":
        certs.make_hub(d, args.host)
    else:
        certs.make_node(d, args.node_id)
    for leftover in (f"{name}.csr", f"{name}.ext"):
        (d / leftover).unlink(missing_ok=True)
    print(f"wrote {d / name}.pem and {d / name}.key")
    return 0


def _setup(args) -> int:
    from argus import profile as profiles
    from argus.setup.importer import ANSWERS_HINT, build_profile, load_answers
    from argus.setup.report import capability_report
    from argus.setup.urdf import UrdfError

    if args.step == "template":
        print(ANSWERS_HINT, end="")
        return 0
    if args.step == "check":
        import rclpy

        from argus.setup.live import live_check

        doc = profiles.load(args.profile)
        if args.motion and not args.yes:
            print("The timeout test drives the robot forward at 0.1 m/s for one second. Keep an e-stop "
                  "in reach and the space ahead clear, or lift the drive wheels (--wheels-up).")
            if args.direct:
                print("--direct: Argo's safety supervisor will not check these motions.")
            if input("Type 'go' to start: ").strip() != "go":
                return 1
        rclpy.init()
        try:
            result = live_check(doc, args.duration, args.motion, args.wheels_up, args.sim_time, args.direct)
        finally:
            rclpy.shutdown()
        print(_colour_results(result.text()))
        return 0 if result.ok else 1
    if args.step == "calibrate":
        import rclpy

        from argus.setup.calibrate import calibrate

        doc = profiles.load(args.profile)
        if not args.yes:
            top = doc["motion"]["limits"]["x"]["velocity_mps"][1]
            print(f"Calibration drives the robot forward up to {top:g} m/s and turns it. It checks the "
                  "scan for room before each run; keep an e-stop in reach and people clear.")
            if args.direct:
                print("--direct: Argo's safety supervisor will not check these motions.")
            if input("Type 'go' to start: ").strip() != "go":
                return 1
        rclpy.init()
        try:
            cal = calibrate(doc, args.sim_time, args.direct)
        finally:
            rclpy.shutdown()
        print(cal.text())
        problems = profiles.problems(cal.profile)
        if problems:
            print("\n".join(problems), file=sys.stderr)
            return 1
        Path(args.out).write_text(profiles.dump(cal.profile))
        print(f"wrote {args.out}")
        return 0 if not cal.problems else 1
    if args.step == "report":
        doc = profiles.load(args.profile)
        problems = profiles.problems(doc)
        if problems:
            print("\n".join(problems), file=sys.stderr)
            return 1
        print(capability_report(doc))
        return 0
    packages = _packages(args.package)
    if packages is None:
        return 2
    try:
        doc, problems, notes = build_profile(args.urdf, load_answers(args.answers), args.base_frame, packages)
    except (UrdfError, ValueError, OSError) as e:
        print(e, file=sys.stderr)
        return 1
    for n in notes:
        print(f"note: {n}", file=sys.stderr)
    if problems:
        print(f"{len(problems)} problem(s) before a profile can be written:", file=sys.stderr)
        for p in problems:
            print(f"  {p}", file=sys.stderr)
        if any(p.endswith("required") or "required property" in p for p in problems):
            print("Start an answers file with: argus setup template > answers.yaml", file=sys.stderr)
        return 1
    Path(args.out).write_text(profiles.dump(doc))
    print(f"wrote {args.out}\n")
    print(capability_report(doc))
    return 0


def _packages(items) -> dict[str, Path] | None:
    packages = {}
    for item in items or []:
        name, sep, directory = item.partition("=")
        if not sep:
            print(f"--package {item}: expected NAME=DIRECTORY", file=sys.stderr)
            return None
        packages[name] = Path(directory)
    return packages


def _design(hub, args) -> int:
    from argus.hub.client import format_design, format_findings

    if args.action == "review":
        name, _, version = args.design.partition("@")
        return _model_text(hub.review_design(name, version, args.ask or ""))
    if args.action == "add":
        from argus.hub.designs import collect
        from argus.setup.urdf import UrdfError

        packages = _packages(args.package)
        if packages is None:
            return 2
        try:
            bundle = collect(args.urdf, args.answers, args.gazebo_model, packages, args.name)
        except (UrdfError, OSError) as e:
            print(e, file=sys.stderr)
            return 1
        reply = hub.add_design(bundle)
        if not reply.ok:
            print(reply.detail, file=sys.stderr)
            return 1
        print(reply.detail)
        if reply.design.findings:
            print(format_findings(reply.design.findings))
        return 0 if reply.design.runnable else 1
    if args.action == "list":
        reply = hub.designs()
        if not reply.designs:
            print("no designs yet; add one with: argus design add URDF -a ANSWERS")
        for d in reply.designs:
            for v in d.versions:
                print(format_design(v))
        return 0
    name, _, version = args.design.partition("@")
    reply = hub.design(name, version)
    if not reply.found:
        print(reply.detail, file=sys.stderr)
        return 1
    print(format_design(reply.design))
    if reply.design.findings:
        print(format_findings(reply.design.findings))
    if reply.capability_report:
        print()
        print(reply.capability_report)
    return 0


def _csv_list(text: str | None) -> list[str]:
    return [t.strip() for t in text.split(",") if t.strip()] if text else []


def _follow(hub, job_id: str, view: bool) -> int:
    from argus import branding
    from argus.hub import viewer
    from argus.hub.client import follow

    if view:
        problems = viewer.problems()
        if problems:
            print("cannot open Gazebo here: " + "; ".join(problems), file=sys.stderr)
            return 1
        print("Gazebo opens on each run as it starts; closing the window does not stop the run.")
    try:
        job = follow(hub, job_id, view)
    except KeyboardInterrupt:
        print(f"\nstopped following; the job carries on (argus jobs watch {job_id[:15]})")
        return 0
    if job is None:
        return 1
    r = job.report
    reached = r.outcomes.get("reached", 0)
    argus_runs = len(r.argus_runs)
    print(f"job {job.id} {branding.word(job.status)}: {reached} of {r.runs} reached; "
          + (branding.word("argus") + f" findings in {argus_runs}" if argus_runs else branding.word("ok") + ", no Argus findings"))
    print(f"report: argus jobs report {job.id[:15]}")
    return 0


def _model_text(reply) -> int:
    """Prints what a model said, or the rule-based text when no model was used."""
    if not reply.ok:
        print(reply.detail, file=sys.stderr)
        return 1
    if reply.detail:
        print(reply.detail, file=sys.stderr)
    print(reply.text)
    if reply.model:
        print(f"\n(from {reply.model}: advice, check it against the findings)", file=sys.stderr)
    return 0


def _read_key() -> str:
    import getpass

    if sys.stdin.isatty():
        return getpass.getpass("API key: ").strip()
    return sys.stdin.readline().strip()


def _sign_in(hub, name: str) -> int:
    started = hub.begin_sign_in(name)
    if not started.ok:
        print(started.detail, file=sys.stderr)
        return 1
    print(f"Sign in to ChatGPT in your browser. If it does not open, open this on this machine:\n{started.url}",
          flush=True)
    try:
        webbrowser.open(started.url)
    except webbrowser.Error:
        pass
    reply = hub.finish_sign_in(name)
    print(reply.detail, file=sys.stdout if reply.ok else sys.stderr)
    return 0 if reply.ok else 1


def _models(hub, args) -> int:
    if args.action == "connect" and args.kind == "chatgpt" and not (args.url or args.key_prompt):
        return _sign_in(hub, args.name or args.kind)
    if args.action == "connect":
        reply = hub.connect_model(args.name or args.kind, args.kind, args.url or "", _read_key() if args.key_prompt else "")
    elif args.action == "use":
        connection, _, model = args.model.partition("/")
        if not model:
            print("name the model as CONNECTION/MODEL", file=sys.stderr)
            return 1
        reply = hub.use_model(args.slot, connection, model)
    elif args.action == "disconnect":
        reply = hub.disconnect_model(args.name)
    elif args.action == "test":
        image = Path(args.image).read_bytes() if args.image else b""
        media = {".jpg": "image/jpeg", ".jpeg": "image/jpeg", ".webp": "image/webp"}.get(
            Path(args.image).suffix.lower(), "image/png") if args.image else ""
        return _model_text(hub.test_model(args.slot, image, media))
    else:
        listed = hub.models()
        if not listed.connections:
            print("no models connected; the hub runs on its rule-based tier (argus models connect)")
        for c in listed.connections:
            again = "sign-in needed: connect again" if c.kind == "chatgpt" else "key needed: connect again"
            state = again if c.needs_key else ("remote" if c.remote else "this machine")
            print(f"{c.name}  {c.kind}  {c.base_url or 'the installed program'}  ({state})")
            for m in c.models:
                print(f"  {m}")
            if c.kind == "claude-code":
                print("  default, or any model name it accepts")
        for s in listed.slots:
            print(f"{s.slot}: {s.connection + '/' + s.model if s.connection else 'empty'}")
        return 0
    print(reply.detail, file=sys.stdout if reply.ok else sys.stderr)
    return 0 if reply.ok else 1


def _say(hub, args) -> int:
    reply = hub.interpret_goal(args.robot, args.text)
    if not reply.ok:
        print(reply.detail, file=sys.stderr)
        return 1
    if reply.kind == "unclear":
        print(f"not sent: {reply.name}", file=sys.stderr)
        return 1
    what = f"go to {reply.name}" if reply.kind == "place" else f"run route {reply.name}"
    print(f"{args.robot}: {what} (read by {reply.model})")
    if not args.yes:
        if not sys.stdin.isatty():
            print("not sent: confirm with --yes when not at a terminal", file=sys.stderr)
            return 1
        if input("send it? [y/N] ").strip().lower() not in ("y", "yes"):
            print("not sent")
            return 0
    sent = hub.go(args.robot, reply.name) if reply.kind == "place" else hub.patrol(args.robot, reply.name)
    print(sent.detail, file=sys.stdout if sent.ok else sys.stderr)
    return 0 if sent.ok else 1


def _jobs(hub, args) -> int:
    from argus.hub.client import format_job_line, format_report

    if args.action == "explain":
        return _model_text(hub.explain(args.job, 0, args.ask or ""))
    if args.action == "compare":
        return _model_text(hub.compare_jobs(args.a, args.b))
    if args.action == "list":
        reply = hub.jobs()
        if not reply.jobs:
            print("no validation jobs yet; start one with: argus validate DESIGN")
        for j in reply.jobs:
            print(format_job_line(j))
        return 0
    if args.action in ("watch", "view"):
        return _follow(hub, args.job, args.action == "view" or args.view)
    if args.action == "report":
        reply = hub.job(args.job)
        if not reply.found:
            print(reply.detail, file=sys.stderr)
            return 1
        print(format_report(reply.job))
        return 0
    if args.action == "cancel":
        reply = hub.cancel_job(args.job)
    elif args.action in ("keep", "unkeep"):
        reply = hub.keep_job(args.job, args.action == "keep")
    else:
        reply = hub.delete_job(args.job)
    print(reply.detail, file=sys.stdout if reply.ok else sys.stderr)
    return 0 if reply.ok else 1


def _runs(hub, args) -> int:
    from argus import branding
    from argus.hub import viewer
    from argus.hub.client import find_run, run_line

    reply = hub.job(args.job)
    if not reply.found:
        print(reply.detail, file=sys.stderr)
        return 1
    run = find_run(reply.job, args.run)
    if run is None:
        print(f"no run {args.run!r} in job {reply.job.id}; runs are numbered 1 to {len(reply.job.runs)}",
              file=sys.stderr)
        return 1
    if args.action == "explain":
        return _model_text(hub.explain(reply.job.id, run.index, args.ask or ""))
    log_dir = Path(run.log_dir)
    findings = [{"category": f.category, "sentence": f.sentence, "time_s": [f.from_s, f.to_s] if f.has_time else None}
                for f in run.findings]
    if args.action == "show":
        print(run_line(run, len(reply.job.runs)))
        if run.detail:
            print(f"  {run.detail}")
        for f in run.findings:
            when = f" [{f.from_s:.1f}-{f.to_s:.1f} s]" if f.has_time else ""
            print(f"  {f.category}{when}: {f.sentence}" + (f" ({', '.join(f.signals)})" if f.signals else ""))
        print()
        print(viewer.format_timeline(viewer.timeline(log_dir, findings)))
        print()
        print(f"log: {branding.link(str(log_dir)) if log_dir.is_dir() else str(log_dir) + ' (not written)'}")
        if log_dir.is_dir():
            print("  " + "  ".join(sorted(p.name for p in log_dir.iterdir())))
        return 0
    if run.status in ("queued", "running"):
        print(f"run {run.index} is {run.status}; watch it live with: argus jobs view {reply.job.id[:15]}",
              file=sys.stderr)
        return 1
    problems = viewer.problems()
    if problems:
        print("cannot open Gazebo here: " + "; ".join(problems), file=sys.stderr)
        return 1
    try:
        window = viewer.replay(log_dir, run.resource_path)
    except FileNotFoundError as e:
        print(e, file=sys.stderr)
        return 1
    print(run_line(run, len(reply.job.runs)))
    print("Gazebo plays the run back; use its play, pause and seek controls. Times below are simulated seconds.")
    print(viewer.format_timeline(viewer.timeline(log_dir, findings)))
    try:
        window.wait()
    except KeyboardInterrupt:
        pass
    finally:
        viewer.close(window)
    return 0


def _stops(items: list[str]) -> list[tuple[str, float]] | None:
    out = []
    for item in items:
        name, sep, dwell = item.partition(":")
        try:
            out.append((name, float(dwell) if sep else 0.0))
        except ValueError:
            print(f"{item}: expected PLACE or PLACE:DWELL_SECONDS", file=sys.stderr)
            return None
    return out


def _places(hub, args) -> int:
    from argus import branding

    if args.command == "go":
        reply = hub.go(args.robot, args.place)
    elif args.command == "patrol":
        reply = hub.patrol(args.robot, args.route)
    elif args.command == "place" and args.action == "list":
        for site in hub.places(args.site or "").sites:
            print(branding.heading(site.site))
            for pl in site.places:
                heading = f", heading {pl.heading_rad:.2f} rad" if pl.has_heading else ""
                print(f"  {pl.name:<16} {pl.x:7.2f} {pl.y:7.2f} {pl.z:6.2f}  within {pl.tolerance_m:g} m{heading}")
            if not site.places:
                print("  no places")
        return 0
    elif args.command == "route" and args.action == "list":
        for site in hub.routes(args.site or "").sites:
            print(branding.heading(site.site))
            for r in site.routes:
                stops = ", ".join(f"{st.place}" + (f" ({st.dwell_s:g} s)" if st.dwell_s else "") for st in r.stops)
                times = f" x{r.cycles}" if r.cycles else ""
                print(f"  {r.name:<16} {r.mode}{times}: {stops}; unreachable: {r.on_unreachable}")
            if not site.routes:
                print("  no routes")
        return 0
    elif args.command == "place" and args.action == "add":
        reply = hub.add_place(args.site, args.name, args.x, args.y, args.z, args.tolerance, args.heading, args.replace)
    elif args.command == "place" and args.action == "here":
        reply = hub.place_here(args.robot, args.name, args.tolerance, replace=args.replace)
    elif args.command == "place":
        reply = hub.remove_place(args.site, args.name)
    elif args.action == "add":
        stops = _stops(args.stops)
        if stops is None:
            return 2
        reply = hub.add_route(args.site, args.name, stops, args.mode, args.on_unreachable, args.cycles or 0,
                              args.replace)
    else:
        reply = hub.remove_route(args.site, args.name)
    print(reply.detail, file=sys.stdout if reply.ok else sys.stderr)
    return 0 if reply.ok else 1


def _client(args) -> int:
    from argus.hub.client import AppClient, HubUnavailable, format_event, format_robots, format_status
    from argus.hub.home import Home

    try:
        with AppClient(Home.at(args.home)) as hub:
            if args.command == "design":
                return _design(hub, args)
            if args.command == "jobs":
                return _jobs(hub, args)
            if args.command in ("place", "route", "go", "patrol"):
                return _places(hub, args)
            if args.command == "runs":
                return _runs(hub, args)
            if args.command == "models":
                return _models(hub, args)
            if args.command == "say":
                return _say(hub, args)
            if args.command == "worlds":
                reply = hub.worlds()
                for w in reply.worlds:
                    print(f"{w.name:<14} {w.description}")
                print(f"\ncontents: {', '.join(reply.contents)} (objects: as shipped; movers: with people "
                      "walking through; empty: the bare room, one run whatever the worlds)")
                return 0
            if args.command == "validate":
                name, _, version = args.design.partition("@")
                reply = hub.validate(name, version, _csv_list(args.worlds), _csv_list(args.contents), args.repeats)
                if not reply.ok:
                    print(reply.detail, file=sys.stderr)
                    return 1
                print(reply.detail)
                if args.detach:
                    print(f"follow it with: argus jobs watch {reply.job.id[:15]} [--view]")
                    return 0
                return _follow(hub, reply.job.id, args.view)
            if args.command == "status":
                print(format_status(hub.status()))
            elif args.command == "robots":
                print(format_robots(hub.robots()))
            elif args.command in ("goal", "cancel"):
                if args.command == "goal":
                    reply = hub.goal(args.robot, args.x, args.y, args.z, args.tolerance)
                else:
                    reply = hub.cancel(args.robot)
                print(reply.detail, file=sys.stdout if reply.ok else sys.stderr)
                return 0 if reply.ok else 1
            else:
                try:
                    for event in hub.watch():
                        print(format_event(event), flush=True)
                except KeyboardInterrupt:
                    pass
    except HubUnavailable as e:
        print(e, file=sys.stderr)
        return 1
    return 0


class _Parser(argparse.ArgumentParser):
    """The top-level help opens with the name; subcommands' help stays plain."""

    def format_help(self) -> str:
        from argus import branding

        return branding.banner() + "\n\n" + super().format_help()


def _colour_results(text: str) -> str:
    """Colours the leading pass/FAIL word of each result line."""
    from argus import branding

    out = []
    for line in text.splitlines():
        word, sep, rest = line.partition("  ")
        out.append(branding.status(word) + sep + rest if word in ("pass", "FAIL") else line)
    return "\n".join(out)


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else list(argv)
    if "--no-color" in argv:
        from argus import branding

        branding.disable()
        argv.remove("--no-color")
    p = _Parser(prog="argus", epilog="--no-color anywhere turns colour off (so does NO_COLOR).")
    sub = p.add_subparsers(dest="command", required=True, parser_class=argparse.ArgumentParser)
    c = sub.add_parser("certs", help="certificates for the hub and robots (mutual TLS)")
    kinds = c.add_subparsers(dest="kind", required=True)
    kinds.add_parser("ca", help="a new certificate authority").add_argument("dir")
    hub = kinds.add_parser("hub", help="the hub's certificate")
    hub.add_argument("dir")
    hub.add_argument("--host", action="append", required=True, help="name or IP address the hub is reached at")
    robot = kinds.add_parser("robot", help="a robot's certificate")
    robot.add_argument("dir")
    robot.add_argument("node_id")
    st = sub.add_parser("setup", help="describe a robot: import its URDF, report its capabilities")
    steps = st.add_subparsers(dest="step", required=True)
    steps.add_parser("template", help="print an answers file to fill in")
    imp = steps.add_parser("import", help="a body profile from a URDF (or xacro) and an answers file")
    imp.add_argument("urdf")
    imp.add_argument("-a", "--answers", help="what the URDF cannot say (see 'setup template')")
    imp.add_argument("-o", "--out", default="profile.yaml")
    imp.add_argument("--base-frame", help="the robot's base link (default base_link)")
    imp.add_argument("--package", action="append", help="NAME=DIRECTORY for package:// mesh paths")
    rep_ = steps.add_parser("report", help="the capability report of a profile")
    rep_.add_argument("profile")
    chk = steps.add_parser("check", help="live checks: topics, types, rates, controller timeout")
    chk.add_argument("profile")
    chk.add_argument("--duration", type=float, default=5.0, help="seconds to measure rates over")
    chk.add_argument("--motion", action="store_true", help="also run the controller timeout test (moves the robot)")
    chk.add_argument("--wheels-up", action="store_true", help="drive wheels are off the ground")
    chk.add_argument("--sim-time", action="store_true", help="use /clock (simulation)")
    chk.add_argument("--yes", action="store_true", help="skip the confirmation before moving")
    chk.add_argument("--direct", action="store_true",
                     help="command the robot directly instead of through Argo's supervisor (argo --setup)")
    cal = steps.add_parser("calibrate", help="stepped motion tests; measured limits written back")
    cal.add_argument("profile")
    cal.add_argument("-o", "--out", required=True, help="where to write the calibrated profile")
    cal.add_argument("--sim-time", action="store_true", help="use /clock (simulation)")
    cal.add_argument("--yes", action="store_true", help="skip the confirmation before moving")
    cal.add_argument("--direct", action="store_true",
                     help="command the robot directly instead of through Argo's supervisor (argo --setup)")
    home = argparse.ArgumentParser(add_help=False)
    home.add_argument("--home", help="the hub's home directory (default ~/.local/share/argus, or ARGUS_HOME)")
    hub = sub.add_parser("hub", help="set up and run the hub")
    hub_steps = hub.add_subparsers(dest="step", required=True)
    init = hub_steps.add_parser("init", parents=[home], help="a hub home: directories, its CA and certificate")
    init.add_argument("--host", action="append",
                      help="name or IP address robots reach the hub at (repeatable; default localhost, 127.0.0.1)")
    start = hub_steps.add_parser("start", parents=[home], help="run the hub until Ctrl-C")
    start.add_argument("--listen", default="0.0.0.0:50051", help="where robots connect (default 0.0.0.0:50051)")
    start.add_argument("--intake", help="bandwidth for map evidence from all robots, e.g. 20Mbit or 2MB "
                                        "(default 16Mbit)")
    sub.add_parser("status", parents=[home], help="the running hub")
    sub.add_parser("robots", parents=[home], help="every registered robot, its link and mission")
    goal = sub.add_parser("goal", parents=[home], help="send a robot to a point in its site frame")
    goal.add_argument("robot")
    goal.add_argument("x", type=float)
    goal.add_argument("y", type=float)
    goal.add_argument("z", type=float, nargs="?", default=0.0)
    goal.add_argument("--tolerance", type=float, default=0.0, help="metres (default 0.4)")
    cancel = sub.add_parser("cancel", parents=[home], help="stop a robot's mission")
    cancel.add_argument("robot")
    sub.add_parser("watch", parents=[home], help="hub events as they happen, until Ctrl-C")
    design = sub.add_parser("design", help="robot designs on the hub")
    design_actions = design.add_subparsers(dest="action", required=True)
    add = design_actions.add_parser("add", parents=[home], help="store a robot design on the hub and check it")
    add.add_argument("urdf", help="the robot's URDF or xacro")
    add.add_argument("-a", "--answers", help="what the URDF cannot say (see 'setup template')")
    add.add_argument("--gazebo-model", help="your own Gazebo model of the robot (SDF or xacro), with its plugins")
    add.add_argument("--package", action="append", help="NAME=DIRECTORY for package:// mesh paths")
    add.add_argument("--name", help="the design's name (default: the URDF's robot name)")
    design_actions.add_parser("list", parents=[home], help="every design and its versions")
    show = design_actions.add_parser("show", parents=[home], help="a design's findings and capability report")
    show.add_argument("design", help="NAME or NAME@VERSION (a version prefix is enough)")
    a = design_actions.add_parser("review", parents=[home], help="suggestions tied to what was measured")
    a.add_argument("design", help="NAME or NAME@VERSION")
    a.add_argument("--ask", help="a question to go with it")
    place = sub.add_parser("place", help="named places in a site")
    place_actions = place.add_subparsers(dest="action", required=True)
    a = place_actions.add_parser("add", parents=[home], help="a place by its coordinates in the site frame")
    a.add_argument("site")
    a.add_argument("name")
    a.add_argument("x", type=float)
    a.add_argument("y", type=float)
    a.add_argument("z", type=float, nargs="?", default=0.0)
    a.add_argument("--tolerance", type=float, default=0.0, help="metres (default 0.4)")
    a.add_argument("--heading", type=float, help="radians, for places a robot must face a way at")
    a.add_argument("--replace", action="store_true", help="replace a place of the same name")
    a = place_actions.add_parser("here", parents=[home], help="a place where a robot is now")
    a.add_argument("robot")
    a.add_argument("name")
    a.add_argument("--tolerance", type=float, default=0.0, help="metres (default 0.4)")
    a.add_argument("--replace", action="store_true", help="replace a place of the same name")
    a = place_actions.add_parser("list", parents=[home], help="every site's places")
    a.add_argument("site", nargs="?")
    a = place_actions.add_parser("remove", parents=[home], help="remove a place no route uses")
    a.add_argument("site")
    a.add_argument("name")
    route = sub.add_parser("route", help="routes through a site's places")
    route_actions = route.add_subparsers(dest="action", required=True)
    a = route_actions.add_parser("add", parents=[home], help="a route: places in order, each with a dwell")
    a.add_argument("site")
    a.add_argument("name")
    a.add_argument("stops", nargs="+", help="PLACE or PLACE:DWELL_SECONDS")
    a.add_argument("--mode", default="once", choices=("once", "loop", "back_and_forth"))
    a.add_argument("--cycles", type=int, help="times round for loop and back_and_forth (default: until cancelled)")
    a.add_argument("--on-unreachable", default="skip", choices=("skip", "stop"),
                   help="skip a stop that cannot be reached, or end the errand there")
    a.add_argument("--replace", action="store_true", help="replace a route of the same name")
    a = route_actions.add_parser("list", parents=[home], help="every site's routes")
    a.add_argument("site", nargs="?")
    a = route_actions.add_parser("remove", parents=[home], help="remove a route")
    a.add_argument("site")
    a.add_argument("name")
    a = sub.add_parser("go", parents=[home], help="send a robot to a named place in its site")
    a.add_argument("robot")
    a.add_argument("place")
    a = sub.add_parser("patrol", parents=[home], help="run a robot along a route in its site")
    a.add_argument("robot")
    a.add_argument("route")
    sub.add_parser("worlds", parents=[home], help="the worlds a validation job can use")
    val = sub.add_parser("validate", parents=[home], help="drive a design through Gazebo worlds and diagnose each run")
    val.add_argument("design", help="NAME or NAME@VERSION (default: its latest version)")
    val.add_argument("--worlds", help="comma separated (default: every world; see 'argus worlds')")
    val.add_argument("--contents", help="comma separated: objects, movers, empty (default objects)")
    val.add_argument("--repeats", type=int, default=1, help="runs per world and contents (default 1)")
    val.add_argument("--view", action="store_true", help="open Gazebo on each run as it goes live")
    val.add_argument("--detach", action="store_true", help="start the job and return")
    jobs = sub.add_parser("jobs", help="validation jobs")
    job_actions = jobs.add_subparsers(dest="action", required=True)
    job_actions.add_parser("list", parents=[home], help="every job and how it went")
    for name, text in (("watch", "follow a job's runs until it ends"), ("view", "watch, with Gazebo open on each run"),
                       ("report", "each run's outcome and findings"), ("cancel", "stop a job"),
                       ("keep", "never delete this job to make room"), ("unkeep", "let it be deleted to make room"),
                       ("delete", "delete a finished job and its logs")):
        a = job_actions.add_parser(name, parents=[home], help=text)
        a.add_argument("job", help="the job's id, or the start of it")
        if name == "watch":
            a.add_argument("--view", action="store_true", help="open Gazebo on each run as it goes live")
    a = job_actions.add_parser("explain", parents=[home], help="what a job's runs show together")
    a.add_argument("job", help="the job's id, or the start of it")
    a.add_argument("--ask", help="a question to go with it")
    a = job_actions.add_parser("compare", parents=[home], help="two jobs run by run over the worlds they share")
    a.add_argument("a", help="the first job")
    a.add_argument("b", help="the second job")
    runs = sub.add_parser("runs", help="runs of a validation job")
    run_actions = runs.add_subparsers(dest="action", required=True)
    for name, text in (("show", "outcome, findings, timeline and log files"),
                       ("replay", "play a finished run back in Gazebo"),
                       ("explain", "why a run went as it did, from the reasoning model")):
        a = run_actions.add_parser(name, parents=[home], help=text)
        a.add_argument("job", help="the job's id, or the start of it")
        a.add_argument("run", help="the run's number, or its name")
        if name == "explain":
            a.add_argument("--ask", help="a question to go with it")
    models = sub.add_parser("models", help="where models come from and what Argus uses them for")
    model_actions = models.add_subparsers(dest="action", required=True)
    a = model_actions.add_parser("connect", parents=[home], help="add a model source, or give one its key again")
    a.add_argument("kind", help="openai-compatible, anthropic, chatgpt or claude-code")
    a.add_argument("url", nargs="?", help="the endpoint's address (openai-compatible)")
    a.add_argument("--name", help="what to call it (default: the kind)")
    a.add_argument("--key-prompt", action="store_true", help="ask for an API key; it stays in the hub's memory only")
    model_actions.add_parser("list", parents=[home], help="connections, their models and the slots")
    a = model_actions.add_parser("use", parents=[home], help="fill a slot")
    a.add_argument("slot", help="reasoning or vision")
    a.add_argument("model", help="CONNECTION/MODEL")
    a = model_actions.add_parser("test", parents=[home], help="ask the model in a slot something small")
    a.add_argument("slot", help="reasoning or vision")
    a.add_argument("--image", help="an image file for the vision slot")
    a = model_actions.add_parser("disconnect", parents=[home], help="remove a connection and forget its key or tokens")
    a.add_argument("name")
    a = sub.add_parser("say", parents=[home], help="send a robot somewhere in words, through the site's places")
    a.add_argument("robot")
    a.add_argument("text", help="the instruction")
    a.add_argument("--yes", action="store_true", help="send it without asking")
    a = sub.add_parser("console", parents=[home], help="the Argus console; plain `argus` on a terminal opens it")
    a.add_argument("--plain", action="store_true", help="a plain prompt instead of the full-screen app")
    if not argv:
        from argus import console

        if console.can_open():
            return console.start()
        p.print_help()
        return 0
    args = p.parse_args(argv)
    if args.command == "console":
        from argus import console

        return console.start(args.home, args.plain)
    if args.command == "hub":
        from argus.hub import run

        if args.step == "init":
            return run.init(args.home, args.host)
        from argus import branding

        print(branding.banner() + "\n")
        return run.start(args.home, args.listen, args.intake)
    if args.command in ("status", "robots", "goal", "cancel", "watch", "design", "worlds", "validate", "jobs", "runs",
                        "place", "route", "go", "patrol", "models", "say"):
        return _client(args)
    return _certs(args) if args.command == "certs" else _setup(args)


if __name__ == "__main__":
    sys.exit(main())
