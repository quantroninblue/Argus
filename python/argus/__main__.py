"""Command line: python3 -m argus <command>.

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

Every hub command takes --home DIR (default ~/.local/share/argus, or ARGUS_HOME).
"""

from __future__ import annotations

import argparse
import sys
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
        print(result.text())
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
    packages = {}
    for item in args.package or []:
        name, sep, directory = item.partition("=")
        if not sep:
            print(f"--package {item}: expected NAME=DIRECTORY", file=sys.stderr)
            return 2
        packages[name] = Path(directory)
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
            print("Start an answers file with: python3 -m argus setup template > answers.yaml", file=sys.stderr)
        return 1
    Path(args.out).write_text(profiles.dump(doc))
    print(f"wrote {args.out}\n")
    print(capability_report(doc))
    return 0


def _client(args) -> int:
    from argus.hub.client import AppClient, HubUnavailable, format_event, format_robots, format_status
    from argus.hub.home import Home

    try:
        with AppClient(Home.at(args.home)) as hub:
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


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="python3 -m argus")
    sub = p.add_subparsers(dest="command", required=True)
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
    args = p.parse_args(argv)
    if args.command == "hub":
        from argus.hub import run

        if args.step == "init":
            return run.init(args.home, args.host)
        return run.start(args.home, args.listen, args.intake)
    if args.command in ("status", "robots", "goal", "cancel", "watch"):
        return _client(args)
    return _certs(args) if args.command == "certs" else _setup(args)


if __name__ == "__main__":
    sys.exit(main())
