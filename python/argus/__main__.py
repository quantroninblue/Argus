"""Command line: python3 -m argus <command>.

  certs ca DIR                     a new certificate authority in DIR
  certs hub DIR --host H [...]     the hub's certificate, valid for each host name or address
  certs robot DIR NODE_ID          a robot's certificate; NODE_ID must match Argo's node_id
  setup template                   an answers file to fill in for import
  setup import URDF -a ANSWERS -o PROFILE
                                   a body profile from a URDF and the answers, with its report
  setup report PROFILE             the capability report of a profile
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
    args = p.parse_args(argv)
    return _certs(args) if args.command == "certs" else _setup(args)


if __name__ == "__main__":
    sys.exit(main())
