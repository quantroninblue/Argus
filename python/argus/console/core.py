"""What the console understands. A line starting with / is a command, and anything else is an
instruction for a robot, read by the reasoning model and always confirmed before it is sent.
Commands run as the ordinary `argus` command, so the console can do what the command can and
nothing behaves differently here."""

from __future__ import annotations

import difflib
import os
import shlex
import signal
import subprocess
import sys
from dataclasses import dataclass, field

from argus.hub.client import AppClient, HubUnavailable
from argus.hub.home import Home

# (group, what to type, what it does)
COMMANDS = (
    ("Console", "/help", "this list"),
    ("Console", "/clear", "clear the timeline"),
    ("Console", "/use ROBOT", "the robot your instructions go to"),
    ("Console", "/quit", "leave; a hub keeps running"),
    ("Hub", "/hub start", "start the hub in the background"),
    ("Hub", "/hub stop", "stop a hub started from here"),
    ("Hub", "/hub init --host NAME", "set up the hub for robots on other machines"),
    ("Hub", "/status", "the running hub"),
    ("Robots", "/robots", "every robot, its link and mission"),
    ("Robots", "/goal ROBOT X Y", "send a robot to a point"),
    ("Robots", "/go ROBOT PLACE", "send a robot to a named place"),
    ("Robots", "/patrol ROBOT ROUTE", "run a robot along a route"),
    ("Robots", "/cancel ROBOT", "stop a robot's mission"),
    ("Sites", "/place list", "named places"),
    ("Sites", "/place add SITE NAME X Y", "add a place by coordinates"),
    ("Sites", "/place here ROBOT NAME", "add a place where a robot is"),
    ("Sites", "/place remove SITE NAME", "remove a place"),
    ("Sites", "/route list", "routes"),
    ("Sites", "/route add SITE NAME STOP:DWELL ...", "add a route"),
    ("Sites", "/route remove SITE NAME", "remove a route"),
    ("Validation", "/design list", "robot designs"),
    ("Validation", "/design show NAME", "a design's findings and capability report"),
    ("Validation", "/design review NAME", "a design review from the reasoning model"),
    ("Validation", "/worlds", "the worlds a job can use"),
    ("Validation", "/validate DESIGN", "drive a design through Gazebo worlds"),
    ("Validation", "/jobs list", "validation jobs"),
    ("Validation", "/jobs report JOB", "each run's outcome and findings"),
    ("Validation", "/jobs explain JOB", "what went wrong in a job"),
    ("Validation", "/jobs compare JOB JOB", "two jobs, run by run"),
    ("Validation", "/runs show JOB RUN", "one run"),
    ("Validation", "/runs explain JOB RUN", "why a run went as it did"),
    ("Models", "/models list", "connections, models and slots"),
    ("Models", "/models connect KIND [URL] [--key-prompt]", "add a model source"),
    ("Models", "/models use SLOT CONNECTION/MODEL", "fill a slot"),
    ("Models", "/models test SLOT", "ask the model something small"),
    ("Models", "/models disconnect NAME", "remove a connection"),
)
PASSED_ON = frozenset(("status", "robots", "goal", "cancel", "go", "patrol", "place", "route", "design", "worlds",
                       "validate", "jobs", "runs", "models", "say", "certs", "setup"))
OWN = ("help", "clear", "use", "quit", "exit", "hub")
HELP_INTRO = "Type an instruction for a robot in plain words, or a command starting with /."


@dataclass
class Action:
    kind: str  # nothing, help, clear, quit, use, hub_start, hub_stop, run, instruction, error
    argv: list[str] = field(default_factory=list)
    text: str = ""


def read(line: str) -> Action:
    line = line.strip()
    if not line:
        return Action("nothing")
    if not line.startswith("/"):
        return Action("instruction", text=line)
    try:
        words = shlex.split(line[1:])
    except ValueError as e:
        return Action("error", text=f"could not read that: {e}")
    if not words:
        return Action("help")
    if any(word in (">", ">>", "|", "<") for word in words):
        return Action("error", text="that needs your shell: run it there as argus " + line[1:])
    words = [os.path.expanduser(os.path.expandvars(word)) for word in words]
    name, rest = words[0], words[1:]
    if name in ("quit", "exit"):
        return Action("quit")
    if name in ("help", "clear"):
        return Action(name)
    if name == "use":
        return Action("use", text=rest[0]) if len(rest) == 1 else Action("error", text="use: /use ROBOT")
    if name == "hub":
        if rest == ["start"]:
            return Action("hub_start")
        if rest == ["stop"]:
            return Action("hub_stop")
        if rest[:1] == ["init"]:
            return Action("run", argv=words)
        return Action("error", text="hub: /hub start, /hub stop or /hub init")
    if name == "watch":
        return Action("error", text="the timeline here already shows what the hub reports")
    if name in PASSED_ON:
        return Action("run", argv=words)
    near = difflib.get_close_matches(name, [*PASSED_ON, *OWN], n=1)
    return Action("error", text=f"no command /{name}" + (f"; did you mean /{near[0]}?" if near else "; /help lists them"))


def matching(prefix: str, limit: int = 6) -> list[tuple[str, str]]:
    """Commands that start with what has been typed, as (usage, what it does)."""
    if not prefix.startswith("/"):
        return []
    typed = prefix.lower()
    return [(usage, what) for _, usage, what in COMMANDS if usage.lower().startswith(typed)][:limit]


def complete(prefix: str) -> str:
    """The text a Tab turns `prefix` into: the known words of the first match."""
    found = matching(prefix)
    if not found:
        return prefix
    # What has been typed in full wins over a longer command that starts the same way.
    found = [f for f in found if f[0].split()[0] == prefix.strip()] or found
    words = []
    for word in found[0][0].split():
        if word.isupper() or word.startswith("[") or word.startswith("--"):
            break
        words.append(word)
    done = " ".join(words)
    return done + " " if len(done) >= len(prefix) else prefix


def help_lines() -> list[tuple[str, str]]:
    """(style, text) lines: a heading per group."""
    lines, group = [("", HELP_INTRO)], ""
    for g, usage, what in COMMANDS:
        if g != group:
            group = g
            lines.append(("heading", g))
        lines.append(("", f"  {usage:<42} {what}"))
    return lines


def command_line(argv: list[str], home: Home) -> list[str]:
    extra = ["--home", str(home.root)] if argv and argv[0] not in ("certs", "setup") else []
    return [sys.executable, "-m", "argus", *argv, *extra]


def child_env() -> dict:
    env = dict(os.environ, PYTHONUNBUFFERED="1", NO_COLOR="1")
    env["PYTHONPATH"] = os.pathsep.join(p for p in sys.path if p)
    return env


def hub_running(home: Home) -> bool:
    try:
        with AppClient(home) as hub:
            hub.status()
        return True
    except HubUnavailable:
        return False


def _pid_file(home: Home):
    return home.root / "console-hub.pid"


def start_hub(home: Home) -> list[str]:
    """Starts the hub as its own process, so it outlives the console. Lines for the user."""
    if hub_running(home):
        return ["the hub is already running"]
    out = []
    if not (home.root / "pki" / "hub.pem").exists():
        done = subprocess.run(command_line(["hub", "init"], home), env=child_env(), capture_output=True, text=True)
        if done.returncode:
            return [(done.stderr or done.stdout).strip() or "could not set up the hub's home"]
        out.append(f"set up a hub home at {home.root}")
    log = open(home.root / "hub.log", "ab")
    process = subprocess.Popen(command_line(["hub", "start"], home), env=child_env(), stdin=subprocess.DEVNULL,
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    log.close()
    _pid_file(home).write_text(str(process.pid))
    out.append(f"starting the hub; its log is {home.root / 'hub.log'}")
    if not os.environ.get("ROS_DISTRO"):
        out.append("ROS 2 is not sourced here, so validation runs will not work from this hub")
    return out


def stop_hub(home: Home) -> str:
    try:
        pid = int(_pid_file(home).read_text())
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            ours = b"argus" in f.read()
    except (OSError, ValueError):
        ours = False
    if not ours:
        return "no hub started from here is running; a hub started elsewhere stops with Ctrl-C where it runs"
    os.kill(pid, signal.SIGTERM)
    _pid_file(home).unlink(missing_ok=True)
    return "stopping the hub"


@dataclass
class Reading:
    """What the reasoning model made of an instruction."""

    send: list[str] | None  # the command that carries it out, or None when nothing can be sent
    words: str


def read_instruction(home: Home, robot: str, text: str) -> Reading:
    with AppClient(home) as hub:
        reply = hub.interpret_goal(robot, text)
    if not reply.ok:
        return Reading(None, reply.detail)
    if reply.kind == "unclear":
        return Reading(None, f"not sent: {reply.name}")
    if reply.kind == "place":
        return Reading(["go", robot, reply.name], f"{robot}: go to {reply.name} (read by {reply.model})")
    return Reading(["patrol", robot, reply.name], f"{robot}: run route {reply.name} (read by {reply.model})")


def only_robot(home: Home) -> str:
    """The robot instructions go to when none was chosen: the only one there is."""
    with AppClient(home) as hub:
        robots = hub.robots().robots
    return robots[0].node_id if len(robots) == 1 else ""
