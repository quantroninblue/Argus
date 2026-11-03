"""The console as a plain prompt, for terminals and machines without the full-screen app."""

from __future__ import annotations

import subprocess

from argus import branding
from argus.console import core
from argus.hub.client import HubUnavailable
from argus.hub.home import Home

try:
    import readline  # noqa: F401  (arrow keys and history for input())
except ImportError:
    pass


def _instruction(home: Home, robot: str, text: str) -> None:
    if not robot:
        print("no robot chosen: /use ROBOT", flush=True)
        return
    reading = core.read_instruction(home, robot, text)
    print(reading.words)
    if reading.send is not None and input("send it? [y/N] ").strip().lower() == "y":
        subprocess.run(core.command_line(reading.send, home), env=core.child_env())


def shell(home: Home) -> int:
    print(branding.banner() + "\n")
    print(f"{core.HELP_INTRO} /help lists the commands.")
    if not core.hub_running(home):
        print("No hub is running here: /hub start")
    robot = ""
    while True:
        try:
            line = input(branding.paint("argus> ", "brand"))
        except (EOFError, KeyboardInterrupt):
            print()
            return 0
        action = core.read(line)
        if action.kind == "quit":
            return 0
        try:
            if action.kind in ("help", "clear"):
                for style, text in core.help_lines() if action.kind == "help" else []:
                    print(branding.paint(text, style) if style else text)
            elif action.kind == "error":
                print(action.text)
            elif action.kind == "use":
                robot = action.text
                print(f"instructions go to {robot}")
            elif action.kind == "hub_start":
                print("\n".join(core.start_hub(home)))
            elif action.kind == "hub_stop":
                print(core.stop_hub(home))
            elif action.kind == "run":
                subprocess.run(core.command_line(action.argv, home), env=dict(core.child_env(), NO_COLOR=""))
            elif action.kind == "instruction":
                _instruction(home, robot or core.only_robot(home), action.text)
        except HubUnavailable as e:
            print(e)
        except KeyboardInterrupt:
            print()
