"""How the argus command looks: the name in large letters, and a few colours.

Bronze for the name (shaded toward amber on terminals with full colour), a light steel blue
for headings, and plain green, amber and red for results. Navy, the other brand colour, does
not read as text on a dark terminal, so it is left out here. Colour only on a terminal that
shows it: never when output is piped, with NO_COLOR set (no-color.org), with TERM=dumb, or
after --no-color. On a terminal the command also sets the window title, makes log paths
clickable, and keeps a status line up to date while a job runs; ARGUS_REDUCED_MOTION=1 stops
it redrawing in place.
"""

from __future__ import annotations

import os
import shutil
import sys

LETTERS = (
    r"    _    ____   ____ _   _ ____",
    r"   / \  |  _ \ / ___| | | / ___|",
    r"  / _ \ | |_) | |  _| | | \___ \ ",
    r" / ___ \|  _ <| |_| | |_| |___) |",
    r"/_/   \_\_| \_\\____|\___/|____/",
)
NARROW = "ARGUS"
SUBTITLE = "robot navigation hub"

# The lettering is shaded from bronze to amber, top to bottom, where 24-bit colour shows.
SHADE_TOP = (184, 128, 60)
SHADE_BOTTOM = (240, 192, 96)

# role: (24-bit colour, nearest of the 256)
_COLOURS = {
    "brand": ((200, 146, 74), 179),
    "heading": ((138, 175, 216), 110),
    "ok": ((108, 197, 139), 78),
    "warn": ((224, 176, 80), 179),
    "fail": ((224, 96, 96), 167),
    "dim": ((140, 140, 140), 245),
}

_disabled = False


def disable() -> None:
    """For --no-color."""
    global _disabled
    _disabled = True


def colour_on(stream=None) -> bool:
    stream = stream or sys.stdout
    if _disabled or os.environ.get("NO_COLOR") or os.environ.get("TERM") == "dumb":
        return False
    return hasattr(stream, "isatty") and stream.isatty()


def _truecolour() -> bool:
    return os.environ.get("COLORTERM", "").lower() in ("truecolor", "24bit")


def paint(text: str, role: str, stream=None) -> str:
    if not colour_on(stream) or role not in _COLOURS:
        return text
    rgb, index = _COLOURS[role]
    code = "38;2;{};{};{}".format(*rgb) if _truecolour() else f"38;5;{index}"
    bold = ";1" if role in ("brand", "heading") else ""
    return f"\033[{code}{bold}m{text}\033[0m"


def _shade(text: str, share: float, stream=None) -> str:
    if not colour_on(stream) or not _truecolour():
        return paint(text, "brand", stream)
    rgb = [round(a + (b - a) * share) for a, b in zip(SHADE_TOP, SHADE_BOTTOM)]
    return "\033[38;2;{};{};{};1m".format(*rgb) + text + "\033[0m"


def banner(width: int | None = None, stream=None) -> str:
    """The name in large letters when the terminal is wide enough, else on one line."""
    if width is None:
        width = shutil.get_terminal_size((80, 24)).columns
    if width < max(len(line) for line in LETTERS) + 2:
        return paint(NARROW, "brand", stream) + "  " + paint(SUBTITLE, "dim", stream)
    lines = [_shade(line, i / (len(LETTERS) - 1), stream) for i, line in enumerate(LETTERS)]
    lines.append(paint(SUBTITLE.rjust(len(LETTERS[-1])), "dim", stream))
    return "\n".join(lines)


def status(word: str, stream=None) -> str:
    """Colours a result word: pass, ok, FAIL, error, warning, note."""
    role = {"pass": "ok", "ok": "ok", "fail": "fail", "error": "fail", "warning": "warn", "warn": "warn",
            "note": "dim"}.get(word.lower())
    return paint(word, role, stream) if role else word


# What each word Argus prints about runs and designs means, for its colour.
_ROLE_OF = {
    # results and outcomes
    "pass": "ok", "ok": "ok", "reached": "ok", "done": "ok", "runnable": "ok",
    "fail": "fail", "failed": "warn", "collided": "fail", "error": "fail", "cannot run": "fail",
    "warning": "warn", "warn": "warn", "interrupted": "warn", "note": "dim",
    # job and run states
    "running": "heading", "queued": "dim", "cancelled": "dim",
    # who a finding blames
    "design": "warn", "argus": "fail", "world": "heading", "inconclusive": "dim",
}


def word(text: str, stream=None) -> str:
    """A word Argus prints about a run, a job, a design or a finding, in its colour."""
    role = _ROLE_OF.get(text.lower())
    return paint(text, role, stream) if role else text


def heading(text: str, stream=None) -> str:
    return paint(text, "heading", stream)


def dim(text: str, stream=None) -> str:
    return paint(text, "dim", stream)


def _terminal(stream=None) -> bool:
    stream = stream or sys.stdout
    return (hasattr(stream, "isatty") and stream.isatty() and os.environ.get("TERM") != "dumb"
            and not _disabled and not os.environ.get("NO_COLOR"))


def reduced_motion() -> bool:
    return os.environ.get("ARGUS_REDUCED_MOTION", "") not in ("", "0")


def set_title(text: str, stream=None) -> None:
    """The terminal window's title."""
    stream = stream or sys.stdout
    if _terminal(stream):
        stream.write(f"\033]0;{text}\007")
        stream.flush()


def link(target: str, label: str | None = None, stream=None) -> str:
    """A path or URL the terminal can open with a click (OSC 8); plain text elsewhere."""
    label = label or target
    if not _terminal(stream):
        return label
    url = target if "://" in target else "file://" + os.path.abspath(target)
    return f"\033]8;;{url}\033\\{label}\033]8;;\033\\"


class StatusLine:
    """One line at the bottom of the output, rewritten in place while something runs. Lines
    printed through `say` go above it. Off when the output is not a terminal or with
    ARGUS_REDUCED_MOTION set: then only `say` prints."""

    def __init__(self, stream=None) -> None:
        self.stream = stream or sys.stdout
        self.live = _terminal(self.stream) and not reduced_motion()
        self.text = ""

    def _clear(self) -> None:
        if self.live and self.text:
            self.stream.write("\r\033[2K")

    def update(self, text: str) -> None:
        if not self.live:
            return
        width = shutil.get_terminal_size((80, 24)).columns
        self._clear()
        self.text = text
        visible = text if len(text) < width else text[:width - 1]
        self.stream.write(paint(visible, "dim", self.stream))
        self.stream.flush()

    def say(self, line: str) -> None:
        self._clear()
        self.stream.write(line + "\n")
        if self.live and self.text:
            self.update(self.text)
        self.stream.flush()

    def close(self) -> None:
        self._clear()
        self.text = ""
        if self.live:
            self.stream.flush()
