"""The full-screen console: a timeline of what the hub and your commands report, a line to type
in, and a status line. Hub events arrive live; everything else goes through core.py."""

from __future__ import annotations

import asyncio
import threading
import time

from rich.text import Text
from textual import work
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Vertical
from textual.screen import ModalScreen
from textual.widgets import Footer, Input, RichLog, Static

from argus import branding
from argus.console import core
from argus.hub.client import AppClient, HubUnavailable
from argus.hub.home import Home

STATUS_PERIOD_S = 2.0
RECONNECT_S = 1.0

COLOURS = {"background": "#14100C", "surface": "#1F1812", "brand": "#C8924A", "amber": "#F0C060",
           "heading": "#8AAFD8", "text": "#EFE7DD", "dim": "#8C8378", "ok": "#6CC58B", "warn": "#E0B050",
           "fail": "#E06060"}
STYLES = {"": COLOURS["text"], "heading": f"bold {COLOURS['heading']}", "ok": COLOURS["ok"],
          "warn": COLOURS["warn"], "fail": COLOURS["fail"], "dim": COLOURS["dim"],
          "you": f"bold {COLOURS['amber']}", "brand": f"bold {COLOURS['brand']}"}
EVENT_STYLES = (("failed", "fail"), ("failsafe", "fail"), ("reached", "ok"), ("done", "ok"), ("connected", "ok"),
                ("disconnected", "warn"), ("skipped", "warn"), ("blocked", "warn"), ("cancelled", "dim"))


def _style_of(line: str) -> str:
    low = line.lower()
    if low.startswith(("error", "fail", "no hub", "no robot")) or " not " in low[:24]:
        return "fail" if low.startswith(("error", "fail")) else "warn"
    return ""


class Composer(Input):
    BINDINGS = [Binding("tab", "complete", show=False, priority=True),
                Binding("up", "older", show=False), Binding("down", "newer", show=False)]

    def action_complete(self) -> None:
        self.value = core.complete(self.value)
        self.cursor_position = len(self.value)

    def action_older(self) -> None:
        self.app.step_history(-1)

    def action_newer(self) -> None:
        self.app.step_history(1)


class Confirm(ModalScreen[bool]):
    BINDINGS = [Binding("y", "answer(True)", "send it"), Binding("n", "answer(False)", "do not send"),
                Binding("escape", "answer(False)", show=False)]

    def __init__(self, words: str) -> None:
        super().__init__()
        self.words = words

    def compose(self) -> ComposeResult:
        with Vertical(id="card"):
            yield Static("Send this to the robot?", id="card-title")
            yield Static(Text(self.words))
            yield Static("y to send, n to leave it", id="card-hint")

    def action_answer(self, value: bool) -> None:
        self.dismiss(value)


class AskKey(ModalScreen[str]):
    BINDINGS = [Binding("escape", "leave", show=False)]

    def compose(self) -> ComposeResult:
        with Vertical(id="card"):
            yield Static("API key", id="card-title")
            yield Input(password=True, id="key")
            yield Static("It stays in the hub's memory only.", id="card-hint")

    def on_input_submitted(self, event: Input.Submitted) -> None:
        self.dismiss(event.value)

    def action_leave(self) -> None:
        self.dismiss("")


class ArgusApp(App[None]):
    TITLE = "Argus"
    ENABLE_COMMAND_PALETTE = False
    BINDINGS = [Binding("ctrl+c", "interrupt", "stop / quit", priority=True),
                Binding("ctrl+l", "clear", "clear"),
                Binding("pageup", "scroll(-1)", "older", show=False),
                Binding("pagedown", "scroll(1)", "newer", show=False)]
    CSS = f"""
    Screen {{ background: {COLOURS['background']}; color: {COLOURS['text']}; }}
    #header {{ height: 1; padding: 0 1; background: {COLOURS['surface']}; color: {COLOURS['brand']}; text-style: bold; }}
    #timeline {{ height: 1fr; padding: 0 1; background: {COLOURS['background']}; scrollbar-size-vertical: 1; }}
    #suggestions {{ height: auto; max-height: 7; padding: 0 2; color: {COLOURS['dim']}; }}
    #composer {{ height: 3; border: round {COLOURS['brand']}; background: {COLOURS['background']}; }}
    #composer:focus {{ border: round {COLOURS['amber']}; }}
    #status {{ height: 1; padding: 0 1; background: {COLOURS['surface']}; color: {COLOURS['dim']}; }}
    Footer {{ background: {COLOURS['background']}; }}
    Confirm, AskKey {{ align: center middle; }}
    #card {{ width: 70; max-width: 90%; height: auto; padding: 1 2; background: {COLOURS['surface']};
             border: round {COLOURS['amber']}; }}
    #card-title {{ color: {COLOURS['amber']}; text-style: bold; padding-bottom: 1; }}
    #card-hint {{ color: {COLOURS['dim']}; padding-top: 1; }}
    """

    def __init__(self, home: Home) -> None:
        super().__init__()
        self.home = home
        self.robot = ""
        self.transcript: list[str] = []
        self.history: list[str] = []
        self._place = 0
        self._child: asyncio.subprocess.Process | None = None
        self._leaving = threading.Event()
        self._hub_up: bool | None = None
        self._last_interrupt = 0.0
        self._watching: AppClient | None = None

    def compose(self) -> ComposeResult:
        yield Static("ARGUS  robot navigation hub", id="header")
        yield RichLog(id="timeline", wrap=True, markup=False, highlight=False)
        yield Static(id="suggestions")
        yield Composer(placeholder="Tell a robot what to do, or / for commands", id="composer")
        yield Static(id="status")
        yield Footer()

    def on_mount(self) -> None:
        for i, line in enumerate(branding.LETTERS):
            share = i / (len(branding.LETTERS) - 1)
            rgb = [round(a + (b - a) * share) for a, b in zip(branding.SHADE_TOP, branding.SHADE_BOTTOM)]
            self._log(Text(line, style="bold #{:02X}{:02X}{:02X}".format(*rgb)))
        self.say(branding.SUBTITLE.rjust(len(branding.LETTERS[-1])), "dim")
        self.say("")
        self.say(f"{core.HELP_INTRO} /help lists them. Tab completes.", "dim")
        self._follow_status()
        self._follow_events()
        self.set_interval(STATUS_PERIOD_S, self._follow_status)
        self.query_one(Composer).focus()

    def on_unmount(self) -> None:
        self._leaving.set()
        if self._watching is not None:
            self._watching.close()  # ends the blocking read in _follow_events
        if self._child is not None and self._child.returncode is None:
            self._child.terminate()

    def _log(self, text: Text) -> None:
        self.transcript.append(text.plain)
        self.query_one(RichLog).write(text, scroll_end=True)

    def say(self, line: str, style: str = "") -> None:
        self._log(Text(line, style=STYLES.get(style, STYLES[""])))

    def on_input_changed(self, event: Input.Changed) -> None:
        if event.input.id != "composer":
            return
        found = core.matching(event.value)
        text = Text()
        for k, (usage, what) in enumerate(found):
            text.append(("\n" if k else "") + f"{usage:<42} ", style=COLOURS["amber"])
            text.append(what, style=COLOURS["dim"])
        self.query_one("#suggestions", Static).update(text)

    def step_history(self, step: int) -> None:
        if not self.history:
            return
        self._place = max(0, min(len(self.history), self._place + step))
        composer = self.query_one(Composer)
        composer.value = self.history[self._place] if self._place < len(self.history) else ""
        composer.cursor_position = len(composer.value)

    def on_input_submitted(self, event: Input.Submitted) -> None:
        if event.input.id != "composer":
            return
        line = event.value
        event.input.value = ""
        if line.strip():
            self.history.append(line)
            self._place = len(self.history)
            self.say(f"> {line}", "you")
        self.act(core.read(line))

    def act(self, action: core.Action) -> None:
        if action.kind == "quit":
            self.exit()
        elif action.kind == "help":
            for style, text in core.help_lines():
                self.say(text, style)
        elif action.kind == "clear":
            self.action_clear()
        elif action.kind == "error":
            self.say(action.text, "warn")
        elif action.kind == "use":
            self.robot = action.text
            self.say(f"instructions go to {self.robot}", "dim")
        elif action.kind == "hub_start":
            self._hub_start()
        elif action.kind == "hub_stop":
            self.say(core.stop_hub(self.home), "dim")
        elif action.kind == "run":
            self._run(action.argv)
        elif action.kind == "instruction":
            self._instruction(action.text)

    @work(exclusive=False)
    async def _run(self, argv: list[str]) -> None:
        if self._child is not None:
            self.say("something is still running; Ctrl-C stops it", "warn")
            return
        key = None
        if "--key-prompt" in argv:
            key = await self.push_screen_wait(AskKey())
            if not key:
                self.say("no key given", "dim")
                return
        await self._child_process(argv, key)

    async def _child_process(self, argv: list[str], key: str | None = None) -> None:
        self._child = await asyncio.create_subprocess_exec(
            *core.command_line(argv, self.home), env=core.child_env(),
            stdin=asyncio.subprocess.PIPE if key else asyncio.subprocess.DEVNULL,
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
        child = self._child
        if key:
            child.stdin.write(key.encode() + b"\n")
            await child.stdin.drain()
            child.stdin.close()
        async for raw in child.stdout:
            line = raw.decode(errors="replace").rstrip()
            self.say(line, _style_of(line))
        code = await child.wait()
        self._child = None
        if code not in (0, -15):
            self.say(f"(ended with code {code})", "dim")
        self._follow_status()

    @work(exclusive=False)
    async def _instruction(self, text: str) -> None:
        try:
            robot = self.robot or await asyncio.to_thread(core.only_robot, self.home)
            if not robot:
                self.say("which robot? /robots lists them, /use ROBOT picks one", "warn")
                return
            self.say("reading that...", "dim")
            reading = await asyncio.to_thread(core.read_instruction, self.home, robot, text)
        except HubUnavailable as e:
            self.say(str(e), "warn")
            return
        if reading.send is None:
            self.say(reading.words, "warn")
            return
        if await self.push_screen_wait(Confirm(reading.words)):
            await self._child_process(reading.send)
        else:
            self.say("not sent", "dim")

    @work(thread=True)
    def _hub_start(self) -> None:
        for line in core.start_hub(self.home):
            self.call_from_thread(self.say, line, "dim")

    @work(thread=True, exclusive=True, group="status")
    def _follow_status(self) -> None:
        try:
            with AppClient(self.home) as hub:
                st, robots, models = hub.status(), hub.robots().robots, hub.models()
        except HubUnavailable:
            self.call_from_thread(self._show_status, None, [], None)
            return
        self.call_from_thread(self._show_status, st, robots, models)

    def _show_status(self, st, robots, models) -> None:
        up = st is not None
        if up != self._hub_up:
            if up:
                self.say(f"connected to the hub at {st.home}", "ok")
            else:
                self.say("no hub is running here: /hub start", "warn")
            self._hub_up = up
        if not up:
            self.query_one("#status", Static).update("hub: not running")
            return
        slot = next((s for s in models.slots if s.slot == "reasoning"), None)
        model = f"{slot.connection}/{slot.model}" if slot and slot.connection else "none"
        chosen = self.robot or (robots[0].node_id if len(robots) == 1 else "none")
        self.query_one("#status", Static).update(
            f"hub: up  |  robots: {st.robots_connected}/{st.robots_registered} connected  |  "
            f"talking to: {chosen}  |  model: {model}")

    @work(thread=True)
    def _follow_events(self) -> None:
        while not self._leaving.is_set():
            try:
                with AppClient(self.home) as hub:
                    self._watching = hub
                    for e in hub.watch():
                        if self._leaving.is_set():
                            return
                        self.call_from_thread(self._event, e)
            except (HubUnavailable, ValueError):
                pass  # ValueError: the channel was closed while reading, on the way out
            self._leaving.wait(RECONNECT_S)

    def _event(self, e) -> None:
        who = f"{e.node_id}: " if e.node_id else ""
        kind = e.kind.replace("_", " ")
        line = f"{time.strftime('%H:%M:%S', time.localtime(e.stamp))}  {who}{kind}" + (f": {e.detail}" if e.detail else "")
        low = kind.lower()
        self.say(line, next((s for word, s in EVENT_STYLES if word in low), "dim"))

    def action_interrupt(self) -> None:
        if self._child is not None and self._child.returncode is None:
            self._child.terminate()
            self.say("stopped", "dim")
            return
        composer = self.query_one(Composer)
        if composer.value:
            composer.value = ""
            return
        now = time.monotonic()
        if now - self._last_interrupt < 2.0:
            self.exit()
        self._last_interrupt = now
        self.say("Ctrl-C again to leave", "dim")

    def action_clear(self) -> None:
        self.query_one(RichLog).clear()
        self.transcript.clear()

    def action_scroll(self, direction: int) -> None:
        log = self.query_one(RichLog)
        (log.scroll_page_up if direction < 0 else log.scroll_page_down)(animate=False)
