"""The console: what a typed line means, and the full-screen app driven as a person would."""

import asyncio

import pytest

from argus.console import core
from test_hub_app import _Loop, running_hub  # noqa: F401


def test_a_line_without_a_slash_is_an_instruction():
    assert core.read("take the dock").kind == "instruction"
    assert core.read("   ").kind == "nothing"


def test_commands_pass_through_with_their_words():
    a = core.read('/place add lab "shelf 3" 1 2')
    assert a.kind == "run" and a.argv == ["place", "add", "lab", "shelf 3", "1", "2"]


def test_a_mistyped_command_says_what_was_meant():
    a = core.read("/stauts")
    assert a.kind == "error" and "/status" in a.text


def test_the_hub_and_console_commands_are_handled_here():
    assert core.read("/hub start").kind == "hub_start"
    assert core.read("/hub stop").kind == "hub_stop"
    assert core.read("/quit").kind == "quit" and core.read("/exit").kind == "quit"
    assert core.read("/use rover-1").text == "rover-1"
    assert core.read("/watch").kind == "error"


def test_paths_and_variables_are_read_as_in_a_shell(monkeypatch):
    monkeypatch.setenv("SHARE", "/opt/share")
    monkeypatch.setenv("HOME", "/home/someone")
    assert core.read("/design add $SHARE/robot.urdf -a ~/answers.yaml").argv == [
        "design", "add", "/opt/share/robot.urdf", "-a", "/home/someone/answers.yaml"]


def test_a_redirect_is_sent_to_the_shell_rather_than_run():
    a = core.read("/setup template > answers.yaml")
    assert a.kind == "error" and "argus setup template > answers.yaml" in a.text


def test_the_hub_can_be_set_up_from_the_console():
    assert core.read("/hub init --host hub.lan").argv == ["hub", "init", "--host", "hub.lan"]


def test_every_listed_command_is_one_the_console_can_run():
    for _, usage, _ in core.COMMANDS:
        assert core.read(usage.split()[0] + (" init" if usage.startswith("/hub") else " x")).kind != "error", usage


def test_tab_stops_before_what_only_you_can_fill_in():
    assert core.complete("/pl") == "/place list "
    assert core.complete("/go") == "/go "
    assert core.complete("/nothing") == "/nothing"
    assert core.complete("plain words") == "plain words"


def test_suggestions_follow_what_is_typed():
    assert [u for u, _ in core.matching("/mod")][0].startswith("/models")
    assert core.matching("hello") == []


textual = pytest.importorskip("textual")

from argus.console.app import ArgusApp, Confirm  # noqa: E402


async def _settle(pilot, app, text, tries=60):
    for _ in range(tries):
        await pilot.pause(0.05)
        if any(text in line for line in app.transcript):
            return
    raise AssertionError(f"{text!r} never appeared in {app.transcript}")


def _drive(home, steps):
    async def run():
        app = ArgusApp(home)
        async with app.run_test(size=(110, 32)) as pilot:
            await steps(app, pilot)
    asyncio.run(run())


def test_the_app_finds_the_hub_and_shows_it(running_hub):  # noqa: F811
    async def steps(app, pilot):
        await _settle(pilot, app, "connected to the hub")
        await pilot.pause(0.2)
        status = str(app.query_one("#status").render())
        assert app.query_one("#composer").region.height == 3 and app.query_one("#timeline").region.height > 15
        assert "0/1 connected" in status and "talking to: rover-1" in status
    _drive(running_hub, steps)


def test_a_command_typed_in_the_app_prints_into_the_timeline(running_hub):  # noqa: F811
    async def steps(app, pilot):
        await _settle(pilot, app, "connected to the hub")
        await pilot.press(*"/robots", "enter")
        await _settle(pilot, app, "rover-1: not connected")
        await pilot.press(*"/place add lab dock 0 0", "enter")
        await _settle(pilot, app, "lab: place dock")
        await pilot.press(*"/place list", "enter")
        await _settle(pilot, app, "within 0.4 m")
        assert not any("places {" in line for line in app.transcript)
    _drive(running_hub, steps)


def test_typing_a_slash_suggests_and_tab_completes(running_hub):  # noqa: F811
    async def steps(app, pilot):
        await pilot.press(*"/mod")
        await pilot.pause(0.1)
        assert "/models list" in str(app.query_one("#suggestions").render())
        await pilot.press("tab")
        await pilot.pause(0.05)
        assert app.query_one("#composer").value == "/models list "
    _drive(running_hub, steps)


def test_a_goal_set_elsewhere_appears_live(running_hub):  # noqa: F811
    from argus.hub.client import AppClient

    async def steps(app, pilot):
        await _settle(pilot, app, "connected to the hub")
        await pilot.pause(0.3)
        with AppClient(running_hub) as hub:
            hub.goal("rover-1", 1.0, 1.0)
        await _settle(pilot, app, "rover-1")
    _drive(running_hub, steps)


def test_an_instruction_with_no_model_says_so_and_sends_nothing(running_hub):  # noqa: F811
    async def steps(app, pilot):
        await _settle(pilot, app, "connected to the hub")
        await pilot.press(*"/place add lab dock 0 0", "enter")
        await _settle(pilot, app, "lab: place dock")
        await pilot.press(*"go to the dock", "enter")
        await _settle(pilot, app, "nothing is in the reasoning slot")
        assert not isinstance(app.screen, Confirm)
    _drive(running_hub, steps)


def test_nothing_moves_without_a_yes(running_hub, monkeypatch):  # noqa: F811
    sent = []

    async def fake(self, argv, key=None):
        sent.append(argv)

    monkeypatch.setattr(ArgusApp, "_child_process", fake)
    monkeypatch.setattr(core, "read_instruction", lambda *a: core.Reading(["go", "rover-1", "dock"], "rover-1: go to dock"))

    async def steps(app, pilot):
        await _settle(pilot, app, "connected to the hub")
        await pilot.press(*"go to the dock", "enter")
        await pilot.pause(0.3)
        assert isinstance(app.screen, Confirm)
        await pilot.press("n")
        await _settle(pilot, app, "not sent")
        assert sent == []
        await pilot.press(*"go to the dock", "enter")
        await pilot.pause(0.3)
        await pilot.press("y")
        await pilot.pause(0.2)
        assert sent == [["go", "rover-1", "dock"]]
    _drive(running_hub, steps)


def test_with_no_hub_the_app_says_how_to_start_one(tmp_path):
    from argus.hub.home import Home

    async def steps(app, pilot):
        await _settle(pilot, app, "no hub is running here")
        await pilot.press(*"/nonsense", "enter")
        await _settle(pilot, app, "no command /nonsense")
    _drive(Home.at(tmp_path / "h"), steps)
