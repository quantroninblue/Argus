"""The argus command's look: the name in large letters, colour only where it shows."""

import io

from argus import branding
from argus.__main__ import main


class _Tty(io.StringIO):
    def isatty(self):
        return True


def test_wide_terminals_get_the_lettering_and_narrow_ones_one_line():
    wide = branding.banner(80, io.StringIO())
    assert wide.splitlines()[:5] == list(branding.LETTERS)
    assert branding.banner(20, io.StringIO()).startswith("ARGUS")


def test_colour_only_on_a_terminal_and_never_with_no_color(monkeypatch):
    monkeypatch.delenv("NO_COLOR", raising=False)
    monkeypatch.setenv("TERM", "xterm-256color")
    assert "\033[" in branding.paint("x", "brand", _Tty())
    assert branding.paint("x", "brand", io.StringIO()) == "x"  # piped
    monkeypatch.setenv("NO_COLOR", "1")
    assert branding.paint("x", "brand", _Tty()) == "x"


def test_everything_shipped_is_ascii():
    assert all(line.isascii() for line in branding.LETTERS)
    assert branding.SUBTITLE.isascii() and branding.NARROW.isascii()


def test_bare_argus_lists_the_commands(capsys):
    assert main([]) == 0
    out = capsys.readouterr().out
    assert "robot navigation hub" in out and "design" in out and "setup" in out


def test_no_color_is_accepted_anywhere(capsys):
    assert main(["--no-color"]) == 0
    assert "\033[" not in capsys.readouterr().out


def _terminal(monkeypatch, colorterm=""):
    monkeypatch.setattr(branding, "_disabled", False)  # an earlier --no-color in this process
    monkeypatch.delenv("NO_COLOR", raising=False)
    monkeypatch.delenv("ARGUS_REDUCED_MOTION", raising=False)
    monkeypatch.setenv("TERM", "xterm-256color")
    monkeypatch.setenv("COLORTERM", colorterm)


def test_the_lettering_is_shaded_on_full_colour_terminals_only(monkeypatch):
    _terminal(monkeypatch, "truecolor")
    lines = branding.banner(80, _Tty()).splitlines()
    first, last = lines[0].split("m")[0], lines[4].split("m")[0]
    assert first.startswith("\033[38;2;") and first != last  # bronze at the top, amber below
    _terminal(monkeypatch)
    assert len({line.split("m")[0] for line in branding.banner(80, _Tty()).splitlines()[:5]}) == 1


def test_who_is_at_fault_and_how_runs_ended_have_their_own_colours(monkeypatch):
    _terminal(monkeypatch)
    tty = _Tty()
    assert branding.word("argus", tty) != branding.word("design", tty) != branding.word("world", tty)
    assert branding.word("reached", tty) == branding.paint("reached", "ok", tty)
    assert branding.word("collided", tty) == branding.paint("collided", "fail", tty)
    assert branding.word("something else", tty) == "something else"
    assert branding.word("argus", io.StringIO()) == "argus"


def test_the_status_line_redraws_in_place_on_a_terminal_only(monkeypatch):
    _terminal(monkeypatch)
    out = _Tty()
    line = branding.StatusLine(out)
    line.update("run 1 of 3")
    line.say("run 1 of 3: reached")
    line.update("run 2 of 3")
    line.close()
    text = out.getvalue()
    assert "\r\033[2K" in text and text.index("run 1 of 3: reached\n") < text.rindex("run 2 of 3")
    piped = io.StringIO()
    quiet = branding.StatusLine(piped)
    quiet.update("run 1 of 3")
    quiet.say("run 1 of 3: reached")
    assert piped.getvalue() == "run 1 of 3: reached\n"
    monkeypatch.setenv("ARGUS_REDUCED_MOTION", "1")
    still = _Tty()
    branding.StatusLine(still).update("run 1 of 3")
    assert still.getvalue() == ""


def test_log_paths_are_links_and_the_title_is_set_on_a_terminal_only(monkeypatch):
    _terminal(monkeypatch)
    assert branding.link("/tmp/run", stream=_Tty()).startswith("\033]8;;file:///tmp/run")
    assert branding.link("/tmp/run", stream=io.StringIO()) == "/tmp/run"
    tty, piped = _Tty(), io.StringIO()
    branding.set_title("Argus: run 2 of 5", tty)
    branding.set_title("Argus: run 2 of 5", piped)
    assert tty.getvalue() == "\033]0;Argus: run 2 of 5\007" and piped.getvalue() == ""
