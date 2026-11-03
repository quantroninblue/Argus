"""The Argus console: what `argus` opens on a terminal. A full-screen app when Textual is
installed (app.py), a plain prompt otherwise (plain.py), both reading the same commands (core.py)."""

from __future__ import annotations

import sys

from argus.hub.home import Home


def can_open() -> bool:
    return sys.stdin.isatty() and sys.stdout.isatty()


def start(root=None, plain: bool = False) -> int:
    home = Home.at(root)
    if not plain:
        try:
            from argus.console.app import ArgusApp
        except ImportError:
            print("The full-screen console needs Textual, which is not installed. Using the plain console.\n")
        else:
            ArgusApp(home).run()
            return 0
    from argus.console.plain import shell

    return shell(home)
