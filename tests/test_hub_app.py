"""The hub's home directory and app interface: init, start, the argus command over the app
socket, events, and who may call."""

import asyncio
import os
import shutil
import socket
import stat
import threading
import time
import tomllib
from pathlib import Path

import pytest

from argus import profile as profiles
from argus.__main__ import main
from argus.hub import Hub, Registry
from argus.hub.app import VERSION, AppServer, Caller, HubAlreadyRunning, authorize, local_caller
from argus.hub.client import AppClient, HubUnavailable, format_robots
from argus.hub.home import Home
from argus.hub.run import serve
from argus.protocol.v1 import link_pb2, session_pb2

ROOT = Path(__file__).resolve().parents[1]


def _home(tmp_path) -> Home:
    home = Home.at(tmp_path / "home")
    home.init(["localhost", "127.0.0.1"])
    shutil.copy(profiles.EXAMPLES_DIR / "planar_differential.yaml", home.profiles / "diff.yaml")
    home.robots_file.write_text("rover-1: {profile: example-planar-differential, site: lab}\n")
    return home


class _Loop:
    """An event loop on its own thread, for running the hub while the test acts as a client."""

    def __init__(self) -> None:
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.thread.start()

    def run(self, coro, timeout=10.0):
        return asyncio.run_coroutine_threadsafe(coro, self.loop).result(timeout)

    def close(self) -> None:
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(5)
        self.loop.close()


@pytest.fixture
def running_hub(tmp_path):
    home = _home(tmp_path)
    loop = _Loop()
    stop = loop.run(_event())
    ready = loop.run(_event())
    task = asyncio.run_coroutine_threadsafe(serve(home, "127.0.0.1:0", stop=stop, ready=ready), loop.loop)
    loop.run(ready.wait())
    yield home
    loop.loop.call_soon_threadsafe(stop.set)
    task.result(10)
    loop.close()


async def _event() -> asyncio.Event:
    return asyncio.Event()


def test_init_makes_a_private_home_and_never_replaces_keys(tmp_path):
    home = Home.at(tmp_path / "home")
    made = home.init(["localhost"])
    assert made
    for p in (home.profiles, home.pki, home.runs, home.run_dir):
        assert p.is_dir()
    for f in ("ca.pem", "ca.key", "hub.pem", "hub.key"):
        assert (home.pki / f).is_file()
    assert not (home.pki / "hub.csr").exists()
    assert stat.S_IMODE(home.run_dir.stat().st_mode) == 0o700
    assert stat.S_IMODE(home.pki.stat().st_mode) == 0o700
    ca_key = (home.pki / "ca.key").read_bytes()
    assert home.init(["localhost"]) == []
    assert (home.pki / "ca.key").read_bytes() == ca_key
    assert home.problems() == []


def test_run_directory_is_closed_to_other_users(tmp_path):
    home = Home.at(tmp_path / "home")
    home.run_dir.mkdir(parents=True, mode=0o755)
    home.run_dir.chmod(0o755)
    home.secure_run_dir()
    assert stat.S_IMODE(home.run_dir.stat().st_mode) == 0o700
    home.run_dir.rmdir()
    os.symlink(tmp_path, home.run_dir)
    with pytest.raises(PermissionError):
        home.secure_run_dir()


def test_hub_start_refuses_an_incomplete_home_and_says_how_to_fix_it(tmp_path, capsys):
    (tmp_path / "profiles").mkdir()
    assert main(["hub", "start", "--home", str(tmp_path)]) == 1
    err = capsys.readouterr().err
    assert "robots.yaml" in err and "hub.key" in err and "hub init" in err


def test_a_home_too_deep_for_a_unix_socket_is_reported(tmp_path):
    home = Home.at(tmp_path / ("d" * 120))
    assert any("too long for a Unix socket" in p for p in home.problems())


def test_commands_without_a_hub_say_none_is_running(tmp_path, capsys):
    assert main(["robots", "--home", str(tmp_path)]) == 1
    assert "no hub is running" in capsys.readouterr().err


def test_status_robots_goal_and_cancel_over_the_app_socket(running_hub, capsys):
    with AppClient(running_hub) as hub:
        st = hub.status()
        assert st.version == VERSION and st.home == str(running_hub.root)
        assert st.robots_registered == 1 and st.robots_connected == 0
        assert st.robot_address.startswith("127.0.0.1:") and not st.robot_address.endswith(":0")
        assert st.caller == str(local_caller())
        assert "rover-1: not connected (never heard)" in format_robots(hub.robots())
        ok = hub.goal("rover-1", 3.0, 1.5)
        assert ok.ok and ok.detail == "rover-1: goal 3 1.5 0, tolerance 0.4 m"
        assert hub.robots().robots[0].mission == "active"
        nobody = hub.goal("nobody", 1.0, 1.0)
        assert not nobody.ok and "no robot 'nobody'" in nobody.detail
        assert hub.cancel("rover-1").ok
        assert hub.robots().robots[0].mission == "cancelled"

    home = ["--home", str(running_hub.root)]
    assert main(["goal", "rover-1", "2", "1", "--tolerance", "0.3"] + home) == 0
    assert main(["cancel", "nobody"] + home) == 1
    assert main(["robots"] + home) == 0
    assert main(["status"] + home) == 0
    out, err = capsys.readouterr()
    assert "rover-1: goal 2 1 0, tolerance 0.3 m" in out
    assert "no robot 'nobody'" in err
    assert "rover-1: not connected" in out and f"Argus {VERSION} at {running_hub.root}" in out


def test_a_second_hub_on_the_same_home_is_refused_and_a_stale_socket_is_replaced(tmp_path):
    home = _home(tmp_path)
    loop = _Loop()
    try:
        first = AppServer(Hub(Registry.from_dir(home.root)), home)
        loop.run(first.start())
        second = AppServer(Hub(Registry.from_dir(home.root)), home)
        with pytest.raises(HubAlreadyRunning):
            loop.run(second.start())
        loop.run(first.stop())
        assert not home.socket.exists()

        stale = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        stale.bind(str(home.socket))
        stale.close()  # a socket file nothing listens on, as after a crash
        third = AppServer(Hub(Registry.from_dir(home.root)), home)
        loop.run(third.start())
        with AppClient(home) as client:
            assert client.status().robots_registered == 1
        loop.run(third.stop())
    finally:
        loop.close()


def test_watch_reports_a_robot_connecting_and_leaving(tmp_path):
    home = _home(tmp_path)
    registry = Registry.from_dir(home.root)
    hub = Hub(registry)
    loop = _Loop()
    app = AppServer(hub, home)
    loop.run(app.start())
    events = []
    client = AppClient(home)

    def watch():
        try:
            for e in client.watch():
                events.append(e)
        except HubUnavailable:
            pass

    watcher = threading.Thread(target=watch, daemon=True)
    watcher.start()
    time.sleep(0.3)  # the stream is open before anything happens

    p = registry.profile("example-planar-differential")
    hello = link_pb2.NodeMessage(seq=1)
    hello.hello.CopyFrom(session_pb2.Hello(node_id="rover-1", protocol_version=1, profile_id=p.id,
                                           profile_hash=p.hash))

    async def connect_then_leave():
        c = hub.connect("rover-1")
        c.receive(hello.SerializeToString(), time.time())
        await asyncio.sleep(0.4)
        c.close()
        await asyncio.sleep(0.4)

    loop.run(connect_then_leave())
    deadline = time.time() + 3
    while time.time() < deadline and not any(e.kind == "robot_disconnected" for e in events):
        time.sleep(0.05)
    client.close()
    watcher.join(3)
    loop.run(app.stop())
    loop.close()
    kinds = [(e.kind, e.node_id) for e in events]
    assert ("robot_connected", "rover-1") in kinds
    assert kinds.index(("robot_connected", "rover-1")) < kinds.index(("robot_disconnected", "rover-1"))


def test_the_hub_process_serves_the_command_and_stops_cleanly_on_sigterm(tmp_path):
    import signal
    import subprocess
    import sys

    home = _home(tmp_path)
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([str(ROOT / "python"), str(ROOT / "build" / "python")]))
    hub = subprocess.Popen([sys.executable, "-m", "argus", "hub", "start", "--home", str(home.root),
                            "--listen", "127.0.0.1:0"], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           text=True)
    try:
        deadline = time.time() + 15
        while not home.socket.exists() and time.time() < deadline and hub.poll() is None:
            time.sleep(0.1)
        robots = subprocess.run([sys.executable, "-m", "argus", "robots", "--home", str(home.root)], env=env,
                                capture_output=True, text=True, timeout=15)
        assert robots.returncode == 0, robots.stderr
        assert "rover-1: not connected" in robots.stdout
    finally:
        hub.send_signal(signal.SIGTERM)
        out, _ = hub.communicate(timeout=15)
    assert hub.returncode == 0, out
    assert "robots connect to 127.0.0.1:" in out and "hub stopped" in out
    assert not home.socket.exists()


def test_init_refuses_a_home_whose_socket_path_would_be_too_long(tmp_path, capsys):
    assert main(["hub", "init", "--home", str(tmp_path / ("d" * 120))]) == 1
    assert "too long for a Unix socket" in capsys.readouterr().err


def test_every_call_goes_through_one_authorisation_point():
    me = local_caller()
    for action in ("status", "robots", "watch", "goal", "cancel"):
        assert authorize(me, action) is None
    assert "could not be identified" in authorize(None, "status")
    assert "may not goal" in authorize(Caller("account", "visitor"), "goal")
    assert "unknown action" in authorize(me, "delete_everything")


def test_version_matches_the_package():
    meta = tomllib.loads((ROOT / "pyproject.toml").read_text())
    assert VERSION == meta["project"]["version"]
