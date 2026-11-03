"""`argus hub init|start`: set up a hub's home and run the hub.

The hub serves robots on HubLink (mutual TLS, `--listen`) and the argus command on its app
socket (see app.py). It runs until Ctrl-C or SIGTERM. Robots connect with certificates signed
by the hub's CA; a robot's certificate name must be its node id.
"""

from __future__ import annotations

import asyncio
import signal
import sys

from .app import AppServer, HubAlreadyRunning
from .gateway import Hub
from .home import Home
from .intake import IntakeConfig, parse_rate
from .places import site_heights
from .registry import Registry
from .server import HubServer, TlsFiles
from .session import HubConfig
from .site import SiteService

DEFAULT_HOSTS = ["localhost", "127.0.0.1"]


async def serve(home: Home, listen: str, intake_bytes_per_s: float | None = None,
                stop: asyncio.Event | None = None, ready: asyncio.Event | None = None) -> None:
    """Runs until `stop` is set (or forever). `ready` is set once both servers accept calls."""
    registry = Registry.from_dir(home.root)
    config = HubConfig()
    if intake_bytes_per_s is not None:
        config.intake = IntakeConfig(bytes_per_s=intake_bytes_per_s)
    sites = SiteService()
    for site, (floor, ceiling) in site_heights(home.sites).items():
        sites.set_heights(site, floor, ceiling)
    hub = Hub(registry, sites=sites, config=config, sites_dir=home.sites)
    robots = HubServer(hub, address=listen,
                       tls=TlsFiles(home.pki / "ca.pem", home.pki / "hub.pem", home.pki / "hub.key"))
    port = await robots.start()
    address = f"{listen.rsplit(':', 1)[0]}:{port}"
    app = AppServer(hub, home, robot_address=address)
    try:
        await app.start()
    except BaseException:
        await robots.stop()
        raise
    print(f"Argus hub at {home.root}: robots connect to {address}, {len(registry.robots())} registered",
          flush=True)
    if ready is not None:
        ready.set()
    try:
        await (stop or asyncio.Event()).wait()
    finally:
        await app.stop()
        await robots.stop()
        print("hub stopped", flush=True)


def start(home_dir: str | None, listen: str, intake: str | None = None) -> int:
    home = Home.at(home_dir)
    problems = home.problems()
    if problems:
        print("\n".join(problems), file=sys.stderr)
        return 1
    try:
        rate = parse_rate(intake) if intake else None
    except ValueError as e:
        print(e, file=sys.stderr)
        return 2

    async def run() -> None:
        stop = asyncio.Event()
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, stop.set)
        await serve(home, listen, rate, stop)

    try:
        asyncio.run(run())
    except (HubAlreadyRunning, PermissionError) as e:
        print(e, file=sys.stderr)
        return 1
    return 0


def init(home_dir: str | None, hosts: list[str] | None) -> int:
    home = Home.at(home_dir)
    try:
        made = home.init(hosts or DEFAULT_HOSTS)
    except PermissionError as e:
        print(e, file=sys.stderr)
        return 1
    for m in made:
        print(f"made {m}")
    problems = home.problems()
    if problems:
        print("\n".join(problems), file=sys.stderr)
        return 1
    print(f"hub home ready at {home.root}" if made else f"nothing to do: {home.root} is complete")
    return 0
