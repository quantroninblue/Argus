"""The hub's home directory: everything a hub keeps between runs.

    profiles/        body profiles (*.yaml)
    robots.yaml      node_id: {profile, site} for every robot allowed to connect
    pki/             ca.pem, ca.key, hub.pem, hub.key (robots' certificates are issued here)
    runs/            run logs
    run/app.sock     the app interface; run/ is readable only by the hub's owner

`~/.local/share/argus` unless `--home` or ARGUS_HOME says otherwise.
"""

from __future__ import annotations

import os
import stat
from dataclasses import dataclass
from pathlib import Path

from argus import certs

DEFAULT_HOME = "~/.local/share/argus"
# A Unix socket path must fit in sockaddr_un (108 bytes with the terminating zero).
MAX_SOCKET_PATH = 107


def default_home() -> Path:
    return Path(os.environ.get("ARGUS_HOME") or DEFAULT_HOME).expanduser()


@dataclass(frozen=True)
class Home:
    root: Path

    @classmethod
    def at(cls, path: str | Path | None = None) -> "Home":
        return cls(Path(path).expanduser().resolve() if path else default_home().resolve())

    @property
    def profiles(self) -> Path:
        return self.root / "profiles"

    @property
    def robots_file(self) -> Path:
        return self.root / "robots.yaml"

    @property
    def pki(self) -> Path:
        return self.root / "pki"

    @property
    def runs(self) -> Path:
        return self.root / "runs"

    @property
    def run_dir(self) -> Path:
        return self.root / "run"

    @property
    def socket(self) -> Path:
        return self.run_dir / "app.sock"

    def init(self, hosts: list[str]) -> list[str]:
        """Creates whatever is missing; returns what was made. Existing keys are never replaced."""
        made = []
        for d in (self.root, self.profiles, self.pki, self.runs):
            if not d.is_dir():
                d.mkdir(parents=True)
                made.append(f"{d}/")
        # openssl writes keys with the process umask; the CA and hub keys stay private by
        # keeping their directory closed to other users.
        self.pki.chmod(0o700)
        self.secure_run_dir()
        if not self.robots_file.exists():
            self.robots_file.write_text("# node_id: {profile: PROFILE_ID, site: SITE_ID}\n")
            made.append(str(self.robots_file))
        if not (self.pki / "ca.key").exists():
            certs.make_ca(self.pki)
            made.append(f"{self.pki / 'ca.pem'} and ca.key")
        if not (self.pki / "hub.key").exists():
            certs.make_hub(self.pki, hosts)
            made.append(f"{self.pki / 'hub.pem'} and hub.key, for {', '.join(hosts)}")
        for leftover in ("hub.csr", "hub.ext"):
            (self.pki / leftover).unlink(missing_ok=True)
        return made

    def problems(self) -> list[str]:
        """What stops a hub from starting here, in words a user can act on."""
        missing = [str(p) for p in (self.robots_file, self.pki / "ca.pem", self.pki / "hub.pem", self.pki / "hub.key")
                   if not p.exists()]
        if not self.profiles.is_dir():
            missing.append(f"{self.profiles}/")
        out = []
        if missing:
            out.append(f"missing {', '.join(missing)}; run: python3 -m argus hub init --home {self.root}")
        if len(str(self.socket)) > MAX_SOCKET_PATH:
            out.append(f"{self.socket} is too long for a Unix socket ({len(str(self.socket))} characters, "
                       f"{MAX_SOCKET_PATH} at most); choose a shorter --home")
        return out

    def secure_run_dir(self) -> None:
        """run/ exists, belongs to this user and is closed to everyone else, so only the hub's
        owner can reach the app socket inside it. Refuses a directory owned by someone else."""
        d = self.run_dir
        d.mkdir(mode=0o700, parents=True, exist_ok=True)
        st = d.lstat()
        if stat.S_ISLNK(st.st_mode) or not stat.S_ISDIR(st.st_mode):
            raise PermissionError(f"{d} is not a directory")
        if st.st_uid != os.getuid():
            raise PermissionError(f"{d} belongs to another user")
        if stat.S_IMODE(st.st_mode) != 0o700:
            d.chmod(0o700)
