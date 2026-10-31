"""Logs of simulated runs, held for the hub's models to read.

Each run log is a directory under the store's root. Reading one with `consume` returns its
contents and deletes it, unless the run is marked to keep (`keep`), so logs only pile up when
someone asked for that.
"""

from __future__ import annotations

import re
import shutil
import tempfile
import time
import uuid
from pathlib import Path

KEEP_MARKER = "KEEP"
# Text files are returned whole up to this size; anything bigger, or binary (a ROS 2 bag), is
# returned as its path.
MAX_TEXT_BYTES = 4 * 1024 * 1024
_RUN_ID = re.compile(r"^[A-Za-z0-9_.-]+$")


class RunLogStore:
    def __init__(self, root: str | Path | None = None) -> None:
        self.root = Path(root) if root else Path(tempfile.gettempdir()) / "argus-run-logs"
        self.root.mkdir(parents=True, exist_ok=True)

    def path(self, run_id: str) -> Path:
        if not _RUN_ID.match(run_id) or run_id in (".", ".."):
            raise ValueError(f"not a run id: {run_id!r}")
        p = self.root / run_id
        if not p.is_dir():
            raise KeyError(run_id)
        return p

    def add(self, log_dir: str | Path, keep: bool = False, run_id: str | None = None) -> str:
        """Moves `log_dir` into the store and returns its run id."""
        run_id = run_id or f"{time.strftime('%Y%m%d-%H%M%S')}-{uuid.uuid4().hex[:6]}"
        if not _RUN_ID.match(run_id) or (self.root / run_id).exists():
            raise ValueError(f"run id {run_id!r} is invalid or taken")
        shutil.move(str(log_dir), str(self.root / run_id))
        if keep:
            self.keep(run_id)
        return run_id

    def ids(self) -> list[str]:
        return sorted(p.name for p in self.root.iterdir() if p.is_dir())

    def keep(self, run_id: str, keep: bool = True) -> None:
        marker = self.path(run_id) / KEEP_MARKER
        if keep:
            marker.touch()
        else:
            marker.unlink(missing_ok=True)

    def is_kept(self, run_id: str) -> bool:
        return (self.path(run_id) / KEEP_MARKER).exists()

    def read(self, run_id: str) -> dict[str, str]:
        """Every file in the run log by relative path: text files with their contents, anything
        else (and directories such as a bag) as "<path>" pointing at it on disk."""
        base = self.path(run_id)
        out: dict[str, str] = {}
        for p in sorted(base.iterdir()):
            if p.name == KEEP_MARKER:
                continue
            if p.is_file() and p.stat().st_size <= MAX_TEXT_BYTES:
                try:
                    out[p.name] = p.read_text()
                    continue
                except UnicodeDecodeError:
                    pass
            out[p.name] = f"<{p}>"
        return out

    def consume(self, run_id: str) -> dict[str, str]:
        """`read`, then delete the run unless it is kept. Paths in the result for a deleted run
        no longer exist."""
        contents = self.read(run_id)
        if not self.is_kept(run_id):
            self.delete(run_id)
        return contents

    def delete(self, run_id: str) -> None:
        shutil.rmtree(self.path(run_id))
