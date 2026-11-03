"""Running the Claude Code program a person has installed and signed in to themselves.

Argus starts the unmodified program and reads its answer. Sign-in stays with that program: Argus
never asks for, reads, stores or passes on its credentials, and offers no way to sign in to it.

Prompts carry run logs, which are data from outside, so the program runs with no tools, no
settings, no plugins or project files, in an empty directory. If it reports any tool at start-up
the run is stopped before it gets further.
"""

from __future__ import annotations

import base64
import json
import os
import shutil
import signal
import subprocess
import tempfile
import threading
from pathlib import Path

from argus.hub.modeltypes import Message, ModelError

DEFAULT_MODEL = "default"  # whatever the person's Claude Code uses
NO_INSTRUCTIONS = "Answer the request."  # an empty one would bring back the program's own
ALLOWED_AT_START = {"EndConversation"}
MAX_OUTPUT_BYTES = 8 * 1024 * 1024
STATUS_TIMEOUT_S = 15.0
NOT_FINISHED = "Claude Code did not finish; check `claude auth status`, its usage limits, and that it is up to date"


def find() -> str | None:
    return shutil.which("claude") or shutil.which("claude", path=str(Path.home() / ".local" / "bin"))


def check() -> None:
    """Raises ValueError unless Claude Code is installed and says it is signed in. Asks the
    program; only the yes or no is kept from its answer."""
    binary = find()
    if binary is None:
        raise ValueError("Claude Code is not installed on this machine")
    try:
        done = subprocess.run([binary, "auth", "status"], capture_output=True, timeout=STATUS_TIMEOUT_S)
        signed_in = json.loads(done.stdout).get("loggedIn") is True
    except (OSError, subprocess.TimeoutExpired, ValueError, AttributeError):
        raise ValueError("Claude Code did not answer; check its installation") from None
    if not signed_in:
        raise ValueError("Claude Code is not signed in; run `claude` and sign in, then connect again")


def arguments(model: str, system: str) -> list[str]:
    args = ["--print", "--input-format", "stream-json", "--output-format", "stream-json", "--verbose",
            "--restricted", "--tools", "", "--disable-slash-commands", "--strict-mcp-config", "--no-chrome",
            "--no-session-persistence", "--setting-sources", "",
            "--settings", json.dumps({"disableAllHooks": True, "autoMemoryEnabled": False}),
            "--system-prompt", system or NO_INSTRUCTIONS, "--permission-mode", "dontAsk", "--max-turns", "1"]
    return args if model == DEFAULT_MODEL else [*args, "--model", model]


def _request(messages: list[Message]) -> bytes:
    text = messages[0].text if len(messages) == 1 else "\n\n".join(f"{m.role}: {m.text}" for m in messages)
    content: list[dict] = [{"type": "text", "text": text}]
    for m in messages:
        content += [{"type": "image", "source": {"type": "base64", "media_type": image.media_type,
                                                 "data": base64.b64encode(image.data).decode()}}
                    for image in m.images]
    return (json.dumps({"type": "user", "message": {"role": "user", "content": content}}) + "\n").encode()


def _stop(child: subprocess.Popen) -> None:
    try:
        os.killpg(child.pid, signal.SIGKILL)
    except (ProcessLookupError, PermissionError):
        pass


def _feed(child: subprocess.Popen, request: bytes) -> None:
    try:
        child.stdin.write(request)
        child.stdin.close()
    except OSError:
        pass  # it exited early; the reader reports that


def _read(child: subprocess.Popen) -> str:
    read = 0
    for line in child.stdout:
        read += len(line)
        if read > MAX_OUTPUT_BYTES:
            raise ModelError("Claude Code wrote more than expected; stopped")
        try:
            event = json.loads(line)
        except ValueError:
            continue
        kind = event.get("type")
        if kind == "system" and event.get("subtype") == "init":
            tools = event.get("tools")
            if isinstance(tools, list) and not ALLOWED_AT_START.issuperset(map(str, tools)):
                raise ModelError("Claude Code started with tools it was told not to load, so the run was "
                                 "stopped; check its managed settings")
        elif kind == "result":
            answer = event.get("result")
            if event.get("is_error") or event.get("subtype") != "success":
                said = answer.strip()[:200] if isinstance(answer, str) else ""
                raise ModelError(f"Claude Code could not answer: {said}" if said else NOT_FINISHED)
            if isinstance(answer, str) and answer:
                return answer
            raise ModelError("the model returned no text")
    raise ModelError(NOT_FINISHED)


def complete(model: str, system: str, messages: list[Message], timeout: float) -> str:
    """One answer from the installed program. Diagnostics on stderr are dropped unread."""
    binary = find()
    if binary is None:
        raise ModelError("Claude Code is no longer installed on this machine")
    with tempfile.TemporaryDirectory(prefix="argus-model-") as empty:
        try:
            child = subprocess.Popen([binary, *arguments(model, system)], cwd=empty, stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, start_new_session=True)
        except OSError as e:
            raise ModelError(f"could not start Claude Code: {e.strerror}") from None
        expired = threading.Event()

        def out_of_time() -> None:
            expired.set()
            _stop(child)

        timer = threading.Timer(timeout, out_of_time)
        timer.daemon = True
        timer.start()
        threading.Thread(target=_feed, args=(child, _request(messages)), daemon=True).start()
        try:
            return _read(child)
        except ModelError:
            if expired.is_set():
                raise ModelError(f"Claude Code took longer than {timeout:.0f} s; stopped") from None
            raise
        finally:
            timer.cancel()
            _stop(child)
            child.wait()
            child.stdout.close()
