"""Model connections and slots on the hub.

A connection is where models come from: an OpenAI-compatible endpoint (the OpenAI API, Gemini's
compatible endpoint, OpenRouter, or a local server such as Ollama or llama.cpp), the Anthropic API,
a person's ChatGPT plan, or the Claude Code program they have installed. A slot is what Argus uses
a model for, one model per slot. Every slot is optional: with nothing connected the hub works as it
always did, on the rule-based tier.

Connections and slots are kept in the hub's home so they survive a restart. Keys and tokens never
are: they live in this process only and are given again after a restart. Nothing here is reachable
from Argo, and no caller waits on a model while driving a robot.
"""

from __future__ import annotations

import base64
import json
import os
import urllib.error
import urllib.parse
import urllib.request
import uuid
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from argus.hub import chatgpt, claude_code
from argus.hub.modeltypes import OPENER, Image, Message, ModelError  # noqa: F401

OPENAI_COMPATIBLE = "openai-compatible"
ANTHROPIC = "anthropic"
CHATGPT = "chatgpt"
CLAUDE_CODE = "claude-code"
KINDS = (OPENAI_COMPATIBLE, ANTHROPIC, CHATGPT, CLAUDE_CODE)
ANTHROPIC_URL = "https://api.anthropic.com"
ANTHROPIC_VERSION = "2026-10-21"

REASONING = "reasoning"
VISION = "vision"
SLOTS = (REASONING, VISION)

MAX_OUTPUT_TOKENS = 16000
REQUEST_TIMEOUT_S = 240.0
LIST_TIMEOUT_S = 20.0
LOOPBACK = ("localhost", "127.0.0.1", "::1")


@dataclass
class Connection:
    name: str
    kind: str
    base_url: str
    uses_key: bool = False
    models: list[str] = field(default_factory=list)
    key: str = field(default="", repr=False)
    client_id: str = ""
    subject: str = ""
    tokens: chatgpt.Tokens | None = field(default=None, repr=False)

    @property
    def remote(self) -> bool:
        return urllib.parse.urlsplit(self.base_url).hostname not in LOOPBACK

    @property
    def needs_key(self) -> bool:
        if self.kind == CHATGPT:
            return self.tokens is None
        return self.uses_key and not self.key


def _post_or_get(url: str, headers: dict, body: dict | None, timeout: float) -> dict:
    data = None if body is None else json.dumps(body).encode()
    request = urllib.request.Request(url, data=data, headers={"content-type": "application/json", **headers})
    try:
        with OPENER.open(request, timeout=timeout) as reply:
            return json.loads(reply.read())
    except urllib.error.HTTPError as e:
        raise ModelError(_http_problem(e.code, e.read())) from None
    except json.JSONDecodeError:
        raise ModelError(f"{url} did not answer in JSON; is it the right address?") from None
    except (urllib.error.URLError, OSError) as e:
        reason = getattr(e, "reason", e)
        raise ModelError(f"could not reach {url}: {reason}") from None


def _http_problem(status: int, body: bytes) -> str:
    said = ""
    try:
        error = json.loads(body).get("error")
        said = error.get("message", "") if isinstance(error, dict) else str(error or "")
    except (ValueError, AttributeError):
        pass
    said = f" ({said[:200]})" if said else ""
    if status in (401, 403):
        return f"the key was refused{said}"
    if 300 <= status < 400:
        return f"the address redirected elsewhere, which Argus does not follow with a key (HTTP {status})"
    if status == 402:
        return f"the account has no credit left{said}"
    if status == 404:
        return f"no such model or address{said}"
    if status == 429:
        return f"rate limited or out of quota; try again later{said}"
    if status >= 500:
        return f"the provider is having trouble (HTTP {status}); try again later{said}"
    return f"the request was refused (HTTP {status}){said}"


def _openai_content(m: Message):
    if not m.images:
        return m.text
    parts: list[dict] = [{"type": "text", "text": m.text}]
    for image in m.images:
        url = f"data:{image.media_type};base64,{base64.b64encode(image.data).decode()}"
        parts.append({"type": "image_url", "image_url": {"url": url}})
    return parts


def _anthropic_content(m: Message):
    if not m.images:
        return m.text
    parts: list[dict] = [{"type": "image", "source": {"type": "base64", "media_type": image.media_type,
                                                      "data": base64.b64encode(image.data).decode()}}
                         for image in m.images]
    parts.append({"type": "text", "text": m.text})
    return parts


def list_models(c: Connection) -> list[str]:
    if c.kind == ANTHROPIC:
        return _anthropic_models(c)
    if c.kind == CHATGPT:
        return chatgpt.list_models(chatgpt.access_token(c))
    if c.kind == CLAUDE_CODE:
        return []  # the program takes a name or an alias and says if it does not know it
    headers = {"authorization": f"Bearer {c.key}"} if c.key else {}
    reply = _post_or_get(f"{c.base_url}/models", headers, None, LIST_TIMEOUT_S)
    return sorted(m["id"] for m in reply.get("data", []) if "id" in m)


def _anthropic_headers(c: Connection) -> dict:
    return {"x-api-key": c.key, "anthropic-version": ANTHROPIC_VERSION}


def _anthropic_models(c: Connection) -> list[str]:
    found: list[str] = []
    after = ""
    while True:
        query = "?limit=100" + (f"&after_id={urllib.parse.quote(after)}" if after else "")
        reply = _post_or_get(f"{c.base_url}/v1/models{query}", _anthropic_headers(c), None, LIST_TIMEOUT_S)
        found += [m["id"] for m in reply.get("data", []) if "id" in m]
        after = reply.get("last_id") or ""
        if not reply.get("has_more") or not after:
            return sorted(found)


def complete(c: Connection, model: str, system: str, messages: list[Message],
             timeout: float = REQUEST_TIMEOUT_S) -> str:
    """The model's text for this conversation. Raises ModelError."""
    if c.kind == CHATGPT:
        return chatgpt.complete(chatgpt.access_token(c), model, system, messages, timeout)
    if c.kind == CLAUDE_CODE:
        return claude_code.complete(model, system, messages, timeout)
    if c.needs_key:
        raise ModelError(f"{c.name} needs its key again after the hub restarted: argus models connect")
    if c.kind == ANTHROPIC:
        reply = _post_or_get(f"{c.base_url}/v1/messages", _anthropic_headers(c),
                             {"model": model, "max_tokens": MAX_OUTPUT_TOKENS, "system": system,
                              "messages": [{"role": m.role, "content": _anthropic_content(m)} for m in messages]},
                             timeout)
        return _anthropic_text(reply)
    headers = {"authorization": f"Bearer {c.key}"} if c.key else {}
    turns = [{"role": "system", "content": system}] if system else []
    turns += [{"role": m.role, "content": _openai_content(m)} for m in messages]
    reply = _post_or_get(f"{c.base_url}/chat/completions", headers, {"model": model, "messages": turns}, timeout)
    return _openai_text(reply)


def _anthropic_text(reply: dict) -> str:
    stop = reply.get("stop_reason")
    if stop == "refusal":
        raise ModelError("the model declined this request")
    text = "".join(b.get("text", "") for b in reply.get("content", []) if b.get("type") == "text")
    if not text:
        raise ModelError(f"the model returned no text (stopped for {stop})")
    if stop == "max_tokens":
        text += "\n[cut off: the model reached its output limit]"
    return text


def _openai_text(reply: dict) -> str:
    choices = reply.get("choices") or []
    if not choices:
        raise ModelError("the model returned no answer")
    text = (choices[0].get("message") or {}).get("content") or ""
    if not text:
        raise ModelError(f"the model returned no text (stopped for {choices[0].get('finish_reason')})")
    if choices[0].get("finish_reason") == "length":
        text += "\n[cut off: the model reached its output limit]"
    return text


def check_address(kind: str, url: str, key: str) -> str:
    """The base URL to use, or raises ValueError. A key is never sent in the clear to another
    machine."""
    if kind not in KINDS:
        raise ValueError(f"unknown kind {kind!r}; choose from {', '.join(KINDS)}")
    if kind in (CHATGPT, CLAUDE_CODE):
        if url or key:
            raise ValueError(f"{kind} takes no address and no key")
        return chatgpt.API if kind == CHATGPT else ""
    url = (url or (ANTHROPIC_URL if kind == ANTHROPIC else "")).rstrip("/")
    parts = urllib.parse.urlsplit(url)
    if parts.scheme not in ("http", "https") or not parts.hostname:
        raise ValueError(f"{url!r} is not an http or https address")
    if kind == ANTHROPIC and not key:
        raise ValueError("the Anthropic API needs a key")
    if key and parts.scheme == "http" and parts.hostname not in LOOPBACK:
        raise ValueError(f"refusing to send a key to {parts.hostname} without https")
    return url


class Models:
    """The hub's connections and slots. Network calls block: callers on the event loop use a
    thread."""

    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self.connections: dict[str, Connection] = {}
        self.slots: dict[str, tuple[str, str]] = {}
        self.host_id = ""  # opaque, says nothing about the person
        self.registrations: dict[str, dict] = {}  # ChatGPT clients by connection name, kept past a sign-out
        self._sign_in: tuple[str, chatgpt.SignIn] | None = None
        self._load()

    def _load(self) -> None:
        if not self.path.exists():
            return
        doc = yaml.safe_load(self.path.read_text()) or {}
        self.host_id = doc.get("host_id", "")
        self.registrations = doc.get("registrations") or {}
        for c in doc.get("connections", []):
            mine = self.registrations.get(c["name"], {}) if c["kind"] == CHATGPT else {}
            self.connections[c["name"]] = Connection(c["name"], c["kind"], c["base_url"], c.get("uses_key", False),
                                                     client_id=mine.get("client_id", ""),
                                                     subject=mine.get("subject", ""))
        for slot, use in (doc.get("slots") or {}).items():
            self.slots[slot] = (use["connection"], use["model"])

    def _save(self) -> None:
        doc = {"connections": [{"name": c.name, "kind": c.kind, "base_url": c.base_url, "uses_key": c.uses_key}
                               for c in self.connections.values()],
               "slots": {slot: {"connection": name, "model": model} for slot, (name, model) in self.slots.items()}}
        if self.host_id:
            doc |= {"host_id": self.host_id, "registrations": self.registrations}
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".tmp")
        fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, "w") as f:
            yaml.safe_dump(doc, f, sort_keys=False)
        temporary.replace(self.path)

    def connect(self, name: str, kind: str, url: str = "", key: str = "") -> Connection:
        """Adds a connection, or gives one its key again, once its model list has answered."""
        base_url = check_address(kind, url, key)
        if kind == CHATGPT:
            raise ValueError("a ChatGPT plan connects by signing in: argus models connect chatgpt")
        if kind == CLAUDE_CODE:
            claude_code.check()
        c = Connection(name, kind, base_url, bool(key), key=key)
        try:
            c.models = list_models(c)
        except ModelError as e:
            raise ValueError(str(e)) from None
        self.connections[name] = c
        self._save()
        return c

    def begin_sign_in(self, name: str) -> str:
        """Opens a sign-in and gives the address for the browser. A new attempt replaces one left
        waiting, which is what a person starting over wants."""
        self.cancel_sign_in()
        if not self.host_id:
            self.host_id = f"urn:uuid:{uuid.uuid4()}"
            self._save()
        known = self.registrations.get(name, {})
        sign_in = chatgpt.SignIn(self.host_id, known.get("client_id", ""), known.get("subject", ""))
        self._sign_in = (name, sign_in)
        return sign_in.url

    def cancel_sign_in(self) -> None:
        if self._sign_in is not None:
            self._sign_in[1].close()
            self._sign_in = None

    def finish_sign_in(self, name: str, timeout: float = chatgpt.SIGN_IN_TIMEOUT_S) -> Connection:
        if self._sign_in is None or self._sign_in[0] != name:
            raise ValueError(f"no sign-in is waiting for {name!r}")
        _, sign_in = self._sign_in

        def registered(client_id: str) -> None:
            self.registrations[name] = {"client_id": client_id,
                                        "subject": self.registrations.get(name, {}).get("subject", "")}
            self._save()

        try:
            done = sign_in.finish(registered, timeout)
        finally:
            if self._sign_in is not None and self._sign_in[1] is sign_in:
                self._sign_in = None
        c = Connection(name, CHATGPT, chatgpt.API, client_id=done.client_id, subject=done.subject, tokens=done.tokens)
        try:
            c.models = list_models(c)
        except ModelError as e:
            raise ValueError(str(e)) from None
        self.registrations[name] = {"client_id": done.client_id, "subject": done.subject}
        self.connections[name] = c
        self._save()
        return c

    def disconnect(self, name: str) -> str:
        """Removes a connection and returns anything the user should know about it. A ChatGPT
        registration is kept, so the same account signs back in as the same client."""
        if name not in self.connections:
            raise ValueError(f"no connection {name!r}")
        c = self.connections.pop(name)
        self.slots = {s: use for s, use in self.slots.items() if use[0] != name}
        self._save()
        if c.tokens is not None and c.tokens.refresh and not chatgpt.revoke(c.client_id, c.tokens):
            return ("forgotten here, but OpenAI did not confirm the sign-in ended; "
                    f"you can end it at {chatgpt.USAGE_PAGE}")
        return ""

    def use(self, slot: str, name: str, model: str) -> None:
        if slot not in SLOTS:
            raise ValueError(f"unknown slot {slot!r}; choose from {', '.join(SLOTS)}")
        c = self.connections.get(name)
        if c is None:
            raise ValueError(f"no connection {name!r}")
        if c.models and model not in c.models:
            raise ValueError(f"{name} has no model {model!r} (argus models list shows what it has)")
        self.slots[slot] = (name, model)
        self._save()

    def filled(self, slot: str) -> bool:
        return slot in self.slots

    def describe(self, slot: str) -> str:
        name, model = self.slots[slot]
        return f"{name}/{model}"

    def ask(self, slot: str, system: str, messages: list[Message]) -> str:
        if slot not in self.slots:
            raise ModelError(f"nothing is in the {slot} slot (argus models use {slot} CONNECTION/MODEL)")
        name, model = self.slots[slot]
        return complete(self.connections[name], model, system, messages)
