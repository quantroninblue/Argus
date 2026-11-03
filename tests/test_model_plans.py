"""Connections that use a person's own plan: ChatGPT sign-in and the installed Claude Code program.
Uses a stand-in for OpenAI's sign-in and Responses services and a stand-in `claude` program: no
network, no account."""

import json
import os
import stat
import threading
import time
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import jwt
import pytest
from cryptography.hazmat.primitives.asymmetric import rsa

from argus.__main__ import main
from argus.hub import chatgpt, claude_code, models
from argus.hub.modeltypes import Image, Message, ModelError
from test_hub_app import _Loop, running_hub  # noqa: F401

CLIENT = "oaiapp_issued"
ACCESS = "access-token-do-not-leak"
REFRESH = "refresh-token-do-not-leak"


class OpenAI:
    """Sign-in and Responses, as far as a sign-in and a request go."""

    def __init__(self):
        stand = self
        self.key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
        self.other_key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
        self.forms = []
        self.responses = []
        self.events = []
        self.status = 200
        self.error = None
        self.scope = chatgpt.SCOPES
        self.subject = "user-1"
        self.nonce = None
        self.audience = None
        self.issuer = None
        self.lifetime = 3600
        self.signing_key = self.key
        self.token_error = None
        self.revoke_status = 200
        self.serial = 0

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def _send(self, status, body, extra=()):
                data = body if isinstance(body, bytes) else json.dumps(body).encode()
                self.send_response(status)
                self.send_header("content-length", str(len(data)))
                for k, v in extra:
                    self.send_header(k, v)
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                if self.path == "/.well-known/openid-configuration":
                    self._send(200, stand.discovery())
                elif self.path == "/jwks.json":
                    public = json.loads(jwt.algorithms.RSAAlgorithm.to_jwk(stand.key.public_key()))
                    self._send(200, {"keys": [public | {"kid": "k1", "use": "sig", "alg": "RS256"}]})
                elif self.path == "/v1/models":
                    assert self.headers["authorization"] == f"Bearer {stand.current_access()}"
                    self._send(200, {"models": [{"slug": "plan-big", "visibility": "list"},
                                                {"slug": "plan-small", "visibility": "list"},
                                                {"slug": "hidden", "visibility": "hide"}]})
                else:
                    self._send(404, {})

            def do_POST(self):
                body = self.rfile.read(int(self.headers["content-length"]))
                if self.path == "/v1/responses":
                    stand.responses.append((dict(self.headers.items()), json.loads(body)))
                    if stand.status != 200:
                        self._send(stand.status, {"error": stand.error}, [("x-request-id", "req_1")])
                    else:
                        lines = "".join(f"event: {e.get('type')}\ndata: {json.dumps(e)}\n\n" for e in stand.events)
                        self._send(200, lines.encode(), [("content-type", "text/event-stream")])
                    return
                form = {k: v[0] for k, v in urllib.parse.parse_qs(body.decode()).items()}
                stand.forms.append((self.path, form))
                if self.path == "/api/accounts/oauth/revoke":
                    self._send(stand.revoke_status, b"")
                elif stand.token_error:
                    self._send(400, {"error": stand.token_error})
                else:
                    self._send(200, stand.token_reply(form))

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.url = f"http://127.0.0.1:{self.server.server_port}"
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def discovery(self):
        return {"issuer": self.url, "authorization_endpoint": f"{self.url}/api/accounts/authorize",
                "token_endpoint": f"{self.url}/api/accounts/oauth/token", "jwks_uri": f"{self.url}/jwks.json",
                "revocation_endpoint": f"{self.url}/api/accounts/oauth/revoke"}

    def current_access(self):
        return f"{ACCESS}-{self.serial}"

    def id_token(self, client_id, nonce):
        now = time.time()
        return jwt.encode({"iss": self.issuer or self.url, "aud": self.audience or client_id, "sub": self.subject,
                           "iat": int(now) - 7200, "exp": int(now) + self.lifetime, "nonce": self.nonce or nonce},
                          self.signing_key, algorithm="RS256", headers={"kid": "k1"})

    def token_reply(self, form):
        self.serial += 1
        reply = {"access_token": self.current_access(), "refresh_token": f"{REFRESH}-{self.serial}",
                 "token_type": "Bearer", "expires_in": 3600, "scope": self.scope}
        if form["grant_type"] == "authorization_code":
            reply["id_token"] = self.id_token(form["client_id"], self.sent_nonce)
        return reply

    def browser(self, url, client_id=CLIENT, **overrides):
        """Does what the person's browser does: signs in and lands on the callback."""
        query = {k: v[0] for k, v in urllib.parse.parse_qs(urllib.parse.urlsplit(url).query).items()}
        self.sent_nonce = query["nonce"]
        answer = {"code": "the-code", "state": query["state"]}
        if client_id:
            answer["client_id"] = client_id
        answer |= overrides
        answer = {k: v for k, v in answer.items() if v is not None}
        try:
            with urllib.request.urlopen(f"{query['redirect_uri']}?{urllib.parse.urlencode(answer)}") as reply:
                return reply.status
        except urllib.error.HTTPError as e:
            return e.code


@pytest.fixture
def openai(monkeypatch):
    stand = OpenAI()
    monkeypatch.setattr(chatgpt, "ISSUER", stand.url)
    monkeypatch.setattr(chatgpt, "API", stand.url + "/v1")
    yield stand
    stand.server.shutdown()
    stand.server.server_close()


@pytest.fixture
def plans(tmp_path):
    return models.Models(tmp_path / "models.yaml")


def _sign_in(plans, openai, name="plan", **browser):
    url = plans.begin_sign_in(name)
    openai.browser(url, **browser)
    return plans.finish_sign_in(name, timeout=2.0)


def _reply(text="ready"):
    return [{"type": "response.output_text.delta", "delta": text[:2]},
            {"type": "response.output_text.delta", "delta": text[2:]},
            {"type": "response.completed", "response": {}}]


def test_a_first_sign_in_registers_this_hub_and_asks_for_plan_use(plans, openai):
    url = plans.begin_sign_in("plan")
    query = {k: v[0] for k, v in urllib.parse.parse_qs(urllib.parse.urlsplit(url).query).items()}
    assert query["client_id"] == "dynamic_agent_client" and query["agent_name_hint"] == "Argus"
    assert query["ext_agent_host_id"].startswith("urn:uuid:")
    assert query["scope"].split()[-1] == "chatgpt.tokens.use.direct"
    assert query["code_challenge_method"] == "S256" and query["resource"] == chatgpt.RESOURCE
    assert query["redirect_uri"].startswith("http://127.0.0.1:") and query["redirect_uri"].endswith("/auth/callback")
    openai.browser(url)
    c = plans.finish_sign_in("plan", timeout=2.0)
    assert c.client_id == CLIENT and c.subject == "user-1" and c.models == ["plan-big", "plan-small"]
    _, form = openai.forms[0]
    assert form["grant_type"] == "authorization_code" and form["client_id"] == CLIENT
    assert form["redirect_uri"] == query["redirect_uri"]


def test_no_token_reaches_the_disk(plans, openai, tmp_path):
    _sign_in(plans, openai)
    plans.use(models.REASONING, "plan", "plan-big")
    saved = (tmp_path / "models.yaml").read_text()
    assert ACCESS not in saved and REFRESH not in saved and "user@" not in saved
    assert CLIENT in saved
    assert stat.S_IMODE((tmp_path / "models.yaml").stat().st_mode) == 0o600
    assert ACCESS not in repr(plans.connections["plan"])


def test_a_restart_asks_for_a_new_sign_in_as_the_same_client(plans, openai, tmp_path):
    _sign_in(plans, openai)
    host = plans.host_id
    plans.use(models.REASONING, "plan", "plan-big")
    again = models.Models(tmp_path / "models.yaml")
    assert again.connections["plan"].needs_key and again.filled(models.REASONING) and again.host_id == host
    with pytest.raises(ModelError, match="signing in again"):
        again.ask(models.REASONING, "", [Message("user", "hello")])
    url = again.begin_sign_in("plan")
    query = urllib.parse.parse_qs(urllib.parse.urlsplit(url).query)
    assert query["client_id"] == [CLIENT] and "agent_name_hint" not in query
    assert query["ext_agent_host_id"] == [host]
    openai.browser(url, client_id=None)
    assert again.finish_sign_in("plan", timeout=2.0).tokens is not None


def test_a_callback_without_the_right_state_changes_nothing(plans, openai):
    url = plans.begin_sign_in("plan")
    assert openai.browser(url, state="forged") == 400
    assert openai.browser(url) == 200
    assert plans.finish_sign_in("plan", timeout=2.0).client_id == CLIENT


def test_a_declined_sign_in_stops_there(plans, openai):
    url = plans.begin_sign_in("plan")
    openai.browser(url, code=None, client_id=None, error="access_denied")
    with pytest.raises(ValueError, match="declined"):
        plans.finish_sign_in("plan", timeout=2.0)
    assert not openai.forms and "plan" not in plans.connections


@pytest.mark.parametrize("change", [{"nonce": "replayed"}, {"audience": "someone-else"}, {"signed_by_other": True},
                                    {"issuer": "https://elsewhere.example"}, {"lifetime": -3600}])
def test_an_id_token_that_does_not_check_out_is_refused(plans, openai, change):
    openai.nonce = change.get("nonce")
    openai.audience = change.get("audience")
    openai.issuer = change.get("issuer")
    openai.lifetime = change.get("lifetime", 3600)
    if "signed_by_other" in change:
        openai.signing_key = openai.other_key
    with pytest.raises(ValueError, match="could not be verified"):
        _sign_in(plans, openai)
    assert "plan" not in plans.connections


def test_a_sign_in_without_plan_use_is_refused_and_the_next_one_asks_for_consent(plans, openai):
    openai.scope = "openid profile email"
    with pytest.raises(ValueError, match="plan use was not allowed"):
        _sign_in(plans, openai)
    assert "plan" not in plans.connections
    url = plans.begin_sign_in("plan")
    query = urllib.parse.parse_qs(urllib.parse.urlsplit(url).query)
    assert query["client_id"] == [CLIENT] and query["prompt"] == ["consent"]
    plans.cancel_sign_in()


def test_a_failed_code_exchange_keeps_the_issued_client(plans, openai):
    openai.token_error = "invalid_grant"
    with pytest.raises(ValueError, match="try again"):
        _sign_in(plans, openai)
    openai.token_error = None
    url = plans.begin_sign_in("plan")
    assert urllib.parse.parse_qs(urllib.parse.urlsplit(url).query)["client_id"] == [CLIENT]
    plans.cancel_sign_in()


def test_a_different_account_cannot_take_over_a_connection(plans, openai):
    _sign_in(plans, openai)
    first = plans.connections["plan"].tokens
    openai.subject = "user-2"
    with pytest.raises(ValueError, match="different ChatGPT account"):
        _sign_in(plans, openai, client_id=None)
    assert plans.connections["plan"].tokens is first


def test_a_callback_for_a_different_client_is_refused(plans, openai):
    _sign_in(plans, openai)
    first = plans.connections["plan"].tokens
    forms = len(openai.forms)
    url = plans.begin_sign_in("plan")
    openai.browser(url, client_id="oaiapp_someone_else")
    with pytest.raises(ValueError, match="different registration"):
        plans.finish_sign_in("plan", timeout=2.0)
    assert len(openai.forms) == forms and plans.connections["plan"].tokens is first
    assert plans.registrations["plan"]["client_id"] == CLIENT


def test_a_sign_in_that_nobody_finishes_times_out(plans, openai):
    plans.begin_sign_in("plan")
    with pytest.raises(ValueError, match="timed out"):
        plans.finish_sign_in("plan", timeout=0.05)


def test_starting_over_replaces_a_waiting_sign_in(plans, openai):
    first = plans.begin_sign_in("plan")
    second = plans.begin_sign_in("plan")
    openai.browser(second)
    assert plans.finish_sign_in("plan", timeout=2.0).client_id == CLIENT
    assert first != second


def test_stopping_a_sign_in_releases_whoever_waits_on_it(plans, openai):
    plans.begin_sign_in("plan")
    problems = []

    def wait():
        try:
            plans.finish_sign_in("plan", timeout=60.0)
        except ValueError as e:
            problems.append(str(e))

    waiter = threading.Thread(target=wait)
    waiter.start()
    time.sleep(0.1)
    plans.cancel_sign_in()
    waiter.join(5.0)
    assert not waiter.is_alive() and "replaced or stopped" in problems[0]


def test_a_request_follows_the_plan_rules_and_reads_the_stream(plans, openai):
    _sign_in(plans, openai)
    plans.use(models.VISION, "plan", "plan-small")
    openai.events = _reply("what it sees")
    text = plans.ask(models.VISION, "ground rules", [Message("user", "look", [Image("image/png", b"\x89PNG")])])
    assert text == "what it sees"
    headers, body = openai.responses[0]
    assert headers["authorization" if "authorization" in headers else "Authorization"] == f"Bearer {openai.current_access()}"
    assert body["store"] is False and body["stream"] is True and body["model"] == "plan-small"
    assert body["instructions"] == "ground rules"
    assert not {"temperature", "top_p", "max_output_tokens", "previous_response_id", "metadata", "user"} & set(body)
    assert all(turn["role"] != "system" for turn in body["input"])
    parts = body["input"][0]["content"]
    assert parts[0] == {"type": "input_text", "text": "look"}
    assert parts[1]["type"] == "input_image" and parts[1]["image_url"].startswith("data:image/png;base64,")


@pytest.mark.parametrize("events, match", [
    ([{"type": "response.output_text.delta", "delta": "half"}], "before it finished"),
    ([{"type": "response.failed", "response": {"error": {"code": "subscription_sharing_usage_limit_exceeded"}}}],
     "usage limit is reached; see https://chatgpt.com/settings/usage"),
    ([{"type": "response.incomplete", "response": {"incomplete_details": {"reason": "content_filter"}}}], "stopped early"),
    ([{"type": "response.refusal.delta", "delta": "no"}, {"type": "response.completed"}], "declined"),
    ([{"type": "error", "code": "subscription_sharing_user_unavailable"}], "try again later"),
])
def test_only_a_completed_stream_counts_as_an_answer(plans, openai, events, match):
    _sign_in(plans, openai)
    plans.use(models.REASONING, "plan", "plan-big")
    openai.events = events
    with pytest.raises(ModelError, match=match):
        plans.ask(models.REASONING, "", [Message("user", "hello")])


def test_a_refused_request_says_why_and_keeps_the_sign_in(plans, openai):
    _sign_in(plans, openai)
    plans.use(models.REASONING, "plan", "plan-big")
    openai.status, openai.error = 401, {"code": "subscription_sharing_invalid_user"}
    with pytest.raises(ModelError, match="did not accept this sign-in .request req_1"):
        plans.ask(models.REASONING, "", [Message("user", "hello")])
    assert plans.connections["plan"].tokens is not None


def test_tokens_renew_before_they_expire_and_the_new_refresh_token_is_kept(plans, openai):
    _sign_in(plans, openai)
    plans.use(models.REASONING, "plan", "plan-big")
    c = plans.connections["plan"]
    c.tokens.expires_at = time.time() + 5
    old_refresh = c.tokens.refresh
    openai.events = _reply("ready")
    assert plans.ask(models.REASONING, "", [Message("user", "hello")]) == "ready"
    path, form = openai.forms[-1]
    assert form["grant_type"] == "refresh_token" and form["refresh_token"] == old_refresh
    assert form["client_id"] == CLIENT and path.endswith("/oauth/token")
    assert c.tokens.refresh != old_refresh and c.tokens.expires_at > time.time() + 1000


def test_a_finished_refresh_token_means_signing_in_again_but_a_blip_does_not(plans, openai):
    _sign_in(plans, openai)
    plans.use(models.REASONING, "plan", "plan-big")
    c = plans.connections["plan"]
    c.tokens.expires_at = 0
    openai.token_error = "server_error"
    with pytest.raises(ModelError, match="could not renew"):
        plans.ask(models.REASONING, "", [Message("user", "hello")])
    assert c.tokens is not None
    openai.token_error = "refresh_token_reused"
    with pytest.raises(ModelError, match="sign-in ended"):
        plans.ask(models.REASONING, "", [Message("user", "hello")])
    assert c.needs_key


def test_disconnecting_ends_the_sign_in_and_keeps_the_registration(plans, openai):
    _sign_in(plans, openai)
    refresh = plans.connections["plan"].tokens.refresh
    assert plans.disconnect("plan") == ""
    path, form = openai.forms[-1]
    assert path.endswith("/revoke") and form["token"] == refresh and form["client_id"] == CLIENT
    url = plans.begin_sign_in("plan")
    assert urllib.parse.parse_qs(urllib.parse.urlsplit(url).query)["client_id"] == [CLIENT]
    plans.cancel_sign_in()


def test_a_revocation_that_is_not_confirmed_is_reported(plans, openai):
    _sign_in(plans, openai)
    openai.revoke_status = 500
    assert "did not confirm" in plans.disconnect("plan")
    assert "plan" not in plans.connections


def test_addresses_other_than_the_published_ones_are_refused(plans, openai, monkeypatch):
    monkeypatch.setattr(OpenAI, "discovery", lambda self: {
        "issuer": self.url, "authorization_endpoint": "https://elsewhere.example/authorize",
        "token_endpoint": f"{self.url}/api/accounts/oauth/token", "jwks_uri": f"{self.url}/jwks.json",
        "revocation_endpoint": f"{self.url}/api/accounts/oauth/revoke"})
    with pytest.raises(ValueError, match="could not be verified"):
        plans.begin_sign_in("plan")


def test_a_plan_connection_takes_no_address_or_key():
    with pytest.raises(ValueError, match="no address and no key"):
        models.check_address(models.CHATGPT, "https://example.com", "")
    with pytest.raises(ValueError, match="no address and no key"):
        models.check_address(models.CLAUDE_CODE, "", "a-key")


def test_a_key_is_not_carried_across_a_redirect(tmp_path):
    seen = []

    class Moved(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            seen.append(self.headers.get("authorization"))
            self.send_response(302)
            self.send_header("location", f"http://127.0.0.1:{self.server.server_port}/elsewhere")
            self.send_header("content-length", "0")
            self.end_headers()

    server = ThreadingHTTPServer(("127.0.0.1", 0), Moved)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    c = models.Connection("x", models.OPENAI_COMPATIBLE, f"http://127.0.0.1:{server.server_port}", True, key="k")
    with pytest.raises(ModelError, match="redirected"):
        models.list_models(c)
    server.shutdown()
    server.server_close()
    assert len(seen) == 1


def test_signing_in_through_the_command(running_hub, openai, capsys, monkeypatch):  # noqa: F811
    home = running_hub
    opened = []

    def browser(url):
        opened.append(url)
        threading.Thread(target=openai.browser, args=(url,), daemon=True).start()
        return True

    monkeypatch.setattr("webbrowser.open", browser)
    assert main(["models", "connect", "chatgpt", "--home", str(home.root)]) == 0
    out = capsys.readouterr().out
    assert "2 models on your ChatGPT plan" in out and "go to OpenAI" in out and opened
    assert main(["models", "list", "--home", str(home.root)]) == 0
    assert "plan-big" in capsys.readouterr().out
    assert main(["models", "disconnect", "chatgpt", "--home", str(home.root)]) == 0
    assert "disconnected" in capsys.readouterr().out


FAKE = """#!/usr/bin/env python3
import json, os, sys, time
script = json.load(open(os.environ["FAKE_CLAUDE"]))
if sys.argv[1:3] == ["auth", "status"]:
    print(json.dumps({"loggedIn": script.get("signed_in", True), "email": "person@example.com"}))
    sys.exit(0)
request = sys.stdin.readline()
json.dump({"argv": sys.argv[1:], "cwd": os.getcwd(), "files": os.listdir("."), "request": json.loads(request)},
          open(os.environ["FAKE_CLAUDE"] + ".seen", "w"))
for event in script["events"]:
    print(json.dumps(event), flush=True)
time.sleep(script.get("hang", 0))
sys.exit(script.get("exit", 0))
"""
INIT = {"type": "system", "subtype": "init", "tools": [], "mcp_servers": []}
RESULT = {"type": "result", "subtype": "success", "is_error": False, "result": "ready from the program"}


@pytest.fixture
def claude(tmp_path, monkeypatch):
    home = tmp_path / "person"
    (home / "bin").mkdir(parents=True)
    binary = home / "bin" / "claude"
    binary.write_text(FAKE)
    binary.chmod(0o755)
    script = tmp_path / "script.json"

    def behave(**what):
        script.write_text(json.dumps({"events": [INIT, RESULT]} | what))
        return script

    behave()
    monkeypatch.setenv("FAKE_CLAUDE", str(script))
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.setenv("PATH", f"{home / 'bin'}:{os.environ['PATH']}")
    behave.script = script
    behave.seen = lambda: json.loads((tmp_path / "script.json.seen").read_text())
    return behave


def _claude_slot(plans):
    plans.connect("claude-code", models.CLAUDE_CODE)
    plans.use(models.REASONING, "claude-code", "default")


def test_claude_code_must_be_installed_and_signed_in(plans, claude, monkeypatch):
    claude(signed_in=False)
    with pytest.raises(ValueError, match="not signed in"):
        plans.connect("claude-code", models.CLAUDE_CODE)
    monkeypatch.setattr(claude_code.shutil, "which", lambda *a, **k: None)
    with pytest.raises(ValueError, match="not installed"):
        plans.connect("claude-code", models.CLAUDE_CODE)
    assert not plans.connections


def test_claude_code_runs_with_nothing_but_the_question(plans, claude, tmp_path):
    _claude_slot(plans)
    secret_log = "log line from a robot"
    assert plans.ask(models.REASONING, "ground rules", [Message("user", secret_log)]) == "ready from the program"
    seen = claude.seen()
    args = seen["argv"]
    assert args[args.index("--tools") + 1] == "" and args[args.index("--setting-sources") + 1] == ""
    assert "--restricted" in args and "--strict-mcp-config" in args and "--no-session-persistence" in args
    assert args[args.index("--permission-mode") + 1] == "dontAsk" and args[args.index("--max-turns") + 1] == "1"
    assert args[args.index("--system-prompt") + 1] == "ground rules"
    assert "--model" not in args and "--bare" not in args
    assert not any(secret_log in a for a in args)
    assert seen["files"] == [] and not seen["cwd"].startswith(str(tmp_path))
    assert seen["request"]["message"]["content"] == [{"type": "text", "text": secret_log}]
    assert not os.path.exists(seen["cwd"])


def test_claude_code_is_told_the_model_and_sees_images(plans, claude):
    plans.connect("claude-code", models.CLAUDE_CODE)
    plans.use(models.VISION, "claude-code", "a-model")
    plans.ask(models.VISION, "", [Message("user", "look", [Image("image/png", b"\x89PNG")])])
    seen = claude.seen()
    assert seen["argv"][seen["argv"].index("--model") + 1] == "a-model"
    assert seen["argv"][seen["argv"].index("--system-prompt") + 1] == claude_code.NO_INSTRUCTIONS
    assert seen["request"]["message"]["content"][1]["source"]["media_type"] == "image/png"


def test_claude_code_is_stopped_if_it_loads_a_tool(plans, claude):
    _claude_slot(plans)
    claude(events=[{"type": "system", "subtype": "init", "tools": ["Bash"]}, RESULT])
    with pytest.raises(ModelError, match="tools it was told not to load"):
        plans.ask(models.REASONING, "", [Message("user", "hello")])


def test_claude_code_failures_are_reported(plans, claude):
    _claude_slot(plans)
    claude(events=[INIT, {"type": "result", "subtype": "error_during_execution", "is_error": True,
                          "result": "usage limit reached"}])
    with pytest.raises(ModelError, match="usage limit reached"):
        plans.ask(models.REASONING, "", [Message("user", "hello")])
    claude(events=[INIT], exit=1)
    with pytest.raises(ModelError, match="did not finish"):
        plans.ask(models.REASONING, "", [Message("user", "hello")])


def test_claude_code_that_hangs_is_stopped(claude):
    claude(events=[INIT], hang=30)
    started = time.monotonic()
    with pytest.raises(ModelError, match="took longer"):
        claude_code.complete("default", "", [Message("user", "hello")], timeout=0.5)
    assert time.monotonic() - started < 5


def test_a_saved_claude_code_connection_survives_a_restart_with_no_secret(plans, claude, tmp_path):
    _claude_slot(plans)
    saved = (tmp_path / "models.yaml").read_text()
    assert "person@example.com" not in saved
    again = models.Models(tmp_path / "models.yaml")
    assert again.ask(models.REASONING, "", [Message("user", "hello")]) == "ready from the program"


def test_claude_code_through_the_command(running_hub, claude, capsys):  # noqa: F811
    home = running_hub
    assert main(["models", "connect", "claude-code", "--home", str(home.root)]) == 0
    out = capsys.readouterr().out
    assert "under your own sign-in" in out and "person@example.com" not in out
    assert main(["models", "use", "reasoning", "claude-code/default", "--home", str(home.root)]) == 0
    capsys.readouterr()
    assert main(["models", "list", "--home", str(home.root)]) == 0
    assert "the installed program" in capsys.readouterr().out
    assert main(["models", "test", "reasoning", "--home", str(home.root)]) == 0
    assert "ready from the program" in capsys.readouterr().out
