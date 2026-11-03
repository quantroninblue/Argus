"""ChatGPT plan sign-in and requests, by the route OpenAI gives open-source, self-hosted apps.

Each person signs in with their own account in their own browser, and OpenAI issues a client for
that person on this hub. Requests go to the public Responses API and count against that person's
plan. The client id and an opaque id for this hub are kept in the hub's home; tokens stay in this
process, so a restarted hub asks for a new sign-in.

Sign-in needs a browser that can reach this machine's 127.0.0.1.
"""

from __future__ import annotations

import base64
import hashlib
import hmac
import json
import secrets
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import jwt

from argus.hub.modeltypes import OPENER, Message, ModelError

ISSUER = "https://auth.openai.com"
API = "https://api.openai.com/v1"
RESOURCE = "https://api.openai.com/v1"
SCOPES = "openid profile email offline_access resource.invoke chatgpt.tokens.use.direct"
PLAN_SCOPE = "chatgpt.tokens.use.direct"
FIRST_CLIENT = "dynamic_agent_client"
APP_NAME = "Argus"
USAGE_PAGE = "https://chatgpt.com/settings/usage"
CALLBACK_PATH = "/auth/callback"

SIGN_IN_TIMEOUT_S = 300.0
AUTH_TIMEOUT_S = 20.0
REFRESH_EARLY_S = 60.0

UNUSABLE_REFRESH = {"invalid_grant", "invalid_refresh_token", "token_expired", "refresh_token_expired",
                    "refresh_token_invalidated", "refresh_token_reused"}

PROBLEMS = {
    "subscription_sharing_user_not_eligible": "this ChatGPT account cannot use its plan in other apps",
    "subscription_sharing_usage_limit_exceeded": f"the ChatGPT plan's usage limit is reached; see {USAGE_PAGE}",
    "subscription_sharing_usage_unavailable": "ChatGPT usage is unavailable right now; try again later",
    "subscription_sharing_user_unavailable": "ChatGPT usage is unavailable right now; try again later",
    "subscription_sharing_unsupported_capability": "a ChatGPT plan does not allow this model or input here",
    "subscription_sharing_route_not_supported": "a ChatGPT plan does not allow this request",
    "subscription_sharing_invalid_user": "OpenAI did not accept this sign-in",
    "chatpass_v2_scope_not_authorized": "this sign-in does not allow plan use",
    "chatpass_v2_invalid_authorization_context": "this sign-in does not allow plan use",
}


class SignInError(ValueError):
    """A sign-in that did not complete, in words for the user."""


@dataclass
class Tokens:
    access: str = field(repr=False)
    refresh: str = field(repr=False)
    expires_at: float
    scopes: list[str]


@dataclass
class Registration:
    client_id: str
    subject: str
    tokens: Tokens


def _json(request: urllib.request.Request, timeout: float = AUTH_TIMEOUT_S) -> tuple[int, dict]:
    """The status and parsed body, for an answer of any status. Raises OSError-family errors when
    nothing answered."""
    try:
        with OPENER.open(request, timeout=timeout) as reply:
            status, body = reply.status, reply.read()
    except urllib.error.HTTPError as e:
        status, body = e.code, e.read()
    try:
        parsed = json.loads(body)
    except ValueError:
        parsed = {}
    return status, parsed if isinstance(parsed, dict) else {}


def _discovery() -> dict:
    """OpenAI's published sign-in addresses, refused unless they are the documented ones."""
    try:
        status, doc = _json(urllib.request.Request(f"{ISSUER}/.well-known/openid-configuration"))
    except OSError as e:
        raise SignInError(f"could not reach ChatGPT sign-in: {getattr(e, 'reason', e)}") from None
    trusted = status == 200 and doc.get("issuer") == ISSUER and \
        doc.get("authorization_endpoint") == f"{ISSUER}/api/accounts/authorize" and \
        doc.get("token_endpoint") == f"{ISSUER}/api/accounts/oauth/token"
    for key in ("jwks_uri", "revocation_endpoint"):
        trusted = trusted and str(doc.get(key, "")).startswith(f"{ISSUER}/")
    if not trusted:
        raise SignInError("ChatGPT sign-in's published configuration could not be verified")
    return doc


def _token_request(endpoint: str, fields: dict) -> tuple[int, dict]:
    request = urllib.request.Request(endpoint, data=urllib.parse.urlencode(fields).encode(),
                                     headers={"content-type": "application/x-www-form-urlencoded",
                                              "accept": "application/json"})
    return _json(request)


def _tokens(reply: dict, previous: Tokens | None = None) -> Tokens:
    access, life = reply.get("access_token"), reply.get("expires_in")
    if not access or str(reply.get("token_type", "")).lower() != "bearer" or \
            not isinstance(life, (int, float)) or life <= 0:
        raise SignInError("ChatGPT sign-in returned an incomplete answer")
    scope = reply.get("scope")
    return Tokens(access, reply.get("refresh_token") or (previous.refresh if previous else ""),
                  time.time() + life,
                  scope.split() if isinstance(scope, str) else (previous.scopes if previous else []))


def _verified_subject(id_token: str, jwks_uri: str, client_id: str, nonce: str = "", subject: str = "") -> str:
    """The account id in a signed ID token, after checking who signed it and who it was for."""
    problem = "ChatGPT sign-in could not be verified; try again"
    try:
        status, keys = _json(urllib.request.Request(jwks_uri))
        if status != 200:
            raise SignInError(problem)
        kid = jwt.get_unverified_header(id_token).get("kid")
        key = next(k for k in jwt.PyJWKSet.from_dict(keys).keys if k.key_id == kid)
        claims = jwt.decode(id_token, key.key, algorithms=["RS256", "ES256"], audience=client_id, issuer=ISSUER,
                            leeway=5, options={"require": ["exp", "iat", "sub"]})
    except (jwt.PyJWTError, StopIteration, OSError, TypeError, ValueError):
        raise SignInError(problem) from None
    if (nonce and not hmac.compare_digest(str(claims.get("nonce", "")).encode(), nonce.encode())) or \
            claims.get("azp", client_id) != client_id:
        raise SignInError(problem)
    if subject and claims["sub"] != subject:
        raise SignInError("that is a different ChatGPT account from the one on this connection; "
                          "use another --name for it")
    return claims["sub"]


class _Callback(BaseHTTPRequestHandler):
    timeout = 10

    def do_GET(self) -> None:
        status, words = self.server.sign_in.callback(self.path, self.headers.get("host", ""))
        body = words.encode()
        self.send_response(status)
        self.send_header("content-type", "text/plain; charset=utf-8")
        self.send_header("content-length", str(len(body)))
        self.send_header("cache-control", "no-store")
        self.send_header("content-security-policy", "default-src 'none'")
        self.send_header("referrer-policy", "no-referrer")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args) -> None:
        pass  # the request line holds the authorization code


class SignIn:
    """One attempt: a listener on 127.0.0.1, the address to open, and the wait for the answer."""

    def __init__(self, host_id: str, client_id: str = "", subject: str = "") -> None:
        self.client_id = client_id
        self.subject = subject
        self.meta = _discovery()
        self.state = secrets.token_urlsafe(32)
        self.nonce = secrets.token_urlsafe(32)
        self.verifier = secrets.token_urlsafe(32)
        self._answered = threading.Event()
        self._claimed = False
        self._code = ""
        self._problem = ""

        self._server = ThreadingHTTPServer(("127.0.0.1", 0), _Callback)
        self._server.daemon_threads = True
        self._server.sign_in = self
        self._host = f"127.0.0.1:{self._server.server_port}"
        self.redirect = f"http://{self._host}{CALLBACK_PATH}"
        threading.Thread(target=self._server.serve_forever, daemon=True).start()

        challenge = base64.urlsafe_b64encode(hashlib.sha256(self.verifier.encode()).digest()).rstrip(b"=").decode()
        query = {"client_id": client_id or FIRST_CLIENT, "ext_agent_host_id": host_id}
        if not client_id:
            query["agent_name_hint"] = APP_NAME
        elif not subject:
            query["prompt"] = "consent"  # registered earlier, but plan use was never granted
        query |= {"response_type": "code", "redirect_uri": self.redirect, "scope": SCOPES, "resource": RESOURCE,
                  "state": self.state, "nonce": self.nonce, "code_challenge_method": "S256",
                  "code_challenge": challenge}
        self.url = f"{self.meta['authorization_endpoint']}?{urllib.parse.urlencode(query)}"

    def callback(self, path: str, host: str) -> tuple[int, str]:
        parts = urllib.parse.urlsplit(path)
        if parts.path != CALLBACK_PATH or host != self._host:
            return 404, "Not found."
        query = urllib.parse.parse_qs(parts.query, keep_blank_values=True)
        state = query.get("state", [])
        if self._claimed or len(state) != 1 or not hmac.compare_digest(state[0].encode(), self.state.encode()):
            return 400, "This sign-in could not be verified. Go back to the terminal."
        self._claimed = True
        code, issued = query.get("code", []), query.get("client_id", [])
        client_id = issued[0] if len(issued) == 1 else self.client_id
        if "error" in query:
            self._problem = "ChatGPT sign-in was declined"
        elif len(code) != 1 or not code[0] or len(issued) > 1:
            self._problem = "ChatGPT sign-in came back incomplete"
        elif not client_id or client_id == FIRST_CLIENT:
            self._problem = "ChatGPT did not finish registering this hub; try again"
        elif self.client_id and client_id != self.client_id:
            self._problem = "ChatGPT answered for a different registration"
        else:
            self._code, self.client_id = code[0], client_id
        self._answered.set()
        if self._problem:
            return 400, "Sign-in did not complete. Go back to the terminal."
        return 200, "Signed in. You can close this window and go back to the terminal."

    def close(self) -> None:
        """Ends the attempt and releases anything waiting on it."""
        self._problem = self._problem or "this ChatGPT sign-in was replaced or stopped"
        self._answered.set()
        self._server.shutdown()
        self._server.server_close()

    def finish(self, on_registered, timeout: float = SIGN_IN_TIMEOUT_S) -> Registration:
        """Waits for the browser, then trades the code for tokens. on_registered(client_id) runs
        before the trade, so a failed one retries with the same client."""
        try:
            if not self._answered.wait(timeout):
                raise SignInError("ChatGPT sign-in timed out")
            if self._problem:
                raise SignInError(self._problem)
            on_registered(self.client_id)
            return self._exchange()
        finally:
            self._answered.set()
            self._server.shutdown()
            self._server.server_close()

    def _exchange(self) -> Registration:
        try:
            status, reply = _token_request(self.meta["token_endpoint"], {
                "grant_type": "authorization_code", "client_id": self.client_id, "code": self._code,
                "code_verifier": self.verifier, "redirect_uri": self.redirect, "resource": RESOURCE})
        except OSError as e:
            raise SignInError(f"could not reach ChatGPT sign-in: {getattr(e, 'reason', e)}") from None
        if status != 200:
            raise SignInError("ChatGPT refused the sign-in; try again" if reply.get("error") == "invalid_grant"
                              else f"ChatGPT sign-in failed (HTTP {status}); try again later")
        tokens = _tokens(reply)
        id_token = reply.get("id_token")
        if not isinstance(id_token, str) or not id_token:
            raise SignInError("ChatGPT sign-in returned an incomplete answer")
        subject = _verified_subject(id_token, self.meta["jwks_uri"], self.client_id, self.nonce, self.subject)
        if PLAN_SCOPE not in tokens.scopes:
            raise SignInError("ChatGPT plan use was not allowed for this sign-in; "
                              "sign in again and allow it, or check the account's plan")
        return Registration(self.client_id, subject, tokens)


_refreshing = threading.Lock()


def access_token(c) -> str:
    """A usable access token for a connection, renewed when close to expiry. Refresh tokens rotate,
    so renewals run one at a time. A token OpenAI will not renew is dropped, which puts the
    connection back to needing a sign-in; a network failure keeps it."""
    with _refreshing:
        tokens = c.tokens
        if tokens is None:
            raise ModelError(f"{c.name} needs signing in again: argus models connect chatgpt --name {c.name}")
        if tokens.expires_at - time.time() > REFRESH_EARLY_S:
            return tokens.access
        if not tokens.refresh:
            c.tokens = None
            raise ModelError(f"{c.name} needs signing in again: argus models connect chatgpt --name {c.name}")
        try:
            meta = _discovery()
            status, reply = _token_request(meta["token_endpoint"], {
                "grant_type": "refresh_token", "client_id": c.client_id, "refresh_token": tokens.refresh,
                "resource": RESOURCE})
            if status == 200:
                renewed = _tokens(reply, tokens)
                if reply.get("id_token"):
                    _verified_subject(reply["id_token"], meta["jwks_uri"], c.client_id, subject=c.subject)
        except (SignInError, OSError) as e:
            raise ModelError(f"could not renew the ChatGPT sign-in: {getattr(e, 'reason', e)}") from None
        if status != 200:
            if reply.get("error") in UNUSABLE_REFRESH:
                c.tokens = None
                raise ModelError(f"the ChatGPT sign-in ended: argus models connect chatgpt --name {c.name}")
            raise ModelError(f"could not renew the ChatGPT sign-in (HTTP {status}); try again later")
        c.tokens = renewed
        return renewed.access


def revoke(client_id: str, tokens: Tokens) -> bool:
    """Whether OpenAI confirmed the refresh token is finished."""
    try:
        status, _ = _token_request(_discovery()["revocation_endpoint"], {
            "token": tokens.refresh, "token_type_hint": "refresh_token", "client_id": client_id})
    except (SignInError, OSError):
        return False
    return status == 200


def _headers(token: str) -> dict:
    return {"authorization": f"Bearer {token}", "content-type": "application/json"}


def _problem(code: str, said: str, param: str = "", request_id: str = "") -> str:
    words = PROBLEMS.get(code) or f"ChatGPT refused the request ({code or said[:200] or 'no reason given'})"
    if param:
        words += f" ({param})"
    if code == "subscription_sharing_invalid_user" and request_id:
        words += f" (request {request_id})"
    return words


def list_models(token: str) -> list[str]:
    """What this account may use, in the order the server gives."""
    request = urllib.request.Request(f"{API}/models", headers=_headers(token))
    try:
        status, doc = _json(request)
    except OSError as e:
        raise ModelError(f"could not reach {API}: {getattr(e, 'reason', e)}") from None
    if status != 200:
        error = doc.get("error")
        error = error if isinstance(error, dict) else {}
        raise ModelError(_problem(error.get("code") or "", error.get("message") or f"HTTP {status}"))
    return [m["slug"] for m in doc.get("models", []) if m.get("visibility") == "list" and m.get("slug")]


def _content(m: Message):
    if not m.images:
        return m.text
    parts: list[dict] = [{"type": "input_text", "text": m.text}]
    for image in m.images:
        data = base64.b64encode(image.data).decode()
        parts.append({"type": "input_image", "image_url": f"data:{image.media_type};base64,{data}"})
    return parts


def _read_stream(reply) -> str:
    """The text of a Responses stream. Only response.completed counts as an answer."""
    text: list[str] = []
    refused = False
    for raw in reply:
        line = raw.decode("utf-8", "replace").strip()
        if not line.startswith("data:"):
            continue
        try:
            event = json.loads(line[5:])
        except ValueError:
            continue
        kind = event.get("type")
        if kind == "response.output_text.delta":
            text.append(event.get("delta", ""))
        elif kind == "response.refusal.delta":
            refused = True
        elif kind == "response.completed":
            if "".join(text):
                return "".join(text)
            raise ModelError("the model declined this request" if refused else "the model returned no text")
        elif kind == "response.incomplete":
            details = (event.get("response") or {}).get("incomplete_details") or {}
            if text and details.get("reason") == "max_output_tokens":
                return "".join(text) + "\n[cut off: the model reached its output limit]"
            raise ModelError(f"the model stopped early ({details.get('reason', 'no reason given')})")
        elif kind == "response.failed":
            error = (event.get("response") or {}).get("error") or {}
            raise ModelError(_problem(error.get("code") or "", error.get("message") or ""))
        elif kind == "error":
            raise ModelError(_problem(event.get("code") or "", event.get("message") or "", event.get("param") or ""))
    raise ModelError("the reply stopped before it finished; try again")


def complete(token: str, model: str, system: str, messages: list[Message], timeout: float) -> str:
    """One Responses request. Plan use refuses temperature, max_output_tokens and similar settings,
    and refuses a system role, so instructions go in their own field."""
    body = {"model": model, "input": [{"role": m.role, "content": _content(m)} for m in messages],
            "store": False, "stream": True}
    if system:
        body["instructions"] = system
    request = urllib.request.Request(f"{API}/responses", data=json.dumps(body).encode(), headers=_headers(token))
    try:
        with OPENER.open(request, timeout=timeout) as reply:
            return _read_stream(reply)
    except urllib.error.HTTPError as e:
        try:
            error = json.loads(e.read()).get("error")
        except (ValueError, AttributeError):
            error = None
        error = error if isinstance(error, dict) else {}
        raise ModelError(_problem(error.get("code") or "", error.get("message") or f"HTTP {e.code}",
                                  error.get("param") or "", e.headers.get("x-request-id") or "")) from None
    except OSError as e:
        raise ModelError(f"could not reach {API}: {getattr(e, 'reason', e)}") from None
