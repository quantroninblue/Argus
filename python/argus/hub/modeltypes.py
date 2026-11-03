"""What every kind of model connection shares."""

from __future__ import annotations

import urllib.request
from dataclasses import dataclass, field


class ModelError(RuntimeError):
    """A model call that failed, in words for the user. Never carries a key or a token."""


@dataclass
class Image:
    media_type: str
    data: bytes


@dataclass
class Message:
    role: str  # "user" or "assistant"
    text: str
    images: list[Image] = field(default_factory=list)


class _NoRedirects(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        return None


# urllib copies request headers across a redirect, so following one could hand a key to another
# host. A redirect comes back as an error instead.
OPENER = urllib.request.build_opener(_NoRedirects())
