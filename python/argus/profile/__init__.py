"""Load, validate and hash Argus body profiles.

Validation runs in two layers. The JSON Schema checks structure and value ranges. The C++
capability library checks how fields relate to each other (sensor frames exist, a body that
cannot strafe has no sideways speed, and so on) and is the final authority.
"""

from __future__ import annotations

import hashlib
import json
from functools import lru_cache
from pathlib import Path
from typing import Any

import jsonschema
import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
SCHEMA_PATH = REPO_ROOT / "schemas" / "body_profile.schema.json"
EXAMPLES_DIR = REPO_ROOT / "schemas" / "examples"


def _native():
    try:
        from argus import capabilities
    except ImportError as exc:
        raise ImportError(
            "argus.capabilities is not built; run tools/check.sh or build with CMake and add "
            "build/python to PYTHONPATH"
        ) from exc
    return capabilities


@lru_cache(maxsize=1)
def schema() -> dict[str, Any]:
    return json.loads(SCHEMA_PATH.read_text())


def load(path: str | Path) -> dict[str, Any]:
    with open(path) as handle:
        return yaml.safe_load(handle)


def dump(doc: dict[str, Any]) -> str:
    return yaml.safe_dump(doc, sort_keys=False)


def schema_problems(doc: Any) -> list[str]:
    validator = jsonschema.Draft202012Validator(schema())
    problems = []
    for error in validator.iter_errors(doc):
        where = ".".join(str(part) for part in error.absolute_path) or "profile"
        problems.append(f"{where}: {error.message}")
    return sorted(problems)


def native_problems(doc: dict[str, Any]) -> list[str]:
    return list(_native().validate_profile_text(dump(doc)))


def problems(doc: dict[str, Any]) -> list[str]:
    """Every problem either layer finds. Empty means the profile is valid."""
    found = schema_problems(doc)
    found += [p for p in native_problems(doc) if p not in found]
    return found


def derive(doc: dict[str, Any], options=None):
    native = _native()
    return native.derive_from_text(dump(doc), options or native.DeriveOptions())


def profile_hash(doc: dict[str, Any]) -> str:
    """Content hash that ignores key order and formatting."""
    canonical = json.dumps(doc, sort_keys=True, separators=(",", ":"))
    return "sha256:" + hashlib.sha256(canonical.encode()).hexdigest()
