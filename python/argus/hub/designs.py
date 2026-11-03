"""Robot designs on the hub: a user's URDF with everything it needs, stored as versions.

A design is what the user brings: a URDF or xacro, the meshes it references, optionally their
own Gazebo model (an SDF or xacro with plugins), and an answers file for what a URDF cannot
say (see `argus.setup.importer`). `collect` gathers it into a bundle on the user's side: xacro
expanded, every referenced mesh copied in, so a version never depends on files that may change
later. The hub stores each bundle under `designs/<name>/<version>/`, the version being a hash
of its contents, and checks it before anything may run it.

Checks report findings instead of fixing anything: Argus never patches the user's physics.
"""

from __future__ import annotations

import hashlib
import json
import re
import shutil
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
import yaml

from argus import profile as profiles
from argus.setup.importer import build_profile
from argus.setup.urdf import UrdfError, expand, identifier, resolve_mesh

URDF_NAME = "robot.urdf"
ANSWERS_NAME = "answers.yaml"
GAZEBO_NAME = "gazebo/model.sdf"
NAME_RE = re.compile(r"^[a-z0-9][a-z0-9_-]{0,63}$")
MAX_BUNDLE_BYTES = 256 * 1024 * 1024
GZ_CHECK_TIMEOUT_S = 60


@dataclass
class Finding:
    severity: str  # error: the design cannot run; warning: it runs but may misbehave; note
    where: str
    text: str

    def __str__(self) -> str:
        return f"{self.severity}: {self.where}: {self.text}" if self.where else f"{self.severity}: {self.text}"


@dataclass
class Bundle:
    name: str
    files: dict[str, bytes] = field(default_factory=dict)  # path inside the version -> contents
    findings: list[Finding] = field(default_factory=list)  # found while collecting

    @property
    def has_answers(self) -> bool:
        return ANSWERS_NAME in self.files

    @property
    def has_gazebo_model(self) -> bool:
        return GAZEBO_NAME in self.files

    def version(self) -> str:
        h = hashlib.sha256()
        for path in sorted(self.files):
            h.update(path.encode() + b"\0" + hashlib.sha256(self.files[path]).digest())
        return h.hexdigest()[:12]

    def size(self) -> int:
        return sum(len(v) for v in self.files.values())


def design_name(text: str) -> str:
    return identifier(text).lower().replace("_", "-").strip("-") or "robot"


def _stored_mesh_path(uri: str, n: int) -> str:
    if uri.startswith("package://"):
        return "packages/" + uri[len("package://"):]
    return f"meshes/{n}_{Path(uri.removeprefix('file://')).name}"


_DAE_IMAGE = re.compile(rb"<init_from>([^<]+)</init_from>")


def _copy_mesh(bundle: Bundle, uri: str, source: Path, stored: str) -> None:
    bundle.files[stored] = source.read_bytes()
    if source.suffix.lower() == ".dae":
        # Collada meshes name their textures relative to themselves; take them along.
        for ref in _DAE_IMAGE.findall(bundle.files[stored]):
            rel = ref.decode(errors="replace").strip()
            if "://" in rel or rel.startswith("/") or "#" in rel:
                continue
            img = source.parent / rel
            if img.is_file():
                bundle.files[str(Path(stored).parent / rel)] = img.read_bytes()


def _collect_meshes(bundle: Bundle, root: ET.Element, tag: str, attr: str | None, base_dir: Path,
                    packages: dict[str, Path], where: str) -> None:
    """Copies every mesh `root` references into the bundle and points the element at the copy.
    Package URIs stay as they are (the hub serves the package from the bundle); file paths
    become paths inside the bundle. A missing visual mesh only changes how the robot looks; a
    missing collision mesh changes its physics."""
    for part in (e for e in root.iter() if e.tag in ("visual", "collision")):
        for el in part.iter(tag):
            uri = (el.get(attr) if attr else el.text or "").strip()
            if uri.startswith("model://"):
                bundle.findings.append(Finding("warning", where, f"{uri}: model:// resources are not taken "
                                                                 "into the design; use package:// or a file path"))
                continue
            if not uri or not re.search(r"\.(stl|dae|obj|glb|gltf)$", uri, re.I):
                continue
            try:
                source = resolve_mesh(uri, base_dir, packages)
            except UrdfError as e:
                severity = "error" if part.tag == "collision" else "warning"
                effect = "" if part.tag == "collision" else " (a visual mesh: the robot will look incomplete)"
                bundle.findings.append(Finding(severity, where, f"{e}{effect}"))
                continue
            stored = _stored_mesh_path(uri, len([p for p in bundle.files if p.startswith("meshes/")]))
            _copy_mesh(bundle, uri, source, stored)
            if not uri.startswith("package://"):
                # Relative to the file that names it: the URDF at the top, the Gazebo model in gazebo/.
                target = stored if tag == "mesh" else "../" + stored
                if attr:
                    el.set(attr, target)
                else:
                    el.text = target


def collect(urdf: str | Path, answers: str | Path | None = None, gazebo_model: str | Path | None = None,
            packages: dict[str, Path] | None = None, name: str | None = None) -> Bundle:
    """A design bundle from the user's files. Problems become findings, so the user sees all of
    them at once; only an unreadable URDF raises."""
    urdf = Path(urdf)
    packages = packages or {}
    root = ET.fromstring(expand(urdf))
    if root.tag != "robot":
        raise UrdfError(f"{urdf}: the root element is <{root.tag}>, not <robot>")
    bundle = Bundle(name=design_name(name or root.get("name") or urdf.stem))
    _collect_meshes(bundle, root, "mesh", "filename", urdf.parent, packages, URDF_NAME)
    bundle.files[URDF_NAME] = ET.tostring(root, encoding="utf-8", xml_declaration=True)
    if answers is not None:
        bundle.files[ANSWERS_NAME] = Path(answers).read_bytes()
    if gazebo_model is not None:
        g = Path(gazebo_model)
        try:
            sdf = ET.fromstring(expand(g) if g.suffix == ".xacro" else g.read_text())
        except (ET.ParseError, UrdfError) as e:
            bundle.findings.append(Finding("error", str(g), f"not readable as SDF: {e}"))
        else:
            _collect_meshes(bundle, sdf, "uri", None, g.parent, packages, GAZEBO_NAME)
            bundle.files[GAZEBO_NAME] = ET.tostring(sdf, encoding="utf-8", xml_declaration=True)
    if bundle.size() > MAX_BUNDLE_BYTES:
        bundle.findings.append(Finding("error", "", f"the design is {bundle.size() / 1e6:.0f} MB; "
                                                    f"{MAX_BUNDLE_BYTES / 1e6:.0f} MB at most"))
    seen = set()
    bundle.findings = [f for f in bundle.findings if not (str(f) in seen or seen.add(str(f)))]
    return bundle


def _inertia_findings(root: ET.Element) -> list[Finding]:
    """Gazebo drops or lumps links without mass, and impossible inertia makes the physics blow
    up; both are design errors, never patched."""
    out = []
    joints = root.findall("joint")
    moving = {j.find("child").get("link") for j in joints if j.get("type") != "fixed"}
    fixed = {j.find("child").get("link") for j in joints if j.get("type") == "fixed"}
    for link in root.findall("link"):
        name = link.get("name")
        inertial = link.find("inertial")
        if inertial is None:
            if name in moving:
                out.append(Finding("error", f"link {name}", "no <inertial> on a moving link; Gazebo drops links "
                                                             "without mass, and the joint with it"))
            elif name in fixed and link.find("collision") is not None:
                out.append(Finding("note", f"link {name}", "no <inertial>; Gazebo merges it into the link it is "
                                                            "fixed to, adding its collision but no mass"))
            continue
        mass = inertial.find("mass")
        m = float(mass.get("value", "0")) if mass is not None else 0.0
        if m <= 0:
            out.append(Finding("error", f"link {name}", f"mass {m:g} kg; a simulated link needs positive mass"))
            continue
        i = inertial.find("inertia")
        if i is None:
            out.append(Finding("error", f"link {name}", "<inertial> has no <inertia>"))
            continue
        get = lambda k: float(i.get(k, "0"))  # noqa: E731
        tensor = np.array([[get("ixx"), get("ixy"), get("ixz")],
                           [get("ixy"), get("iyy"), get("iyz")],
                           [get("ixz"), get("iyz"), get("izz")]])
        ev = np.linalg.eigvalsh(tensor)
        if ev.min() <= 0:
            out.append(Finding("error", f"link {name}", "inertia is not positive definite "
                                                        f"(principal moments {', '.join(f'{v:.3g}' for v in ev)})"))
            continue
        a, b, c = sorted(ev)
        if a + b < c * (1 - 1e-6):
            out.append(Finding("error", f"link {name}", "inertia breaks the triangle inequality (no real "
                                                        f"body has principal moments {a:.3g}, {b:.3g}, {c:.3g})"))
    return out


def _gazebo_findings(version_dir: Path) -> list[Finding]:
    if shutil.which("gz") is None:
        return [Finding("note", "", "Gazebo is not installed here, so the model was not checked by Gazebo")]
    out = []
    for rel in (URDF_NAME, GAZEBO_NAME):
        f = version_dir / rel
        if not f.exists():
            continue
        try:
            r = subprocess.run(["gz", "sdf", "-k", str(f)], capture_output=True, text=True, timeout=GZ_CHECK_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            out.append(Finding("warning", rel, "Gazebo's check did not finish"))
            continue
        errors, warnings = _gz_messages(r.stdout + r.stderr)
        if r.returncode != 0 or errors:
            out.append(Finding("error", rel, "Gazebo rejects it: " + "; ".join(errors or warnings or ["no reason given"])[:800]))
        elif warnings:
            out.append(Finding("warning", rel, "Gazebo warns: " + "; ".join(warnings)[:800]))
    return out


_ANSI = re.compile(r"\x1b\[[0-9;]*m")
_GZ_HEADER = re.compile(r"(Warning|Error) \[[^\]]*\]")
_GZ_PART = re.compile(r"Msg:\s*(.*?)(?=Error Code \d+:|$)", re.S)


def _gz_messages(text: str) -> tuple[list[str], list[str]]:
    """Gazebo's output as (errors, warnings), colour codes and repeats removed. Gazebo runs its
    messages together, each starting "Warning [file:line]" or "Error [file:line]", with one or
    more "Error Code N: Msg: ..." parts even inside warnings; it ends with "Valid." when it
    accepts the file."""
    text = re.sub(r"Valid\.\s*$", "", _ANSI.sub("", text).strip())
    errors, warnings = [], []
    parts = _GZ_HEADER.split(text)
    for kind, body in zip(parts[1::2], parts[2::2]):
        msg = " ".join(" ".join(m.split()) for m in _GZ_PART.findall(body)) or " ".join(body.split())
        bucket = errors if kind == "Error" else warnings
        if msg and msg not in bucket:
            bucket.append(msg)
    return errors, warnings


def check(version_dir: Path) -> tuple[dict | None, list[Finding]]:
    """The body profile (None when it cannot be built) and every finding for a stored version."""
    findings = []
    urdf = version_dir / URDF_NAME
    root = ET.fromstring(urdf.read_bytes())
    findings += _inertia_findings(root)
    packages = {p.name: p for p in (version_dir / "packages").iterdir()} if (version_dir / "packages").is_dir() else {}
    answers = {}
    if (version_dir / ANSWERS_NAME).exists():
        try:
            answers = yaml.safe_load((version_dir / ANSWERS_NAME).read_text()) or {}
        except yaml.YAMLError as e:
            findings.append(Finding("error", ANSWERS_NAME, f"not valid YAML: {e}"))
        if not isinstance(answers, dict):
            findings.append(Finding("error", ANSWERS_NAME, "expected a mapping at the top level"))
            answers = {}
    else:
        findings.append(Finding("error", ANSWERS_NAME, "missing; a URDF cannot say the robot's speed and braking "
                                                       "limits, command topic or failsafe "
                                                       "(start one with: argus setup template)"))
    doc = None
    try:
        doc, problems, notes = build_profile(urdf, answers, packages=packages)
    except (UrdfError, ValueError) as e:
        findings.append(Finding("error", URDF_NAME, str(e)))
    else:
        findings += [Finding("error", "profile", p) for p in problems]
        findings += [Finding("note", URDF_NAME, n) for n in notes]
        if problems:
            doc = None
    findings += _gazebo_findings(version_dir)
    if doc is not None and (version_dir / GAZEBO_NAME).exists():
        from argus.gazebo.design_model import DesignModelError, mount_findings

        try:
            findings += [Finding("warning", "sensors", f) for f in mount_findings(version_dir, doc)]
        except (DesignModelError, ET.ParseError):
            pass  # the Gazebo model's own problems are reported above
    return doc, findings


@dataclass
class DesignVersion:
    name: str
    version: str
    created: float
    has_answers: bool
    has_gazebo_model: bool
    profile_id: str
    findings: list[Finding]
    path: Path

    @property
    def runnable(self) -> bool:
        return self.profile_id != "" and not any(f.severity == "error" for f in self.findings)


class DesignStore:
    def __init__(self, root: Path) -> None:
        self.root = root

    def add(self, bundle: Bundle) -> tuple[DesignVersion, bool]:
        """Stores and checks a bundle; (version, whether it was new). The same contents always
        give the same version, so adding twice changes nothing."""
        if not NAME_RE.match(bundle.name):
            raise ValueError(f"design name {bundle.name!r}: lower-case letters, digits, '-' and '_' only")
        for rel in bundle.files:
            p = Path(rel)
            if p.is_absolute() or ".." in p.parts:
                raise ValueError(f"design file {rel!r} is outside the design")
        version = bundle.version()
        final = self.root / bundle.name / version
        if (final / "design.json").exists():
            return self._load(final), False
        final.parent.mkdir(parents=True, exist_ok=True)
        tmp = Path(tempfile.mkdtemp(prefix=f".{version}-", dir=final.parent))
        try:
            for rel, data in bundle.files.items():
                (tmp / rel).parent.mkdir(parents=True, exist_ok=True)
                (tmp / rel).write_bytes(data)
            doc, findings = check(tmp)
            findings = bundle.findings + findings
            if doc is not None:
                (tmp / "profile.yaml").write_text(profiles.dump(doc))
            meta = {"name": bundle.name, "version": version, "created": time.time(),
                    "has_answers": bundle.has_answers, "has_gazebo_model": bundle.has_gazebo_model,
                    "profile_id": doc["id"] if doc else "", "findings": [asdict(f) for f in findings]}
            (tmp / "design.json").write_text(json.dumps(meta, indent=1))
            try:
                tmp.rename(final)
            except OSError:
                if not (final / "design.json").exists():
                    raise
                shutil.rmtree(tmp, ignore_errors=True)  # the same version, stored by another add
                return self._load(final), False
        except BaseException:
            shutil.rmtree(tmp, ignore_errors=True)
            raise
        return self._load(final), True

    def _load(self, path: Path) -> DesignVersion:
        m = json.loads((path / "design.json").read_text())
        return DesignVersion(name=m["name"], version=m["version"], created=m["created"], has_answers=m["has_answers"],
                             has_gazebo_model=m["has_gazebo_model"], profile_id=m["profile_id"],
                             findings=[Finding(**f) for f in m["findings"]], path=path)

    def versions(self, name: str) -> list[DesignVersion]:
        d = self.root / name
        if not d.is_dir():
            return []
        out = [self._load(p) for p in d.iterdir() if (p / "design.json").exists()]
        return sorted(out, key=lambda v: v.created)

    def names(self) -> list[str]:
        if not self.root.is_dir():
            return []
        return sorted(p.name for p in self.root.iterdir() if p.is_dir() and not p.name.startswith("."))

    def get(self, name: str, version: str | None = None) -> DesignVersion | None:
        versions = self.versions(name)
        if version is None:
            return versions[-1] if versions else None
        return next((v for v in versions if v.version.startswith(version)), None)

    def profile(self, v: DesignVersion) -> dict | None:
        p = v.path / "profile.yaml"
        return profiles.load(p) if p.exists() else None
