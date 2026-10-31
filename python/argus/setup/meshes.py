"""Axis-aligned bounds of mesh files (STL, OBJ, COLLADA), in metres in the mesh's own frame."""

from __future__ import annotations

import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def _stl(data: bytes) -> np.ndarray:
    if len(data) >= 84:
        (count,) = struct.unpack_from("<I", data, 80)
        if 84 + 50 * count == len(data):
            rec = np.frombuffer(data, dtype=np.uint8, count=50 * count, offset=84).reshape(count, 50)
            return rec[:, 12:48].copy().view("<f4").reshape(-1, 3).astype(float)
    pts = []
    for line in data.decode("ascii", errors="replace").splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[0] == "vertex":
            pts.append([float(v) for v in parts[1:]])
    return np.array(pts, dtype=float)


def _obj(text: str) -> np.ndarray:
    pts = [[float(v) for v in line.split()[1:4]] for line in text.splitlines() if line.startswith("v ")]
    return np.array(pts, dtype=float)


def _dae(text: str) -> np.ndarray:
    root = ET.fromstring(text)
    ns = root.tag[:root.tag.index("}") + 1] if root.tag.startswith("{") else ""
    unit = root.find(f"{ns}asset/{ns}unit")
    scale = float(unit.get("meter", "1")) if unit is not None else 1.0
    up = (root.findtext(f"{ns}asset/{ns}up_axis") or "Y_UP").strip()
    pts = []
    for mesh in root.iter(f"{ns}mesh"):
        verts = mesh.find(f"{ns}vertices")
        if verts is None:
            continue
        src_ids = [i.get("source", "").lstrip("#") for i in verts.findall(f"{ns}input") if i.get("semantic") == "POSITION"]
        for src in mesh.findall(f"{ns}source"):
            if src.get("id") in src_ids:
                arr = src.find(f"{ns}float_array")
                vals = np.array(arr.text.split(), dtype=float)
                pts.append(vals[: len(vals) // 3 * 3].reshape(-1, 3))
    if not pts:
        return np.zeros((0, 3))
    p = np.vstack(pts) * scale
    # Gazebo and RViz convert Y-up COLLADA files to Z-up the same way.
    if up == "Y_UP":
        p = p[:, [0, 2, 1]] * np.array([1.0, -1.0, 1.0])
    elif up == "X_UP":
        p = p[:, [2, 1, 0]] * np.array([-1.0, 1.0, 1.0])
    return p


def mesh_bounds(path: Path) -> tuple[np.ndarray, np.ndarray]:
    suffix = path.suffix.lower()
    if suffix == ".stl":
        pts = _stl(path.read_bytes())
    elif suffix == ".obj":
        pts = _obj(path.read_text(errors="replace"))
    elif suffix == ".dae":
        pts = _dae(path.read_text(errors="replace"))
    else:
        raise ValueError(f"mesh {path}: unsupported format {suffix} (STL, OBJ and DAE are read)")
    if len(pts) == 0:
        raise ValueError(f"mesh {path}: no vertices found")
    return pts.min(axis=0), pts.max(axis=0)
