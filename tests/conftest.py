import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BUILD_PYTHON = ROOT / "build" / "python"

for path in (ROOT / "python", BUILD_PYTHON):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

if not (BUILD_PYTHON / "argus").is_dir():
    raise RuntimeError("build/python is missing; run tools/check.sh (or build with CMake) first")
