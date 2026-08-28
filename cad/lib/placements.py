"""Assembly placements extracted from the SolidWorks full-assembly STEP.

reference/placements.json is written by tools/reference/extract_placements.py. Each occurrence record
carries `rel` (placement relative to its parent node - what the assemblies compose) and
`world` (absolute, in the arm frame), each as {position, rotation_xyz_deg, matrix_3x4}.
Rotation convention: build123d `Location.to_tuple()` - intrinsic XYZ Euler angles in degrees,
rebuilt with `Location(position, rotation_xyz_deg)`. Keys: "<part>#<n>" per part occurrence
and "<module>#<n>" for sub-assembly nodes (e.g. "gripper#1"). A module record with
`designed: true` ("cycloidal_drive#1") carries only the node's pose: its contents are built by
assemblies/<module>.py from code, and its `solidworks` block keeps the SolidWorks node's
totals / bbox as a cross-check.
"""
from __future__ import annotations

import json
import pathlib

from build123d import Location

PLACEMENTS_PATH = pathlib.Path(__file__).resolve().parent.parent / "reference" / "placements.json"


def _load() -> dict:
    if not PLACEMENTS_PATH.exists():
        raise FileNotFoundError(f"missing {PLACEMENTS_PATH} - run tools/reference/extract_placements.py")
    return json.loads(PLACEMENTS_PATH.read_text(encoding="utf-8"))


DATA: dict = _load()
OCCURRENCES: dict[str, dict] = {o["key"]: o for o in DATA["occurrences"]}


def to_location(record: dict) -> Location:
    return Location(tuple(record["position"]), tuple(record["rotation_xyz_deg"]))


def location(key: str, frame: str = "rel") -> Location:
    """Location of occurrence `key`; frame = "rel" (to its parent node) or "world"."""
    return to_location(OCCURRENCES[key][frame])


def keys(*, part: str | None = None, parent: str | None = None, kind: str | None = None,
         designed: bool | None = None) -> list[str]:
    """Occurrence keys, optionally filtered by part name, parent key, kind (part|module) or
    whether the record is a designed (code-driven) module."""
    return [
        k
        for k, o in OCCURRENCES.items()
        if (part is None or o["part"] == part)
        and (parent is None or o.get("parent") == parent)
        and (kind is None or o["kind"] == kind)
        and (designed is None or bool(o.get("designed", False)) == designed)
    ]
