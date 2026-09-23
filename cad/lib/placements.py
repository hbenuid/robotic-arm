"""Assembly placements extracted from the SolidWorks full-assembly STEP.

reference/placements.json is written by tools/reference/extract_placements.py. Each occurrence record
carries `rel` (placement relative to its parent node - what the assemblies compose) and
`world` (absolute, in the arm frame), each as {position, rotation_xyz_deg, matrix_3x4}.
Rotation convention: build123d `Location.to_tuple()` - intrinsic XYZ Euler angles in degrees,
rebuilt with `Location(position, rotation_xyz_deg)`. Keys: "<part>#<n>" per part occurrence
and "<module>#<n>" for sub-assembly nodes (e.g. "gripper#1"). A module record with
`designed: true` ("cycloidal_drive#1") carries only the node's pose: its contents are built by
assemblies/<module>.py from code, and its `solidworks` block keeps the SolidWorks node's
totals / bbox as a cross-check. A part record with a `mount` block (keys listed under `mounted`) is
an occurrence the SolidWorks capture never contained - the belt joints' motors and their MKS boards,
declared as frames in lib/mounts.py and materialised by tools/reference/mount_placements.py
(parent None, rel == world = host world * mount frame). A MODULE record with a `mount` block
("forearm_roll_drive#1", listed under both `designed_modules` and `mounted`) is a designed module the capture
never placed - its pose is a lib/mounts.py ModuleMount, its contents assemblies/<module>.py.
"""
from __future__ import annotations

import json
import pathlib

from cadgen import build123d as bd

PLACEMENTS_PATH = pathlib.Path(__file__).resolve().parent.parent / "reference" / "placements.json"


MISSING_HINT = f"missing {PLACEMENTS_PATH} - run tools/reference/extract_placements.py"


def _load() -> dict:
    """The document; an empty one while the file does not exist (so the tools that WRITE it can import
    this module) - location() / keys() then raise MISSING_HINT."""
    if not PLACEMENTS_PATH.exists():
        return {"expected": {"leaf_occurrences": 0, "solids": 0, "solid_volume": 0.0},
                "designed_modules": [], "mounted": [], "occurrences": [], "skipped": []}
    return json.loads(PLACEMENTS_PATH.read_text(encoding="utf-8"))


DATA: dict = _load()
OCCURRENCES: dict[str, dict] = {o["key"]: o for o in DATA["occurrences"]}


def to_location(record: dict) -> bd.Location:
    return bd.Location(tuple(record["position"]), tuple(record["rotation_xyz_deg"]))


def to_record(loc: bd.Location) -> dict:
    """The inverse of to_location(): a Location as a placements.json frame record (the writers'
    format - tools/reference/extract_placements.py and mount_placements.py)."""
    pos, rot = (tuple(v) for v in tuple(loc))   # (position Vector, XYZ-Euler-degrees Vector)
    t = loc.wrapped.Transformation()
    return {
        "position": [round(v, 6) for v in pos],
        "rotation_xyz_deg": [round(v, 6) for v in rot],
        "matrix_3x4": [[round(t.Value(i, j), 9) for j in (1, 2, 3, 4)] for i in (1, 2, 3)],
    }


def location(key: str, frame: str = "rel") -> bd.Location:
    """Location of occurrence `key`; frame = "rel" (to its parent node) or "world"."""
    if not OCCURRENCES:
        raise FileNotFoundError(MISSING_HINT)
    return to_location(OCCURRENCES[key][frame])


def keys(*, part: str | None = None, parent: str | None = None, kind: str | None = None,
         designed: bool | None = None, mounted: bool | None = None) -> list[str]:
    """Occurrence keys, optionally filtered by part name, parent key, kind (part|module),
    whether the record is a designed (code-driven) module, or whether it is a mounted occurrence
    (lib/mounts.py: a part record with a `mount` block)."""
    if not OCCURRENCES:
        raise FileNotFoundError(MISSING_HINT)
    return [
        k
        for k, o in OCCURRENCES.items()
        if (part is None or o["part"] == part)
        and (parent is None or o.get("parent") == parent)
        and (kind is None or o["kind"] == kind)
        and (designed is None or bool(o.get("designed", False)) == designed)
        and (mounted is None or ("mount" in o) == mounted)
    ]
