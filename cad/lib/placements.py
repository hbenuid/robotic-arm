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

The design may also MOVE capture records without editing the file: SHIFTS translates every record beyond a link
whose length the design changed, or whose far end it moved across the link (location() applies it; to_location() /
to_record() stay raw for the writers).
"""
from __future__ import annotations

import json
import pathlib
from dataclasses import dataclass

from cadgen import build123d as bd

from lib.forearm.params import DEFAULT as _FOREARM
from lib.forearm.params import LEGACY as _FOREARM_LEGACY
from lib.upper_arm.layout import arm_slide as _arm_slide
from lib.upper_arm.params import DEFAULT as _UPPER_ARM
from lib.upper_arm.params import LEGACY as _UPPER_ARM_LEGACY
from lib.wrist.params import DEFAULT as _WRIST
from lib.wrist.params import LEGACY as _WRIST_LEGACY
from lib.yaw_coupler.params import CAPTURE_FACE_X as _CAPTURE_FACE_X
from lib.yaw_coupler.params import DEFAULT as _YAW_COUPLER

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

# Occurrences of the capture the DESIGN has replaced: their records stay (the file is an immutable input) but no
# assembly table, link or total claims them - keys() leaves them out unless asked (retired=True).
#   j3_coupler#1  the elbow coupler: since 2026-09-23 the forearm roll drive's block carries its lip, boss, journal
#                 and stub (lib/forearm/params.py RollDriveParams) and the elbow 90T bolts straight into the block.
#   gt2_pulley_90t#1 / #2  the elbow's and the wrist's 90T: SolidWorks mated each hub flat onto its coupler's stub, where
#                 the lower 6806 has no room; lib/mounts.py re-seats them as gt2_pulley_90t#3 / #4 (PULLEY_SEAT_SHIFT).
RETIRED: tuple[str, ...] = ("j3_coupler#1", "gt2_pulley_90t#1", "gt2_pulley_90t#2")


@dataclass(frozen=True)
class LinkShift:
    """A link whose length the design changed: every capture record beyond it moves `along_x` along the link's +X -
    and `across_y` along its +Y, where the design moved the link's far end across it.

    `along_x` / `across_y` are the link parameters' DEFAULT - LEGACY (the part's geometry and these poses have one
    source); `axis_w` / `across_w` are the anchor record's +X / +Y in W - data, so no file read is needed
    (tests/test_placements.py checks them against the record); `moves` lists the TOP-LEVEL capture records beyond the
    link, the retired hosts included (a module's children follow its world pose; the mounted records never:
    mount_placements.py bakes their hosts' shifts in)."""

    link: str
    anchor: str
    axis_w: tuple[float, float, float]
    along_x: float
    moves: tuple[str, ...]
    across_w: tuple[float, float, float] = (0.0, 0.0, 0.0)
    across_y: float = 0.0


_BEYOND_WRIST_ROLL = ("gripper_clamp_bracket#1", "nema17_pancake#1", "gt2_pulley_20t#1", "gripper#1")
_BEYOND_WRIST_PITCH = ("gt2_pulley_90t#2", "j3_coupler#2", "wrist_link#1") + _BEYOND_WRIST_ROLL
_BEYOND_ELBOW = ("j2_link#1", "j3_coupler#1", "gt2_pulley_90t#1") + _BEYOND_WRIST_PITCH

SHIFTS: tuple[LinkShift, ...] = (
    # the upper arm: its elbow axis at slab.elbow_x from the shoulder axis, and its elbow end slid along +Y (N) with the
    # arm rising off the drive's shell (lib/upper_arm/params.py ArmParams)
    LinkShift("j1_link", "j1_link#1", (-0.673104702, 0.73826898, 0.043462315),   # [REFERENCE]
              _UPPER_ARM.slab.elbow_x - _UPPER_ARM_LEGACY.slab.elbow_x, _BEYOND_ELBOW,
              (0.064435732, 0.0, 0.997921859), _arm_slide(_UPPER_ARM) - _arm_slide(_UPPER_ARM_LEGACY)),   # [REFERENCE] its +Y
    # the forearm: its wrist_pitch axis at web.wrist_x from the elbow axis (along -X)
    LinkShift("j2_link", "j2_link#1", (0.680611117, -0.731325624, -0.043947003),   # [REFERENCE]
              _FOREARM.web.wrist_x - _FOREARM_LEGACY.web.wrist_x, _BEYOND_WRIST_PITCH),
    # the wrist body: the gripper bracket bolts to its end face at tower.block_x1
    LinkShift("wrist_link", "wrist_link#1", (-0.865418936, 0.497923174, 0.055880029),   # [REFERENCE]
              _WRIST.tower.block_x1 - _WRIST_LEGACY.tower.block_x1, _BEYOND_WRIST_ROLL),
    # the yoke: the drive held with the middle of its discs on the base_yaw axis (lib/yaw_coupler/params.py
    # ForkParams.face_x, the capture's CAPTURE_FACE_X), so the drive, the upper arm and everything beyond move along its
    # +X - the drive's axis, N
    LinkShift("j1_coupler", "j1_coupler#1", (0.064435732, 0.0, 0.997921859),   # [REFERENCE]
              _YAW_COUPLER.fork.face_x - _CAPTURE_FACE_X, ("cycloidal_drive#1", "j1_link#1") + _BEYOND_ELBOW),
    # the forearm roll: its axis runs ForearmConfig.elbow_offset above the elbow axis (the roll motor sits on the elbow
    # axis, the roll belt's centre distance under the ring - lib/forearm/params.py RollDriveParams), so the forearm and
    # everything beyond it sit that far across, along j2_link's +Y; the elbow's own records (the retired j3_coupler#1
    # and gt2_pulley_90t#1, which the elbow 90T and its bolts are hosted on) stay
    LinkShift("forearm_roll_drive", "j2_link#1", (0.680611117, -0.731325624, -0.043947003),   # [REFERENCE]
              0.0, ("j2_link#1",) + _BEYOND_WRIST_PITCH,
              (0.729805826, 0.682028468, -0.047123502), _FOREARM.elbow_offset - _FOREARM_LEGACY.elbow_offset),   # [REFERENCE] its +Y
)


def shift(key: str) -> tuple[float, float, float]:
    """The translation (W, mm) SHIFTS applies to occurrence `key`'s world pose: (0, 0, 0) for any other record."""
    t = [0.0, 0.0, 0.0]
    for s in SHIFTS:
        if key in s.moves:
            for i in range(3):
                t[i] += s.along_x * s.axis_w[i] + s.across_y * s.across_w[i]
    return (t[0], t[1], t[2])


def record_shift(record: dict) -> tuple[float, float, float]:
    """What a record's WORLD pose moves by: its own shift, or its parent module's (the children follow it)."""
    return shift(record.get("parent") or record["key"])


def shifted(key: str, point: tuple[float, float, float]) -> tuple[float, float, float]:
    """A capture point (W) that moves with occurrence `key` (robot/frames.py's joint origins)."""
    d = shift(key)
    return (point[0] + d[0], point[1] + d[1], point[2] + d[2])


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


def record_location(record: dict, frame: str = "rel") -> bd.Location:
    """A record's Location with SHIFTS applied: its world pose moves by record_shift(); so does `rel` of a top-level
    record (rel == world), never a child's (relative to its parent, which carries the shift)."""
    loc = to_location(record[frame])
    if frame == "world" or record.get("parent") is None:
        d = record_shift(record)
        if any(d):
            loc = bd.Location(d) * loc
    return loc


def location(key: str, frame: str = "rel") -> bd.Location:
    """Location of occurrence `key`; frame = "rel" (to its parent node) or "world"; SHIFTS applied."""
    if not OCCURRENCES:
        raise FileNotFoundError(MISSING_HINT)
    return record_location(OCCURRENCES[key], frame)


def keys(*, part: str | None = None, parent: str | None = None, kind: str | None = None,
         designed: bool | None = None, mounted: bool | None = None, retired: bool = False) -> list[str]:
    """Occurrence keys, optionally filtered by part name, parent key, kind (part|module),
    whether the record is a designed (code-driven) module, or whether it is a mounted occurrence
    (lib/mounts.py: a part record with a `mount` block). The RETIRED keys are left out unless
    retired=True (then every record of the file is listed)."""
    if not OCCURRENCES:
        raise FileNotFoundError(MISSING_HINT)
    return [
        k
        for k, o in OCCURRENCES.items()
        if (retired or k not in RETIRED)
        and (part is None or o["part"] == part)
        and (parent is None or o.get("parent") == parent)
        and (kind is None or o["kind"] == kind)
        and (designed is None or bool(o.get("designed", False)) == designed)
        and (mounted is None or ("mount" in o) == mounted)
    ]
