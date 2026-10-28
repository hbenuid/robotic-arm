"""Assembly placements extracted from the SolidWorks full-assembly STEP.

reference/placements.json is written by tools/reference/extract_placements.py. Each occurrence record
carries `rel` (placement relative to its parent node - what the assemblies compose) and
`world` (absolute, in the arm frame), each as {position, rotation_xyz_deg, matrix_3x4}.
Rotation convention: build123d `Location.orientation` - intrinsic XYZ Euler angles in degrees,
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
whose length the design changed, or whose far end it moved across the link, and TURNS then turns the records of a link
the design turned about an axis of its own (location() applies both; to_location() / to_record() stay raw for the
writers).
"""
from __future__ import annotations

import json
import math
import pathlib
from dataclasses import dataclass

import build123d as bd

from lib.forearm.params import DEFAULT as _FOREARM
from lib.forearm.params import LEGACY as _FOREARM_LEGACY
from lib.upper_arm.layout import arm_slide as _arm_slide
from lib.upper_arm.params import DEFAULT as _UPPER_ARM
from lib.upper_arm.params import LEGACY as _UPPER_ARM_LEGACY
from lib.wrist.layout import stack_positions as _wrist_stack
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
#                 and stub (lib/forearm/params.py RollDriveParams) and the elbow 108T bolts straight into the block.
#   gt2_pulley_90t#1 / #2  the elbow's and the wrist's 90T: SolidWorks mated each hub flat onto its coupler's stub, where
#                 the lower 6806 has no room; lib/mounts.py re-seats them (PULLEY_SEAT_SHIFT) as gt2_pulley_108t#1 (the
#                 elbow's, more teeth: lib/pulley/params.py ELBOW) / gt2_pulley_90t#4.
#   nema17_pancake#1 / gt2_pulley_20t#1  the wrist-roll pancake and its 20T on the SolidWorks roll axis: since 2026-10-14
#                 the wrist roll drive (assemblies/wrist_roll_drive.py) carries both, the pancake on the wrist-pitch axis.
RETIRED: tuple[str, ...] = ("j3_coupler#1", "gt2_pulley_90t#1", "gt2_pulley_90t#2", "nema17_pancake#1", "gt2_pulley_20t#1")


@dataclass(frozen=True)
class LinkShift:
    """A link (or a drive on it) the design changed: every capture record beyond it moves `along_x` along the link's
    +X, where the design changed the link's length - and `across_y` along its +Y, where the design moved the link's far
    end across it, and `normal_z` along its +Z, where the design moved it off the link's plane.

    `along_x` / `across_y` / `normal_z` come from the design's parameters - mostly their DEFAULT - LEGACY, or a
    designed station less its capture value (the comment over each SHIFTS row) - so the part's geometry and these
    poses have one source; `axis_w` / `across_w` / `normal_w` are the anchor record's +X / +Y / +Z in W - data, so no
    file read is needed (tests/test_placements.py checks them against the record); `moves` lists the TOP-LEVEL capture
    records beyond the link, the retired hosts included (a module's children follow its world pose; the mounted
    records never: mount_placements.py bakes their hosts' shifts in)."""

    link: str
    anchor: str
    axis_w: tuple[float, float, float]
    along_x: float
    moves: tuple[str, ...]
    across_w: tuple[float, float, float] = (0.0, 0.0, 0.0)
    across_y: float = 0.0
    normal_w: tuple[float, float, float] = (0.0, 0.0, 0.0)
    normal_z: float = 0.0


_BEYOND_WRIST_ROLL = ("nema17_pancake#1", "gt2_pulley_20t#1", "gripper#1")
# The gripper's connector, its plate's hub-side face (y plate_t) along wrist_link#1's +X in the capture: the station the
# wrist roll drive's LinkShift moves the gripper from (tests/test_placements.py checks it against the records)
_CONNECTOR_CAPTURE_X = 79.65   # [REFERENCE] the connector's origin (its plate's servo-side face) at 84.15, less the 4.5 plate
_BEYOND_WRIST_PITCH = ("gt2_pulley_90t#2", "j3_coupler#2", "wrist_link#1") + _BEYOND_WRIST_ROLL
_BEYOND_ELBOW = ("j2_link#1", "j3_coupler#1", "gt2_pulley_90t#1") + _BEYOND_WRIST_PITCH

SHIFTS: tuple[LinkShift, ...] = (
    # the upper arm: its elbow axis at slab.elbow_x from the shoulder axis, and its elbow end slid along +Y (N) with the
    # arm rising off the drive's shell (lib/upper_arm/params.py ArmParams)
    LinkShift("j1_link", "j1_link#1", (-0.673104702, 0.73826898, 0.043462315),   # [REFERENCE]
              _UPPER_ARM.slab.elbow_x - _UPPER_ARM_LEGACY.slab.elbow_x, _BEYOND_ELBOW,
              (0.064435732, 0.0, 0.997921859), _arm_slide(_UPPER_ARM) - _arm_slide(_UPPER_ARM_LEGACY)),   # [REFERENCE] its +Y
    # the forearm: its wrist_pitch axis at web.wrist_x from the elbow axis (along -X); the wrist dropped along the axis
    # with the web toward its belt side (-Z: lib/forearm/params.py WEB_DROP), on the boss's bearings
    LinkShift("j2_link", "j2_link#1", (0.680611117, -0.731325624, -0.043947003),   # [REFERENCE]
              _FOREARM.web.wrist_x - _FOREARM_LEGACY.web.wrist_x, _BEYOND_WRIST_PITCH,
              normal_w=(0.064435732, 0.0, 0.997921859),   # [REFERENCE] its +Z
              normal_z=_FOREARM.web.z0 - _FOREARM_LEGACY.web.z0),
    # the wrist roll drive: the gripper rolls about R', WristRollParams.centre_distance across from the pitch axis (along
    # wrist_link's Y, on its `side`: -Y) on the plate's underside (axis_h over its coupler face: along its -Z from the
    # SolidWorks roll axis at EndFaceParams.axis_z), its connector bolted to the roll pulley's face (along its +X)
    LinkShift("wrist_roll_drive", "wrist_link#1", (-0.865418936, 0.497923174, 0.055880029),   # [REFERENCE]
              _WRIST.plate.pitch_x + _wrist_stack(_WRIST)["z_connector"] - _CONNECTOR_CAPTURE_X, _BEYOND_WRIST_ROLL,
              (0.49688842, 0.867221144, -0.032084044), _WRIST.roll.side * _WRIST.roll.centre_distance,   # [REFERENCE] its +Y
              (-0.064435732, 0.0, -0.997921859),   # [REFERENCE] its +Z
              (_WRIST.plate.thickness - _WRIST.roll.axis_h) - _WRIST_LEGACY.end_face.axis_z),
    # the yoke: the drive held with the middle of its discs on the base_yaw axis (lib/yaw_coupler/params.py
    # ForkParams.face_x, the capture's CAPTURE_FACE_X), so the drive, the upper arm and everything beyond move along its
    # +X - the drive's axis, N
    LinkShift("j1_coupler", "j1_coupler#1", (0.064435732, 0.0, 0.997921859),   # [REFERENCE]
              _YAW_COUPLER.fork.face_x - _CAPTURE_FACE_X, ("cycloidal_drive#1", "j1_link#1") + _BEYOND_ELBOW),
    # the forearm roll: its axis runs ForearmConfig.elbow_offset above the elbow axis (the roll motor sits on the elbow
    # axis, the drive's centre distance under the ring - lib/forearm/params.py RollDriveParams), so the forearm and
    # everything beyond it sit that far across, along j2_link's +Y; the elbow's own records (the retired j3_coupler#1
    # and gt2_pulley_90t#1, which the elbow 108T and its bolts are hosted on) stay
    LinkShift("forearm_roll_drive", "j2_link#1", (0.680611117, -0.731325624, -0.043947003),   # [REFERENCE]
              0.0, ("j2_link#1",) + _BEYOND_WRIST_PITCH,
              (0.729805826, 0.682028468, -0.047123502), _FOREARM.elbow_offset - _FOREARM_LEGACY.elbow_offset),   # [REFERENCE] its +Y
)


@dataclass(frozen=True)
class LinkTurn:
    """A link the design turned about an axis of its own: every capture record in `moves` (the link's own record and all
    beyond it, as a LinkShift's) turns `deg` about the line through `point` along `axis` - both in the anchor record's
    frame, the anchor SHIFTED (the turn comes after the translations). `deg` is the link parameter's DEFAULT - LEGACY;
    `axis_w` / `across_w` are the anchor's +X / +Y in W, data as a LinkShift's (tests/test_placements.py checks them).
    A designed module mounted on a turned record keeps its pose - it drives the joint the turn is about:
    tools/reference/mount_placements.py resolves a ModuleMount's host unturned (location(turned=False))."""

    link: str
    anchor: str
    axis_w: tuple[float, float, float]
    across_w: tuple[float, float, float]
    point: tuple[float, float, float]
    axis: tuple[float, float, float]
    deg: float
    moves: tuple[str, ...]


TURNS: tuple[LinkTurn, ...] = (
    # the forearm half a turn about its roll axis (y 0, z axis_z, along j2_link's +X; lib/forearm/params.py roll_turn_deg):
    # the wrist-pitch motor on the upper arm's side of it, the wrist and the gripper rolled with it; the roll drive (its
    # rotor too) keeps its pose and j2_link's wall bolts onto the pulley turned (its 4 bolts at 0 / 90 / 180 / 270 deg)
    LinkTurn("j2_link", "j2_link#1", (0.680611117, -0.731325624, -0.043947003), (0.729805826, 0.682028468, -0.047123502),   # [REFERENCE]
             (0.0, 0.0, _FOREARM.roll_end.axis_z), (1.0, 0.0, 0.0), _FOREARM.roll_turn_deg - _FOREARM_LEGACY.roll_turn_deg,
             ("j2_link#1",) + _BEYOND_WRIST_PITCH),
)


def _cross(a, b) -> tuple[float, float, float]:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _dot(a, b) -> float:
    return sum(x * y for x, y in zip(a, b, strict=True))


def _turns_of(key: str) -> list[LinkTurn]:
    return [t for t in TURNS if key in t.moves and t.deg]


def turn_line(t: LinkTurn) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """(a point on the turn's axis, its unit direction) in W: `point` / `axis` of the anchor's record, shifted."""
    o, d = OCCURRENCES[t.anchor]["world"]["position"], shift(t.anchor)
    ex, ey = t.axis_w, t.across_w
    ez = _cross(ex, ey)
    c = tuple(o[i] + d[i] + t.point[0] * ex[i] + t.point[1] * ey[i] + t.point[2] * ez[i] for i in range(3))
    u = tuple(t.axis[0] * ex[i] + t.axis[1] * ey[i] + t.axis[2] * ez[i] for i in range(3))
    n = math.sqrt(_dot(u, u))
    return (c[0], c[1], c[2]), (u[0] / n, u[1] / n, u[2] / n)


def _rotate(v, u, deg: float) -> tuple[float, float, float]:
    """`v` turned `deg` about the unit direction `u` (Rodrigues)."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    k, d = _cross(u, v), _dot(u, v)
    return (v[0] * c + k[0] * s + u[0] * d * (1.0 - c), v[1] * c + k[1] * s + u[1] * d * (1.0 - c),
            v[2] * c + k[2] * s + u[2] * d * (1.0 - c))


def is_turned(key: str) -> bool:
    """Whether a TURN moves occurrence `key` (a module's children follow it: pass the module's key)."""
    return bool(_turns_of(key))


def turned(key: str, direction: tuple[float, float, float]) -> tuple[float, float, float]:
    """A capture direction (W) that turns with occurrence `key` (robot/frames.py's axes beyond a turned link)."""
    for t in _turns_of(key):
        direction = _rotate(direction, turn_line(t)[1], t.deg)
    return direction


def shift(key: str) -> tuple[float, float, float]:
    """The translation (W, mm) SHIFTS applies to occurrence `key`'s world pose: (0, 0, 0) for any other record."""
    t = [0.0, 0.0, 0.0]
    for s in SHIFTS:
        if key in s.moves:
            for i in range(3):
                t[i] += s.along_x * s.axis_w[i] + s.across_y * s.across_w[i] + s.normal_z * s.normal_w[i]
    return (t[0], t[1], t[2])


def record_shift(record: dict) -> tuple[float, float, float]:
    """What a record's WORLD pose moves by: its own shift, or its parent module's (the children follow it)."""
    return shift(record.get("parent") or record["key"])


def shifted(key: str, point: tuple[float, float, float], *, turned: bool = True) -> tuple[float, float, float]:
    """A capture point (W) that moves with occurrence `key` (robot/frames.py's joint origins): SHIFTS, then TURNS
    (`turned=False`: SHIFTS only - for a point ON a turn's axis, which the turn leaves where it is)."""
    d = shift(key)
    p = (point[0] + d[0], point[1] + d[1], point[2] + d[2])
    for t in _turns_of(key) if turned else ():
        c, u = turn_line(t)
        r = _rotate((p[0] - c[0], p[1] - c[1], p[2] - c[2]), u, t.deg)
        p = (c[0] + r[0], c[1] + r[1], c[2] + r[2])
    return p


def _turn_location(t: LinkTurn) -> bd.Location:
    c, u = turn_line(t)
    f = bd.Location(bd.Plane(origin=c, z_dir=u))
    return f * bd.Location((0.0, 0.0, 0.0), (0.0, 0.0, t.deg)) * f.inverse()


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


def record_location(record: dict, frame: str = "rel", *, turned: bool = True) -> bd.Location:
    """A record's Location with SHIFTS and TURNS applied: its world pose moves by record_shift() and turns with its TURNS
    (`turned=False`: shifted only); so does `rel` of a top-level record (rel == world), never a child's (relative to its
    parent, which carries both)."""
    loc = to_location(record[frame])
    if frame == "world" or record.get("parent") is None:
        d = record_shift(record)
        if any(d):
            loc = bd.Location(d) * loc
        if turned:
            for t in _turns_of(record.get("parent") or record["key"]):
                loc = _turn_location(t) * loc
    return loc


def location(key: str, frame: str = "rel", *, turned: bool = True) -> bd.Location:
    """Location of occurrence `key`; frame = "rel" (to its parent node) or "world"; SHIFTS and TURNS applied (`turned=False`:
    SHIFTS only - the frame a turned link had before its turn)."""
    if not OCCURRENCES:
        raise FileNotFoundError(MISSING_HINT)
    return record_location(OCCURRENCES[key], frame, turned=turned)


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
