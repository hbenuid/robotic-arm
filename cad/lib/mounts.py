"""The motor mounts of the belt joints, their bearings, the two re-seated 90T pulleys, and the pose of a code-driven
module the SolidWorks capture never placed - occurrences declared here, not extracted.

The SolidWorks arm cut a NEMA 17 pad into `base` (base_yaw), `j1_link` (elbow_pitch) and `j2_link`
(wrist_pitch) but never placed the motors, so reference/placements.json has no record for them.
This module DECLARES them: the 48 mm kit motor (parts/cycloidal/nema17_48mm, the drive's) on the base's pad,
a 40 mm kit motor (parts/joints/nema17_40mm) on each of the two link pads, and each motor's MKS SERVO42D board
(parts/joints/mks_servo42d) on its rear face. Each mount is a frame AS DATA
(`(position mm, rotation_xyz_deg)`, lib.datum.to_location - kernel-free, like every frame a module
declares) in the HOST occurrence's frame; tools/reference/mount_placements.py turns them into ordinary
placements.json part records (world = host world * frame; solids / volume / bbox from the built part),
which tools/reference/extract_placements.py appends on every extraction, so everything downstream
(assemblies/arm.py OCCURRENCES + GROUPS, robot/frames.py LINKS, the inertials, tools/bom.py) reads them
like any SolidWorks occurrence. The drive's own board is placed by assemblies/cycloidal_drive.py.

Part frame of the motor: mounting face z=0, body -Z, shaft +Z, D-flat +Y, cable connector on -Y. So a
mount frame puts +Z on the joint axis (the shaft through the pad toward the driven pulley) and its origin
on the pad face at the bolt-pattern centre; the spin about the axis (90 deg steps, the pattern is square)
only sets which way the connector faces [ESTIMATE]. The board's frame is the motor's, shifted to the rear face.

A MODULE mount (ModuleMount, MODULE_MOUNTS) does the same for a designed module (assemblies/<module>.py,
lib/reference.py DESIGNED_MODULES) whose pose no SolidWorks node gives - the forearm roll drive: its frame in
the host's frame puts the module's +Z on the joint axis it drives; mount_placements.py writes a
`kind: "module", designed: true` record with a `mount` block (no solids / volume: the module's own
totals are its EXPECTED), listed under both `designed_modules` and `mounted`, which world_rows() expands
like the drive's SolidWorks-placed record.

Geometry (lib/params.py, kernel-verified 2026-09-21 - tests/test_mounts.py re-checks it):
  base_yaw     base plate -Y face, pattern centre BASE_MOTOR_PATTERN_CENTRE; the 48 mm body hangs in -Y, shaft +Y
               through the plate, belt slot toward the yaw axis; motor + board reach BASE_MOTOR_STACK_PROUD (6.1 mm)
               BELOW the base's bottom face (56 mm of depth under the plate) - the base needs feet or a cut-out
  elbow_pitch  j1_link's 48 x 48 pad (outer face y = J1_MOTOR_PAD_FACE_Y, the -N side), pattern on the
               shoulder axis; shaft +N through the pad opening into the elbow drive, whose second stage runs through
               j1_link's x 128 seats (not modelled - docs/open_issues.md) to the elbow 90T
  wrist_pitch  j2_link's web (+Z face z = J2_MOTOR_WEB_FACE_Z), motor axis at x = J2_MOTOR_SLIDE_X on the side
               slots (lib/forearm/params.py: where the stock wrist belt puts it, its plug clear of the roll wall);
               body +N, shaft -N through the web, the 20T under it

The BEARINGS (parts/joints/bearing_6806, BEARING_MOUNTS): the 6806-2RS pair each belt joint's housing takes, one
each side of the lip that splits its bore - in the base (base_yaw), in j1_link's elbow end (elbow_pitch), in
j2_link's wrist boss (wrist_pitch) - each standing on the lip (its axis +Z on the joint axis), the coupler side
first. They ride with the housing's link.
The PULLEYS (PULLEY_MOUNTS): the elbow's and the wrist's 90T, whose SolidWorks poses (gt2_pulley_90t#1 / #2,
retired - lib/placements.py) left no room for the lower bearing, re-declared PULLEY_SEAT_SHIFT further out along
their own axis (+Y): the host of such a mount is the capture pose it corrects, its spin kept exactly.
"""
from __future__ import annotations

from dataclasses import dataclass

from lib.base.params import DEFAULT as _BASE
from lib.forearm import module_frame_in_host
from lib.forearm.params import DEFAULT as _FOREARM
from lib.params import (
    BASE_MOTOR_PATTERN_CENTRE,
    BEARING_6806_WIDTH,
    CYCLOIDAL_MOTOR_BODY_LEN,
    J1_MOTOR_PAD_FACE_Y,
    J2_MOTOR_SLIDE_X,
    J2_MOTOR_WEB_FACE_Z,
    NEMA17_40_BODY_LEN,
    PULLEY_SEAT_SHIFT,
)
from lib.upper_arm.params import DEFAULT as _UPPER_ARM

MOTOR_48, MOTOR_40, BOARD = "nema17_48mm", "nema17_40mm", "mks_servo42d"
MOTORS = (MOTOR_48, MOTOR_40)
BEARING, PULLEY = "bearing_6806", "gt2_pulley_90t"

# The part-frame axis tools/reference/mount_placements.py checks for each mounted part, and whether it lies ON its
# joint's axis (a motor's shaft runs beside its joint, parallel; a bearing or a pulley sits on it).
AXES: dict[str, tuple[tuple[float, float, float], bool]] = {
    MOTOR_48: ((0.0, 0.0, 1.0), False), MOTOR_40: ((0.0, 0.0, 1.0), False),
    BEARING: ((0.0, 0.0, 1.0), True), PULLEY: ((0.0, 1.0, 0.0), True),
}


def board_frame(body_length: float) -> tuple:
    """The board's frame in its motor's: z=0 at the motor's rear face (the board stack in -Z)."""
    return ((0.0, 0.0, -body_length), (0.0, 0.0, 0.0))


BOARD_FRAME_48 = board_frame(CYCLOIDAL_MOTOR_BODY_LEN)   # behind a 48 mm motor
BOARD_FRAME_40 = board_frame(NEMA17_40_BODY_LEN)          # behind a 40 mm motor


@dataclass(frozen=True)
class Mount:
    key: str        # occurrence key "<part>#<n>" (placements.json, assemblies/arm.py, robot/frames.py LINKS)
    part: str
    host: str       # the occurrence it is bolted to: a placements.json key, or the motor key for a board
    link: str       # the robot/frames.py link it rides with
    joint: str      # the joint it drives
    frame: tuple    # ((x, y, z), (rx, ry, rz)) in the host's frame - lib.datum.to_location(frame)
    note: str = ""


MOTOR_MOUNTS: tuple[Mount, ...] = (
    Mount("nema17_48mm#1", MOTOR_48, "base#1", "base_link", "base_yaw",
          (BASE_MOTOR_PATTERN_CENTRE, (-90.0, 0.0, 270.0)),
          "the 48 mm motor under the base plate, shaft up through it; motor + board hang BASE_MOTOR_STACK_PROUD below "
          "the base's bottom face; connector toward +X [ESTIMATE]"),
    Mount("mks_servo42d#1", BOARD, "nema17_48mm#1", "base_link", "base_yaw", BOARD_FRAME_48),
    Mount("nema17_40mm#2", MOTOR_40, "j1_link#1", "upper_arm_link", "elbow_pitch",
          ((0.0, J1_MOTOR_PAD_FACE_Y, 0.0), (-90.0, 0.0, 90.0)),
          "on j1_link's pad, on the shoulder axis (on its 4 holes), shaft +N; connector toward the elbow [ESTIMATE]"),
    Mount("mks_servo42d#2", BOARD, "nema17_40mm#2", "upper_arm_link", "elbow_pitch", BOARD_FRAME_40),
    Mount("nema17_40mm#3", MOTOR_40, "j2_link#1", "forearm_link", "wrist_pitch",
          ((J2_MOTOR_SLIDE_X, 0.0, J2_MOTOR_WEB_FACE_Z), (180.0, 0.0, 90.0)),
          "on j2_link's web (+Z face), shaft -N; slide position J2_MOTOR_SLIDE_X [ESTIMATE]; connector toward the elbow [ESTIMATE]"),
    Mount("mks_servo42d#3", BOARD, "nema17_40mm#3", "forearm_link", "wrist_pitch", BOARD_FRAME_40),
)


def _bearing_pair(key_n: int, host: str, link: str, joint: str, lip: tuple, at: tuple, rot: tuple, what: str) -> tuple[Mount, Mount]:
    """A joint's two 6806s on the lip (lip = its two faces along the joint axis, `at` = the axis' place across it):
    the coupler-side one standing on the lip's upper face, the pulley-side one under its lower face."""
    def frame(station: float) -> tuple:
        return (tuple(station if c is None else c for c in at), rot)
    return (Mount(f"bearing_6806#{key_n}", BEARING, host, link, joint, frame(lip[1]), f"{what}: the upper (coupler-side) 6806, on the lip"),
            Mount(f"bearing_6806#{key_n + 1}", BEARING, host, link, joint, frame(lip[0] - BEARING_6806_WIDTH),
                  f"{what}: the lower (pulley-side) 6806, under the lip"))


BEARING_MOUNTS: tuple[Mount, ...] = (
    *_bearing_pair(1, "base#1", "base_link", "base_yaw", _BASE.bore.lip_y, (0.0, None, 0.0), (-90.0, 0.0, 0.0),
                   "the base's bore (axis +Y)"),
    *_bearing_pair(3, "j1_link#1", "upper_arm_link", "elbow_pitch", _UPPER_ARM.elbow.lip_y, (_UPPER_ARM.slab.elbow_x, None, 0.0),
                   (-90.0, 0.0, 0.0), "j1_link's elbow bore (axis +Y)"),
    *_bearing_pair(5, "j2_link#1", "forearm_link", "wrist_pitch", _FOREARM.boss.lip_z, (_FOREARM.web.wrist_x, 0.0, None),
                   (0.0, 0.0, 0.0), "j2_link's wrist boss (axis +Z)"),
)
PULLEY_MOUNTS: tuple[Mount, ...] = (
    Mount("gt2_pulley_90t#3", PULLEY, "gt2_pulley_90t#1", "elbow_link", "elbow_pitch", ((0.0, PULLEY_SEAT_SHIFT, 0.0), (0.0, 0.0, 0.0)),
          "the elbow 90T, PULLEY_SEAT_SHIFT out from its SolidWorks pose (retired): its hub in the lower elbow bearing"),
    Mount("gt2_pulley_90t#4", PULLEY, "gt2_pulley_90t#2", "wrist_pitch_link", "wrist_pitch", ((0.0, PULLEY_SEAT_SHIFT, 0.0), (0.0, 0.0, 0.0)),
          "the wrist 90T, PULLEY_SEAT_SHIFT out from its SolidWorks pose (retired): its hub in the lower wrist bearing"),
)
MOUNTS: tuple[Mount, ...] = MOTOR_MOUNTS + BEARING_MOUNTS + PULLEY_MOUNTS
BY_KEY: dict[str, Mount] = {m.key: m for m in MOUNTS}


@dataclass(frozen=True)
class ModuleMount:
    key: str        # occurrence key "<module>#<n>" (placements.json designed_modules, assemblies/arm.py, LINKS with ":<body>")
    module: str     # the designed module (assemblies/<module>.py, lib/reference.py DESIGNED_MODULES)
    host: str       # the SolidWorks occurrence the module's stator is bolted to
    joint: str      # the joint the module IS (robot/frames.py): the module's +Z lies on its axis
    frame: tuple    # ((x, y, z), (rx, ry, rz)) in the host's frame - lib.datum.to_location(frame)
    note: str = ""


MODULE_MOUNTS: tuple[ModuleMount, ...] = (
    ModuleMount("forearm_roll_drive#1", "forearm_roll_drive", "j2_link#1", "forearm_roll", module_frame_in_host(),
                "the roll drive on j2_link's roll axis (y 0, z FOREARM_ROLL_AXIS_Z): module +Z = host -X toward the wrist, "
                "module +X = host +Z (N), +Y = up in the swing plane (the motor side); the block IS the elbow coupler: its stub turns in "
                "j1_link's bore and the elbow 90T bolts into it (j3_coupler#1 retired, lib/placements.py RETIRED)"),
)
MODULES_BY_KEY: dict[str, ModuleMount] = {m.key: m for m in MODULE_MOUNTS}


def keys() -> list[str]:
    """The mounted part occurrence keys, in declaration order (motor, its board, ...)."""
    return [m.key for m in MOUNTS]


def module_keys() -> list[str]:
    """The mounted designed-module keys, in declaration order."""
    return [m.key for m in MODULE_MOUNTS]
