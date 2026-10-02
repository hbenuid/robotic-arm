"""The motor mounts of the belt joints, their bearings, their driven pulleys (the elbow's and the wrist's 90T re-seated,
the base_yaw 120T placed) and the bolts that clamp them, and the pose of a code-driven module the SolidWorks capture never
placed - occurrences declared here, not extracted.

The SolidWorks arm cut a NEMA 17 pad into `base` (base_yaw), `j1_link` (elbow_pitch) and `j2_link`
(wrist_pitch) but never placed the motors, so reference/placements.json has no record for them.
This module DECLARES them: the 48 mm kit motor (parts/cycloidal/nema17_48mm, the drive's) on the base's pad - now
on the base's bolt-on motor mount (BASE_MOUNTS: parts/base/base_motor_mount and its M4 screws + nuts),
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
  base_yaw     the motor mount's plate -Y face, pattern centre BASE_MOTOR_PATTERN_CENTRE (the slots' middle); the
               48 mm body hangs in -Y, shaft +Y through the plate into the plane of the base_yaw 120T's teeth; motor +
               board end BASE_MOTOR_TABLE_CLEAR above the base's bottom face (lowered under them, lib/base/params.py
               BOARD_CLEAR)
  elbow_pitch  j1_link's motor plate under the arm's outer face (y = J1_MOTOR_PAD_FACE_Y, the -N side;
               lib/upper_arm/params.py ArmParams), pattern on the pad's axis (PadParams.x: ELBOW_MOTOR_CENTRES short of
               the elbow axis - the shoulder axis is the drive's yoke's); the body +N down the square hole through the
               arm, standing out on the forearm's side; shaft -N through the plate, its 20T on the belt to the elbow 90T
               (a stock 280-2GT; not modelled - docs/open_issues.md)
  wrist_pitch  j2_link's web (+Z face z = J2_MOTOR_WEB_FACE_Z), motor axis at x = J2_MOTOR_SLIDE_X on the side
               slots (lib/forearm/params.py: where the stock wrist belt puts it, its plug clear of the roll wall);
               body +N, shaft -N through the web, the 20T under it

The BEARINGS (parts/joints/bearing_6806, BEARING_MOUNTS): the 6806-2RS pair each belt joint's housing takes, one
each side of the lip that splits its bore - in the base (base_yaw), in j1_link's elbow end (elbow_pitch), in
j2_link's wrist boss (wrist_pitch) - each standing on the lip (its axis +Z on the joint axis), the coupler side
first. They ride with the housing's link.
The THRUST BEARING (THRUST_MOUNTS): the base_yaw joint's axial load - j1_coupler and all it carries - on a needle cage
(parts/base/bearing_axk6590) between two washers (parts/base/washer_as6590) in the base's groove round its seat ring:
the lower washer on the groove's floor and the cage on it ride with the base, the upper washer under j1_coupler's seat
(lib/yaw_coupler/params.py DEFAULT) with the coupler.
The PULLEYS (PULLEY_MOUNTS): the elbow's and the wrist's 90T, whose SolidWorks poses (gt2_pulley_90t#1 / #2,
retired - lib/placements.py) left no room for the lower bearing, re-declared PULLEY_SEAT_SHIFT further out along
their own axis (+Y): the host of such a mount is the capture pose it corrects. The wrist's keeps its spin exactly; the
elbow's turns RollDriveParams.pulley_bolt_deg about its axis with the elbow block's bolt pattern (lib/forearm/params.py:
its nut channels would otherwise stop short of the ring cavity). The base_yaw pulley (gt2_pulley_120t#1, the 90T with
120 teeth: lib/pulley/params.py YAW) the capture never had: hosted on j1_coupler, upside down under it (its +Y down the axis) - its hub end on the stub's end in the plane of
the lip's lower face, its journal in the lower base bearing, its ring under that bearing's inner ring - and turned
HubParams.hole_deg about the axis onto the stub's diagonal holes (lib/yaw_coupler/params.py). Bolted there it holds the
coupler down on the thrust bearing: the lower bearing's outer ring stands under the base's lip.
The PULLEY BOLTS (FASTENER_MOUNTS): each pulley's 4x M4 screws and nuts, purchased pattern parts
(parts/joints/{elbow,wrist}_pulley_{screws,nuts}, parts/base/yaw_pulley_{screws,nuts}) centred on the joint axis (their
+Z on it). A screw set is hosted on its pulley, its heads' bearing face on the pulley's outer face
(GT2_PULLEY_90T_FACE_Y) and its shanks into the hub; a nut set is hosted on its screw set, its bearing face where the
host's seat lies along the screws - the elbow block's channel seats (RollDriveParams.nut_seat_x), j3_coupler's pocket
floors (CouplerParams.nut_depth), j1_coupler's pockets in the floor of the pocket over its hub (HubParams.nut_depth
under YokeParams.pocket_y0).
"""
from __future__ import annotations

from dataclasses import dataclass

from lib.base.layout import joint_stations
from lib.base.params import DEFAULT as _BASE
from lib.bearings import THRUST_WASHER_WIDTH
from lib.coupler.params import DEFAULT as _COUPLER
from lib.forearm import module_frame_in_host
from lib.forearm.params import DEFAULT as _FOREARM
from lib.params import (
    BASE_MOTOR_PATTERN_CENTRE,
    BEARING_6806_WIDTH,
    CYCLOIDAL_MOTOR_BODY_LEN,
    GT2_PULLEY_90T_FACE_Y,
    J1_ARM_SLIDE,
    J1_MOTOR_PAD_FACE_Y,
    J2_MOTOR_SLIDE_X,
    J2_MOTOR_WEB_FACE_Z,
    NEMA17_40_BODY_LEN,
    PULLEY_SEAT_SHIFT,
)
from lib.upper_arm.params import DEFAULT as _UPPER_ARM
from lib.yaw_coupler.params import DEFAULT as _YAW_COUPLER

MOTOR_48, MOTOR_40, BOARD = "nema17_48mm", "nema17_40mm", "mks_servo42d"
MOTORS = (MOTOR_48, MOTOR_40)
BEARING, PULLEY, YAW_PULLEY = "bearing_6806", "gt2_pulley_90t", "gt2_pulley_120t"
THRUST_CAGE, THRUST_WASHER = "bearing_axk6590", "washer_as6590"
PULLEY_BOLTS = ("elbow_pulley_screws", "elbow_pulley_nuts", "wrist_pulley_screws", "wrist_pulley_nuts",
                "yaw_pulley_screws", "yaw_pulley_nuts")
BASE_MOUNT, BASE_MOUNT_SCREWS, BASE_MOUNT_NUTS = "base_motor_mount", "base_motor_mount_screws", "base_motor_mount_nuts"
YOKE_CAP = "j1_coupler_cap"

# The part-frame axis tools/reference/mount_placements.py checks for each mounted part, and whether it lies ON its
# joint's axis (a motor's shaft runs beside its joint, parallel; a bearing, a pulley or a pulley-bolt pattern sits on it).
AXES: dict[str, tuple[tuple[float, float, float], bool]] = {
    MOTOR_48: ((0.0, 0.0, 1.0), False), MOTOR_40: ((0.0, 0.0, 1.0), False),
    BEARING: ((0.0, 0.0, 1.0), True), PULLEY: ((0.0, 1.0, 0.0), True), YAW_PULLEY: ((0.0, 1.0, 0.0), True),
    THRUST_CAGE: ((0.0, 0.0, 1.0), True), THRUST_WASHER: ((0.0, 0.0, 1.0), True),
    **{part: ((0.0, 0.0, 1.0), True) for part in PULLEY_BOLTS},
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


# The base's motor lobe, a part of its own bolted to the base (lib/base/params.py JointParams), and the 4x M4 that hold
# it: the mount in the base's part frame; the screws' +Z down the base's -X from the heads on the mount's ears (the
# pattern's (x, y) = the base's (z, y)); the nuts on the screws, their bearing faces nut.h inside the base's posts.
_JOINT = joint_stations(_BASE)
BASE_MOUNTS: tuple[Mount, ...] = (
    Mount("base_motor_mount#1", BASE_MOUNT, "base#1", "base_link", "base_yaw", ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
          "the motor's box (its plate the motor's seat), bolted to the base's posts at the joint face; built in the base's part frame"),
    Mount("base_motor_mount_screws#1", BASE_MOUNT_SCREWS, "base_motor_mount#1", "base_link", "base_yaw",
          ((_JOINT["x_head"], 0.0, 0.0), (0.0, -90.0, 0.0)),
          "the mount's 4x M4 x screw_len: heads on its ears' outer faces, along -X through the ears and the base's posts"),
    Mount("base_motor_mount_nuts#1", BASE_MOUNT_NUTS, "base_motor_mount_screws#1", "base_link", "base_yaw",
          ((0.0, 0.0, round(_JOINT["x_head"] - _JOINT["x_nut_face"], 6)), (0.0, 0.0, 0.0)),
          "the screws' nuts in the base posts' hex pockets, their outer faces flush with the posts' back faces, a corner up"),
)
MOTOR_MOUNTS: tuple[Mount, ...] = (
    Mount("nema17_48mm#1", MOTOR_48, "base_motor_mount#1", "base_link", "base_yaw",
          (BASE_MOTOR_PATTERN_CENTRE, (-90.0, 0.0, 270.0)),
          "the 48 mm motor under the motor mount's plate (in the base's part frame), shaft up through it, at the slots' "
          "middle; motor + board end BASE_MOTOR_TABLE_CLEAR above the base's bottom face; connector toward +X [ESTIMATE]"),
    Mount("mks_servo42d#1", BOARD, "nema17_48mm#1", "base_link", "base_yaw", BOARD_FRAME_48),
    Mount("nema17_40mm#2", MOTOR_40, "j1_link#1", "upper_arm_link", "elbow_pitch",
          ((_UPPER_ARM.pad.x, J1_MOTOR_PAD_FACE_Y, 0.0), (90.0, 0.0, 0.0)),
          "on j1_link's motor plate (on its 4 holes), ELBOW_MOTOR_CENTRES short of the elbow axis, the body +N down the "
          "hole through the arm, shaft -N through the plate; connector toward -Z [ESTIMATE] (toward the elbow the roll "
          "block would pass it 0.2 mm off)"),
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
    *_bearing_pair(3, "j1_link#1", "upper_arm_link", "elbow_pitch", tuple(y + J1_ARM_SLIDE for y in _UPPER_ARM.elbow.lip_y),
                   (_UPPER_ARM.slab.elbow_x, None, 0.0), (-90.0, 0.0, 0.0), "j1_link's elbow bore (axis +Y; slid with the elbow end)"),
    *_bearing_pair(5, "j2_link#1", "forearm_link", "wrist_pitch", _FOREARM.boss.lip_z, (_FOREARM.web.wrist_x, 0.0, None),
                   (0.0, 0.0, 0.0), "j2_link's wrist boss (axis +Z)"),
)
# The thrust stack along the base_yaw axis (+Y of both hosts; each part's +Z up it, standing on its lower face).
_UP = (-90.0, 0.0, 0.0)
_GROOVE_FLOOR = _BASE.cap.groove_y0
THRUST_MOUNTS: tuple[Mount, ...] = (
    Mount("washer_as6590#1", THRUST_WASHER, "base#1", "base_link", "base_yaw", ((0.0, _GROOVE_FLOOR, 0.0), _UP),
          "the thrust bearing's lower washer, on the floor of the base's groove"),
    Mount("bearing_axk6590#1", THRUST_CAGE, "base#1", "base_link", "base_yaw", ((0.0, round(_GROOVE_FLOOR + THRUST_WASHER_WIDTH, 6), 0.0), _UP),
          "the needle cage on the lower washer, round the base's seat ring"),
    Mount("washer_as6590#2", THRUST_WASHER, "j1_coupler#1", "shoulder_link", "base_yaw",
          ((0.0, round(_YAW_COUPLER.hub.recess_y1 - THRUST_WASHER_WIDTH, 6), 0.0), _UP),
          "the upper washer, under j1_coupler's seat (its recess's ceiling) - it turns with the coupler"),
)
PULLEY_MOUNTS: tuple[Mount, ...] = (
    Mount("gt2_pulley_90t#3", PULLEY, "gt2_pulley_90t#1", "elbow_link", "elbow_pitch",
          ((0.0, PULLEY_SEAT_SHIFT, 0.0), (0.0, _FOREARM.drive.pulley_bolt_deg, 0.0)),
          "the elbow 90T, PULLEY_SEAT_SHIFT out from its SolidWorks pose (retired): its hub in the lower elbow bearing; "
          "turned pulley_bolt_deg about its axis with the block's bolt pattern (the nut channels clear of the ring cavity)"),
    Mount("gt2_pulley_90t#4", PULLEY, "gt2_pulley_90t#2", "wrist_pitch_link", "wrist_pitch", ((0.0, PULLEY_SEAT_SHIFT, 0.0), (0.0, 0.0, 0.0)),
          "the wrist 90T, PULLEY_SEAT_SHIFT out from its SolidWorks pose (retired): its hub in the lower wrist bearing"),
    Mount("gt2_pulley_120t#1", YAW_PULLEY, "j1_coupler#1", "shoulder_link", "base_yaw",
          ((0.0, round(_YAW_COUPLER.hub.stub_y0 + GT2_PULLEY_90T_FACE_Y[0], 6), 0.0), (180.0, _YAW_COUPLER.hub.hole_deg, 0.0)),
          "the base_yaw 120T under j1_coupler, hub up: its hub end on the stub's end, its journal in the lower base bearing, "
          "its ring under that bearing's inner ring (it holds the coupler down on the thrust bearing); turned hole_deg "
          "about its axis onto the stub's diagonal holes"),
)
# A pulley's screw set: +Z into the hub (the pulley's -Y) from its outer face; the pattern on the pulley's own X / Z axes.
_SCREWS_ON_FACE = ((0.0, GT2_PULLEY_90T_FACE_Y[1], 0.0), (90.0, 0.0, 0.0))
_HUB_LEN = GT2_PULLEY_90T_FACE_Y[1] - GT2_PULLEY_90T_FACE_Y[0]
FASTENER_MOUNTS: tuple[Mount, ...] = (
    Mount("elbow_pulley_screws#1", "elbow_pulley_screws", "gt2_pulley_90t#3", "elbow_link", "elbow_pitch", _SCREWS_ON_FACE,
          "the elbow 90T's 4x M4 x pulley_screw_len: heads on its outer face, down through its hub and the block's stub"),
    Mount("elbow_pulley_nuts#1", "elbow_pulley_nuts", "elbow_pulley_screws#1", "elbow_link", "elbow_pitch",
          ((0.0, 0.0, round(_FOREARM.drive.nut_seat_x - (_FOREARM.drive.stub_x[0] - _HUB_LEN), 6)), (0.0, 0.0, 0.0)),
          "the elbow screws' nuts on the block's channel seats (nut_seat_x), a flat toward the axis"),
    Mount("wrist_pulley_screws#1", "wrist_pulley_screws", "gt2_pulley_90t#4", "wrist_pitch_link", "wrist_pitch", _SCREWS_ON_FACE,
          "the wrist 90T's 4x M4 x pulley_screw_len: heads on its outer face, down through its hub and j3_coupler's stub"),
    Mount("wrist_pulley_nuts#1", "wrist_pulley_nuts", "wrist_pulley_screws#1", "wrist_pitch_link", "wrist_pitch",
          ((0.0, 0.0, round(_COUPLER.stub_y1 + _HUB_LEN - _COUPLER.nut_depth, 6)), (0.0, 0.0, 0.0)),
          "the wrist screws' nuts on j3_coupler's pocket floors (nut_depth), a corner along the coupler's Z"),
    Mount("yaw_pulley_screws#1", "yaw_pulley_screws", "gt2_pulley_120t#1", "shoulder_link", "base_yaw", _SCREWS_ON_FACE,
          "the base_yaw 120T's 4x M4 x pulley_screw_len: heads on its outer face (under it), up through its hub and j1_coupler's stub"),
    Mount("yaw_pulley_nuts#1", "yaw_pulley_nuts", "yaw_pulley_screws#1", "shoulder_link", "base_yaw",
          ((0.0, 0.0, round(_HUB_LEN + _YAW_COUPLER.yoke.pocket_y0 - _YAW_COUPLER.hub.nut_depth - _YAW_COUPLER.hub.stub_y0, 6)),
           (0.0, 0.0, 0.0)),
          "the base_yaw screws' nuts in j1_coupler's hex pockets, flush with the floor of the pocket over its hub "
          "(nut_depth under pocket_y0), a corner along the coupler's Z"),
)
# The yoke's cap: the upper half of j1_coupler's motor-side leg round the drive's motor sleeve, built in the coupler's frame.
YOKE_MOUNTS: tuple[Mount, ...] = (
    Mount("j1_coupler_cap#1", YOKE_CAP, "j1_coupler#1", "shoulder_link", "base_yaw", ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
          "the motor-side leg's upper half on j1_coupler, round the drive's motor sleeve; built in the coupler's part frame"),
)
MOUNTS: tuple[Mount, ...] = (BASE_MOUNTS + YOKE_MOUNTS + MOTOR_MOUNTS + BEARING_MOUNTS + THRUST_MOUNTS + PULLEY_MOUNTS
                             + FASTENER_MOUNTS)
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
