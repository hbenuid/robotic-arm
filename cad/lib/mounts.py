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
lib/reference.py DESIGNED_MODULES) whose pose no SolidWorks node gives - the forearm and wrist roll drives: its frame in
the host's frame puts the module's +Z on the joint axis it drives; mount_placements.py writes a
`kind: "module", designed: true` record with a `mount` block (no solids / volume: the module's own
totals are its EXPECTED), listed under both `designed_modules` and `mounted`, which world_rows() expands
like the drive's SolidWorks-placed record.

Geometry (lib/params.py, kernel-verified 2026-09-21 - tests/test_mounts.py re-checks it):
  base_yaw     the motor mount's plate -Y face, pattern centre BASE_MOTOR_PATTERN_CENTRE (its round holes); the
               48 mm body hangs in -Y, shaft +Y through the plate into the plane of the base_yaw 120T's teeth; motor +
               board end BASE_MOTOR_TABLE_CLEAR above the base's bottom face (lowered under them, lib/base/params.py
               BOARD_CLEAR)
  elbow_pitch  the web across j1_link's motor hole (its top y = J1_MOTOR_PAD_FACE_Y, the -N side;
               lib/upper_arm/params.py ArmParams), pattern on the pad's axis (PadParams.x: ELBOW_MOTOR_CENTRES short of
               the elbow axis - the shoulder axis is the drive's yoke's); the body +N down the square hole through the
               slab, standing out on the forearm's side; shaft -N through the web, its 20T (gt2_pulley_20t#2, under the
               web, MOTOR_PULLEY_MOUNTS) level with the elbow 90T, on the belt to it (elbow_belt#1, BELT_MOUNTS); held down by its 4x M3 SHCS (elbow_motor_screws#1, MOTOR_SCREW_MOUNTS), up
               through the web from its underside, the heads flush in its pockets - the drive motor's screws and seat
  wrist_pitch  j2_link's web (+Z face z = J2_MOTOR_WEB_FACE_Z), motor axis at x = J2_MOTOR_SLIDE_X on the side
               slots (lib/forearm/params.py: where the stock wrist belt puts it - no idler -, its body clear of the
               wall screws' heads, its connector sideways); body +N, shaft -N through the web, the 20T under it (the
               forearm turned about its roll axis: lib/placements.py TURNS - the motor on the upper arm's side)

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
+Z on it). A screw set is hosted on its pulley, its heads' bearing face on the floors of the pulley's counterbores
(GT2_PULLEY_90T_HEAD_SEAT under its outer face, GT2_PULLEY_90T_FACE_Y) and its shanks into the hub; a nut set is hosted
on its screw set, its bearing face where the host's seat lies along the screws - the elbow block's channel seats
(RollDriveParams.nut_seat_x), j3_coupler's pocket floors (CouplerParams.nut_depth), j1_coupler's pockets in the floor of
the pocket over its hub (HubParams.nut_depth under YokeParams.pocket_y0).
The elbow belt's TENSIONER (IDLER_MOUNTS; lib/upper_arm/params.py ElbowIdlerParams): the idler (parts/joints/
gt2_idler_20t), its printed spacer, its M5 axle and nut on j1_link, on the stepped slot's line where the stock belt puts
the idler (lib/upper_arm/layout.py elbow_idler_z), each +Z up j1_link's +Y (N): the idler level with the belt, the spacer
between it and the slab's underside, the screw's head under the idler, the nut on its slot's floor (lib/params.py
J1_IDLER_*). The base_yaw belt's (YAW_IDLER_MOUNTS; lib/base/params.py YawIdlerParams): the same kit on the base's motor
mount, where the stock belt puts the idler on the belt's open run (lib/base/layout.py yaw_idler_xz): the idler standing on
its lower face level with the belt, each other part's +Z down the base's -Y - the spacer's land under the idler, its
riser on the mount's plate, the screw's head on the idler, the nut on the plate's underside between its rails
(lib/params.py YAW_IDLER_*). The forearm roll's tensioner is a row of its module (assemblies/forearm_roll_drive.py).
The wrist belt has no idler: its motor sits at the stock belt's centre distance (lib/forearm/params.py IdlerParams - the
idler, its M5 axle and its nut stay modelled, UNPLACED).
"""
from __future__ import annotations

from dataclasses import dataclass

from lib.base.layout import joint_stations, yaw_belt_frame, yaw_idler_xz
from lib.base.params import DEFAULT as _BASE
from lib.bearings import THRUST_WASHER_WIDTH
from lib.coupler.params import DEFAULT as _COUPLER
from lib.forearm import module_frame_in_host, wrist_belt_frame
from lib.forearm.params import DEFAULT as _FOREARM
from lib.params import (
    BASE_MOTOR_20T_HUB_Z,
    BASE_MOTOR_PATTERN_CENTRE,
    BEARING_6806_WIDTH,
    CYCLOIDAL_MOTOR_BODY_LEN,
    GT2_IDLER_WIDTH,
    GT2_PULLEY_90T_FACE_Y,
    GT2_PULLEY_90T_HEAD_SEAT,
    J1_ARM_SLIDE,
    J1_BELT_Y,
    J1_IDLER_LEDGE_Y,
    J1_IDLER_Y,
    J1_MOTOR_20T_HUB_Z,
    J1_MOTOR_PAD_FACE_Y,
    J2_MOTOR_20T_HUB_Z,
    J2_MOTOR_SLIDE_X,
    J2_MOTOR_WEB_FACE_Z,
    NEMA17_40_BODY_LEN,
    PULLEY_SEAT_SHIFT,
    YAW_IDLER_NUT_Y,
    YAW_IDLER_Y,
)
from lib.upper_arm.layout import elbow_idler_z
from lib.upper_arm.params import DEFAULT as _UPPER_ARM
from lib.wrist.layout import module_frame_in_host as wrist_module_frame_in_host
from lib.yaw_coupler.params import DEFAULT as _YAW_COUPLER

MOTOR_48, MOTOR_40, BOARD = "nema17_48mm", "nema17_40mm", "mks_servo42d"
MOTORS = (MOTOR_48, MOTOR_40)
BEARING, PULLEY, YAW_PULLEY, MOTOR_PULLEY = "bearing_6806", "gt2_pulley_90t", "gt2_pulley_120t", "gt2_pulley_20t"
THRUST_CAGE, THRUST_WASHER = "bearing_axk6590", "washer_as6590"
PULLEY_BOLTS = ("elbow_pulley_screws", "elbow_pulley_nuts", "wrist_pulley_screws", "wrist_pulley_nuts",
                "yaw_pulley_screws", "yaw_pulley_nuts")
BASE_MOUNT, BASE_MOUNT_SCREWS, BASE_MOUNT_NUTS = "base_motor_mount", "base_motor_mount_screws", "base_motor_mount_nuts"
YOKE_MOTOR_LEG = "j1_motor_leg"
MOTOR_SCREWS = "elbow_motor_screws"
IDLER_KIT = ("gt2_idler_20t", "elbow_idler_spacer", "elbow_idler_screw", "elbow_idler_nut", "yaw_idler_spacer",
             "yaw_idler_screw", "yaw_idler_nut")

# The part-frame axis tools/reference/mount_placements.py checks for each mounted part, and whether it lies ON its
# joint's axis (a motor's shaft runs beside its joint, parallel; a bearing, a pulley or a pulley-bolt pattern sits on it).
AXES: dict[str, tuple[tuple[float, float, float], bool]] = {
    MOTOR_48: ((0.0, 0.0, 1.0), False), MOTOR_40: ((0.0, 0.0, 1.0), False), MOTOR_PULLEY: ((1.0, 0.0, 0.0), False),
    MOTOR_SCREWS: ((0.0, 0.0, 1.0), False),
    BEARING: ((0.0, 0.0, 1.0), True), PULLEY: ((0.0, 1.0, 0.0), True), YAW_PULLEY: ((0.0, 1.0, 0.0), True),
    THRUST_CAGE: ((0.0, 0.0, 1.0), True), THRUST_WASHER: ((0.0, 0.0, 1.0), True),
    **{part: ((0.0, 0.0, 1.0), True) for part in PULLEY_BOLTS},
    **{part: ((0.0, 0.0, 1.0), False) for part in IDLER_KIT},
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
          "the 48 mm motor under the motor mount's plate (in the base's part frame), shaft up through it, on its round "
          "holes; motor + board end BASE_MOTOR_TABLE_CLEAR above the base's bottom face; connector toward +X [ESTIMATE]"),
    Mount("mks_servo42d#1", BOARD, "nema17_48mm#1", "base_link", "base_yaw", BOARD_FRAME_48),
    Mount("nema17_40mm#2", MOTOR_40, "j1_link#1", "upper_arm_link", "elbow_pitch",
          ((_UPPER_ARM.pad.x, J1_MOTOR_PAD_FACE_Y, 0.0), (90.0, 0.0, 0.0)),
          "on the web across j1_link's motor hole (on its 4 holes), ELBOW_MOTOR_CENTRES short of the elbow axis, the body "
          "+N down the hole through the slab, shaft -N through the web; connector toward -Z [ESTIMATE] (toward the elbow "
          "the roll block would pass it 0.2 mm off)"),
    Mount("mks_servo42d#2", BOARD, "nema17_40mm#2", "upper_arm_link", "elbow_pitch", BOARD_FRAME_40),
    Mount("nema17_40mm#3", MOTOR_40, "j2_link#1", "forearm_link", "wrist_pitch",
          ((J2_MOTOR_SLIDE_X, 0.0, J2_MOTOR_WEB_FACE_Z), (180.0, 0.0, 180.0)),
          "on j2_link's web (+Z face), shaft -N; slide position J2_MOTOR_SLIDE_X (the stock wrist belt's); connector sideways toward "
          "j2_link's -Y - away from the elbow axis once the forearm is turned -, its plug's room beside the motor (toward the "
          "wall the motor would stop short of the belt's centre distance)"),
    Mount("mks_servo42d#3", BOARD, "nema17_40mm#3", "forearm_link", "wrist_pitch", BOARD_FRAME_40),
)
# The belt joints' motor-side pulleys, on their motors' shafts (the vendor 20T: its bore along its +X, its hub face at
# x 0, hub toward the motor), each its *_20T_HUB_Z out from the motor's face - its tooth band level with the driven
# pulley's (lib/params.py: each belt's plane).
MOTOR_PULLEY_MOUNTS: tuple[Mount, ...] = (
    Mount("gt2_pulley_20t#2", MOTOR_PULLEY, "nema17_40mm#2", "upper_arm_link", "elbow_pitch",
          ((0.0, 0.0, J1_MOTOR_20T_HUB_Z), (0.0, -90.0, 0.0)),
          "the elbow motor's 20T on its shaft, hub toward the motor, J1_MOTOR_20T_HUB_Z out from its face (under the "
          "web): its tooth band level with the elbow 90T's"),
    Mount("gt2_pulley_20t#3", MOTOR_PULLEY, "nema17_48mm#1", "base_link", "base_yaw",
          ((0.0, 0.0, BASE_MOTOR_20T_HUB_Z), (0.0, -90.0, 0.0)),
          "the base_yaw motor's 20T on its shaft, its hub in the motor plate's window: its tooth band level with the 120T's"),
    Mount("gt2_pulley_20t#4", MOTOR_PULLEY, "nema17_40mm#3", "forearm_link", "wrist_pitch",
          ((0.0, 0.0, J2_MOTOR_20T_HUB_Z), (0.0, -90.0, 0.0)),
          "the wrist motor's 20T on its shaft, its hub up in the web's pilot slot: its tooth band level with the wrist 90T's"),
)
# The elbow motor's 4x M3 SHCS: +Z up j1_link's +Y from the heads' bearing faces on the floors of the pockets in the
# underside of the web across its motor hole (motor_plate_t under the motor's face), the pattern on the web's holes.
MOTOR_SCREW_MOUNTS: tuple[Mount, ...] = (
    Mount("elbow_motor_screws#1", MOTOR_SCREWS, "j1_link#1", "upper_arm_link", "elbow_pitch",
          ((_UPPER_ARM.pad.x, round(J1_MOTOR_PAD_FACE_Y - _UPPER_ARM.arm.motor_plate_t, 6), 0.0), (-90.0, 0.0, 0.0)),
          "the elbow motor's 4x M3 x J1_MOTOR_SCREW_LEN SHCS (the drive motor's): heads flush in the pockets in the underside "
          "of the web across j1_link's motor hole, up through it into the motor's tapped holes"),
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
# A pulley's screw set: +Z into the hub (the pulley's -Y) from its counterbores' floors; the pattern on the pulley's own
# X / Z axes. A nut set's frame runs from those floors: the hub's end lies _SEAT_TO_END down the screws.
_SCREWS_IN_COUNTERBORES = ((0.0, round(GT2_PULLEY_90T_FACE_Y[1] - GT2_PULLEY_90T_HEAD_SEAT, 6), 0.0), (90.0, 0.0, 0.0))
_SEAT_TO_END = GT2_PULLEY_90T_FACE_Y[1] - GT2_PULLEY_90T_HEAD_SEAT - GT2_PULLEY_90T_FACE_Y[0]
FASTENER_MOUNTS: tuple[Mount, ...] = (
    Mount("elbow_pulley_screws#1", "elbow_pulley_screws", "gt2_pulley_90t#3", "elbow_link", "elbow_pitch", _SCREWS_IN_COUNTERBORES,
          "the elbow 90T's 4x M4 x pulley_screw_len: heads in its counterbores, down through its hub and the block's stub"),
    Mount("elbow_pulley_nuts#1", "elbow_pulley_nuts", "elbow_pulley_screws#1", "elbow_link", "elbow_pitch",
          ((0.0, 0.0, round(_FOREARM.drive.nut_seat_x - (_FOREARM.drive.stub_x[0] - _SEAT_TO_END), 6)), (0.0, 0.0, 0.0)),
          "the elbow screws' nuts on the block's channel seats (nut_seat_x), a flat toward the axis"),
    Mount("wrist_pulley_screws#1", "wrist_pulley_screws", "gt2_pulley_90t#4", "wrist_pitch_link", "wrist_pitch", _SCREWS_IN_COUNTERBORES,
          "the wrist 90T's 4x M4 x pulley_screw_len: heads in its counterbores, down through its hub and j3_coupler's stub"),
    Mount("wrist_pulley_nuts#1", "wrist_pulley_nuts", "wrist_pulley_screws#1", "wrist_pitch_link", "wrist_pitch",
          ((0.0, 0.0, round(_COUPLER.stub_y1 + _SEAT_TO_END - _COUPLER.nut_depth, 6)), (0.0, 0.0, 0.0)),
          "the wrist screws' nuts on j3_coupler's pocket floors (nut_depth), a corner along the coupler's Z"),
    Mount("yaw_pulley_screws#1", "yaw_pulley_screws", "gt2_pulley_120t#1", "shoulder_link", "base_yaw", _SCREWS_IN_COUNTERBORES,
          "the base_yaw 120T's 4x M4 x pulley_screw_len: heads in its counterbores (under it), up through its hub and j1_coupler's stub"),
    Mount("yaw_pulley_nuts#1", "yaw_pulley_nuts", "yaw_pulley_screws#1", "shoulder_link", "base_yaw",
          ((0.0, 0.0, round(_SEAT_TO_END + _YAW_COUPLER.yoke.pocket_y0 - _YAW_COUPLER.hub.nut_depth - _YAW_COUPLER.hub.stub_y0, 6)),
           (0.0, 0.0, 0.0)),
          "the base_yaw screws' nuts in j1_coupler's hex pockets, flush with the floor of the pocket over its hub "
          "(nut_depth under pocket_y0), a corner along the coupler's Z"),
)
# The yoke's motor leg: j1_coupler's motor-side leg, its own part - a ring round the drive's motor sleeve bolted to the
# coupler's disc and to the sleeve - built in the coupler's frame.
YOKE_MOUNTS: tuple[Mount, ...] = (
    Mount("j1_motor_leg#1", YOKE_MOTOR_LEG, "j1_coupler#1", "shoulder_link", "base_yaw", ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
          "the motor-side leg on j1_coupler's disc, round the drive's motor sleeve; built in the coupler's part frame"),
)
# The belts (purchased parts, a smooth band round each loop - lib/belts.py, parts/*/<joint>_belt.py), each in the link
# that carries its motor (both its pulleys' axes are fixed there): its frame on the motor's axis at the belt's plane,
# +X toward the driven pulley, +Z along the axes. The forearm roll's is a row of its module
# (assemblies/forearm_roll_drive.py).
BELT_MOUNTS: tuple[Mount, ...] = (
    Mount("yaw_belt#1", "yaw_belt", "base#1", "base_link", "base_yaw", yaw_belt_frame(_BASE),
          "the base_yaw belt inside the base: the motor's 20T to the 120T under j1_coupler, round its idler"),
    Mount("elbow_belt#1", "elbow_belt", "j1_link#1", "upper_arm_link", "elbow_pitch",
          ((_UPPER_ARM.pad.x, J1_BELT_Y, 0.0), (90.0, 0.0, 0.0)),
          "the elbow belt under j1_link's web: the elbow motor's 20T to the elbow 90T"),
    Mount("wrist_belt#1", "wrist_belt", "j2_link#1", "forearm_link", "wrist_pitch", wrist_belt_frame(_FOREARM),
          "the wrist belt under j2_link's web: the wrist motor's 20T to the wrist 90T"),
)
# The elbow belt's tensioner (lib/upper_arm/params.py ElbowIdlerParams), on j1_link at the idler's place for the stock
# belt - the stepped slot's x, elbow_idler_z() across -, each part's +Z up j1_link's +Y (N): the idler standing on its
# lower face level with the belt, the spacer's land on its upper face, the screw's head under it, the nut on its slot's
# floor (lib/params.py J1_IDLER_*).
_IDLER_X, _IDLER_Z = _UPPER_ARM.slots.stepped_x, round(elbow_idler_z(_UPPER_ARM), 6)
IDLER_MOUNTS: tuple[Mount, ...] = (
    Mount("gt2_idler_20t#1", "gt2_idler_20t", "j1_link#1", "upper_arm_link", "elbow_pitch",
          ((_IDLER_X, J1_IDLER_Y, _IDLER_Z), _UP),
          "the elbow belt's tensioner under j1_link's slab, level with the belt, on its back on the 90T -> 20T run"),
    Mount("elbow_idler_spacer#1", "elbow_idler_spacer", "j1_link#1", "upper_arm_link", "elbow_pitch",
          ((_IDLER_X, round(J1_IDLER_Y + GT2_IDLER_WIDTH, 6), _IDLER_Z), _UP),
          "its printed spacer: the land on the idler's inner ring, the riser on the slab's underside"),
    Mount("elbow_idler_screw#1", "elbow_idler_screw", "j1_link#1", "upper_arm_link", "elbow_pitch",
          ((_IDLER_X, J1_IDLER_Y, _IDLER_Z), _UP),
          "its M5 axle: the head under the idler, up through the idler, the spacer and the floor's slot into the nut"),
    Mount("elbow_idler_nut#1", "elbow_idler_nut", "j1_link#1", "upper_arm_link", "elbow_pitch",
          ((_IDLER_X, J1_IDLER_LEDGE_Y, _IDLER_Z), _UP),
          "its nut on the floor of its slot down from the slab's top face, its flats across the link's X"),
)
# The base_yaw belt's tensioner (lib/base/params.py YawIdlerParams), on the motor mount at the idler's place for the stock
# belt (yaw_idler_xz()), in the base's part frame: the idler standing on its lower face level with the belt (+Z up the
# base's +Y); the spacer, the screw and the nut turned over (+Z down the base's -Y) - the spacer's land under the idler,
# the screw's head on its top face, the nut's bearing face on the plate's underside (lib/params.py YAW_IDLER_*).
_DOWN = (90.0, 0.0, 0.0)
_YAW_IDLER_X, _YAW_IDLER_Z = (round(v, 6) for v in yaw_idler_xz(_BASE))
YAW_IDLER_MOUNTS: tuple[Mount, ...] = (
    Mount("gt2_idler_20t#2", "gt2_idler_20t", "base_motor_mount#1", "base_link", "base_yaw",
          ((_YAW_IDLER_X, YAW_IDLER_Y, _YAW_IDLER_Z), _UP),
          "the base_yaw belt's tensioner on the motor mount's plate, level with the belt, on its back on the 120T -> 20T run"),
    Mount("yaw_idler_spacer#1", "yaw_idler_spacer", "base_motor_mount#1", "base_link", "base_yaw",
          ((_YAW_IDLER_X, YAW_IDLER_Y, _YAW_IDLER_Z), _DOWN),
          "its printed spacer: the land under the idler's inner ring, the riser on the mount's plate"),
    Mount("yaw_idler_screw#1", "yaw_idler_screw", "base_motor_mount#1", "base_link", "base_yaw",
          ((_YAW_IDLER_X, round(YAW_IDLER_Y + GT2_IDLER_WIDTH, 6), _YAW_IDLER_Z), _DOWN),
          "its M5 axle: the head on the idler, down through the idler, the spacer and the plate's slot into the nut"),
    Mount("yaw_idler_nut#1", "yaw_idler_nut", "base_motor_mount#1", "base_link", "base_yaw",
          ((_YAW_IDLER_X, YAW_IDLER_NUT_Y, _YAW_IDLER_Z), _DOWN),
          "its nut on the plate's underside between the rails, its flats across the slot"),
)
MOUNTS: tuple[Mount, ...] = (BASE_MOUNTS + YOKE_MOUNTS + MOTOR_MOUNTS + MOTOR_PULLEY_MOUNTS + MOTOR_SCREW_MOUNTS + BEARING_MOUNTS
                             + THRUST_MOUNTS + PULLEY_MOUNTS + FASTENER_MOUNTS + BELT_MOUNTS + IDLER_MOUNTS + YAW_IDLER_MOUNTS)
BY_KEY: dict[str, Mount] = {m.key: m for m in MOUNTS}


@dataclass(frozen=True)
class ModuleMount:
    key: str        # occurrence key "<module>#<n>" (placements.json designed_modules, assemblies/arm.py, LINKS with ":<body>")
    module: str     # the designed module (assemblies/<module>.py, lib/reference.py DESIGNED_MODULES)
    host: str       # the SolidWorks occurrence the module's stator is bolted to
    joint: str      # the joint the module IS (robot/frames.py): the module's +Z lies on its axis
    frame: tuple    # ((x, y, z), (rx, ry, rz)) in the host's frame BEFORE any lib/placements.py TURNS (the module drives the
    #                 joint a turn is about, so it keeps its pose: tools/reference/mount_placements.py) - lib.datum.to_location(frame)
    note: str = ""
    turned: bool = False   # True: in the host's frame AFTER its TURNS - a module beyond the turned link (the wrist roll drive),
    #                        which turns with its host like every record there




MODULE_MOUNTS: tuple[ModuleMount, ...] = (
    ModuleMount("forearm_roll_drive#1", "forearm_roll_drive", "j2_link#1", "forearm_roll", module_frame_in_host(),
                "the roll drive on j2_link's roll axis (y 0, z FOREARM_ROLL_AXIS_Z), in its frame before the forearm's turn: module "
                "+Z = host -X toward the wrist, module +X = host +Z (N), +Y = up in the swing plane (the motor side); the block IS the "
                "elbow coupler: its stub turns in j1_link's bore and the elbow 90T bolts into it (j3_coupler#1 retired, "
                "lib/placements.py RETIRED)"),
    ModuleMount("wrist_roll_drive#1", "wrist_roll_drive", "wrist_link#1", "wrist_roll", wrist_module_frame_in_host(),
                "the wrist roll drive on wrist_link's roll axis R' (WRIST_ROLL_OFFSET along +Y from the pitch axis, "
                "WRIST_ROLL_AXIS_H over the plate's coupler face): module +Z = host +X toward the gripper, module +X = host -Z "
                "(away from the plate), +Y = host +Y (from the pancake toward R'); wrist_link IS its frame (the stator), the "
                "pancake and its 20T replace nema17_pancake#1 / gt2_pulley_20t#1 (lib/placements.py RETIRED); beyond the "
                "forearm's turn, it turns with wrist_link", turned=True),
)
MODULES_BY_KEY: dict[str, ModuleMount] = {m.key: m for m in MODULE_MOUNTS}


def keys() -> list[str]:
    """The mounted part occurrence keys, in declaration order (motor, its board, ...)."""
    return [m.key for m in MOUNTS]


def module_keys() -> list[str]:
    """The mounted designed-module keys, in declaration order."""
    return [m.key for m in MODULE_MOUNTS]
