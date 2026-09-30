"""BaseConfig - every dimension of the base (the arm's foot and the base_yaw housing), in its part frame.

Frame (= the SolidWorks part frame of base, which placements.json places at identity: the capture frame W): origin on
the base_yaw axis, +Y up the axis (the bottom face - the arm's mounting face - at y = shell.y0, lib/datum.py
BASE_BOTTOM_Y), the round half toward -X, the motor end toward +X. Every feature is a prism or a bore along Y.

The body: a D-shaped wall (a half round of r on -X, straight sides out to the flat +X end) from the bottom face up
to the motor plate; above the plate only the round half and the straight sides' stubs (out to where the inside of a
side meets the outer round) carry on up to the cap; the cap (a disc of r plus those stubs, a 45 degree chamfer
under it) holds the base_yaw bearing bore in a boss below and on top a seat ring in an annular groove - the base_yaw
thrust bearing (lib/bearings.py THRUST_*) lies in the groove, centred on the ring, under j1_coupler. The 48 mm motor
(lib/mounts.py nema17_48mm#1) bolts to the plate's underside, its shaft up through the plate's window.

Two configurations: LEGACY reproduces the SolidWorks reference (the part's REFERENCE_BUILD -
tests/test_reference_match.py); DEFAULT is what the parts build: the bearing bore sized for the 6806-2RS pair, the seat
ring sized for the thrust bearing's bore, and the +X lobe cut off at a joint face (JointParams): in its place a
narrower bolt-on motor mount (MountParams, parts/base/base_motor_mount), its motor seat slotted where a stock belt puts
the motor.

Every number below was measured on the reference 2026-09-25 (vertex / face census; tests/base/test_base.py
re-checks the builds against it): [REFERENCE] unless tagged. Units mm, degrees where named *_deg (in the XZ plane,
from +X toward +Z). Frozen dataclasses; variants via dataclasses.replace.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

from lib.bearings import THRUST_BORE
from lib.belts import GT2_PULLEY_20T_TEETH, GT2_PULLEY_120T_TEETH, centre_distance
from lib.cycloidal.params import DEFAULT_CONFIG as _DRIVE
from lib.fasteners import M3_CLEAR, M4_CLEAR, M4_NUT, M4_SHCS, NutSize, ShcsSize
from lib.forearm.params import LEGACY as _FOREARM
from lib.motors import MKS_SERVO42D_STACK, NEMA17_PILOT_DIA


@dataclass(frozen=True)
class ShellParams:
    """The D-shaped wall: the round half's outer radius r (= the straight sides' half width), `wall` thick, the flat
    +X end at x1; full height up to the plate's top face, the round half and the sides' stubs on up to the cap. A
    notch through the +X end at the bottom (the base_yaw motor's cable)."""

    r: float = 53.33946
    wall: float = 5.0
    x1: float = 114.838755
    y0: float = -100.9             # the bottom face
    notch_half_z: float = 15.0
    notch_y1: float = -72.996154   # the notch: from the bottom face up to here


@dataclass(frozen=True)
class CapParams:
    """The cap: its underside flat inside a 45 degree chamfer that starts on the wall's inside at chamfer_y0, the top
    face with an annular groove round a raised seat ring (the thrust bearing's washers and cage in the groove, centred on
    the ring; j1_coupler stands on them), the bearing boss under the
    underside, an r 0.5 round on the groove's outer edge, the ring's top edge and both ends of the bore."""

    chamfer_y0: float = -24.4
    underside_y: float = -14.4
    top_y: float = -5.492748
    groove_r: tuple = (32.55, 45.1)
    groove_y0: float = -8.492748
    ring_top_y: float = -5.092748
    boss_r: float = 31.735297
    boss_y0: float = -22.4
    fillet: float = 0.5


@dataclass(frozen=True)
class BoreParams:
    """The base_yaw bearing bore on the axis: the upper seat from the ring's top down to the lip, the lip (from the
    cap's underside up), the lower seat through the boss."""

    upper_dia: float = 42.4
    lip_dia: float = 31.733886
    lip_y: tuple = (-14.4, -13.092748)
    lower_dia: float = 43.4


@dataclass(frozen=True)
class PlateParams:
    """The motor plate across the D, its top face flush with the shell's full-height top: a central opening of
    opening_r that runs out to the wall from the +Z side round -X to a radial line (open_deg), and a curved slot of
    width arc_slot_w on the +X side (centreline arc_slot_r, round ends at +/- arc_slot_half_deg)."""

    y: tuple = (-44.9, -39.9)
    opening_r: float = 42.33946
    open_deg: tuple = (90.0, 253.002235)
    arc_slot_r: float = 49.33946
    arc_slot_w: float = 4.0
    arc_slot_half_deg: float = 45.954036


@dataclass(frozen=True)
class MotorParams:
    """The base_yaw motor's seat on the plate's underside: 4 holes on the NEMA 17 square about `centre` (x, z), a
    window for the pilot and the shaft's pulley, the belt slot from the shell's outer round out to slot_x1, and a U
    rim hanging below the plate round the motor's face on three sides (open toward the axis, its ends on the outer
    round: the motor slides along the slot). The belt slot and the rim are the SolidWorks base's only: the motor
    mount (build_motor_mount) has neither - its slots hold the motor."""

    centre: tuple = (78.971441, 0.08362)
    hole_dia: float = 3.2
    window_x: tuple = (67.991297, 90.151584)
    window_half_z: float = 21.3
    slot_x1: float = 100.271441
    slot_half_z: float = 9.7
    rim_half: float = 21.3         # the rim's inside: +/- this about the centre (the +X side at slot_x1)
    rim_wall: float = 2.0
    rim_y0: float = -51.9
    travel: float = 0.0            # [DESIGN] the motor's slide along X, +/- this about the centre: the holes become
    #                                slots, the belt's tension (0: the SolidWorks round holes)


@dataclass(frozen=True)
class JointParams:
    """[DESIGN] Where the motor mount (base_motor_mount) bolts to the base: the plane x = split_x across the D below the
    plate, the base on -X, the mount on +X. The base ends there in a post inside each side wall, post_t thick (along X),
    in to the mount's inside (MountParams: the window between the posts, full height, opens the mount into the base),
    from the bottom face up to the plate's underside; 2 M4 per side along X, through the mount's ear and the post,
    bolt_inset under the plate's underside and above the bottom face: the heads on the ears' outside, the nuts pressed
    into hex pockets in the posts (their outer faces flush with the posts' back faces, a corner up), the tips out into
    the base."""

    split_x: float = 55.0          # just past the tower's round (shell.r): the base keeps a flat full-width end
    post_t: float = 8.0
    bolt_inset: float = 8.0
    bolt_dia: float = M4_CLEAR
    screw: ShcsSize = M4_SHCS
    screw_len: float = 20.0        # the tip 4.0 past the nut's outer face
    nut: NutSize = M4_NUT
    nut_pocket_af: float = 6.85    # a press on the 7.0 nut (the elbow block's, j3_coupler's): it stays when its screw is out


@dataclass(frozen=True)
class MountParams:
    """[DESIGN] The motor mount (base_motor_mount): a box round the 48 mm motor + its MKS board, `room` clear of the
    board's square on every side (in Z; along X to the end wall at the slots' middle - the -X side opens into the base
    through the window between its posts: the cables' way), `wall` thick, from the joint face to its end wall and from
    the bottom face (it stands on the table) up to the plate's top; the plate (the motor's seat, MotorParams) across its
    top; open underneath. An ear outside each side wall at the joint end, ear_w wide (in Z) and ear_t thick (along X),
    full height, carries the side's 2 M4 (JointParams).

    The walls are trusses (lib/base/layout.py truss_panels()): below the plate each keeps a frame `strut` wide - a rail
    under the plate, a rail on the table, a post at each end (from the outer corner, so the corner posts take in the
    other wall) - and a V of two `strut` wide struts from the frame's top corners down to the middle of its bottom
    rail; the three triangles between are open. Printed plate-down, the middle one is self-supporting and the side
    ones bridge only their table edge. The walls carry little: the belt's pull goes along the plate into the ears, and
    the motor + board weigh ~0.44 kg."""

    room: float = 10.0
    wall: float = 4.0
    ear_w: float = 12.0
    ear_t: float = 8.0
    strut: float = 6.0


@dataclass(frozen=True)
class BaseConfig:
    shell: ShellParams = ShellParams()
    cap: CapParams = CapParams()
    bore: BoreParams = BoreParams()
    plate: PlateParams = PlateParams()
    motor: MotorParams = MotorParams()
    joint: JointParams | None = None   # None: one part, the lobe included (the SolidWorks base)
    mount: MountParams | None = None   # the bolt-on motor mount (with joint)


LEGACY = BaseConfig()     # the SolidWorks part, exactly

# What the part builds: the base_yaw bore takes the 6806-2RS pair (lib/bearings.py) like the wrist's bore - both
# seats the wrist's Ø42.2 (the SolidWorks Ø42.4 / Ø43.4 let the bearings wobble, the lower one 0.7 mm a side) and its
# Ø37.65 lip, which stops the outer rings only (the SolidWorks Ø31.73 lip ran under the upper bearing's inner ring).
# The seat ring centres the thrust bearing (the washers' and the cage's bore THRUST_BORE): the SolidWorks Ø65.1 would
# not go into a Ø65 bore; RING_CLEAR a side.
# The motor has a bolt-on mount of its own (base_motor_mount, JointParams + MountParams: a box round the motor and its
# board with room for the wiring, in place of the SolidWorks lobe the base's full width). It sits where a stock belt puts it
# round the base_yaw pulleys - the 120T on the joint (parts/base/gt2_pulley_120t), the 20T on the motor: YAW_BELT sets
# 83.97 from the axis (the SolidWorks centre, 78.97, belonged to a 90T), the holes are slots of
# +/- MOTOR_TRAVEL along X (the belt's tension), the window lets the pilot slide with them (WINDOW_CLEAR a side - only
# the pilot's height in Z: the SolidWorks window's +/- 21.3 would run into the -X slots).
# The bottom face (the mounting face, lib/datum.py BASE_BOTTOM_Y) sits BOARD_CLEAR under the motor's MKS board: the
# SolidWorks base's -100.9 left the 48 mm motor + board (48 + 14.1 under the plate) hanging 6.1 below it. The base's
# walls, its posts and the whole motor mount reach down to it.
RING_CLEAR = 0.1          # [DESIGN]
BOARD_CLEAR = 5.0         # [DESIGN] the base_yaw motor's board above the table
YAW_BELT = 320            # [ESTIMATE] base_yaw belt, 320-2GT (lib/belts.py STANDARD_2GT_LENGTHS): 20T motor - 120T joint
MOTOR_TRAVEL = 2.5        # [DESIGN] the roll motor's slots' +/- 2.5 (lib/forearm/params.py)
WINDOW_CLEAR = 0.2        # [DESIGN]
_MOTOR_X = round(centre_distance(YAW_BELT, GT2_PULLEY_120T_TEETH, GT2_PULLEY_20T_TEETH), 6)
_WINDOW_HALF = NEMA17_PILOT_DIA / 2.0 + MOTOR_TRAVEL + WINDOW_CLEAR
_BOTTOM_Y = round(LEGACY.plate.y[0] - _DRIVE.motor.body_length - MKS_SERVO42D_STACK - BOARD_CLEAR, 6)
DEFAULT = replace(LEGACY, shell=replace(LEGACY.shell, y0=_BOTTOM_Y),                                  # [DESIGN]
                  bore=replace(LEGACY.bore, upper_dia=_FOREARM.boss.seat_dia, lower_dia=_FOREARM.boss.seat_dia,
                                       lip_dia=_FOREARM.boss.lip_dia),                                    # [DESIGN]
                  cap=replace(LEGACY.cap, groove_r=(THRUST_BORE / 2.0 - RING_CLEAR, LEGACY.cap.groove_r[1])),   # [DESIGN]
                  motor=replace(LEGACY.motor, centre=(_MOTOR_X, LEGACY.motor.centre[1]), hole_dia=M3_CLEAR,
                                window_x=(round(_MOTOR_X - _WINDOW_HALF, 6), round(_MOTOR_X + _WINDOW_HALF, 6)),
                                window_half_z=NEMA17_PILOT_DIA / 2.0 + WINDOW_CLEAR, travel=MOTOR_TRAVEL),   # [DESIGN]
                  joint=JointParams(), mount=MountParams())                                             # [DESIGN]
