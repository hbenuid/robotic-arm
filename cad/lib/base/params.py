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
ring sized for the thrust bearing's bore, and the +X lobe cut off at a joint face (JointParams) into a part of its own,
bolted back on (parts/base/base_motor_mount), its motor seat slotted where a stock belt puts the motor.

Every number below was measured on the reference 2026-09-25 (vertex / face census; tests/base/test_base.py
re-checks the builds against it): [REFERENCE] unless tagged. Units mm, degrees where named *_deg (in the XZ plane,
from +X toward +Z). Frozen dataclasses; variants via dataclasses.replace.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

from lib.bearings import THRUST_BORE
from lib.belts import GT2_PULLEY_20T_TEETH, GT2_PULLEY_90T_TEETH, centre_distance
from lib.fasteners import M3_CLEAR, M4_CLEAR, M4_NUT, M4_SHCS, NutSize, ShcsSize
from lib.forearm.params import LEGACY as _FOREARM
from lib.motors import NEMA17_PILOT_DIA


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
    round: the motor slides along the slot)."""

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
    """[DESIGN] Where the motor lobe comes off (base_motor_mount): the plane x = split_x across the D below the plate,
    the base on -X, the mount on +X. On each side a rib inside each side wall, rib_w deep (in Z, from the wall's
    inside) and rib_t thick (along X), from the bottom face up to the plate's underside; 2 M4 per rib along X, bolt_inset
    under the plate's underside and above the bottom face: the heads on the mount's rib, the nuts pressed into hex
    pockets in the base's rib (their outer faces flush with its back face, a corner up), the tips out into the base."""

    split_x: float = 55.0          # just past the tower's round (shell.r): the base keeps a flat full-width end
    rib_w: float = 12.0
    rib_t: float = 8.0
    bolt_inset: float = 8.0
    bolt_dia: float = M4_CLEAR
    screw: ShcsSize = M4_SHCS
    screw_len: float = 20.0        # the tip 4.0 past the nut's outer face
    nut: NutSize = M4_NUT
    nut_pocket_af: float = 6.85    # a press on the 7.0 nut (the elbow block's, j3_coupler's): it stays when its screw is out


@dataclass(frozen=True)
class BaseConfig:
    shell: ShellParams = ShellParams()
    cap: CapParams = CapParams()
    bore: BoreParams = BoreParams()
    plate: PlateParams = PlateParams()
    motor: MotorParams = MotorParams()
    joint: JointParams | None = None   # None: one part, the lobe included (the SolidWorks base)


LEGACY = BaseConfig()     # the SolidWorks part, exactly

# What the part builds: the base_yaw bore takes the 6806-2RS pair (lib/bearings.py) like the wrist's bore - both
# seats the wrist's Ø42.2 (the SolidWorks Ø42.4 / Ø43.4 let the bearings wobble, the lower one 0.7 mm a side) and its
# Ø37.65 lip, which stops the outer rings only (the SolidWorks Ø31.73 lip ran under the upper bearing's inner ring).
# The seat ring centres the thrust bearing (the washers' and the cage's bore THRUST_BORE): the SolidWorks Ø65.1 would
# not go into a Ø65 bore; RING_CLEAR a side.
# The motor lobe is its own part (base_motor_mount, JointParams). Its motor sits where a stock belt puts it:
# the SolidWorks centre (78.97 from the axis) wants a 274.2 mm belt; YAW_BELT sets 81.97, the holes are slots of
# +/- MOTOR_TRAVEL along X (the belt's tension), the window lets the pilot slide with them (WINDOW_CLEAR a side - only
# the pilot's height in Z: the SolidWorks window's +/- 21.3 would run into the -X slots) and the rim's +X side follows
# the slots out.
RING_CLEAR = 0.1          # [DESIGN]
YAW_BELT = 280            # [ESTIMATE] base_yaw belt, 280-2GT (lib/belts.py STANDARD_2GT_LENGTHS): 20T motor - 90T joint
MOTOR_TRAVEL = 2.5        # [DESIGN] the roll motor's slots' +/- 2.5 (lib/forearm/params.py)
WINDOW_CLEAR = 0.2        # [DESIGN]
_MOTOR_X = round(centre_distance(YAW_BELT, GT2_PULLEY_90T_TEETH, GT2_PULLEY_20T_TEETH), 6)
_WINDOW_HALF = NEMA17_PILOT_DIA / 2.0 + MOTOR_TRAVEL + WINDOW_CLEAR
DEFAULT = replace(LEGACY, bore=replace(LEGACY.bore, upper_dia=_FOREARM.boss.seat_dia, lower_dia=_FOREARM.boss.seat_dia,
                                       lip_dia=_FOREARM.boss.lip_dia),                                    # [DESIGN]
                  cap=replace(LEGACY.cap, groove_r=(THRUST_BORE / 2.0 - RING_CLEAR, LEGACY.cap.groove_r[1])),   # [DESIGN]
                  motor=replace(LEGACY.motor, centre=(_MOTOR_X, LEGACY.motor.centre[1]), hole_dia=M3_CLEAR,
                                window_x=(round(_MOTOR_X - _WINDOW_HALF, 6), round(_MOTOR_X + _WINDOW_HALF, 6)),
                                window_half_z=NEMA17_PILOT_DIA / 2.0 + WINDOW_CLEAR,
                                slot_x1=round(_MOTOR_X + LEGACY.motor.rim_half + MOTOR_TRAVEL, 6), travel=MOTOR_TRAVEL),   # [DESIGN]
                  joint=JointParams())                                                                  # [DESIGN]
