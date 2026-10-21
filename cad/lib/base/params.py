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
bolt-on motor mount (MountParams, parts/base/base_motor_mount) - the base's D carried on to a round end, its foot round
it -, its motor seat on round holes YAW_MOTOR_X out and the belt's tensioner on its plate (YawIdlerParams); the walls
j1_coupler's cone carried on down (ShellParams.r / draft: BASE_R, BASE_DRAFT) onto a Ø200 foot - a flange, a straight
chamfer up the wall, screw holes (FootParams); a band of slits through both parts' walls (SlitParams).

Every number below was measured on the reference 2026-09-25 (vertex / face census; tests/base/test_base.py
re-checks the builds against it): [REFERENCE] unless tagged. Units mm, degrees where named *_deg (in the XZ plane,
from +X toward +Z). Frozen dataclasses; variants via dataclasses.replace.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

from lib.bearings import THRUST_BORE
from lib.cycloidal.params import DEFAULT_CONFIG as _DRIVE
from lib.fasteners import M3_CLEAR, M4_CLEAR, M4_NUT, M4_SHCS, M5_CLEAR, M5_NUT, M5_SHCS, NutSize, ShcsSize
from lib.forearm.params import LEGACY as _FOREARM
from lib.motors import MKS_SERVO42D_STACK, NEMA17_PILOT_DIA


@dataclass(frozen=True)
class ShellParams:
    """The D-shaped wall: the round half's outer radius r at the cap's top face (= the straight sides' half width),
    `wall` thick (along the horizontal), the flat +X end at x1; full height up to the plate's top face, the round half
    and the sides' stubs on up to the cap. With a `draft` the wall leans out going down - its radius r + draft * the
    depth under the cap's top face, inside and out: the round half a cone, the sides planes. A notch through the +X end
    at the bottom (the base_yaw motor's cable)."""

    r: float = 53.33946
    wall: float = 5.0
    x1: float = 114.838755
    y0: float = -100.9             # the bottom face
    notch_half_z: float = 15.0
    notch_y1: float = -72.996154   # the notch: from the bottom face up to here
    draft: float = 0.0             # dr / dy, outward going down (0: the SolidWorks base's vertical walls)


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
    mount (build_motor_mount) has neither - its holes hold the motor."""

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
    in to the window between the posts (MountParams: full height, it opens the mount into the base), from the bottom
    face up to the plate's underside; the mount's own posts (MountParams.post_t) face them. 2 M4 per side along X,
    bolt_out out of the window's edge, bolt_inset under the plate's underside and above the bottom face, through the
    mount's post and the base's: the heads on the mount's posts' inner faces, the nuts pressed into hex pockets in the
    base's posts (their outer faces flush with the posts' back faces, a corner up), the tips out into the base."""

    split_x: float = 55.0          # just past the tower's round (shell.r): the base keeps a flat full-width end
    post_t: float = 8.0
    bolt_inset: float = 8.0
    bolt_out: float = 10.0         # the bolts' axis out of the window's edge (z +/- 41.58): the nut's pocket 6.6 clear of it
    bolt_dia: float = M4_CLEAR
    screw: ShcsSize = M4_SHCS
    screw_len: float = 20.0        # the tip 4.0 past the nut's outer face
    nut: NutSize = M4_NUT
    nut_pocket_af: float = 6.85    # a press on the 7.0 nut (the elbow block's, j3_coupler's): it stays when its screw is out


@dataclass(frozen=True)
class MountParams:
    """[DESIGN] The motor mount (base_motor_mount): the base's other half. Its walls are the base's D carried on at full
    width - the same leaning planes, shell.wall thick - out to a round end that matches the base's round half (its
    radius the walls' at each height, its centre lib/base/layout.py mount_end_x(): the end reaches mount_x1() at the
    plate's top, `room` past the MKS board's square there), from the bottom face up to the plate's top; the plate (the
    motor's seat, MotorParams) across its top; open underneath; the foot (FootParams) carried on round it. Open toward
    the base between its two posts at the joint, which mirror the base's (post_t deep along X, from the window -
    `room` clear of the board's square in Z, lib/base/layout.py mount_inner_half(): the cables' way into the base - out
    to the wall, from the bottom face up to the plate's underside): the 4 M4 run through both pairs of posts, the heads
    on the mount's posts' inner faces (turned from underneath, the base off the table), into the M4 nuts pressed into
    the base's posts (JointParams)."""

    room: float = 10.0
    post_t: float = 8.0


@dataclass(frozen=True)
class SlitParams:
    """[DESIGN] The slits through the base's and the motor mount's walls: one band of upright slits, `w` wide and `h`
    tall with round ends, cut straight across through the wall (their sides upright, their round tops w / 2 - no bridge
    and no support, the base printed foot down, the mount plate down). The band's foot `over_flare` above the foot's
    chamfer, under the plate. Round the whole outline at `pitch` along the wall's mid-surface at the band's middle: the
    base's round half and the mount's round end as many slits as fit there at `pitch` (one on each end of the half
    round), the straight sides at multiples of `pitch` from the base_yaw axis - none within joint_clear of the two
    parts' posts at the joint (lib/base/layout.py slits())."""

    w: float = 3.0
    h: float = 32.0
    over_flare: float = 4.0        # the band 28 .. 60 above the table: 7.1 under the plate's underside
    pitch: float = 10.0
    joint_clear: float = 2.0


@dataclass(frozen=True)
class FootParams:
    """[DESIGN] The foot: a D flange on the table (a round of r about the axis on -X, the straight sides z = +/- r out
    to the joint face), t thick, hollow like the walls (the base's inside runs through it); a straight chamfer - the
    flare - from the flange's top at flare_r up to the wall's outside flare_h above the bottom face. 6 screw holes
    (hole_dia) down through the flange for fixing the base to the table: on the circle hole_r at hole_deg (in the XZ
    plane, from +X toward +Z: the round half's 4) and at x = side_hole_x on the sides (z = +/- hole_r), each with a
    spot-face of spot_dia from the flange's top up through the chamfer - the screw's head (or its washer) on a flat.
    The motor mount carries the same foot on round its end (the flange's round of r and the chamfer about its end's
    centre, lib/base/layout.py mount_end_x()), with mount_hole_deg's 4 holes on hole_r about that centre."""

    r: float = 100.0               # Ø200: the stretched arm's centre of mass (131 mm out) still past it standing free -
    #                                the holes fix it; it prints on any bed (155 x 200 with the base cut at the joint)
    t: float = 6.0
    flare_r: float = 95.5          # the chamfer's foot on the flange: 4.5 of flat rim outside it
    flare_h: float = 24.0          # ... its top on the wall: ~39.7 degrees from the table
    hole_dia: float = M5_CLEAR     # M5 (or a 5 mm wood screw)
    hole_r: float = 92.5
    spot_dia: float = 11.0         # an M5 washer's 10 and room
    hole_deg: tuple = (112.5, 157.5, 202.5, 247.5)
    side_hole_x: float = 43.0
    mount_hole_deg: tuple = (-67.5, -22.5, 22.5, 67.5)   # the motor mount's 4, on hole_r round its end's centre: the
    #                                                      base's round half's, mirrored


@dataclass(frozen=True)
class YawIdlerParams:
    """[DESIGN] The base_yaw belt's tensioner (cad/AGENTS.md "Belts"): a smooth GT2 idler (parts/joints/gt2_idler_20t) on
    the belt's back on its run from the 120T back to the motor's 20T (the +Z strand), on the belt's long open run over
    the motor mount's plate, `x` out from the base_yaw axis, pushed into the loop until the stock YAW_BELT fits
    YAW_MOTOR_X (lib/belts.py idler_on_run; lib/base/layout.py yaw_belt_loop, yaw_idler_xz). It stands on the plate,
    level with the belt, on a printed spacer (parts/base/yaw_idler_spacer: its riser on the plate's top, its land on the
    idler's bearing inner ring), on an M5 SHCS down from its top (parts/base/yaw_idler_screw) - the head up, turned from
    above: the mount stands on the table - through the idler, the spacer and a slot through the plate into a nut
    (parts/base/yaw_idler_nut) under the plate, held flats-on between two rails grown down from the plate's underside
    (the mount prints plate-down: the rails stand up off it): it slides with the screw. Both run across the belt between
    slot_y's ends, in the belt's frame (lib/base/layout.py yaw_belt_frame: its +Y the base's +Z, turned with the motor's
    small z). Its stations along Y: lib/params.py YAW_IDLER_*."""

    run: int = 1                          # the loop's run it presses (lib/belts.py belt_runs): the 120T back to the 20T, the +Z strand
    x: float = 85.0                       # its axis this far out from the base_yaw axis, along the belt's centre line: on the
    #                                       mount's plate, the nut's slot under it clear of the motor's face
    slot_y: tuple = (-3.1, 24.1)          # its axis' range across the belt (the belt frame's y), the slots' end centres: in to 3.0
    #                                       off the far strand's teeth (the longest belt it takes), out to 1.0 past just touching
    #                                       the open loop's strand
    riser_dia: float = 12.0               # the spacer's riser, on the plate's top (as ElbowIdlerParams.riser_dia) ...
    land_dia: float = 7.0                 # [ESTIMATE] ... its land on the idler's bearing inner ring only (ElbowIdlerParams.land_dia)
    land_h: float = 1.0                   # the land over the riser: the idler's flange clear of the riser
    slot_w: float = M5_CLEAR              # the axle's slot through the plate, the spacer's bore
    screw: ShcsSize = M5_SHCS             # [DATASHEET] ISO 4762 M5 (parts/base/yaw_idler_screw), its head on the idler
    screw_len: float = 25.0               # [DESIGN] M5 x 25: through the idler, the spacer, the plate and the nut, its tip 2 pitches past it
    nut: NutSize = M5_NUT                 # [DATASHEET] ISO 4032 M5 (parts/base/yaw_idler_nut)
    nut_fit: float = 0.2                  # [DESIGN] the nut's slot: its flats + this across the slot, its corners + this past each end
    rail_wall: float = 3.0                # [DESIGN] the rails either side of the nut's slot, and its closed ends
    rail_h: float = 5.0                   # [DESIGN] the rails' depth under the plate: the nut (4.7) between them


@dataclass(frozen=True)
class BaseConfig:
    shell: ShellParams = ShellParams()
    cap: CapParams = CapParams()
    bore: BoreParams = BoreParams()
    plate: PlateParams = PlateParams()
    motor: MotorParams = MotorParams()
    joint: JointParams | None = None   # None: one part, the lobe included (the SolidWorks base)
    mount: MountParams | None = None   # the bolt-on motor mount (with joint)
    foot: FootParams | None = None     # None: the walls stand straight on the table (the SolidWorks base)
    idler: YawIdlerParams | None = None   # the belt's tensioner on the motor mount's plate (with mount)
    slits: SlitParams | None = None    # the slits through the base's and the mount's walls (with mount)


LEGACY = BaseConfig()     # the SolidWorks part, exactly

# What the part builds: the base_yaw bore takes the 6806-2RS pair (lib/bearings.py) like the wrist's bore - both
# seats the wrist's Ø42.2 (the SolidWorks Ø42.4 / Ø43.4 let the bearings wobble, the lower one 0.7 mm a side) and its
# Ø37.65 lip, which stops the outer rings only (the SolidWorks Ø31.73 lip ran under the upper bearing's inner ring).
# The seat ring centres the thrust bearing (the washers' and the cage's bore THRUST_BORE): the SolidWorks Ø65.1 would
# not go into a Ø65 bore; RING_CLEAR a side.
# The motor has a bolt-on mount of its own (base_motor_mount, JointParams + MountParams: in place of the SolidWorks lobe,
# the base's D carried on at full width to a round end that matches the base's, its foot round it - one body with the
# base -, a band of slits through both parts' walls, SlitParams). It sits YAW_MOTOR_X out, short
# of the stock YAW_BELT's own centre distance round the base_yaw pulleys - the 120T on the joint
# (parts/base/gt2_pulley_120t), the 20T on the motor (the SolidWorks centre, 78.97, belonged to a 90T) -, on round holes:
# the belt's tensioner is an idler on its back on the mount's plate (YawIdlerParams), which takes up what the stock belt
# has over the open loop (cad/AGENTS.md "Belts"). The window is the pilot's, WINDOW_CLEAR a side.
# The bottom face (the mounting face, lib/datum.py BASE_BOTTOM_Y) sits BOARD_CLEAR under the motor's MKS board: the
# SolidWorks base's -100.9 left the 48 mm motor + board (48 + 14.1 under the plate) hanging 6.1 below it. The base's
# walls, its posts and the whole motor mount reach down to it.
# The tower is j1_coupler's disc carried on down: the disc is BASE_R at its rim (the coupler's frame origin, the ring's
# top ring_top_y) and drafted in at BASE_DRAFT going up (lib/yaw_coupler/params.py), so the base's walls take the same
# cone on down - r at the cap's top face the cone's there (flush under the rim, RIM_CLEAR above it), BASE_DRAFT
# outward going down - where the SolidWorks Ø106.7 stood under the coupler's Ø116. At the table it stands on the foot
# (FootParams: Ø200, a straight chamfer up the wall, 6 screw holes), the arm's stance: stretched out, its centre of mass
# is 131 mm off the axis (165 with 0.5 kg in the gripper), the SolidWorks base reached 53.
BASE_R = 58.0             # [DESIGN] j1_coupler's disc at its rim (Ø116, the SolidWorks Ø106): room under the fork's legs for their root ...
BASE_DRAFT = 12.0 / 63.0  # [DESIGN] ... its side drafted in at dr/dy (10.8 degrees) going up, the base's walls out going down
RING_CLEAR = 0.1          # [DESIGN]
BOARD_CLEAR = 5.0         # [DESIGN] the base_yaw motor's board above the table
YAW_BELT = 400            # [DESIGN] base_yaw belt, 400-2GT (a multipack length, lib/belts.py STANDARD_2GT_LENGTHS): 20T motor -
#                           120T joint, round its idler (YawIdlerParams)
YAW_BELT_Y = -31.1        # [REFERENCE] the base_yaw belt's mid-plane in the base's frame: the 120T's tooth band (its mount under
#                           j1_coupler, lib/mounts.py) - the motor's 20T level with it (tests/test_mounts.py)
YAW_MOTOR_X = 122.0       # [DESIGN] the base_yaw motor's axis this far out along X: the open loop (392.4) shorter than the stock
#                           belt, the idler (YawIdlerParams) taking up the rest inside its slot - it takes 392.4 .. 405.3
WINDOW_CLEAR = 0.2        # [DESIGN]
_WINDOW_HALF = NEMA17_PILOT_DIA / 2.0 + WINDOW_CLEAR
_BOTTOM_Y = round(LEGACY.plate.y[0] - _DRIVE.motor.body_length - MKS_SERVO42D_STACK - BOARD_CLEAR, 6)
_TOP_R = round(BASE_R + BASE_DRAFT * (LEGACY.cap.ring_top_y - LEGACY.cap.top_y), 6)   # the cone at the cap's top face
DEFAULT = replace(LEGACY, shell=replace(LEGACY.shell, y0=_BOTTOM_Y, r=_TOP_R, draft=BASE_DRAFT),      # [DESIGN]
                  bore=replace(LEGACY.bore, upper_dia=_FOREARM.boss.seat_dia, lower_dia=_FOREARM.boss.seat_dia,
                                       lip_dia=_FOREARM.boss.lip_dia),                                    # [DESIGN]
                  cap=replace(LEGACY.cap, groove_r=(THRUST_BORE / 2.0 - RING_CLEAR, LEGACY.cap.groove_r[1])),   # [DESIGN]
                  motor=replace(LEGACY.motor, centre=(YAW_MOTOR_X, LEGACY.motor.centre[1]), hole_dia=M3_CLEAR,
                                window_x=(round(YAW_MOTOR_X - _WINDOW_HALF, 6), round(YAW_MOTOR_X + _WINDOW_HALF, 6)),
                                window_half_z=_WINDOW_HALF),                                            # [DESIGN]
                  joint=JointParams(), mount=MountParams(), foot=FootParams(), idler=YawIdlerParams(),
                  slits=SlitParams())                                                                  # [DESIGN]
