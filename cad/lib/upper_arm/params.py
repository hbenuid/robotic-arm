"""UpperArmConfig - every dimension of the upper arm, j1_link (the link between the shoulder_pitch and elbow_pitch
axes), in its part frame.

Frame (= the SolidWorks part frame of j1_link, which placements.json places): origin on the shoulder_pitch axis,
+X along the link to the elbow_pitch axis at x = SlabParams.elbow_x, +Y = N (both pitch axes; LEGACY's y = +1.5 top
face bolts to the cycloidal drive's hub, the drive's axis this frame's Y and its +Z this frame's -Y; the elbow motor
hangs off the pad on the -Y side), Z across the link (the width). Every feature is a prism or a bore along Y.

Two configurations: LEGACY reproduces the SolidWorks reference (the part's REFERENCE_BUILD -
tests/test_reference_match.py), DEFAULT is what the part builds: SHORTENING nearer the shoulder at the elbow (every
elbow-end feature moves with the axis - lib/placements.py SHIFTS moves what lies beyond), no cap-locating sockets
(the caps were removed 2026-09-25), no through slots, the NEMA 17 holes on a true square about the pad's axis, the
elbow's clearance for the elbow block (the top face down to the relief's floor, on the floor of a recess 1.5 mm deeper
than the SolidWorks one - level with the upper 6806's top: no recess left), and the drive's turning shell: the arm,
one flat slab from the elbow half's underside to the relief's floor, end to end, narrower than the plate (ArmParams
width, end_r), rises off the shell, printed as one with the shell's body (ArmParams), slid along Y until its underside
is flush with the shell's hub end (the face the part prints on); the elbow motor moved out along the link to
ELBOW_MOTOR_CENTRES from the elbow
axis, where the second stage's seat was (gone), and turned over onto the arm's other side, down a square hole through
the slab onto a web across it, flush with the slab's underside (the pad's tube gone); the elbow belt's tensioner
(ElbowIdlerParams) on the stepped slot's line, its slots in that slot's place.

Every number below was measured on the reference 2026-09-24 (planar / cylindrical face census;
tests/upper_arm/test_j1_link.py re-checks the builds against it): [REFERENCE] unless tagged.
Units mm, degrees where named *_deg. Frozen dataclasses; variants via dataclasses.replace.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

from lib.cycloidal import DEFAULT_CONFIG as _DRIVE
from lib.cycloidal import shell_ends as _shell_ends
from lib.fasteners import M3_CLEAR, M3_SHCS, M5_CLEAR, M5_NUT, M5_SHCS, NutSize, ShcsSize
from lib.motors import NEMA17_BOLT_SP


@dataclass(frozen=True)
class SlabParams:
    """The plate between the two axes: a stadium (r at both ends) from the shoulder axis to the elbow axis, its top
    face inset by a lip (the r 44.5 x 1.5 step the drive's hub bears on) over an r 0.5 round."""

    elbow_x: float = 210.0     # the elbow_pitch axis (the shoulder axis is the origin)
    r: float = 45.0            # both round ends and the half width
    y0: float = -11.771993     # underside (the shoulder half; the elbow half steps down - ElbowParams)
    y1: float = 0.0            # the lip's root
    lip_r: float = 44.5        # the lip: the stadium inset by 0.5 ...
    lip_top: float = 1.5       # ... up to the top face (the cycloidal hub's arm-mount face)
    round_r: float = 0.5       # the outer edge at y1


@dataclass(frozen=True)
class ElbowParams:
    """The elbow half: thicker below (a 45 degree chamfer, then a step down), and the elbow_pitch bearing stack on
    its axis - a recess from the top, the bore, a lip and a seat from below; DEFAULT gives the elbow block, which turns
    over the top face, its floor (`relief_y`): the plate would cut its lip away down to it round the axis
    (`relief_r`), DEFAULT's arm (ArmParams) - a slab from the underside y0 up to that floor - has its whole top there."""

    chamfer_x: float = 145.5          # the chamfer starts at the underside ...
    step_x: float = 153.5             # ... and meets the step wall here (45 degrees: 8 down)
    y0: float = -24.0                 # the elbow half's underside
    recess_dia: float = 80.0          # from the top face ...
    recess_y: float = -4.5            # ... down to here
    bore_dia: float = 42.0            # recess floor .. lip
    lip_dia: float = 37.64
    lip_y: tuple = (-15.0, -13.0)
    seat_dia: float = 42.2            # lip .. underside
    relief_r: float = 0.0             # [DESIGN] the top face down to relief_y within this radius of the elbow axis;
    #                                   0: none (the SolidWorks part)
    relief_y: float = 0.0             # [DESIGN] the relief's floor (0: the lip's root, SlabParams.y1); DEFAULT's slab top


@dataclass(frozen=True)
class PadParams:
    """The elbow motor's pad under the shoulder end: a 48 square tube hanging from the underside (a 45 degree flare
    at its root), a floor with the pilot opening and the 4 NEMA 17 holes, a window through the middle of each wall
    (the +X one runs into the pilot opening - the elbow belt's exit), and R10 fills where the tube meets the
    plate's square opening (a cove along Z on the -X wall, one along X on the +X half of each Z wall). Every feature is
    about the pad's axis at x (the plate's square opening over it too)."""

    x: float = 0.0                         # the pad's axis along the link (the SolidWorks pad: on the shoulder axis)
    half: float = 24.0                     # the tube's outer square, +/-
    face_y: float = -32.5                  # the mounting face (the motor's face bears on it; lib/params.py J1_MOTOR_PAD_FACE_Y)
    flare: float = 3.0                     # the root flare: 45 degrees, `flare` out over `flare` down
    cavity_x: tuple = (-19.25, 21.25)      # the inside: walls 4.75 thick at -X and +/-Z, 2.75 at +X ...
    cavity_half_z: float = 19.25
    floor_y: float = -23.5                 # ... above the 9 thick floor
    pilot_half: float = 13.25              # the floor's square opening (the motor's Ø22 pilot)
    window_half: float = 10.33767          # every wall's window, +/- about its middle
    hole_dia: float = 3.2                  # the M3 clearance holes ...
    holes: tuple = ((15.349, -15.95, 3.2), (15.349, 15.249, 3.2), (-15.65, 15.05, 3.2), (-15.65, -15.751, 3.0))   # (x, z, dia) - LEGACY: 0.38 off the axis, uneven
    cove_r: float = 10.0                   # the fills' radius (their axes: lib/upper_arm/layout.py cove_axes)


@dataclass(frozen=True)
class HubParams:
    """The shoulder end of the plate: the square opening over the pad and the 4 bolts of the cycloidal drive's
    output hub (4x M4 into the hub's captive nuts; lib/cycloidal/layout.py arm_mount_points)."""

    opening_half: float = 21.25       # the square opening through the plate
    bolt_dia: float = 4.4             # M4 clearance
    bolt_circle_dia: float = 50.0     # = the drive's arm_mount_bolt_circle_dia
    bolt_angle_deg: float = 0.775     # the first hole's angle in the XZ plane (atan2(z, x)), then every 90 degrees


@dataclass(frozen=True)
class SlotParams:
    """Stadium slots across the plate (along Z, `half_len` = the end centres' |z|): two through slots of unknown
    purpose (DEFAULT has none: the second stage's seat, moved with the elbow, lands on them), and one at stepped_x,
    47 from the elbow axis, with a 10 wide counterbore from below, across the elbow belt's strands - the elbow belt's
    tensioner slot: DEFAULT's idler (ElbowIdlerParams) has slots of its own in its place, just off its line."""

    through_x: tuple = (70.5, 100.5)
    width: float = 4.0
    through_half_len: float = 24.0
    stepped_x: float = 163.0
    stepped_half_len: float = 25.0
    counterbore_w: float = 10.0
    counterbore_y: float = -10.0      # the counterbore's floor (from the underside up)


@dataclass(frozen=True)
class BearingParams:
    """At x: a Ø22.2 counterbore from each side (608 bearings?) around a Ø8.4 hole through a 2 mm web, the
    lower one in a Ø40 boss proud of the underside - the elbow drive's second stage (an intermediate pulley shaft between
    the pad's motor and the elbow's driven pulley; not modelled - docs/open_issues.md; the SolidWorks capture holds nothing there).
    82 from the elbow axis in LEGACY and _SHORTENED; DEFAULT has none (bearing=None: the elbow belt runs in one stage)."""

    x: float = 128.0
    seat_dia: float = 22.2
    web_y: tuple = (-8.0, -6.0)
    hole_dia: float = 8.4
    boss_dia: float = 40.0
    boss_y: float = -15.0


@dataclass(frozen=True)
class SocketParams:
    """Blind Ø5.15 x 2 locating sockets in the underside - dowel seats for j1_cap, which mirrored them (LEGACY only:
    the cap is gone). (x, z) on the shoulder half's underside, then on the elbow half's."""

    dia: float = 5.15
    depth: float = 2.0
    shoulder: tuple = ((-40.0, 0.0), (0.0, 40.0), (0.0, -40.0), (60.0, 40.0), (60.0, -40.0), (120.0, 40.0), (120.0, -40.0))
    elbow: tuple = ((181.75, 40.0), (181.75, -40.0), (250.0, 0.0))


@dataclass(frozen=True)
class ArmParams:
    """DEFAULT's shoulder end: PAROL6-like, the arm rises straight off the cycloidal drive's turning shell, printed as one
    with the shell's body (lib/cycloidal/housing.py build_shell_body), not bolted to the shell's end. The arm is one flat
    slab, solid (a placeholder for its mass: docs/open_issues.md): `width` across (Z, about the link's centreline) from
    the shoulder axis, its round end `end_r` about the elbow axis - narrower than that, the slab keeps its width over the
    elbow idler's slots and tapers straight onto the end past them (lib/upper_arm/layout.py slab_taper_x), the bend
    `bend_r`; wider, it flares into the end through `bend_r` fillets -, from the elbow half's underside
    (ElbowParams.y0) up to the relief's floor (ElbowParams.relief_y), end to end - the elbow block turns over its top face, the forearm roll drive's motor and
    board in it (they turn with the elbow and never roll: their height along Y is fixed) clear it further up - fused
    into the shell's wall from fuse_r out, where the body's two windows under it are solid (lib/cycloidal/params.py
    ShellParams.arm_windows; its other windows open on the discs). The slab's underside is flush with the shell's hub
    end, the web across the motor hole flush with both: one flat face, the one the part prints on (no support under the
    arm) - the plate as built slides along +Y until its underside is there (lib/upper_arm/layout.py arm_slide), the slab
    with it; what lies beyond the elbow moves with it (lib/placements.py SHIFTS), the elbow's belt stays under the arm,
    past the shell's hub end, level with j1_coupler's hub leg. The
    elbow motor stands on the arm's motor side (+Y, with the forearm), on the pad's axis (PadParams.x): its body down a
    square hole through the slab (motor_hole_half), its face on the web left across the hole (its top at y_outer), held
    by 4 socket head cap screws up through it: motor_plate_t of web under their heads, the heads flush in pockets in its
    underside (motor_screw, motor_pocket_dia), its shaft (-Y) through the web to its 20T in the belt's plane; the pad's
    tube is LEGACY's only. The drive's frame in this one: its axis this frame's Y, its +Z this frame's -Y, its z
    drive_z_at_y0 at y = 0, its +X at drive_x_deg (atan2(z, x))."""

    y_outer: float                   # [DESIGN] the elbow motor's face: the web (motor_plate_t + the heads' head_h) above the
    #                                  shell's hub end (lib/cycloidal/layout.py shell_ends, through drive_z_at_y0), where
    #                                  the slab's underside and the web's are - the face the part prints on
    fuse_r: float = 61.0             # [DESIGN] the arm from here out: in the shell's wall (its bore 54 .. od 64.6), past
    #                                  the housing bolts' holes (60.7; their nut pockets are cut again through the arm)
    motor_plate_t: float = 8.0       # [DESIGN] the web across the elbow motor's hole, under its face, its 4 holes PadParams.holes,
    #                                  to the screws' heads: the M3 x 12 (lib/params.py J1_MOTOR_SCREW_LEN) less the thread in
    #                                  the motor ...
    motor_screw: ShcsSize = M3_SHCS  # [DATASHEET] ... the motor's 4x M3 SHCS, their heads flush in pockets in the web's
    #                                  underside, head_h deep - the web motor_plate_t + head_h thick, flush with the slab ...
    motor_pocket_dia: float = _DRIVE.motor.motor_bolt_head_dia + _DRIVE.tolerances.bolt_clearance_add   # [DESIGN] ... the
    #                                  drive's motor plate's head pockets (5.7)
    motor_pilot_dia: float = 22.5    # [DESIGN] ... the hole for the motor's Ø22 pilot boss, the shaft and its 20T through it
    motor_hole_half: float = 21.6    # [DESIGN] the square hole through the slab round the motor's 42.3 body
    drive_z_at_y0: float = 66.5      # [REFERENCE] the drive's z at this frame's y = 0 (placements.json cycloidal_drive#1,
    #                                  j1_link#1: the SolidWorks hub face bolted to the top face, y 1.5, at the port's z 65)
    drive_x_deg: float = -47.584167  # [REFERENCE] the drive's +X in this frame, atan2(z, x), as the capture pose places it
    width: float = 76.0              # [DESIGN] the slab across, symmetric: 16.4 of it beside the motor's hole, 8.1 past the
    #                                  idler's nut slot at its outer end (tests/upper_arm/test_elbow_idler.py) - the plate's 90
    #                                  slimmed for mass
    end_r: float = 36.0              # [DESIGN] its round end about the elbow axis: 14.9 of wall round the lower 6806's seat,
    #                                  the 108T's flange (r 35.3) under it just inside; the roll frame's octagonal end
    #                                  (lib/forearm/params.py RollDriveParams.end_r) stays the plate's r 45, overhanging it
    bend_r: float = 20.0             # [DESIGN] where the width turns into the taper onto the end (or flares into it)


@dataclass(frozen=True)
class ElbowIdlerParams:
    """[DESIGN] The elbow belt's tensioner (cad/AGENTS.md "Belts"): a smooth GT2 idler (parts/joints/gt2_idler_20t) on
    the belt's back on its run from the 108T back to the motor's 20T (the +Z strand), `along` from the motor's axis -
    beside the stepped slot's line (SlotParams.stepped_x - the SolidWorks slit was for it) -, pushed into the loop until
    the stock ELBOW_BELT fits ELBOW_MOTOR_CENTRES (lib/belts.py idler_on_run; lib/upper_arm/layout.py elbow_belt_loop,
    elbow_idler_x, elbow_idler_z). It hangs
    under the slab, level with the belt, under a printed spacer (parts/joints/elbow_idler_spacer: the slab's underside
    is the face the part prints on, so no riser grows from it - the spacer's riser stands on it, its land on the idler's
    bearing inner ring), on an M5 SHCS up from under it (parts/joints/elbow_idler_screw) through the idler, the spacer
    and a slot through the slab's floor into a nut (parts/joints/elbow_idler_nut) held flats-on in a slot down from the
    slab's top face, on the floor: both slots run along Z (the push) between slot_z's ends, and the nut slides with the
    screw (turned from below). Its stations along N: lib/params.py J1_IDLER_*."""

    run: int = 1                          # the loop's run it presses (lib/belts.py belt_runs): the 108T back to the 20T, the +Z strand
    along: float = 29.1                   # its axis this far from the motor's along the link: 1.9 nearer the motor than the
    #                                       stepped slot's line (31.0), so its flange keeps 5.0 off the 108T's pitch circle at the
    #                                       slots' inner end; the nut's slot 3.4 off the motor's hole (tests/upper_arm/test_elbow_idler.py)
    slot_z: tuple = (-4.0, 25.1)          # its axis' range along Z, the slots' end centres: in to 3.0 off the far strand's teeth
    #                                       (the longest belt it takes), out past just touching the open loop's strand
    riser_dia: float = 12.0               # the spacer's riser, on the slab's underside (as RollIdlerParams.riser_w) ...
    land_dia: float = 7.0                 # [ESTIMATE] ... its land on the idler's bearing inner ring only (RollIdlerParams.land_w)
    land_h: float = 1.0                   # the land over the riser: the idler's flange clear of the riser
    slot_w: float = M5_CLEAR              # the axle's slot through the floor, the spacer's bore
    screw: ShcsSize = M5_SHCS             # [DATASHEET] ISO 4762 M5 (parts/joints/elbow_idler_screw), its head under the idler
    screw_len: float = 25.0               # [DESIGN] M5 x 25: through the idler, the spacer, the floor and the nut, its tip 2 pitches past it
    nut: NutSize = M5_NUT                 # [DATASHEET] ISO 4032 M5 (parts/joints/elbow_idler_nut)
    nut_fit: float = 0.2                  # [DESIGN] the nut's slot: its flats + this across X, its corners + this past each end
    floor: float = 6.5                    # [DESIGN] the slab under the nut's slot (from the underside): the nut's seat


@dataclass(frozen=True)
class UpperArmConfig:
    slab: SlabParams = SlabParams()
    elbow: ElbowParams = ElbowParams()
    pad: PadParams = PadParams()
    hub: HubParams = HubParams()
    slots: SlotParams = SlotParams()
    bearing: BearingParams | None = BearingParams()   # None: no second-stage seat
    sockets: SocketParams | None = SocketParams()
    arm: ArmParams | None = None                      # DEFAULT: the arm rising off the drive's turning shell, not bolted to its hub
    idler: ElbowIdlerParams | None = None             # DEFAULT: the elbow belt's tensioner, its slots in the stepped slot's place


LEGACY = UpperArmConfig()     # the SolidWorks part, exactly

SHORTENING = -40.0   # [DESIGN] the elbow axis this much nearer the shoulder than the SolidWorks part's: the arm's reach
#                      is long for its NEMA 17 drives (the shoulder's and the elbow's holding torque)


def shortened(cfg: UpperArmConfig, dx: float) -> UpperArmConfig:
    """`cfg` with the elbow axis `dx` along X: every feature at the elbow end - the thick half's chamfer and step, the
    counterbored slot, the second stage's seat - moves with it (the relief and the bearing stack sit on the axis
    itself); the shoulder end (the pad, the hub bolts, the through slots) stays."""
    return replace(
        cfg,
        slab=replace(cfg.slab, elbow_x=cfg.slab.elbow_x + dx),
        elbow=replace(cfg.elbow, chamfer_x=cfg.elbow.chamfer_x + dx, step_x=cfg.elbow.step_x + dx),
        slots=replace(cfg.slots, stepped_x=cfg.slots.stepped_x + dx),
        bearing=None if cfg.bearing is None else replace(cfg.bearing, x=cfg.bearing.x + dx),
    )


# What the part builds: SHORTENING at the elbow end; the caps are gone, so are their sockets; the through slots too
# (no use for them was ever found); the motor's 4 holes on the NEMA 17 square about the pad's axis; the hub holes where
# the drive's arm-mount bolts are - the drive's 45 degree pattern (lib/cycloidal/layout.py arm_mount_angles) as the
# capture pose places it in this frame (placements.json cycloidal_drive#1 and j1_link#1; tests/upper_arm/test_j1_link.py
# re-derives it) - until the drive's shell turns (below).
_SHORTENED = shortened(replace(
    LEGACY,
    sockets=None,
    slots=replace(LEGACY.slots, through_x=()),   # [DESIGN]
    pad=replace(LEGACY.pad, holes=tuple((sx * NEMA17_BOLT_SP / 2.0, sz * NEMA17_BOLT_SP / 2.0, M3_CLEAR)
                                        for sx, sz in ((1, -1), (1, 1), (-1, 1), (-1, -1)))),   # [DESIGN] M3 clearance (LEGACY's 3.2)
    hub=replace(LEGACY.hub, bolt_angle_deg=-2.584167),   # [REFERENCE] the drive's bolts, 3.36 degrees from the SolidWorks holes
    # the roll frame (lib/forearm/ RollDriveParams) turns with the elbow over this top face: the top face down to the
    # recess floor (0.5 under the SolidWorks recess's -4.5), level with the upper 6806's top - its seat exactly 7.0
    # deep, the slab no deeper than the elbow's bearing stack, no recess left -; the frame's Ø62 boss rides 2.0 above it,
    # the most the boss can get, its flat underside 11.0 above it out to the tower ELBOW_ROLL_OFFSET across the elbow
    # axis (tests/forearm/test_roll_drive.py)
    elbow=replace(LEGACY.elbow, relief_r=60.0, relief_y=-6.0, recess_y=-6.0),   # [DESIGN]
), SHORTENING)
# The drive's shell turns (lib/cycloidal/params.py ShellParams) and the arm rises off it, flush with its hub end
# (ArmParams), the held hub and the motor's sleeve on the j1_coupler yoke at the shell's two ends - so the elbow motor
# leaves the shoulder axis: its pad moves out along the link to ELBOW_MOTOR_CENTRES from the elbow axis, where the
# second stage's seat was (gone: a stock belt runs the motor's 20T straight to the elbow's 108T, lib/params.py ELBOW_RATIO).
ELBOW_BELT = 300   # [DESIGN] a stock 300-2GT closed belt, the elbow motor's 20T to the elbow's 108T, round its idler
ELBOW_MOTOR_CENTRES = 78.0   # [DESIGN] the elbow motor's axis this far from the elbow's: the open loop (294.2) shorter than the
#                              stock belt, the idler (ElbowIdlerParams) taking up the rest - it takes 294.2 .. 315.7; the
#                              motor's board passes the elbow's roll frame closest at the elbow's lower limit (tests/forearm/test_roll_drive.py)
_ARM = ArmParams(0.0)
_HUB_END_Y = _ARM.drive_z_at_y0 - _shell_ends(_DRIVE)[1]   # 5.5: the shell's hub end, the face the part prints on
DEFAULT = replace(_SHORTENED, bearing=None,
                  arm=replace(_ARM, y_outer=_HUB_END_Y + _ARM.motor_plate_t + _ARM.motor_screw.head_h),   # 16.5
                  pad=replace(_SHORTENED.pad, x=round(_SHORTENED.slab.elbow_x - ELBOW_MOTOR_CENTRES, 6)),   # 92.0
                  idler=ElbowIdlerParams())
