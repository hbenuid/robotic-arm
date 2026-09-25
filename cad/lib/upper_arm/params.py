"""UpperArmConfig - every dimension of the upper arm, j1_link (the link between the shoulder_pitch and elbow_pitch
axes), in its part frame.

Frame (= the SolidWorks part frame of j1_link, which placements.json places): origin on the shoulder_pitch axis,
+X along the link to the elbow_pitch axis at x = elbow_x (210), +Y = N (both pitch axes; the cycloidal drive's hub
bolts onto the y = +1.5 top face, the elbow motor hangs off the pad on the -Y side), Z across the link (the width).
Every feature is a prism or a bore along Y.

Two configurations: LEGACY reproduces the SolidWorks reference (the part's REFERENCE_BUILD -
tests/test_reference_match.py), DEFAULT is what the part builds: no cap-locating sockets (the caps were removed
2026-09-25), the NEMA 17 holes on a true square about the shoulder axis and the hub holes on the drive's pattern.

Every number below was measured on the reference 2026-09-24 (planar / cylindrical face census;
tests/upper_arm/test_j1_link.py re-checks the builds against it): [REFERENCE] unless tagged.
Units mm, degrees where named *_deg. Frozen dataclasses; variants via dataclasses.replace.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

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
    its axis - a recess from the top, the bore, a lip and a seat from below."""

    chamfer_x: float = 145.5          # the chamfer starts at the underside ...
    step_x: float = 153.5             # ... and meets the step wall here (45 degrees: 8 down)
    y0: float = -24.0                 # the elbow half's underside
    recess_dia: float = 80.0          # from the top face ...
    recess_y: float = -4.5            # ... down to here
    bore_dia: float = 42.0            # recess floor .. lip
    lip_dia: float = 37.64
    lip_y: tuple = (-15.0, -13.0)
    seat_dia: float = 42.2            # lip .. underside


@dataclass(frozen=True)
class PadParams:
    """The elbow motor's pad under the shoulder end: a 48 square tube hanging from the underside (a 45 degree flare
    at its root), a floor with the pilot opening and the 4 NEMA 17 holes, a window through the middle of each wall
    (the +X one runs into the pilot opening - the elbow belt's exit), and R10 fills where the tube meets the
    plate's square opening (a cove along Z on the -X wall, one along X on the +X half of each Z wall)."""

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
    purpose, and one at x 163 with a 10 wide counterbore from below (next to the elbow belt's strands - a
    tensioner slot?)."""

    through_x: tuple = (70.5, 100.5)
    width: float = 4.0
    through_half_len: float = 24.0
    stepped_x: float = 163.0
    stepped_half_len: float = 25.0
    counterbore_w: float = 10.0
    counterbore_y: float = -10.0      # the counterbore's floor (from the underside up)


@dataclass(frozen=True)
class BearingParams:
    """At x 128: a Ø22.2 counterbore from each side (608 bearings?) around a Ø8.4 hole through a 2 mm web, the
    lower one in a Ø40 boss proud of the underside. Purpose unknown (the SolidWorks capture holds nothing there)."""

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
class UpperArmConfig:
    slab: SlabParams = SlabParams()
    elbow: ElbowParams = ElbowParams()
    pad: PadParams = PadParams()
    hub: HubParams = HubParams()
    slots: SlotParams = SlotParams()
    bearing: BearingParams = BearingParams()
    sockets: SocketParams | None = SocketParams()


LEGACY = UpperArmConfig()     # the SolidWorks part, exactly

# What the part builds: the caps are gone, so are their sockets; the motor's 4 holes on the NEMA 17 square about the
# shoulder axis (the motor is placed there - lib/mounts.py nema17_40mm#2); the hub holes where the drive's arm-mount
# bolts are - the drive's 45 degree pattern (lib/cycloidal/layout.py arm_mount_angles) as the capture pose places it
# in this frame (placements.json cycloidal_drive#1 and j1_link#1; tests/upper_arm/test_j1_link.py re-derives it).
DEFAULT = replace(
    LEGACY,
    sockets=None,
    pad=replace(LEGACY.pad, holes=tuple((sx * NEMA17_BOLT_SP / 2.0, sz * NEMA17_BOLT_SP / 2.0, LEGACY.pad.hole_dia)
                                        for sx, sz in ((1, -1), (1, 1), (-1, 1), (-1, -1)))),   # [DESIGN]
    hub=replace(LEGACY.hub, bolt_angle_deg=-2.584167),   # [REFERENCE] the drive's bolts, 3.36 degrees from the SolidWorks holes
)
