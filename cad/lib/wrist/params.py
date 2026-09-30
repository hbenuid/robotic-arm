"""WristConfig - every dimension of the wrist body, wrist_link (the wrist_pitch stage: j3_coupler#2's flange bolts to
its plate on the pitch axis, gripper_clamp_bracket to its block's end face on the roll axis), in its part frame.

Frame (= the SolidWorks part frame of wrist_link, which placements.json places): origin on the plate's underside at
the centre of the tower's round end, +Z up through the plate toward the coupler and the forearm; the wrist_pitch axis
is the line (PlateParams.pitch_x, 0) along Z, the wrist_roll axis the line (y 0, z EndFaceParams.axis_z) along X,
toward +X (the pancake motor, the gripper). Symmetric about y = 0 but for the back opening (TowerParams.back_y).

The body: a plate with a round end on each axis - j3_coupler's seat on the pitch axis (a through hole clear of the
wrist 90T's nuts, the flange's 4 M4 into hex nut pockets in the underside); on the other end the tower: an R39 wall
round a bore, a wedge under a slope inside it (the slope's edge with the bore filleted), cheeks either side of a slot
in the bore, and a block out to the end face the bracket bolts to - the NEMA 17 pattern's M3 and the bracket's M4,
drilled through the whole part along X (the SolidWorks through-all: the z 6.5 pair score the plate's top).

Two configurations: LEGACY reproduces the SolidWorks reference (the part's REFERENCE_BUILD -
tests/test_reference_match.py); DEFAULT is what the part builds: the block SHORTENING shorter (its end face nearer
the pitch axis - lib/placements.py SHIFTS moves what bolts to it), the slope steeper so the weight-saving pocket
behind it stays inside the shorter block, the NEMA 17 pattern's upper pair of M3 dropped (nothing uses them; they
would run through the pocket).

Every number below was measured on the reference 2026-09-27 (face / vertex census; tests/wrist/test_wrist_link.py
re-checks the builds against it): [REFERENCE] unless tagged. SolidWorks' float noise (<= 6e-4) is rounded off the end
face's holes and the block. Units mm. Frozen dataclasses; variants via dataclasses.replace.
"""
from __future__ import annotations

from dataclasses import dataclass, replace


@dataclass(frozen=True)
class PlateParams:
    """The plate, z 0..thickness: a round end on the pitch axis, flats at y = +/-half_width, a round end on the origin
    (the tower's wall). SolidWorks drew the pitch end 0.0007 over the flats' half width: the flats clip it."""

    thickness: float = 5.0
    pitch_x: float = -45.445774        # the wrist_pitch axis (x, 0), j3_coupler#2's
    pitch_r: float = 39.000699         # the round end on it
    half_width: float = 39.0


@dataclass(frozen=True)
class SeatParams:
    """j3_coupler's seat on the pitch axis: a through hole (clear of the wrist 90T's nuts in the coupler flange's
    underside), the flange's 4 M4 on the axes at bolt_r - holes from the plate's top down to hex nut pockets
    nut_depth deep in the underside, a flat toward the axis."""

    hole_dia: float = 29.885736
    bolt_r: float = 35.0
    bolt_dia: float = 4.1
    nut_af: float = 6.85
    nut_depth: float = 2.0


@dataclass(frozen=True)
class TowerParams:
    """The tower on the origin end: the wall (r wall_r) from its back face at x0 on, z1 high, round a bore (r bore_r)
    open above a slope - the plane from (slope_x0, the plate's top) up to (slope_x1, z1), the bore's edge on it filleted
    fillet_r (a wedge of material under it); cheeks inside the bore either side of a slot (y = +/-slot_half), their
    front faces at cheek_x (the back of the bore behind them opened down to the slope - on +Y out to back_y, on -Y to
    the bore: SolidWorks' one asymmetry), up to cheek_z1; the block the bracket bolts to, out to its end face at
    block_x1, +/-block_half_width wide, block_z high."""

    x0: float = 0.948349
    z1: float = 44.0
    wall_r: float = 39.0
    bore_r: float = 30.0
    slope_x0: float = 2.407088
    slope_x1: float = 30.0             # at z1 the slope touches the bore (bore_r)
    fillet_r: float = 10.0
    slot_half: float = 21.0
    cheek_x: float = 5.0
    cheek_z1: float = 43.0
    back_y: float = 30.0
    block_x1: float = 40.0             # the end face: gripper_clamp_bracket's seat
    block_half_width: float = 32.0
    block_z: tuple = (1.0, 43.0)


@dataclass(frozen=True)
class EndFaceParams:
    """The end face's holes, on the roll axis (y 0, z axis_z): the NEMA 17 pattern's M3 on an m3_sp square (the
    bracket's motor) - the rows at z = axis_z + r * m3_sp / 2 for r in m3_rows -, the bracket's M4 at y = +/-m4_y,
    z = axis_z +/- m4_dz; every one drilled along X through the whole part."""

    axis_z: float = 22.0
    m3_sp: float = 31.0
    m3_dia: float = 3.2
    m3_rows: tuple = (-1.0, 1.0)
    m4_y: float = 26.5
    m4_dz: float = 15.5
    m4_dia: float = 4.2


@dataclass(frozen=True)
class WristConfig:
    plate: PlateParams = PlateParams()
    seat: SeatParams = SeatParams()
    tower: TowerParams = TowerParams()
    end_face: EndFaceParams = EndFaceParams()


LEGACY = WristConfig()     # the SolidWorks part, exactly

SHORTENING = -20.0   # [DESIGN] the end face this much nearer the pitch axis than the SolidWorks part's: the arm's
#                      reach is long for its NEMA 17 drives

# What the part builds: the block SHORTENING shorter; the slope's top at slope_x1 14 - the steepest whose cut stays
# inside the fillet's core (slope_x at the core's top < block_x1 + 1), 6 mm of block left over the pocket at the top;
# the upper M3 pair gone.
DEFAULT = replace(
    LEGACY,
    tower=replace(LEGACY.tower, block_x1=LEGACY.tower.block_x1 + SHORTENING, slope_x1=14.0),   # [DESIGN]
    end_face=replace(LEGACY.end_face, m3_rows=(-1.0,)),                                          # [DESIGN]
)
