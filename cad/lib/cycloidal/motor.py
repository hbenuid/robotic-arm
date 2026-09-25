"""The NEMA 17 motor geometry of the drive repo's build_nema17_motor (spec section 3.1) for any
MotorParams - THE shaft and pilot boss of every motor in the arm.

nema17_motor() is the simplified whole motor: parts/cycloidal/nema17_48mm.py's envelope, and - via
dataclasses.replace - the 40 mm motor's (parts/joints/nema17_40mm.py). pilot() and shaft() are the
interface features on their own: tools/reference/split_mks_motor.py fuses them onto the real motor
bodies of the SolidWorks kit exports (whose own bosses and shafts are cut off at the mounting face),
so the vendor motors carry exactly the drive's shaft - the one the eccentric shaft's D-bore
(parts/cycloidal/cycloidal_eccentric_shaft.py) was designed around.

Frame: mounting face at z=0, the body in -Z, the pilot boss and the shaft in +Z, the D-flat on +Y.
D-flat convention (the drive repo's, kept as built): the flat lies m.shaft_dcut_flat / 2 from the
axis, i.e. flat-to-round = shaft radius + flat / 2 (4.75 for the 5 mm shaft); the eccentric shaft's
D-bore is cut the same way (+ d_bore_clearance_add). A standard 5 mm D-shaft (and the kit exports)
has 4.5 flat-to-round (flat 2.0 from the axis) - change both together if that is ever adopted.
Kernel lazy: `bd.` names only inside the functions (lib/geom.py convention).
"""
from __future__ import annotations

from cadgen import build123d as bd

from lib.geom import cylinder, single_solid
from lib.units import NUDGE


def flat_offset(m) -> float:
    """Distance of the D-flat plane from the shaft axis (the drive repo's convention, see above)."""
    return m.shaft_dcut_flat / 2.0


def pilot(m, bore: float | None = None) -> bd.Solid:
    """The Ø pilot_dia x pilot_height centring boss standing on the mounting face (z 0..pilot_height),
    optionally with a Ø `bore` hole for a separately modelled shaft."""
    boss = cylinder(m.pilot_dia / 2.0, m.pilot_height)
    if bore:
        boss = boss - cylinder(bore / 2.0, m.pilot_height + 2 * NUDGE, z0=-NUDGE)
    return single_solid(boss)


def shaft(m) -> bd.Solid:
    """The Ø shaft_dia x shaft_length shaft from the mounting face: round for
    shaft_length - shaft_dcut_length, then the D-cut to the tip, flat on +Y."""
    shaft_r = m.shaft_dia / 2.0
    round_len = m.shaft_length - m.shaft_dcut_length                      # 4
    shaft_round = cylinder(shaft_r, round_len)
    dcut = cylinder(shaft_r, m.shaft_dcut_length, z0=round_len)
    flat_y = flat_offset(m)                                                # 2.25: material beyond it removed
    dcut = dcut - bd.Pos(0, flat_y + 5.0, round_len + m.shaft_dcut_length / 2.0) * bd.Box(10.0, 10.0, m.shaft_dcut_length + 4 * NUDGE)
    return single_solid(shaft_round + dcut)


def nema17_motor(m, bolt_points) -> bd.Solid:
    """Square body (m.body_width x m.body_length in -Z) + pilot() + shaft(), minus the 4 blind M3 holes
    at `bolt_points` ((x, y) pairs on the mounting face)."""
    body = bd.Box(m.body_width, m.body_width, m.body_length, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MAX))
    result = body + pilot(m) + shaft(m)
    for xy in bolt_points:
        result = result - cylinder(m.bolt_dia / 2.0, m.bolt_hole_depth + NUDGE, xy, z0=-m.bolt_hole_depth)
    return single_solid(result)
