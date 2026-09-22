"""The simplified NEMA 17 motor solid (the drive repo's build_nema17_motor, spec section 3.1) for any
MotorParams: parts/cycloidal/nema17_48mm.py's envelope, and - via dataclasses.replace - the 40 mm
motor's (parts/joints/nema17_40mm.py).

Frame: mounting face at z=0, the body in -Z, the pilot boss and the shaft in +Z, the D-flat on +Y.
Kernel lazy: `bd.` names only inside the function (lib/cycloidal/geom.py convention).
"""
from __future__ import annotations

from cadgen import build123d as bd

from lib.cycloidal.geom import cylinder, single_solid
from lib.units import NUDGE


def nema17_motor(m, bolt_points) -> bd.Solid:
    """Square body (m.body_width x m.body_length in -Z) + pilot boss + round-then-D-cut shaft in +Z,
    minus the 4 blind M3 holes at `bolt_points` ((x, y) pairs on the mounting face)."""
    shaft_r = m.shaft_dia / 2.0
    body = bd.Box(m.body_width, m.body_width, m.body_length, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MAX))
    pilot = cylinder(m.pilot_dia / 2.0, m.pilot_height)
    round_len = m.shaft_length - m.shaft_dcut_length                      # 4
    shaft_round = cylinder(shaft_r, round_len)
    dcut = cylinder(shaft_r, m.shaft_dcut_length, z0=round_len)
    flat_y = m.shaft_dcut_flat / 2.0                                       # 2.25: material beyond it removed
    dcut = dcut - bd.Pos(0, flat_y + 5.0, round_len + m.shaft_dcut_length / 2.0) * bd.Box(10.0, 10.0, m.shaft_dcut_length + 4 * NUDGE)
    result = body + pilot + shaft_round + dcut
    for xy in bolt_points:
        result = result - cylinder(m.bolt_dia / 2.0, m.bolt_hole_depth + NUDGE, xy, z0=-m.bolt_hole_depth)
    return single_solid(result)
