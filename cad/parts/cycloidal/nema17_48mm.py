"""nema17_48mm - NEMA 17 stepper, 48 mm body, 5 mm D-shaft (the drive's input motor).

Simplified model from cycloidal_drive@2f1f67d src/purchased_parts.py (build_nema17_motor):
42.3 mm square body in -Z, 22 mm pilot boss and the 22 mm shaft (4 mm round + 18 mm D-cut, flat
on +Y) in +Z, 4x M3 blind holes on the 31 mm square. Mounting face at z=0 - the drive's stack
datum. reference/nema17_48mm.step is that builder's export (kind "cots"); vendor/nema17_48mm.step,
when present, is a step.parts catalog model re-oriented by VENDOR_TO_REF into this frame.
Not the wrist's pancake motor (parts/nema17_pancake.py).
"""
import pathlib

from build123d import Align, Box, Location, Pos
from cadgen import step
from lib.cycloidal import DEFAULT_CONFIG, DriveConfig, motor_bolt_points
from lib.cycloidal.geom import cylinder, single_solid
from lib.params import CYCLOIDAL_MOTOR_MASS_G, NUDGE
from parts.cycloidal._cots import hybrid

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = CYCLOIDAL_MOTOR_MASS_G
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = Location()   # set after inspecting a step.parts model (see vendor/README.md)


def _envelope(cfg: DriveConfig = DEFAULT_CONFIG):
    m = cfg.motor
    shaft_r = m.shaft_dia / 2.0
    body = Box(m.body_width, m.body_width, m.body_length, align=(Align.CENTER, Align.CENTER, Align.MAX))
    pilot = cylinder(m.pilot_dia / 2.0, m.pilot_height)
    round_len = m.shaft_length - m.shaft_dcut_length                      # 4
    shaft_round = cylinder(shaft_r, round_len)
    dcut = cylinder(shaft_r, m.shaft_dcut_length, z0=round_len)
    flat_y = m.shaft_dcut_flat / 2.0                                       # 2.25: material beyond it removed
    dcut = dcut - Pos(0, flat_y + 5.0, round_len + m.shaft_dcut_length / 2.0) * Box(10.0, 10.0, m.shaft_dcut_length + 4 * NUDGE)
    result = body + pilot + shaft_round + dcut
    for xy in motor_bolt_points(cfg):
        result = result - cylinder(m.bolt_dia / 2.0, m.bolt_hole_depth + NUDGE, xy, z0=-m.bolt_hole_depth)
    return single_solid(result)


@step
def nema17_48mm():
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    nema17_48mm()   # build: writes the sibling nema17_48mm.step (preview: ./cadtool show parts/cycloidal/nema17_48mm.py)
