"""j1_coupler - the base_yaw coupler: the yoke that turns on the base about the base_yaw axis and holds the cycloidal
drive (parametric build123d, lib/yaw_coupler/body.py build_yaw_coupler(cfg), in the SolidWorks part frame -
origin on the base_yaw axis at the base's seat-ring top, +Y up the axis, the drive's axis along X at y 90).

A drafted disc (flats at x +/-40, ears of a Ø96 disc beyond them, a recess in its underside - DEFAULT: Ø90.4, its
ceiling the seat on the base_yaw thrust bearing, the rim 0.5 over the base's top face) on a stub through the upper base
bearing (DEFAULT: Ø30, on to the lip's lower face, where the base_yaw 120T's hub end meets it), a bore (DEFAULT: the
pulley's Ø12.5) and 4 holes on the diagonals up to the pocket over the hub (DEFAULT: the 90T's bolt circle, M4 clearance,
the nuts flush in hex pockets in the pocket's floor); on the disc a ring (DEFAULT: lowered under the drive's turning
shell) and DEFAULT's fork round the drive (ForkParams) on the drive's own axis: two alike thin legs past the turning
shell's two ends, centred on the base_yaw axis with the drive's discs, each a disc round the axis on a leg down to the
disc, straddling a flat on a low bridge - the -X one the held hub bolts to, the +X one round the motor plate's sleeve,
split at the axis; its upper half the cap, parts/base/j1_coupler_cap, on 2 M3s into nuts in the leg's side slots; the
ring under the drive flat to flat. Every number: lib/yaw_coupler/params.py
(YawCouplerConfig; measured on the reference 2026-09-27; LEGACY: the Ø90.05 recess on the base's face, a Ø29.8 stub
0.2 into the lip, a Ø15 bore, 4x Ø3.3, the -X cheek and the middle body under the housing's Ø116 cradle, a channel and
two V-grooves round the 8-pillar housing's bottom and +/-45 degree pillars, 3 nut pockets).

SolidWorks product: 'Base couple updated 62126 _J1 coupler'
Source export:      step/Base couple updated 62126 _J1 coupler.STEP
Reference: mm units, 1 solid(s), volume 199961.4 mm^3,
           bbox size (96, 63.976, 106.264) mm, bbox min (-48, -8.2, -53.132) mm.
In the arm: x1 (j1_coupler#1).

Conversion: build_yaw_coupler(LEGACY) reproduces the reference (REFERENCE_BUILD - tests/test_reference_match.py); the
model builds DEFAULT (tests/yaw_coupler/).
"""
import pathlib

from cadgen import step

from lib.datum import IDENTITY
from lib.yaw_coupler import DEFAULT, LEGACY
from lib.yaw_coupler.body import build_yaw_coupler

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/solidworks/<NAME>.step
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled in the SolidWorks part frame


def REFERENCE_BUILD():
    """The configuration that reproduces the SolidWorks part (the reference-match lock)."""
    return build_yaw_coupler(LEGACY)


@step
def j1_coupler():
    """The coupler at its local origin (on the base_yaw axis at the seat-ring top); the assembly owns placement."""
    part = build_yaw_coupler(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    j1_coupler()   # build: writes the sibling j1_coupler.step
