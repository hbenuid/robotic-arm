"""j1_coupler - the base_yaw coupler: the yoke that turns on the base about the base_yaw axis and cradles the cycloidal
drive's housing (parametric build123d, lib/yaw_coupler/body.py build_yaw_coupler(cfg), in the SolidWorks part frame -
origin on the base_yaw axis at the base's seat-ring top, +Y up the axis, the drive's axis along X at y 90).

A drafted disc (flats at x +/-40, ears of a Ø96 disc beyond them, a Ø90.05 recess in its underside) on a Ø29.8 stub
into the upper base bearing, a Ø15 bore and 4x Ø3.3 holes on the diagonals up to the pocket over the hub; on the disc
a ring and the yoke flaring out of it: the -X cheek and the middle body under the housing's Ø116 cradle, a channel
and two V-grooves round the housing's pillars, the nut pockets of 3 housing bolts in the cheek's outer face. Every
number: lib/yaw_coupler/params.py (YawCouplerConfig; measured on the reference 2026-09-27).

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
