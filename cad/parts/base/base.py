"""base - the arm's foot and the base_yaw housing (parametric build123d, lib/base/body.py build_base(cfg), in the
SolidWorks part frame - origin on the base_yaw axis, +Y up it, the bottom face at y = -100.9, the round half on -X).

A D-shaped wall (r 53.34 round half, straight sides to the flat +X end, 5 thick) from the bottom face up to a 5 mm
motor plate at y -44.9..-39.9; above the plate the round half and the sides' stubs carry on up to the cap, open over
the plate on the +X side (the base_yaw belt's room). The cap: a 45 degree chamfer under it, the bearing bore on the
axis (the upper seat from the seat ring's top, the lip, the lower seat through a Ø63.47 boss - DEFAULT: Ø42.2 / Ø37.65
/ Ø42.2 for the 6806-2RS pair, the wrist's; LEGACY: Ø42.4 / Ø31.73 / Ø43.4), an annular groove round the raised seat
ring for the base_yaw thrust bearing j1_coupler turns on (DEFAULT: the ring Ø64.8 inside the bearing's Ø65 bore;
LEGACY: Ø65.1). The plate: a central opening that runs out to the
wall on the -X side, a curved slot on +X, and the 48 mm base_yaw motor's seat on its underside (4 holes on the NEMA 17
square, the pilot window, the belt slot toward the axis, a U rim round the motor's face; lib/mounts.py places
nema17_48mm#1 on it). A cable notch through the +X end at the bottom. Every number: lib/base/params.py (BaseConfig;
measured on the reference 2026-09-25).

SolidWorks product: 'base of robot arm 62126'
Source export:      step/base of robot arm 62126.STEP
Reference: mm units, 1 solid(s), volume 286169.1 mm^3,
           bbox size (168.178, 95.807, 106.679) mm, bbox min (-53.339, -100.9, -53.339) mm.
In the arm: x1 (base#1).

Conversion: build_base(LEGACY) reproduces the reference (REFERENCE_BUILD - tests/test_reference_match.py); the
model builds DEFAULT (tests/base/ lock what DEFAULT changes).
"""
import pathlib

from cadgen import step

from lib.base import DEFAULT, LEGACY
from lib.base.body import build_base
from lib.datum import IDENTITY

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/solidworks/<NAME>.step
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled in the SolidWorks part frame


def REFERENCE_BUILD():
    """The configuration that reproduces the SolidWorks part (the reference-match lock)."""
    return build_base(LEGACY)


@step
def base():
    """The base at its local origin (the base_yaw axis); the assembly owns placement."""
    part = build_base(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    base()   # build: writes the sibling base.step
