"""j3_coupler - the driven side of the wrist_pitch belt joint (parametric build123d, lib/coupler/body.py
build_coupler(cfg), in the SolidWorks part frame - origin on the joint axis at the flange's underside, +Y up the axis
toward the stub's end).

A Ø78 flange (4x M4 counterbored on the axes at r 35), a Ø58/62 dust-lip ring on its top, the Ø40 journal and the
Ø30 stub the wrist bearings turn on (DEFAULT: a Ø33 shoulder between them on the upper 6806's inner ring, and the
stub 3 mm longer, on through the lip to the re-seated pulley), a Ø12.5 bore; the wrist 90T bolts flat onto the
stub's end with 4x M4 on the axes at r 11, their nuts in hex pockets in the flange's underside. Every number: lib/coupler/params.py (CouplerParams;
measured on the reference 2026-09-25).

SolidWorks product: 'Joint 2 coupler 62226_J3 Coupler'
Source export:      step/Joint 2 coupler 62226_J3 Coupler.STEP
Reference: mm units, 1 solid(s), volume 55568.5 mm^3,
           bbox size (78, 22, 78) mm, bbox min (-39, 0, -39) mm.
In the arm: x1 (j3_coupler#2, the wrist; j3_coupler#1 at the elbow is RETIRED - lib/placements.py: the forearm roll drive's
block carries its lip / boss / journal / stub since 2026-09-23).

Conversion: build_coupler(LEGACY) reproduces the reference (REFERENCE_BUILD - tests/test_reference_match.py); the
model builds DEFAULT (tests/coupler/ lock what DEFAULT changes).
"""
import pathlib

from cadgen import step

from lib.coupler import DEFAULT, LEGACY
from lib.coupler.body import build_coupler
from lib.datum import IDENTITY

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/solidworks/<NAME>.step
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled in the SolidWorks part frame


def REFERENCE_BUILD():
    """The configuration that reproduces the SolidWorks part (the reference-match lock)."""
    return build_coupler(LEGACY)


@step
def j3_coupler():
    """The coupler at its local origin (the flange's underside on the joint axis); the assembly owns placement."""
    part = build_coupler(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    j3_coupler()   # build: writes the sibling j3_coupler.step
