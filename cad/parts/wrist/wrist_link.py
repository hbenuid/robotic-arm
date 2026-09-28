"""wrist_link - the wrist body, the wrist_pitch stage (parametric build123d, lib/wrist/link.py build_wrist_link(cfg), in
the SolidWorks part frame - origin on the plate's underside at the centre of the tower's round end, +Z up through the
plate; the pitch axis along Z at x -45.446, the roll axis along X at z 22).

A 5 mm plate with a round end on each axis: j3_coupler#2's seat on the pitch axis (a Ø29.9 hole clear of the wrist
90T's nuts, the flange's 4x M4 at r 35 on the axes into hex nut pockets in the underside); on the other end the
tower - an R39 wall round an R30 bore, a wedge under a slope inside it (its edge with the bore filleted R10), cheeks
either side of a 42 mm slot, a 64 x 42 block out to the end face gripper_clamp_bracket bolts to (the NEMA 17
pattern's 4x M3 and the bracket's 4x M4, drilled along X through the whole part). Every number: lib/wrist/params.py
(WristConfig; measured on the reference 2026-09-27).

SolidWorks product: 'final component arm qwrist movement'
Source export:      step/final component arm qwrist movement.STEP
Reference: mm units, 1 solid(s), volume 113601.4 mm^3,
           bbox size (124.446, 78, 44) mm, bbox min (-84.446, -39, -0) mm.
In the arm: x1 (wrist_link#1).

Conversion: build_wrist_link(LEGACY) reproduces the reference (REFERENCE_BUILD - tests/test_reference_match.py); the
model builds DEFAULT (= LEGACY; tests/wrist/ lock the features).
"""
import pathlib

from cadgen import step

from lib.datum import IDENTITY
from lib.wrist import DEFAULT, LEGACY
from lib.wrist.link import build_wrist_link

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/solidworks/<NAME>.step
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled in the SolidWorks part frame


def REFERENCE_BUILD():
    """The configuration that reproduces the SolidWorks part (the reference-match lock)."""
    return build_wrist_link(LEGACY)


@step
def wrist_link():
    """The wrist body at its local origin (the plate's underside, the tower's axis); the assembly owns placement."""
    part = build_wrist_link(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    wrist_link()   # build: writes the sibling wrist_link.step
