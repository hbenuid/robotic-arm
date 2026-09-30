"""j2_link - the forearm web: the link between the elbow_pitch and wrist_pitch pivots (parametric build123d,
lib/forearm/link.py build_link(cfg), in the SolidWorks part frame - origin on the elbow pivot, the wrist pivot at
x = WebParams.wrist_x, +Z = N toward the motor-body side).

Elbow end (DEFAULT, the forearm roll): the flange wall at x -56..-48 - full width, z -10..60 - with the roll shaft's
Ø40 x 2 spigot recess on its elbow face, 4x M3 on Ø32 and the Ø24 cable bore on the roll axis (y 0, z 25 = the wrist
centre's N-station); the elbow block (parts/joints/forearm_roll_block) is the elbow coupler now, so no disc and no
j3_coupler#1. LEGACY: the Ø90 disc whose z=0 face bolts to j3_coupler#1 (Ø54.89 bore, 4x M4 into
captive hex nuts dropped in from the top). Web z 8..19 with the wrist-pitch motor's slide (a central slot for the
pilot boss - 22.3 wide in DEFAULT, 20 in LEGACY -, two 3.2 mm side slots for the 31 mm bolt square, shortened in
DEFAULT to the slide range the stock wrist belt allows; lib/mounts.py places nema17_40mm#3 on it) and, in LEGACY only,
the Ø5.18 x 2 locating sockets of the removed caps (j2_cap_1 / j2_cap_2, gone 2026-09-25). Wrist end: the Ø90 boss with the Ø42.2 wrist_pitch bearing seat (lipped) and the
Ø80 recess. Every number: lib/forearm/params.py (ForearmConfig; measured on the reference 2026-09-22).

SolidWorks product: 'Joint 2 change 8126'
Source export:      step/Joint 2 change 8126.STEP
Reference: mm units, 1 solid(s), volume 285470.5 mm^3,
           bbox size (300, 90, 33.5) mm, bbox min (-255, -45, 0) mm.
In the arm: x1 (j2_link#1).

Conversion: build_link(LEGACY) reproduces the reference (REFERENCE_BUILD - tests/test_reference_match.py);
the model builds DEFAULT (tests/forearm/ lock what DEFAULT adds).
"""
import pathlib

from cadgen import step

from lib.datum import IDENTITY
from lib.forearm import DEFAULT, LEGACY
from lib.forearm.link import build_link

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/solidworks/<NAME>.step
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled in the SolidWorks part frame
REF_BBOX_TOL = 0.02


def REFERENCE_BUILD():
    """The configuration that reproduces the SolidWorks part (the reference-match lock)."""
    return build_link(LEGACY)


@step
def j2_link():
    """The forearm web at its local origin (the elbow pivot); the assembly owns placement."""
    part = build_link(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    j2_link()   # build: writes the sibling j2_link.step
