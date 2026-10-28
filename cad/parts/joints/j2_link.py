"""j2_link - the forearm web: the link between the elbow_pitch and wrist_pitch pivots (parametric build123d,
lib/forearm/link.py build_link(cfg), in the SolidWorks part frame - origin on the elbow pivot, the wrist pivot at
x = WebParams.wrist_x, +Z = N toward the motor-body side).

Elbow end (DEFAULT, the forearm roll): the flange wall at x -56..-48 - a Ø60 round flange on the roll axis (y 0, z 25
= the capture's wrist centre N-station, whole all round under the roll pulley's ring) standing on a 60 mm foot, the
web's end, level with its round bottom; its elbow-side band cut back to RollEndParams.relief_r about the roll axis
(the roll tensioner's screw head passes there as the forearm rolls) - with the roll pulley's Ø40 x 2 spigot recess on
its elbow face, 4x M3 on Ø31 (the bottom one's head just over the web) and the Ø24 cable bore on the roll axis, braced
by two gussets on the web either side of the wrist motor; the web runs straight from the wall to the wrist, as wide as
the wall, no taper. The roll frame
(parts/joints/forearm_roll_block) is the elbow coupler now, so no disc and no j3_coupler#1; the forearm sits
ELBOW_ROLL_OFFSET across the elbow axis (lib/placements.py SHIFTS). LEGACY: the Ø90 disc whose
z=0 face bolts to j3_coupler#1 (Ø54.89 bore, 4x M4 into
captive hex nuts dropped in from the top). Web z 8..19 in LEGACY, WEB_DROP lower in DEFAULT (toward the belt side, the
boss and the wrist with it: -5..6), with the wrist-pitch motor's seat - in LEGACY a slide (a
central 20 mm slot for the pilot boss, two 3.2 mm side slots for the 31 mm bolt square), in DEFAULT round holes
(lib/forearm/params.py MotorHoleParams) WRIST_MOTOR_CENTRES from the wrist, the 31 mm square's counterbored in the
underside for the motor's 4x M3 (wrist_motor_screws, heads flush); lib/mounts.py places nema17_40mm#3 on it -,
in DEFAULT the wrist belt's idler mount beside it (a rib under the web, an M5 slot along y through both, the nut's
channel down from the top face - lib/forearm/params.py IdlerParams) and, in LEGACY only,
the Ø5.18 x 2 locating sockets of the removed caps (j2_cap_1 / j2_cap_2, gone 2026-09-25). Wrist end: the boss with the
Ø42.2 wrist_pitch bearing seat (lipped) - LEGACY Ø90 with the Ø80 recess j3_coupler's flange dips into, DEFAULT as wide
as the web (its round end), ending at the seat's top under the coupler's dust lip, no recess. Every number:
lib/forearm/params.py (ForearmConfig; measured on the reference 2026-09-22).

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


@step(out="j2_link.step")
def j2_link():
    """The forearm web at its local origin (the elbow pivot); the assembly owns placement."""
    part = build_link(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    j2_link()   # build: writes the sibling j2_link.step
