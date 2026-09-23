"""j3_coupler - import wrapper around the SolidWorks reference (reference/j3_coupler.step).

SolidWorks product: 'Joint 2 coupler 62226_J3 Coupler'
Source export:      step/Joint 2 coupler 62226_J3 Coupler.STEP
Reference: mm units, 1 solid(s), volume 55568.5 mm^3,
           bbox size (78, 22, 78) mm, bbox min (-39, 0, -39) mm.
In the arm: x1 (j3_coupler#2, the wrist; j3_coupler#1 at the elbow is RETIRED - lib/placements.py: the forearm roll drive's
block carries its lip / boss / journal / stub since 2026-09-23).

SolidWorks config name 'J3 Coupler'; used at J2 and J3.

Not yet parametric: j3_coupler() returns the reference geometry in the SolidWorks part-file
frame. See parts/_templates/wrapper.py for how to convert it to build123d.
"""
import pathlib

from cadgen import step
from lib import reference
from lib.datum import IDENTITY, to_location

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/<NAME>.step
CONVERTED = False             # True once j3_coupler() is parametric build123d
LOCAL_FROM_REF = IDENTITY   # reference frame -> this part's local frame (identity = SolidWorks frame)


@step
def j3_coupler():
    """Return the reference geometry as a labelled Solid/Compound in this part's local frame."""
    shape = reference.load(REFERENCE).moved(to_location(LOCAL_FROM_REF))
    shape.label = NAME
    return shape


if __name__ == "__main__":
    j3_coupler()   # build: writes the sibling j3_coupler.step (preview: ./cadtool show parts/joints/j3_coupler.py)
